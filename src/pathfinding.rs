use bevy::log::info;
use bevy::prelude::{Vec2, Reflect};
use bevy::platform::collections::{HashSet, HashMap};
use bevy::platform::collections::hash_map::Entry;
use serde::{Serialize, Deserialize};
use smallvec::SmallVec;
use std::{
    cmp::Ordering,
    fmt::Debug,
    sync::Arc
};
use std::fmt;
use std::collections::BinaryHeap;

use crate::types::{PGPolygon, PGVertex};
use crate::pgnavmesh::PGNavmesh;

pub(crate) const DEBUG: bool = false;

const PRECISION: f32 = 1000.0;     // Root hashing grid: 1 / PRECISION = 1 mm

/// A point closer than this to a line counts as lying on it. World units.
const SIDE_TOLERANCE: f32 = 1e-3;
/// Two points closer than this are the same point. Squared world units (1 mm).
const POINT_EPS_SQ: f32 = 1e-6;
/// Successor intervals shorter than this are dropped. Squared world units (1 cm).
const MIN_INTERVAL_SQ: f32 = 1e-4;
/// Slack on the intersection time `t`, which is relative to segment length.
const T_EPS: f32 = 1e-4;

#[derive(Debug, PartialEq, Clone, Reflect, Serialize, Deserialize)]
pub struct Path {
    pub length: f32,
    pub path: SmallVec<[Vec2;10]>,
}
impl Path {
    pub fn len(&self) -> usize {
        return self.path.len();
    }

    pub fn add(&mut self, other: &mut Path){
        self.length += other.length;
        self.path.extend(other.path.drain(..));
    }

    pub fn offset_inward(
        &self, 
        start_point: Vec2, 
        agent_radius: f32
    ) -> Path {
        let pts = &self.path;
        let n = pts.len();
        if n < 1 {
            return self.clone();
        }

        let mut new_pts: SmallVec<[Vec2; 10]> = SmallVec::with_capacity(n);

        for i in 0..n {
            let curr = pts[i];

            // Determine incoming and outgoing directions
            let prev = if i == 0 { start_point } else { pts[i - 1] };
            let next = if i < n - 1 { pts[i + 1] } else { curr };

            let dir_in = (curr - prev).normalize_or_zero();
            let dir_out = (next - curr).normalize_or_zero();

            // Normals for each direction (pointing left)
            let n1 = Vec2::new(-dir_in.y, dir_in.x);
            let n2 = Vec2::new(-dir_out.y, dir_out.x);

            // Smooth average normal
            let mut normal = (n1 + n2).normalize_or_zero();

            if i == 0 {
                normal = n2.normalize_or_zero(); // use forward normal
            } else if i == n - 1 {
                normal = n1.normalize_or_zero(); // use backward normal
            }
            
            let cross = dir_in.perp_dot(dir_out);
            if cross > 0.0 {
                normal = -normal;
            }

            let offset_point = curr + normal * agent_radius;
            new_pts.push(offset_point);
        }

        // Recompute path length
        let mut total_len = 0.0;
        for w in new_pts.windows(2) {
            total_len += (w[1] - w[0]).length();
        }

        Path {
            length: total_len,
            path: new_pts,
        }
    }
}

const NO_PARENT: u32 = u32::MAX;

pub enum SearchStep {
    Found(Path),
    NotFound,
    /// run() hit its expansion budget.
    Aborted,
    Continue,
}

#[derive(Default)]
pub struct PathFinder {
    arena: Vec<Node>,
    queue: BinaryHeap<HeapEntry>,
    node_buffer: Vec<u32>,
    root_history: HashMap<Root, f32>,
    visited: Vec<u32>,
    to: Vec2,
    polygon_to: u32,
}

impl PathFinder {
    #[allow(dead_code)]
    pub fn best(&self) -> Option<&Node> {
        self.queue.peek().map(|e| &self.arena[e.idx as usize])
    }

    pub fn start(&mut self, nm: &PGNavmesh, from: (Vec2, u32), to: (Vec2, u32)) {
        self.arena.clear();
        self.queue.clear();
        self.node_buffer.clear();
        self.root_history.clear();
        self.visited.clear();
        self.to = to.0;
        self.polygon_to = to.1;

        // Node 0 is the empty start node.
        self.arena.push(Node {
            parent: NO_PARENT,
            new_root: false,
            root: from.0,
            interval: (Vec2::ZERO, Vec2::ZERO),
            edge: (0, 0),
            polygon_from: from.1,
            polygon_to: from.1,
            distance_start_to_root: 0.0,
            heuristic: 0.0,
        });

        let starting_polygon = nm.polygon(&from.1);

        for edge in starting_polygon.edges_index() {
            let start: &PGVertex = nm.vertex(&edge[0]);
            let end: &PGVertex = nm.vertex(&edge[1]);
            let other_sides = start.common(&end, &from.1);

            if DEBUG {
                info!(" [debug] other sides: {:?} between {} and {}", other_sides, start.index, end.index);
            }

            for other_side in other_sides.iter() {
                self.try_add_node(
                    nm,
                    0,
                    from.0,
                    *other_side,
                    (start.xz(), edge[0]),
                    (end.xz(), edge[1]),
                );
            }
        }

        self.flush_nodes();
    }

    fn try_add_node(
        &mut self,
        nm: &PGNavmesh,
        parent_idx: u32,
        root: Vec2,
        other_side: u32,
        start: (Vec2, u32),
        end: (Vec2, u32),
    ) {
        if !nm.polygons.contains_key(&other_side) {
            return;
        }

        let parent = self.arena[parent_idx as usize];

        let new_root = Root(root) != Root(parent.root);
        let mut g = parent.distance_start_to_root;
        if new_root {
            g += parent.root.distance(root);
        }

        let h = heuristic(root, self.to, (start.0, end.0));
        if g.is_nan() || h.is_nan() {
            return;
        }

        // Keep the node only if it improves (or ties) the best known g for this root.
        let keep = match self.root_history.entry(Root(root)) {
            Entry::Occupied(mut o) => {
                if *o.get() < g {
                    false
                } else {
                    o.insert(g);
                    true
                }
            }
            Entry::Vacant(v) => {
                v.insert(g);
                true
            }
        };
        if !keep {
            return;
        }

        let idx = self.arena.len() as u32;
        self.arena.push(Node {
            parent: parent_idx,
            new_root,
            root,
            interval: (start.0, end.0),
            edge: (start.1, end.1),
            polygon_from: parent.polygon_to,
            polygon_to: other_side,
            distance_start_to_root: g,
            heuristic: h,
        });
        self.node_buffer.push(idx);
    }

    #[inline(always)]
    fn flush_nodes(&mut self) {
        for idx in self.node_buffer.drain(..) {
            let n = &self.arena[idx as usize];
            self.queue.push(HeapEntry {
                f: n.distance_start_to_root + n.heuristic,
                g: n.distance_start_to_root,
                idx,
            });
        }
    }

    /// Walks the parent chain once and returns the root points in order.
    fn build_path(&self, mut idx: u32) -> SmallVec<[Vec2; 10]> {
        let mut path: SmallVec<[Vec2; 10]> = SmallVec::new();
        while idx != NO_PARENT {
            let n = &self.arena[idx as usize];
            if n.new_root {
                path.push(n.root);
            }
            idx = n.parent;
        }
        path.reverse();
        path
    }

    /// One expansion step.
    pub fn search(&mut self, nm: &PGNavmesh) -> SearchStep {
        let Some(entry) = self.queue.pop() else {
            return SearchStep::NotFound;
        };
        let node = self.arena[entry.idx as usize];

        if DEBUG {
            info!(" [debug] popped off: {:?}", node);
        }

        if let Some(best) = self.root_history.get(&Root(node.root)) {
            if *best < node.distance_start_to_root {
                return SearchStep::Continue; // a better path to this root exists
            }
        }

        if node.polygon_to == self.polygon_to {
            let mut path = self.build_path(entry.idx);
            if let Some(turn) = turning_point(node.root, self.to, node.interval) {
                path.push(turn);
            }
            path.push(self.to);
            return SearchStep::Found(Path {
                path,
                length: node.distance_start_to_root + node.heuristic,
            });
        }

        self.successors(nm, entry.idx);
        SearchStep::Continue
    }

    /// Runs until a result, or gives up after `max_expansions` pops.
    pub fn run(&mut self, nm: &PGNavmesh, max_expansions: u32) -> SearchStep {
        for _ in 0..max_expansions {
            match self.search(nm) {
                SearchStep::Continue => {}
                done => return done,
            }
        }
        SearchStep::Aborted
    }

 #[inline(always)]
    fn edges_between(nm: &PGNavmesh, node: &Node) -> SmallVec<[Successor; 10]> {
        let mut successors = SmallVec::new();

        let polygon = nm.polygon(&node.polygon_to);
        let edge: Vec2 = nm.vertices[&node.edge.1].xz();
        let right_index: usize = polygon.vertices.iter().enumerate()
            .find(|(_, v_index)| {nm.vertex(v_index).xz().distance_squared(edge) < 0.001})
            .map(|(i, _)| i)
            .unwrap_or_else(|| {
                    let mut distances = polygon
                        .vertices
                        .iter()
                        .map(|v_index| {(nm.vertex(v_index).xz()).distance_squared(edge)})
                        .enumerate()
                        .collect::<Vec<_>>();
                    distances.sort_unstable_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
                    distances.first().unwrap().0
            })+ 1;

        let left_index = polygon.vertices.len() + right_index - 2;
        let mut typ = SuccessorType::RightNonObservable;

        for [edge0, edge1] in polygon.circular_edges_index(right_index..=left_index) {

            let mut start_point = nm.vertex(&edge0).xz();
            let end_point  = nm.vertex(&edge1).xz();
            let edge_side = start_point.side((node.root, node.interval.0));

            match edge_side{
                EdgeSide::Right => {
                    if let Some(intersect) = line_intersect_segment(
                        (node.root, node.interval.0),
                        (start_point, end_point),
                    ) {
                        if intersect.distance_squared(start_point) > 1.0e-6
                            && intersect.distance_squared(end_point) > 1.0e-6
                        {
                            successors.push(Successor {
                                interval: (start_point, intersect),
                                edge: [edge0, edge1],
                                typ,
                            });
                            start_point = intersect;
                        } else {
                        }
                        if intersect.distance_squared(end_point) > 1.0e-6 {
                            typ = SuccessorType::Observable;
                        }
                    }
                }
                EdgeSide::Left => {
                    if typ == SuccessorType::RightNonObservable {
                        typ = SuccessorType::Observable;
                    }
                }

                EdgeSide::Edge => {
                    let endpoint_side = end_point.side((node.root, node.interval.0));
                    match endpoint_side {
                        EdgeSide::Edge | EdgeSide::Left => {
                            typ = SuccessorType::Observable;
                        }
                        _ => (),
                    }
                }
            }
            let mut end_intersection_p = None;
            let mut found_intersection = false;
            let end_root_int1_side = end_point.side((node.root, node.interval.1));

            if end_root_int1_side == EdgeSide::Left {
                if let Some(intersect) = line_intersect_segment((node.root, node.interval.1), (start_point, end_point)){
                    if intersect.distance_squared(end_point) > 1.0e-6 {
                        end_intersection_p = Some(intersect);
                    } else {}
                    found_intersection = true;
                }
            }

            successors.push(Successor {
                interval: (start_point, end_intersection_p.unwrap_or(end_point)),
                edge: [edge0, edge1],
                typ,
            });

            match end_root_int1_side {
                EdgeSide::Left => {
                    if found_intersection {
                        typ = SuccessorType::LeftNonObservable;
                    }
                    if let Some(intersect) = end_intersection_p {
                        successors.push(Successor {
                            interval: (intersect, end_point),
                            edge: [edge0, edge1],
                            typ,
                        });
                    }
                }
                EdgeSide::Edge => match end_point.side((node.root, node.interval.0)) {
                    EdgeSide::Edge | EdgeSide::Left => {
                        typ = SuccessorType::LeftNonObservable;
                    }
                    _ => (),
                },
                _ => (),
            }
        }
        
        // println!("Successors for {:?}: {:?}", node, successors);
        return successors;
    }

    fn successors(&mut self, nm: &PGNavmesh, start_idx: u32) {
        self.visited.clear();
        let mut node_idx = start_idx;
        let mut node = self.arena[node_idx as usize];

        loop {
            let right_vertex = nm.vertex(&node.edge.0);
            let left_vertex = nm.vertex(&node.edge.1);

            for successor in Self::edges_between(nm, &node).iter() {
                let [edge_0, edge_1] = successor.edge;
                let start: &PGVertex = nm.vertex(&edge_0);
                let end: &PGVertex = nm.vertex(&edge_1);
                let start_loc = start.xz();
                let end_loc = end.xz();
                let other_sides = start.common(&end, &node.polygon_to);

                if other_sides.len() == 0 {
                    continue;
                }
                if successor.interval.0.distance_squared(successor.interval.1) < MIN_INTERVAL_SQ {
                    continue;
                }

                for other_side in other_sides.iter() {
                    if *other_side == u32::MAX {
                        continue;
                    }
                    let other_side_polygon = nm.polygon(other_side);

                    // Dead-end pruning: leads to one polygon only, and it isn't the target.
                    if self.polygon_to != *other_side && other_side_polygon.neighbours.len() == 1 {
                        continue;
                    }
                    // Don't walk back where we came from.
                    if node.polygon_from == *other_side {
                        continue;
                    }

                    let root: Vec2 = match successor.typ {
                        SuccessorType::RightNonObservable => {
                            if successor.interval.0.distance_squared(start_loc) > POINT_EPS_SQ {
                                continue;
                            }
                            let at_corner = right_vertex.xz().distance_squared(node.interval.0) < POINT_EPS_SQ;
                            if right_vertex.is_corner() && at_corner {
                                node.interval.0
                            } else {
                                continue;
                            }
                        }
                        SuccessorType::Observable => node.root,
                        SuccessorType::LeftNonObservable => {
                            if successor.interval.1.distance_squared(end_loc) > POINT_EPS_SQ {
                                continue;
                            }
                            let at_corner = left_vertex.xz().distance_squared(node.interval.1) < POINT_EPS_SQ;
                            if left_vertex.is_corner() && at_corner {
                                node.interval.1
                            } else {
                                continue;
                            }
                        }
                    };

                    self.try_add_node(
                        nm,
                        node_idx,
                        root,
                        *other_side,
                        (successor.interval.0, edge_0),
                        (successor.interval.1, edge_1),
                    );
                }
            }

            // Corridor shortcut: exactly one new node that isn't the goal -> keep walking it
            // instead of going through the heap.
            let follow = self.node_buffer.len() == 1
                && self.arena[self.node_buffer[0] as usize].polygon_to != self.polygon_to;

            if !follow {
                break;
            }

            let next_idx = self.node_buffer.pop().unwrap();
            let next = self.arena[next_idx as usize];

            let exact_reverse = next.root == node.root
                && next.polygon_to == node.polygon_from
                && next.polygon_from == node.polygon_to
                && next.interval.0 == node.interval.1
                && next.interval.1 == node.interval.0;
            if exact_reverse {
                break; // TODO as before: shouldn't happen, find out what triggers it
            }
            if self.visited.contains(&next.polygon_to) {
                break; // loop detected
            }
            self.visited.push(next.polygon_to);

            node = next;
            node_idx = next_idx;
        }

        self.flush_nodes();
    }

}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Node {
    /// Index of the parent in the arena, or NO_PARENT.
    pub parent: u32,
    /// True if this node's root differs from its parent's, so it adds a path point.
    pub new_root: bool,
    pub root: Vec2,
    pub interval: (Vec2, Vec2),
    pub edge: (u32, u32),
    pub polygon_from: u32,
    pub polygon_to: u32,
    pub distance_start_to_root: f32,
    pub heuristic: f32,
}

#[derive(Clone, Copy)]
struct HeapEntry {
    f: f32,
    g: f32,
    idx: u32,
}


impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
impl Eq for HeapEntry {}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for HeapEntry {
    // Same ordering as before: lowest f first, ties broken toward larger g.
    fn cmp(&self, other: &Self) -> Ordering {
        match self.f.total_cmp(&other.f) {
            Ordering::Less => Ordering::Greater,
            Ordering::Equal => self.g.total_cmp(&other.g),
            Ordering::Greater => Ordering::Less,
        }
    }
}


// impl Node {
//     fn empty(
//         from: (Vec2, u32)
//     ) -> Self {
//         let empty_node = Node {
//             path: SmallVec::new(),
//             root: from.0,
//             interval: (Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0)),
//             edge: (0, 0),
//             polygon_from: from.1,
//             polygon_to: from.1,
//             distance_start_to_root: 0.0,
//             heuristic: 0.0,
//         };
//         return empty_node;
//     }
// }

// impl fmt::Debug for Node {
//     fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
//         return write!(f, "Root: {} from: {} to: {} cost: {} dist: {}, path: {:?}", 
//         self.root, self.polygon_from, self.polygon_to, self.heuristic, self.distance_start_to_root, self.path);
//     }
// }



impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for Node {}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        match (self.distance_start_to_root + self.heuristic)
            .total_cmp(&(other.distance_start_to_root + other.heuristic))
        {
            Ordering::Less => Ordering::Greater,
            Ordering::Equal => self
                .distance_start_to_root
                .total_cmp(&other.distance_start_to_root),
            Ordering::Greater => Ordering::Less,
        }
    }
}




#[derive(Debug, Clone, Copy)]
pub struct Root(Vec2);

impl Root {
    #[inline(always)]
    fn key(&self) -> (i32, i32) {
        // round(), not truncation: `as i32` alone makes the bucket around 0 double width
        (
            (self.0.x * PRECISION).round() as i32,
            (self.0.y * PRECISION).round() as i32,
        )
    }
}

impl PartialEq for Root {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.key() == other.key()
    }
}
impl Eq for Root {}

impl std::hash::Hash for Root {
    #[inline(always)]
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.key().hash(state);
    }
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum EdgeSide {
    Left,
    Right,
    Edge,
}


/// Computes heuristic distance from a [`super::SearchNode`] represented by the given root and interval to the goal.
#[inline(always)]
pub fn heuristic(root: Vec2, goal: Vec2, interval: (Vec2, Vec2)) -> f32 {
    // If the goal is on the same side of the interval with the root, then we mirror it.
    let goal = if root.side(interval) == goal.side(interval) {
        goal.mirror(interval)
    } else {
        goal
    };

    // Filter out the trivial cases.
    if root == interval.0 || root == interval.1 {
        root.distance(goal)
    } else {
        // If the point is not in the estimated "line of sight", then the heuristic will
        // be an approximated taut path length, otherwise it will be the exact distance
        // between the root and the goal.
        match intersection_time((root, goal), interval) {
            x if x < 0.0 => root.distance(interval.0) + interval.0.distance(goal),
            x if x > 1.0 => root.distance(interval.1) + interval.1.distance(goal),
            _ => root.distance(goal),
        }
    }
}

/// Returns intersection time (which is the ratio of the supposed
/// intersection-defined segment "part" to the total segment length)
/// at which the given segment will be intersected by the given line.
#[inline(always)]
pub(crate) fn intersection_time(line: (Vec2, Vec2), segment: (Vec2, Vec2)) -> f32 {
    let (l0, l1) = (line.0.as_dvec2(), line.1.as_dvec2());
    let (s0, s1) = (segment.0.as_dvec2(), segment.1.as_dvec2());

    let local_line = l0 - l1;
    ((l0 - s0).perp_dot(local_line) / local_line.perp_dot(s0 - s1)) as f32
}


pub trait Vec2Helper {
    fn side(self, edge: (Vec2, Vec2)) -> EdgeSide;
    fn mirror(self, edge: (Vec2, Vec2)) -> Vec2;
}

impl Vec2Helper for Vec2 {
    /// Determines where relative to the given line the point is.
    /// Returns [EdgeSide].
    #[inline(always)] 
    fn side(self, line: (Vec2, Vec2)) -> EdgeSide {
        let p = self.as_dvec2() - line.0.as_dvec2();
        let l = line.1.as_dvec2() - line.0.as_dvec2();

        // cross = |l| * (signed distance of the point from the line)
        let cross = l.perp_dot(p);
        let tol = SIDE_TOLERANCE as f64;

        if cross * cross <= tol * tol * l.length_squared() {
            EdgeSide::Edge // also covers a zero-length line
        } else if cross > 0.0 {
            EdgeSide::Left
        } else {
            EdgeSide::Right
        }
    }

    /// Mirrors a point across a line defined by two points.
    /// Returns the reflected point.
    #[inline(always)]
    fn mirror(self, line: (Vec2, Vec2)) -> Vec2 {
        let local_point = self - line.0;
        let local_line = line.1 - line.0;

        let local_reflect_point = 2.0 * local_point.project_onto(local_line) - local_point;

        line.0 + local_reflect_point
    }
}


/// Returns the point at which the path between the given root and goal should turn, if any.
#[inline(always)]
pub(crate) fn turning_point(root: Vec2, goal: Vec2, interval: (Vec2, Vec2)) -> Option<Vec2> {
    let goal = if root.side(interval) == goal.side(interval) {
        goal.mirror(interval)
    } else {
        goal
    };

    if root == interval.0 {
        None
    } else if goal.side((root, interval.0)) == EdgeSide::Right {
        Some(interval.0)
    } else if goal.side((root, interval.1)) == EdgeSide::Left {
        Some(interval.1)
    } else {
        None
    }

}

#[derive(Debug, PartialEq, Clone, Copy)]
enum SuccessorType {
    LeftNonObservable,
    Observable,
    RightNonObservable,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Successor {
    interval: (Vec2, Vec2),
    edge: [u32;2],
    typ: SuccessorType,
}

/// Returns the intersection point of the given line and segment, if it exists.
#[inline(always)]
pub(crate) fn line_intersect_segment(
    line: (Vec2, Vec2), 
    segment: (Vec2, Vec2)
) -> Option<Vec2> {
    let intersection_time = intersection_time(line, segment);
    if !(-f32::EPSILON..=(1.0 + f32::EPSILON)).contains(&intersection_time) || intersection_time.is_nan() {
        None
    } else {
        Some(segment.0 + intersection_time * (segment.1 - segment.0))
    }
}
