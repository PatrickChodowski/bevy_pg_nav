use bevy::prelude::{Vec2, Reflect};
use bevy::prelude::Vec3Swizzles;
use bevy::platform::collections::{HashSet, HashMap};
use bevy::platform::collections::hash_map::Entry;
use serde::{Serialize, Deserialize};
use smallvec::SmallVec;
use std::{
    cmp::Ordering,
    fmt::Debug,
};
use std::fmt;
use std::collections::BinaryHeap;
use crate::navmesh::{NavMesh, Polygon, Vertex};
use crate::types::NavType;

const PRECISION: f32 = 1000.0;
const EPSILON: f32 = 1e-4;

#[derive(Debug, PartialEq, Clone, Reflect, Serialize, Deserialize)]
pub struct Path {
    pub length: f32,
    pub path: Vec<Vec2>,
}
impl Path {
    pub fn len(&self) -> usize {
        return self.path.len();
    }
}

pub enum SearchStep {
    Found(Path),
    NotFound,
    Continue,
}

pub struct PathFinder<'m> {
    queue: BinaryHeap<Node>,
    node_buffer: Vec<Node>,
    root_history: HashMap<Root, f32>,
    pub from: Vec2,
    pub to: Vec2,
    pub polygon_from: usize,
    pub polygon_to: usize,
    pub navmesh: &'m NavMesh,
    blockers: Vec<NavType>
}


impl<'m> PathFinder<'m> {
    pub fn best(&self) -> Option<&Node> {
        for n in self.queue.iter(){
            return Some(n);
        }
        return None;
    }

    pub fn setup(
        navmesh: &'m NavMesh,
        from: (Vec2, usize),
        to:   (Vec2, usize),
        blockers: Option<Vec<NavType>>
    ) -> Self {

        // If no blockers are provided, apply NavType::Blocker by default
        let path_blockers: Vec<NavType>;
        if let Some(blockers) = blockers {
            path_blockers = blockers;
        } else {
            path_blockers = vec![
                NavType::Blocker
            ]
        }

        let mut path_finder = PathFinder{
            queue: BinaryHeap::with_capacity(15),
            node_buffer: Vec::with_capacity(10),
            root_history: HashMap::default(),
            from: from.0,
            to: to.0,
            polygon_from: from.1,
            polygon_to: to.1,
            navmesh,
            blockers: path_blockers.clone()
        };

        let empty_node = Node::empty(from);
        let starting_polygon: &&Polygon = &navmesh.polygon(&from.1).unwrap();

        for edge in starting_polygon.edges_index() {

            let start: &Vertex = if let Some(v) = navmesh.vertex(&edge[0]) {
                v
            } else {
                continue;
            };
            let end: &Vertex = if let Some(v) = navmesh.vertex(&edge[1]) {
                v
            } else {
                continue;
            };

            let other_sides = start.common(&end, &from.1, &path_blockers);


            for other_side in other_sides.iter(){

                path_finder.try_add_node(
                    from.0,
                    **other_side,
                    (start.xz(), edge[0]),
                    (end.xz(), edge[1]),
                    &empty_node,
                );
                
            }

        }

        path_finder.flush_nodes();
        return path_finder;

    }


    fn try_add_node(
        &mut self,
        root:       Vec2,
        other_side: usize,
        start:      (Vec2, usize),
        end:        (Vec2, usize),
        node:       &Node,
    ){
        if self.navmesh.polygons.get(&other_side).is_none(){
            return;
        }

        let mut new_f = node.distance_start_to_root;
        let mut path = node.path.clone();
        if root != node.root {
            path.push(root);
            new_f += node.root.distance(root);
        }
        let heuristic_to_end: f32;
        heuristic_to_end = heuristic(root, self.to, (start.0, end.0));
        
        if new_f.is_nan() || heuristic_to_end.is_nan() {
            // println!("newf or heuristic to end is nan");
            return;
        }

        let new_node = Node {
            path,
            root,
            interval: (start.0, end.0),
            edge: (start.1, end.1),
            polygon_from: node.polygon_to,
            polygon_to: other_side,
            distance_start_to_root: new_f,
            heuristic: heuristic_to_end,
        };

        match self.root_history.entry(Root(root)) {
            Entry::Occupied(mut o) => {
                if o.get() < &new_node.distance_start_to_root {
                    // println!("x already got a better path");
                } else {
                    o.insert(new_node.distance_start_to_root);
                    self.node_buffer.push(new_node);
                }
            }
            Entry::Vacant(v) => {
                v.insert(new_node.distance_start_to_root);
                self.node_buffer.push(new_node);
            }
        }
        
    }

    pub fn search(&mut self) -> SearchStep {
        if let Some(next_node) = self.pop_node() {

            // println!("popped off: {:?} ({})", next_node, next_node.polygon_from);

            // println!("Root history: {:?}", self.root_history);

            if let Some(o) = self.root_history.get(&Root(next_node.root)) {
                if o < &next_node.distance_start_to_root {
                    // println!("node is dominated!");
                    return SearchStep::Continue;
                }
            }
            if next_node.polygon_to == self.polygon_to {
                let mut path = next_node.path;
                if let Some(turn) = turning_point(next_node.root, self.to, next_node.interval) {
                    // println!("New turn: {}", turn);
                    path.push(turn);
                }
                let complete = next_node.polygon_to == self.polygon_to;
                if complete {
                    path.push(self.to);
                }
                return SearchStep::Found(Path {
                    path,
                    length: next_node.distance_start_to_root + next_node.heuristic
                });
            }
            self.successors(next_node);
            return SearchStep::Continue;
        } else {
            return SearchStep::NotFound
        }
    }

    #[inline(always)]
    fn flush_nodes(&mut self) {
        self.queue.extend(self.node_buffer.drain(..));
    }
    #[inline(always)]
    fn pop_node(&mut self) -> Option<Node> {
        self.queue.pop()
    }

    #[inline(always)]
    fn successors(&mut self, mut node: Node) {
        let mut visited = HashSet::new();
        loop {
            for successor in self.edges_between(&node).iter() {

                let start: &Vertex = self.navmesh.vertex(&successor.edge.0).unwrap();
                let end: &Vertex = self.navmesh.vertex(&successor.edge.1).unwrap();
                let start_loc: Vec2 = start.xz();
                let end_loc: Vec2 = end.xz();
                let other_sides = start.common(&end, &node.polygon_to, &self.blockers);

                if other_sides.len() == 0 {
                    // println!("No ends: no Successors otherside available between {:?} and {:?} polygon_to: {}", start, end, &node.polygon_to);
                    continue;
                }

                for other_side in other_sides.iter(){

                    // prune edges that only lead to one other polygon, and not the target: dead end pruning
                    if &self.polygon_to != *other_side && self.navmesh.polygons[*other_side].neighbours.len() == 1{
                        // println!("Dead End: Prune edges leading to only one polygon that is not target");
                        continue;
                    }

                    if &node.polygon_from == *other_side {
                        // println!("If other side is polygon from: not going back");
                        continue;
                    }

                    const EPSILON: f32 = 1.0e-10;

                    let root: Vec2 = match successor.typ {
                        SuccessorType::RightNonObservable => {
                            if successor.interval.0.distance_squared(start_loc) > EPSILON {
                                continue;
                            }
                            let vertex: &Vertex = self.navmesh.vertices.get(&node.edge.0).unwrap();
                            let is_corner = vertex.is_corner(&self.blockers);
                            let dist = vertex.xz().distance_squared(node.interval.0);

                            if is_corner {
                                // println!("      Vertex: {:?} IS RIGHT A CORNER distance: {}", vertex, dist);
                            }
                            if is_corner && dist< EPSILON{
                            // if is_corner {
                                node.interval.0 
                            } else {
                                continue;
                            }
                        }
                        SuccessorType::Observable => node.root,
                        SuccessorType::LeftNonObservable => {
                            if (successor.interval.1).distance_squared(end_loc) > EPSILON {
                                continue;
                            }
                            let vertex: &Vertex = self.navmesh.vertices.get(&node.edge.1).unwrap();
                            let dist = vertex.xz().distance_squared(node.interval.1);
                            let is_corner = vertex.is_corner(&self.blockers);

                            if is_corner {
                                // println!("      Vertex: {:?} IS LEFT A CORNER", vertex);
                            }

                            if is_corner && dist < EPSILON{
                                node.interval.1
                            } else {
                                continue;
                            }
                        }
                    };

                    if root != node.root {
                        // println!("  New root: {:?}", root);
                    }

                    if successor.interval.0.distance_squared(successor.interval.1) < EPSILON {
                        // println!("Successor distance interval too short");
                        continue;
                    }

                    self.try_add_node(
                        root,
                        **other_side,
                        (successor.interval.0, successor.edge.0), //start
                        (successor.interval.1, successor.edge.1), // end
                        &node,
                    );
                }
            }

            if self.node_buffer.len() == 1 && self.node_buffer[0].polygon_to != self.polygon_to {

                let previous_node = node;
                node = self.node_buffer.drain(..).next().unwrap();


                if node.root == previous_node.root
                    && node.polygon_to == previous_node.polygon_from
                    && node.polygon_from == previous_node.polygon_to
                    && node.interval.0 == previous_node.interval.1
                    && node.interval.1 == previous_node.interval.0
                {
                    // panic!("going the exact reverse way as we went into this polygon");
                    // going the exact reverse way as we went into this polygon
                    // TODO: shouldn't happen, identify cases that trigger this
                    break;
                }
                if !visited.insert(node.polygon_to) {
                    // infinite loop, exit now
                    // panic!("infinite loop, exit now");
                    break;
                }
            } else {
                break;
            }
        }
        self.flush_nodes();
    }


    #[inline(always)]
    fn edges_between(&self, node: &Node) -> SmallVec<[Successor; 10]> {
        let mut successors = SmallVec::new();

        if let Some(polygon) = self.navmesh.polygon(&node.polygon_to) {
            let edge: Vec2 = self.navmesh.vertices[&node.edge.1].loc.xz();
            let right_index = polygon
                .vertices
                .iter()
                .enumerate()
                .find(|(_, v)| {(self.navmesh.vertices[&v.index].loc.xz()).distance_squared(edge) < 0.001})
                .map(|(i, _)| i)
                .unwrap_or_else(|| {
                    let mut distances = polygon
                        .vertices
                        .iter()
                        .map(|v| {(self.navmesh.vertices[&v.index].loc.xz()).distance_squared(edge)})
                        .enumerate()
                        .collect::<Vec<_>>();
                    distances.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
                    distances.first().unwrap().0
                })+ 1;
            let left_index = polygon.vertices.len() + right_index - 2;
            let mut typ = SuccessorType::RightNonObservable;
            for edge in polygon.circular_edges_index(right_index..=left_index) {

                let Some(start) = self.navmesh.vertex(&edge[0]) else {panic!("Lack of vertex for {}", &edge[0])};
                let Some(end)  = self.navmesh.vertex(&edge[1]) else {panic!("Lack of vertex for {}", &edge[1])};

                let mut start_point = Vec2::new(start.loc.x, start.loc.z);
                let end_point = Vec2::new(end.loc.x, end.loc.z);
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
                                    edge: edge.into(),
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
                let end_root_int1 = end_point.side((node.root, node.interval.1));

                if end_root_int1 == EdgeSide::Left {
                    if let Some(intersect) =
                        line_intersect_segment((node.root, node.interval.1), (start_point, end_point))
                    {

                        if intersect.distance_squared(end_point) > 1.0e-6 {
                            end_intersection_p = Some(intersect);
                        } else {
                        }
                        found_intersection = true;
                    }
                }
                successors.push(Successor {
                    interval: (start_point, end_intersection_p.unwrap_or(end_point)),
                    edge: edge.into(),
                    typ,
                });
                match end_root_int1 {
                    EdgeSide::Left => {
                        if found_intersection {
                            typ = SuccessorType::LeftNonObservable;
                        }
                        if let Some(intersect) = end_intersection_p {
                            successors.push(Successor {
                                interval: (intersect, end_point),
                                edge: edge.into(),
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
        }

        // println!("Successors for {:?}: {:?}", node, successors);
        return successors;
    }

}



#[derive(PartialEq)]
pub struct Node {
    path:                   Vec<Vec2>,
    root:                   Vec2,
    interval:               (Vec2, Vec2),
    edge:                   (usize, usize),
    polygon_from:           usize,
    polygon_to:             usize,
    distance_start_to_root: f32,
    heuristic:              f32,
}

impl Node {
    fn empty(
        from: (Vec2, usize)
    ) -> Self {
        let empty_node = Node {
            path: vec![],
            root: from.0,
            interval: (Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0)),
            edge: (0, 0),
            polygon_from: from.1,
            polygon_to: from.1,
            distance_start_to_root: 0.0,
            heuristic: 0.0,
        };
        return empty_node;
    }

//     fn is(&self, indices: &Vec<usize>) -> bool {
//         indices.contains(&self.polygon_from) || indices.contains(&self.polygon_to)
//     }
}

impl fmt::Debug for Node {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return write!(f, "Root: {} from: {} to: {} cost: {} dist: {}, path: {:?}", 
        self.root, self.polygon_from, self.polygon_to, self.heuristic, self.distance_start_to_root, self.path);
    }
}



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




#[derive(Debug)]
pub struct Root(Vec2);

impl PartialEq for Root {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl Eq for Root {}

impl std::hash::Hash for Root {
    #[inline(always)]
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        ((self.0.x * PRECISION) as i32).hash(state);
        ((self.0.y * PRECISION) as i32).hash(state);
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
    // What's happening here is that we're effectively finding a "partial" area (wedge product) defined by supposed
    // intersection (line end to segment end x full line), and then divide it by "total" area (full line x full segment).
    // Apparently this is equal to the so called intersection time, which is the ratio of the supposed segment
    // "part" defined by intersection to the total segment length. If the "part" is biggger than 1, then the line
    // does not intersect the segment but intersects the line on which the segment lies.
    let local_line = line.0 - line.1;

    (line.0 - segment.0).perp_dot(local_line) / (local_line).perp_dot(segment.0 - segment.1)
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
        let local_point = self - line.0;
        let local_line = line.1 - line.0;

        match local_line.perp_dot(local_point) {
            x if x.abs() < EPSILON => EdgeSide::Edge,

            // ORIGINAL:
            // x if x > 0.0 => EdgeSide::Left,
            // _ => EdgeSide::Right,
            
            // my hack, no idea why:
            x if x > 0.0 => EdgeSide::Right,
            _ => EdgeSide::Left,
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
    edge: (usize, usize),
    typ: SuccessorType,
}

/// Returns the intersection point of the given line and segment, if it exists.
#[inline(always)]
pub(crate) fn line_intersect_segment(
    line: (Vec2, Vec2), 
    segment: (Vec2, Vec2)
) -> Option<Vec2> {
    let intersection_time = intersection_time(line, segment);
    if !(-EPSILON..=(1.0 + EPSILON)).contains(&intersection_time) || intersection_time.is_nan() {
        None
    } else {
        Some(segment.0 + intersection_time * (segment.1 - segment.0))
    }
}
