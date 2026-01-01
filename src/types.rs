use std::ops::RangeInclusive;
use bevy::prelude::*;
use bevy::platform::collections::{HashSet, HashMap};
use serde::{Deserialize, Serialize};

use crate::pathfinding::{Path, SearchStep, PathFinder, DEBUG};
use crate::plugin::{ORIGIN_HEIGHT, PGNavmeshType};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PGVertex {
    pub index:    usize,
    pub loc:      Vec3,
    pub polygons: Vec<usize>
}
impl PGVertex {
    pub fn is_corner(&self) -> bool {
        self.polygons.contains(&usize::MAX)
    }

    pub fn xz(&self) -> Vec2 {
        return self.loc.xz();
    }

    pub fn common(
        &self, 
        other: &PGVertex,
        except: &usize
    ) -> Vec<usize> {

        return self.polygons.iter()
            .filter(|p_index| other.polygons.contains(*p_index) && p_index != &except)
            .map(|x| *x)
            .collect::<Vec<usize>>();
    }
}



#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PGPolygon {
    pub index:      usize,
    pub vertices:   Vec<usize>,
    pub neighbours: HashSet<usize>
}

impl PGPolygon {
    pub fn has_point(
        &self, 
        loc: &Vec2,
        pgn: &PGNavmesh
    ) -> bool {
        const EPSILON: f32 = 0.0000001;

        let a = pgn.vertex(&self.vertices[0]).xz();
        let b = pgn.vertex(&self.vertices[1]).xz();
        let c = pgn.vertex(&self.vertices[2]).xz();

        // Calculate barycentric coordinates
        let v0x = c.x - a.x;
        let v0y = c.y - a.y;
        let v1x = b.x - a.x;
        let v1y = b.y - a.y;
        let v2x = loc.x - a.x;
        let v2y = loc.y - a.y;
        
        let dot00 = v0x * v0x + v0y * v0y;
        let dot01 = v0x * v1x + v0y * v1y;
        let dot02 = v0x * v2x + v0y * v2y;
        let dot11 = v1x * v1x + v1y * v1y;
        let dot12 = v1x * v2x + v1y * v2y;
        
        let denom = dot00 * dot11 - dot01 * dot01;
        if denom.abs() < EPSILON {
            return false; // Degenerate triangle
        }
        
        let inv_denom = 1.0 / denom;
        let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
    
        // Check if point is in triangle
        return u >= 0.0 && v >= 0.0 && (u + v) <= 1.0;
    }

    pub fn locs(&self, pgn: &PGNavmesh) -> [Vec3; 3] {
        let a: Vec3 = pgn.vertex(&self.vertices[0]).loc;
        let b: Vec3 = pgn.vertex(&self.vertices[1]).loc;
        let c: Vec3 = pgn.vertex(&self.vertices[2]).loc;
        return [a,b,c];
    }

    pub fn center(&self, pgn: &PGNavmesh) -> Vec3 {
        let [a,b,c] = self.locs(pgn);
        return (a + b + c)/3.0;
    }

    pub fn ray_intersection(
        &self, 
        origin:    &Vec3, 
        direction: &Vec3,
        pgn:       &PGNavmesh
    ) -> Option<(Vec3, f32)> {

        const EPSILON: f32 = 0.0000001;
        let [a,b,c] = self.locs(pgn);

        let edge1: Vec3 = b - a;
        let edge2: Vec3 = c - a;
        let h: Vec3 = direction.cross(edge2);
        let p: f32 = edge1.dot(h);

        // Ray is parallel to triangle
        if p > -EPSILON && p < EPSILON {
            return None;
        }
        let f: f32 = 1.0 / p;
        let s: Vec3 = origin - a;
        let u = f * s.dot(h);
        
        if u < 0.0 || u > 1.0 {
            return None;
        }
        
        let q = s.cross(edge1);
        let v = f * direction.dot(q);
        
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        
        // Calculate t (distance along ray)
        let t = f * edge2.dot(q);
        
        if t > EPSILON {
            let intersection_point = origin + direction * t;
            return Some((intersection_point, t));
        } else {
            return None;
        }
    }

    fn edges(&self, pgn: &PGNavmesh) -> [(Vec2, Vec2); 3] {
        let [a3,b3,c3] = self.locs(pgn);
        let a = a3.xz();
        let b = b3.xz();
        let c = c3.xz();
        return [(a,b),(b,c),(c,a)];
    }
    
    pub fn circular_edges_index(
        &self,
        bounds: RangeInclusive<usize>,
    ) -> impl Iterator<Item = [usize; 2]> + '_ {
        self.edges_index()
            .chain(self.edges_index())
            .skip(*bounds.start())
            .take(*bounds.end() + 1 - *bounds.start())
    }
    pub fn edges_index(&self) -> impl Iterator<Item = [usize; 2]> + '_ {
        self.vertices
            .windows(2)
            .map(|pair| [pair[0], pair[1]])
            .chain(std::iter::once([
                self.vertices[self.vertices.len() - 1],
                self.vertices[0],
        ]))
    }

    fn ray_side_intersection(
        &self, 
        origin: Vec3, 
        direction: Vec3, 
        len: f32,
        pgn: &PGNavmesh
    ) -> (usize, f32) {

        let ray_segment = (origin.xz(), Vec2::new(origin.x + direction.x, origin.z + direction.z));
        let edges: [(Vec2, Vec2); 3] = self.edges(pgn);
        let mut distances = Vec::with_capacity(2);

        for e in edges.iter() {
            if let Some(t) = _line_segments_intersect(e, &ray_segment) {
                let dist = t * len;
                distances.push(dist);
            }
        }
        match distances.len(){
            0 => {return(0, len);}
            _ => {
                distances.sort_by(|a, b| a.partial_cmp(b).unwrap());
                let min_dist = *distances.iter().next().unwrap();
                return (distances.len(), min_dist);
            }
        }
    }


    fn closest_point(&self, p: Vec2, pgn: &PGNavmesh) -> Vec2 {
        let [a3, b3, c3] = self.locs(pgn);
        let a = a3.xz();
        let b = b3.xz();
        let c = c3.xz();
                
        // Check if P in vertex region outside A
        let ab = b - a;
        let ac = c - a;
        let ap = p - a;
        let d1 = ab.dot(ap);
        let d2 = ac.dot(ap);
        if d1 <= 0.0 && d2 <= 0.0 {
            return a;
        }
        
        // Check if P in vertex region outside B
        let bp = p - b;
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);
        if d3 >= 0.0 && d4 <= d3 {
            return b;
        }
        
        // Check if P in edge region of AB
        let vc = d1 * d4 - d3 * d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let v = d1 / (d1 - d3);
            return a + ab * v;
        }
        
        // Check if P in vertex region outside C
        let cp = p - c;
        let d5 = ab.dot(cp);
        let d6 = ac.dot(cp);
        if d6 >= 0.0 && d5 <= d6 {
            return c;
        }
        
        // Check if P in edge region of AC
        let vb = d5 * d2 - d1 * d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let w = d2 / (d2 - d6);
            return a + ac * w;
        }
        
        // Check if P in edge region of BC
        let va = d3 * d6 - d5 * d4;
        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + (c - b) * w;
        }
        
        // P inside face region
        let denom = 1.0 / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;
        a + ab * v + ac * w
    }
}



#[inline(always)]
fn _cross(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

#[inline(always)]
fn _line_segments_intersect(
    seg1: &(Vec2, Vec2), 
    seg2: &(Vec2, Vec2)
) -> Option<f32> {

    let (p1, p2) = seg1;
    let (p3, p4) = seg2;

    let d1 = p2-p1;
    let d2 = p4-p3;
    let d3 = p3-p1;
    let denom = _cross(d1, d2);

    if denom.abs() < f32::EPSILON {
        return None; // parallel
    }

    let t = _cross(d3, d2) / denom;
    let u = _cross(d3, d1) / denom;

    if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
        Some(t)
    } else {
        None
    }
}

#[derive(Component, Clone, Debug, bevy::asset::Asset, bevy::reflect::TypePath, Serialize, Deserialize)]
pub struct PGNavmesh {
    pub polygons:     HashMap<usize, PGPolygon>,
    pub vertices:     HashMap<usize, PGVertex>,
    pub water_height: f32,
    pub search_limit: usize,
    pub typ:          PGNavmeshType,
    pub chunk_id:     String,
    pub map_name:     String
}

impl Default for PGNavmesh {
    fn default() -> Self {
        PGNavmesh{
            polygons: HashMap::default(),
            vertices: HashMap::default(),
            water_height: 180.0,
            search_limit: 1000,
            typ: PGNavmeshType::Terrain,
            chunk_id: "065".to_string(),
            map_name: "hedeby".to_string()
        }
    }
}


impl PGNavmesh {
    pub fn filename(&self) -> String {
        let filename = match self.typ {
            PGNavmeshType::Terrain => {
                format!("./assets/navmesh/{}_{}_terrain.navmesh.json", self.map_name, self.chunk_id)
            }
            PGNavmeshType::Water => {
                format!("./assets/navmesh/{}_{}_water.navmesh.json", self.map_name, self.chunk_id)
            }
        };
        return filename;
    }


    pub fn get_polygon_height(
        &self, 
        loc:          Vec2
    ) -> Option<(&PGPolygon, f32)> {

        if let Some(poly) = self.has_point(loc){
            let origin = Vec3::new(loc.x, ORIGIN_HEIGHT, loc.y);
            let direction = Vec3::NEG_Y;

            if let Some((_world_pos, dist)) = poly.ray_intersection(&origin, &direction, &self){
                let dist = dist.round() as i32;
                let height: i32 = ORIGIN_HEIGHT as i32 - dist;
                return Some((poly, height as f32));
            } else {
                // TODO solve this
                warn!("Navmesh ray calculation went wrong for {} and {:?}", poly.index, origin);
            }
        }

        return None;
    }

    pub fn ray_intersection(&self, origin: &Vec3, direction: &Vec3) -> Option<(Vec3, f32, usize)>  {
        let direction = direction.normalize();
        // info!("direction: {:?}", direction);
        for (polygon_index, polygon) in self.polygons.iter(){
            if let Some((world_pos, _dist)) = polygon.ray_intersection(&origin, &direction, &self){
                return Some((world_pos, _dist, *polygon_index));
            }
        }
        return None;
    }

    pub fn has_point(&self, loc: Vec2) -> Option<&PGPolygon> {
        let origin: Vec3 = Vec3::new(loc.x, ORIGIN_HEIGHT, loc.y);
        for (_polygon_id, polygon) in self.polygons.iter(){
            if polygon.ray_intersection(&origin, &Vec3::NEG_Y, &self).is_some(){
                return Some(polygon);
            }
        }
        return None;
    }

    pub fn path_points(
        &self, 
        from:     Vec2, 
        to0:       Vec2,
        agent_radius: f32
    ) -> Option<(Path, usize, usize)> {

        info!("path points");

        let mut to = to0;

        let Some(starting_polygon) = self.has_point(from) else {
            // if DEBUG {
                info!("no starting polygon index");
            // }
            return None;
        };

        let mut maybe_ending_polygon: Option<&PGPolygon> = self.has_point(to);
        if maybe_ending_polygon.is_none() {
            // if DEBUG {
                info!("no ending polygon index for point, searching for nearest one");
            // }

            if let Some((new_target, new_target_polygon_id)) = self.find_nearest_point_from(from, to){
                // if DEBUG {
                    info!("Found nearest one: {} ({})", new_target, new_target_polygon_id);
                // }

                to = new_target;
                maybe_ending_polygon = Some(self.polygon(&new_target_polygon_id));
            } else {
                // if DEBUG {
                    info!("couldnt find the nearest one to {}", to);
                // }
            }
        }

        let Some(ending_polygon) = maybe_ending_polygon else {
            // if DEBUG {
                info!("no ending polygon index");
            // }
            return None;
        };


        // if DEBUG {
            info!(" [Debug] find path between {:?} and {} (from {} to {})", starting_polygon.index, ending_polygon.index, from, to);
            info!(" start polygon: {:?}", starting_polygon);
            info!(" end polygon: {:?}", ending_polygon);
        // }

        if starting_polygon.index == ending_polygon.index {
            let path = Path {
                length: from.distance(to),
                path: vec![to].into(),
            };
            if DEBUG {
                info!(" [Debug] same polygon found path");
            }
            return Some((path, starting_polygon.index, ending_polygon.index));
        }

        // println!("Need to find path between {}: ({}) and {}: ({})", starting_polygon.index, from, ending_polygon.index, to);
        let mut path_finder = PathFinder::setup(
            self,
            (from, starting_polygon.index),
            (to, ending_polygon.index)
        );

        for _s in 0..self.search_limit {
            if DEBUG {
                info!(" [Debug] {}", _s);
            }
             match path_finder.search() {
                SearchStep::Found(path) => {
                    let offset_path = path.offset_inward(from, agent_radius);
                    if DEBUG {
                        info!(" [Debug] found");
                    }
                    
                    return Some((offset_path, starting_polygon.index, ending_polygon.index));
                }
                SearchStep::NotFound => {
                    if DEBUG {
                        info!(" [Debug] not found");
                    }
                    return None;
                }
                SearchStep::Continue => {}
            }
        }
        if DEBUG {
            info!(" [Debug] Reached pathfinding limit");
        }

        return None;
    }

    pub fn vertex(&self, id: &usize) -> &PGVertex {
        return self.vertices.get(id).unwrap();
    }

    pub fn polygon(&self, id: &usize) -> &PGPolygon {
        return self.polygons.get(id).expect(&format!("expected polygon {}", id));
    }

    pub fn cleanup_lower(&mut self){

        if self.typ != PGNavmeshType::Terrain {
            return;
        }

        if DEBUG {
            info!(" [debug] Start Cleanup navmesh");
        }


        let mut polygons_to_rm: HashSet<usize> = HashSet::with_capacity(self.polygons.len());
        let mut possible_vertices_to_rm: HashSet<usize> = HashSet::with_capacity(self.vertices.len());
        let mut vertices_to_rm: HashSet<usize> = HashSet::with_capacity(self.vertices.len());

        // if all 3 vertices are below water, remove polygon

        if DEBUG {
            info!(" [debug] water height: {}", self.water_height);
            info!(" [debug] total vertices {}", self.vertices.len());
            info!(" [debug] total polygons {}", self.polygons.len());
        }

        for (polygon_index, polygon) in self.polygons.iter(){

            if polygon.vertices.len() != 3 {
                if DEBUG {
                    error!(" [debug] Polygon {} has wrong number of vertices: {}", polygon_index, polygon.vertices.len());
                }

            }

            let mut low_count: usize = 0;

            for v_index in polygon.vertices.iter(){

                let v = self.vertex(v_index);

                // info!("v loc: {}", v.loc);
                if v.loc.y < self.water_height {
                    low_count += 1;
                    possible_vertices_to_rm.insert(*v_index);
                }
            }

            if low_count == 3 {
                polygons_to_rm.insert(*polygon_index);
            }
        }

        // info!("possible vertices to remove count: {}", possible_vertices_to_rm.len());

        for poss_v in possible_vertices_to_rm.iter(){

            let vertex_data = self.vertex(poss_v);
            let mut low_count: usize = 0;
            for v_polygon in vertex_data.polygons.iter(){
                if polygons_to_rm.contains(v_polygon){
                    low_count += 1;
                }
            }

            if low_count == vertex_data.polygons.len(){
                vertices_to_rm.insert(*poss_v);
            }

        }
        if DEBUG {
            info!(" [debug] polygons to remove count: {}", polygons_to_rm.len());
            info!(" [debug] vertices to remove count: {}", vertices_to_rm.len()); 
        }


        self.polygons.retain(|key, _| !polygons_to_rm.contains(key));
        if DEBUG {
            info!(" [debug] polygons count after: {}", self.polygons.len());
        }

        self.vertices.retain(|key, _| !vertices_to_rm.contains(key));
        if DEBUG {
            info!(" [debug] vertices count after: {}", self.vertices.len()); 
        }

        for (_polygon_index, polygon) in self.polygons.iter_mut(){
            polygon.neighbours.retain(|n| !polygons_to_rm.contains(n));
        }

        for (_vertex_id, vertex) in self.vertices.iter_mut(){
            vertex.polygons.retain(|n| !polygons_to_rm.contains(n));
        }

    }

    pub(crate) fn reorder_vertex_polygons(&mut self) {

        let mut mapping_polygons: HashMap<usize, Vec<usize>> = HashMap::new();

        for (vertex_id, vertex) in self.vertices.iter() {

            // For each polygon using a vertex, sort them in CCW order
            let mut v_polygons = vertex
                .polygons
                .iter()
                .filter(|p| **p != usize::MAX)
                .cloned()
                .collect::<Vec<_>>();

            // Sort by the angle between the Y axis and the direction from the vertex to the center of the polygon
            v_polygons.sort_unstable_by_key(|p| {
                let center: Vec2 = self.polygon(p).center(&self).xz();
                let direction = center - vertex.xz();
                let angle = Vec2::Y.angle_to(direction); // for sure Y?
                (angle * 100000.0) as i32
            });

            v_polygons.dedup_by_key(|p| *p);
            if v_polygons.is_empty() {
                v_polygons.push(usize::MAX);
            } else {
                // Reintroduce empty markers
                // For two following polygons on a vertex, check their previous / next vertices
                // If they are different, there is a hole between them

                let first = v_polygons[0];
                let last = *v_polygons.last().unwrap();
                if first == last {
                    v_polygons.push(usize::MAX);
                } else {
                    v_polygons = v_polygons
                        .windows(2)
                        .map(|pair| [pair[0], pair[1]])
                        .chain(std::iter::once([last, first]))
                        .flat_map(|[pair0, pair1]| {

                            let mut polygon0 = self.polygon(&pair0).vertices.clone();
                            polygon0.reverse();
                            let mut found = false;
                            let Some(previous0) =
                                polygon0.iter().cycle().take(polygon0.len() * 2).find(|v| {
                                    if found {
                                        return true;
                                    }
                                    if self.vertex(v).loc.distance_squared(vertex.loc)< 0.0001 {
                                        found = true;
                                    }
                                    false
                                })
                            else {
                                return vec![pair0, usize::MAX];
                            };
                            let polygon1 = self.polygon(&pair1).vertices.clone();
                            let mut found = false;
                            let Some(next1) =
                                polygon1.iter().cycle().take(polygon1.len() * 2).find(|v| {
                                    if found {
                                        return true;
                                    }
                                    if self.vertex(v).loc.distance_squared(vertex.loc)< 0.0001 {
                                        found = true;
                                    }
                                    false
                                })
                            else {
                                return vec![pair0, usize::MAX];
                            };

                            if self.vertex(previous0).loc != self.vertex(next1).loc {
                                vec![pair0, usize::MAX]
                            } else {
                                vec![pair0]
                            }
                        })
                        .collect();
                }
            }
            mapping_polygons.insert(*vertex_id, v_polygons);
        }

        for (vertex_id, v_polygons) in mapping_polygons.iter(){
            let vertex_data = self.vertices.get_mut(vertex_id).unwrap();
            vertex_data.polygons = v_polygons.clone();
        }

    }

    pub fn ray_side_intersection(
        &self, 
        origin: Vec3, 
        direction: Vec3, 
        len: f32,
        origin_polygon: usize,
    )  -> (bool, f32) {

        let mut polys_to_check: Vec<&usize> = Vec::with_capacity(10);
        let mut polys_buffer: Vec<&usize> = Vec::with_capacity(10);

        let safety: usize = 5;
        let mut i: usize = 0;

        let Some(origin_poly) = self.polygons.get(&origin_polygon) else {return (false, 0.0);};
        polys_to_check.extend(origin_poly.neighbours.iter());

        while i < safety {
 
            for poly_id in polys_to_check.iter(){
                let Some(poly) = self.polygons.get(*poly_id) else {continue};
                let (intersections, min_dist) = poly.ray_side_intersection(origin, direction, len, &self);
                let blocker: bool = poly_id == &&usize::MAX;

                match (intersections, blocker) {
                    (0, _) => {continue; /* Wrong side */}
                    (1, false) => {return (false, len); /* Reached end of the ray, no blockers*/}
                    (_, true) => {return (true, min_dist); /* Reached end of the ray, its a blocker*/}
                    (2, false) => {
                        // Clear polys to check
                        // Break loop, clear polys_to_check, add current neighbours of current poly to check
                        polys_buffer.extend(poly.neighbours.iter());
                        break;
                    }
                    (_,_) => {panic!("Should not happen never");}
                }

            }
            polys_to_check.clear();
            polys_to_check.extend(polys_buffer.iter());
            polys_buffer.clear();
            i += 1;

        }
        return (false, 0.0);
    }



    pub fn find_nearest_point_from(&self, origin: Vec2, target: Vec2) -> Option<(Vec2, usize)> {

        let start_polygon = self.has_point(origin).unwrap();
        let mut visited: HashSet<usize> = HashSet::new();
        let mut to_visit: Vec<usize> = vec![start_polygon.index];
        let mut best_point: Option<Vec2> = None;
        let mut best_dist = f32::INFINITY;
        let mut best_polygon_id = start_polygon.index;

        while let Some(poly_id) = to_visit.pop() {

            if visited.contains(&poly_id) {
                continue;
            }

            visited.insert(poly_id);
            let polygon = self.polygon(&poly_id);
            let closest = polygon.closest_point(target, &self);
            let dist = closest.distance_squared(target);
            
            if dist < best_dist {
                best_dist = dist;
                best_point = Some(closest);
                best_polygon_id = poly_id;
            }

            for neighbor_id in polygon.neighbours.iter(){
                if !visited.contains(neighbor_id) {
                    to_visit.push(*neighbor_id);
                }    
            }
        }

        return best_point.map(|p| (p, best_polygon_id));
    }
}
