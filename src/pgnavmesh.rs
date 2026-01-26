use bevy::prelude::*;
use bevy::platform::collections::{HashSet, HashMap};
use serde::{Deserialize, Serialize};
use bevy_pg_core::prelude::AABB;

use crate::bvh::{BVH, BHVType, BVHNode, aabb_intersects_triangle};
use crate::pathfinding::{Path, SearchStep, PathFinder, DEBUG};
use crate::plugin::ORIGIN_HEIGHT;
use crate::types::{PGPolygon, PGVertex};

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug, Reflect, Serialize, Deserialize)]
pub enum PGNavmeshType {
    Terrain,
    Water
}

#[derive(Component, Clone, Debug, bevy::asset::Asset, bevy::reflect::TypePath, Serialize, Deserialize)]
pub struct PGNavmesh {
    pub polygons:     HashMap<usize, PGPolygon>,
    pub vertices:     HashMap<usize, PGVertex>,
    pub water_height: f32,
    pub search_limit: usize,
    pub typ:          PGNavmeshType,
    pub bvh:          BVH,
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
            map_name: "hedeby".to_string(),
            bvh: BVH::empty()
        }
    }
}


impl PGNavmesh {
    pub fn ray_intersection(&self, origin: &Vec3, direction: &Vec3) -> Option<(&PGPolygon, Vec3)> {
        let loc = origin.xz();
        for node in self.bvh.data.iter(){
            if node.aabb.has_point(loc){
                match &node.typ {
                    BHVType::Leaf(polygons) => {
                        for polygon_id in polygons.iter(){
                            let polygon = self.polygon(polygon_id);
                                if let Some(world_pos) = polygon.ray_intersection(&origin, &direction, &self) {
                                    return Some((polygon, world_pos));
                                }

                        }
                    }
                    BHVType::Branch(aabbs) => {}
                }
            }
        };
        return None;
    }

    pub fn has_point(&self, loc: &Vec2) -> Option<(&PGPolygon, Vec3)> {
        let origin: Vec3 = Vec3::new(loc.x, ORIGIN_HEIGHT, loc.y);
        let direction: Vec3 = Vec3::NEG_Y;
        return self.ray_intersection(&origin, &direction);
    }

    pub fn clamp_move(
        &self,
        target_pos: Vec2,
        current_poly_index: usize
    ) -> (Vec3, usize) {

        let mut closest_point = target_pos;
        let mut min_dist_sq = f32::MAX;
        let mut best_poly_index = current_poly_index;

        let mut candidates = vec![current_poly_index];
        let current_poly = self.polygon(&current_poly_index);
        candidates.extend(&current_poly.neighbours);
        
        for &idx in &candidates {
            let poly = self.polygon(&idx);
            let [a, b, c] = poly.locs_2d(&self);
            let clamped_2d = closest_point_on_triangle_2d(target_pos, a,b,c);
            let dist_sq = clamped_2d.distance_squared(target_pos);
            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                closest_point = clamped_2d;
                best_poly_index = idx;
            }
        }

        let poly = self.polygon(&best_poly_index);
        let height = poly.get_height(closest_point, &self);

        return (Vec3::new(closest_point.x, height, closest_point.y), best_poly_index);
    }



    pub fn path_points(
        &self, 
        from0:        &Vec2, 
        to0:          &Vec2,
        agent_radius: f32
    ) -> Option<(Path, usize, usize)> {

        let from = *from0;
        let to = *to0;
        let starting_polygon: &PGPolygon = self.has_point(&from).map(|p| p.0).unwrap();
        let ending_polygon: &PGPolygon = self.has_point(&to).map(|p| p.0).unwrap();

        if DEBUG {
            info!(" [Debug] find path between {:?} and {} (from {} to {})", starting_polygon.index, ending_polygon.index, from, to);
            // info!(" start polygon: {:?}", starting_polygon);
            // info!(" end polygon: {:?}", ending_polygon);
        }

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
        return self.vertices.get(id).expect(&format!("expected vertex {}", id));
    }

    pub fn polygon(&self, id: &usize) -> &PGPolygon {
        return self.polygons.get(id).expect(&format!("expected polygon {}", id));
    }

    pub (crate) fn bake(&mut self) {
        let mut bvh = BVH::empty();
        const LAYER_NODE_COUNT: usize = 16;

        let mut navmesh_aabb = AABB::new_min_max(); 
        for (_vertex_id, vertex) in self.vertices.iter(){

            if vertex.loc.x < navmesh_aabb.min_x {
                navmesh_aabb.min_x = vertex.loc.x
            }
            if vertex.loc.z < navmesh_aabb.min_z {
                navmesh_aabb.min_z = vertex.loc.z
            }
            if vertex.loc.x > navmesh_aabb.max_x {
                navmesh_aabb.max_x = vertex.loc.x
            }
            if vertex.loc.z > navmesh_aabb.max_z {
                navmesh_aabb.max_z = vertex.loc.z
            }
        }

        let mut id: u32 = 0;
        let small_aabbs = navmesh_aabb.split(LAYER_NODE_COUNT);
        let mut nodes: Vec<BVHNode> = Vec::new();

        for aabb in small_aabbs.iter(){
            let mut node_polygons: Vec<usize> = Vec::new();
            for (polygon_id, polygon) in self.polygons.iter(){
                let [a,b,c] = polygon.locs_2d(self);
                if aabb_intersects_triangle(aabb, a,b,c){
                    node_polygons.push(*polygon_id);
                }
            }

            info!("ID: {} polygon count: {}", id, node_polygons.len());

            let node = BVHNode{id: id, aabb: *aabb, typ: BHVType::Leaf(node_polygons)};
            nodes.push(node);
            id += 1;
        }
        bvh.data = nodes;
        self.bvh = bvh;
    }

    pub(crate) fn islands_removal(&mut self){
        let mut visited: HashSet<usize> = HashSet::new();
        let mut islands: Vec<Vec<usize>> = Vec::new();

        for (polygon_id, _polygon) in self.polygons.iter(){

            if visited.contains(polygon_id){
                continue;
            }

            let mut island: Vec<usize> = Vec::new();
            let mut queue: Vec<usize> = vec![*polygon_id];

            while queue.len() > 0 {

                let current = queue.pop().unwrap(); // checked in while condition
                if visited.contains(&current) {
                    continue;
                }
                visited.insert(current);
                island.push(current);

                for n_poly_id in self.polygon(&current).neighbours.iter(){
                    if n_poly_id == &usize::MAX {
                        continue;
                    }
                    if visited.contains(n_poly_id){
                        continue;
                    }
                    queue.push(*n_poly_id);
                }

            }
            islands.push(island);

        }

        // Remove small islands
        let mut polygons_to_rm: Vec<usize> = Vec::new();
        let mut vertices_to_rm: Vec<usize> = Vec::new();

        const ISLAND_THRESHOLD: usize = 10;
        for island in islands.iter_mut(){
            if island.len() <= ISLAND_THRESHOLD {
                for poly_id in island.iter(){
                    vertices_to_rm.extend(self.polygon(poly_id).vertices.iter());
                }
                polygons_to_rm.extend(island.drain(..)); 
            }
        }

        for polygon_id in polygons_to_rm.iter(){
            self.polygons.remove(polygon_id);
        }

        for vertex_id in vertices_to_rm.iter(){
            self.vertices.remove(vertex_id);
        }

    }

    pub(crate) fn cleanup_lower(&mut self){

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

    pub fn find_nearest_point_from_outside(&self, origin: &Vec2) -> Option<(Vec2, usize)> {
        let mut best_point = None;
        let mut best_dist_sq = f32::INFINITY;
        let mut best_polygon_id = 0;
        
        for (&poly_id, polygon) in self.polygons.iter() {
            let closest = polygon.closest_point(origin, &self);
            let dist_sq = closest.distance_squared(*origin);
            
            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_point = Some(closest);
                best_polygon_id = poly_id;
            }
        }
        
        best_point.map(|p| (p, best_polygon_id))
    }

    pub fn find_nearest_point_from(&self, origin: &Vec2, target: &Vec2) -> Option<(Vec2, usize)> {

        let (start_polygon, _world_pos) = self.has_point(origin).unwrap();
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
            let dist = closest.distance_squared(*target);
            
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

}



// Search for the point in query of navmeshes
pub fn find_point<'a>(
    point:     &Vec2, 
    navs:      &Query<(Entity, &PGNavmesh)>
) -> Option<(Entity, Vec3, PGNavmeshType)> {

    let mut highest_nav_entity: Option<Entity> = None;
    let mut highest_world_pos: Vec3 = Vec3::MIN;
    let mut highest_navmesh: Option<&PGNavmesh> = None;

    for (navmesh_entity, navmesh) in navs.iter(){
        if let Some((_polygon, world_pos)) = navmesh.has_point(point){
            if world_pos.y > highest_world_pos.y {
                highest_world_pos = world_pos;
                highest_nav_entity = Some(navmesh_entity);
                highest_navmesh = Some(navmesh);
            }
        }
    }

    if let Some(nav_entity) = highest_nav_entity {
        return Some(
            (nav_entity, highest_world_pos, highest_navmesh.unwrap().typ)
        );
    }

    return None;
}



fn closest_point_on_segment_2d(p: Vec2, a: Vec2, b: Vec2) -> Vec2 {
    let ab = b - a;
    let t = (p - a).dot(ab) / ab.length_squared();
    let t_clamped = t.clamp(0.0, 1.0);
    a + ab * t_clamped
}
/// Checks if a 2D point is inside a 2D triangle
fn is_point_in_triangle_2d(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool {
    let v0 = c - a;
    let v1 = b - a;
    let v2 = p - a;

    let dot00 = v0.dot(v0);
    let dot01 = v0.dot(v1);
    let dot02 = v0.dot(v2);
    let dot11 = v1.dot(v1);
    let dot12 = v1.dot(v2);

    let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0)
}

/// Finds the closest point on a 2D triangle to a 2D point
fn closest_point_on_triangle_2d(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> Vec2 {
    // 1. If inside, return the point itself
    if is_point_in_triangle_2d(p, a, b, c) {
        return p;
    }

    // 2. If outside, clamp to the closest edge
    let closest_ab = closest_point_on_segment_2d(p, a, b);
    let closest_bc = closest_point_on_segment_2d(p, b, c);
    let closest_ca = closest_point_on_segment_2d(p, c, a);

    let d_ab = p.distance_squared(closest_ab);
    let d_bc = p.distance_squared(closest_bc);
    let d_ca = p.distance_squared(closest_ca);

    // Return the closest of the three edge points
    if d_ab < d_bc && d_ab < d_ca { closest_ab }
    else if d_bc < d_ca { closest_bc }
    else { closest_ca }
}