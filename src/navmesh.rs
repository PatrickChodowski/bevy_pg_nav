
use bevy::color::palettes::css::WHITE;
use bevy::prelude::*;
use bevy::platform::collections::HashMap;
use bevy::math::{Vec3A};
use std::ops::RangeInclusive;
use ordered_float::*;
use serde::{Serialize,Deserialize};

use crate::tools::NavRay;
use crate::pathfinding::{PathFinder, Path, SearchStep};
use crate::types::{NavQuad, Neighbours, QuadAABB, NavType};

const ORIGIN_HEIGHT: f32 = 1000.0;
const VERTEX_SIMILARITY_THRESHOLD: f32 = 1.0;

#[derive(Resource, Clone, Debug, Serialize, Deserialize, bevy::asset::Asset, bevy::reflect::TypePath)]
pub struct NavMesh {
    pub polygons: HashMap<usize, Polygon>,
    pub vertices: HashMap<usize, Vertex>,
    pub water_height: f32
}

impl Default for NavMesh {
    fn default() -> Self {
        NavMesh{
            polygons: HashMap::default(),
            vertices: HashMap::default(),
            water_height: 0.0
        }
    }
}

impl NavMesh {

    pub fn has_point(&self, loc: Vec2) -> Option<&Polygon> {
        for (_polygon_id, polygon) in self.polygons.iter(){
            if polygon.aabb.has_point(loc){
                return Some(polygon);
            }
        }
        return None;
    }

    pub fn path_points(
        &self, 
        from: Vec2, 
        to: Vec2, 
        blockers: Option<Vec<NavType>>,
        search_limit: usize
    ) -> Option<(Path, usize, usize)> {
        let Some(starting_polygon) = self.has_point(from) else {
            println!("no starting polygon index");
            return None;
        };
        let Some(ending_polygon) = self.has_point(to) else {
            println!("no ending polygon index");
            return None;
        };


        if starting_polygon.index == ending_polygon.index {
            let path = Path {
                length: from.distance(to),
                path: vec![to],
            };
            return Some((path, starting_polygon.index, ending_polygon.index));
        }

        // println!("Need to find path between {}: ({}) and {}: ({})", starting_polygon.index, from, ending_polygon.index, to);
        let mut path_finder = PathFinder::setup(
            self,
            (from, starting_polygon.index),
            (to, ending_polygon.index),
            blockers
        );

        for _s in 0..search_limit {

            // if let Some(best) = path_finder.best(){
            //     println!("   Step: {} Best: {:?}", _s, best)
            // }

             match path_finder.search() {
                SearchStep::Found(path) => {
                    // info!("Found path between {} and {}: {:?}", from, to, path);
                    return Some((path, starting_polygon.index, ending_polygon.index));
                }
                SearchStep::NotFound => {
                    return None;
                }
                SearchStep::Continue => {}
            }
        }
        return None;
    }

    pub fn type_for(&self, loc: Vec2) -> NavType {
        return self.has_point(loc).unwrap().typ;
    }

    pub (super) fn polygon(&self, index: &usize) -> Option<&Polygon> {
        self.polygons.get(index)
    }
    pub (super) fn vertex(&self, index: &usize) -> Option<&Vertex> {
        self.vertices.get(index)
    }

    pub(crate) fn from_hash_navquads(navquads: &mut HashMap<usize, NavQuad>) -> Self {
        let mut navmesh = NavMesh::default();
        let mut polygons: HashMap<usize, Polygon> = HashMap::with_capacity(3000);
        for (_quad_id, quad) in navquads.iter_mut(){
            let polygon = Polygon::from_navquad(quad);
            polygons.insert(polygon.index, polygon);
        }
        navmesh.polygons = polygons;
        navmesh.process_vertices();
        return navmesh;
    }


    ///  Function that would process vertices based on the hashmap of polygons 
    fn process_vertices(&mut self){


        let mut ordered_vs: HashMap<[OrderedFloat<f32>;3], Vertex> = HashMap::default();
        let mut global_vertex_index: usize = 1;

        for (polygon_index, polygon) in self.polygons.iter(){
            let locs: Vec<Vec3A> = polygon.vertices.iter().map(|v| v.loc).collect::<Vec<Vec3A>>();
            if locs.len() != 4 {
                panic!("On this stage there should be 4 vertices locs in {}", polygon_index);
            }

            let converted_vertices: [[OrderedFloat<f32>;3]; 4] = polygon.convert_vertices();
            for (conv_vertex, loc) in converted_vertices.iter().zip(locs) {

                let mut polys_to_insert: Vec<(NavType, usize)> = vec![polygon.mindata()];
                for (_neighbour_type, neighbour_index) in polygon.neighbours.iter(){
                    if let Some(neighbour) = self.polygon(neighbour_index){
                        let n_has_point = neighbour.has_point_int(loc.xz());
                        if n_has_point {
                            polys_to_insert.push(neighbour.mindata());
                        }
                    }
                }

                // Tests for equality, but should test also for close enough...
                if let Some(v) = ordered_vs.get_mut(conv_vertex){
                    for (poly_type, poly_index) in polys_to_insert.iter(){
                        v.polygons.insert(*poly_type, *poly_index);
                    }
                } else {
                    
                    let mut found_close_enough: bool = false;

                    // Check for close enough
                    for (k, v) in ordered_vs.iter_mut(){

                        let dist = ordered_float_distance(conv_vertex, k);
                        if dist <= VERTEX_SIMILARITY_THRESHOLD {
                            for (poly_type, poly_index) in polys_to_insert.iter(){
                                v.polygons.insert(*poly_type, *poly_index);
                            }
                            found_close_enough = true;
                            break;
                        }
                    }

                    if found_close_enough == false {
                        let vertex = Vertex {
                            loc,
                            index: global_vertex_index,
                            polygons: Neighbours::from_pairs(&polys_to_insert)
                        };
                        global_vertex_index += 1;
                        ordered_vs.insert(*conv_vertex, vertex);
                    }
                }
            }
        }

        let mapping_by_index = self._get_vertices_mapping_by_height(&ordered_vs);
        let duplicates: Vec<usize> = mapping_by_index
            .iter()
            .filter(|(key, value)| **key == value.0)
            .map(|(key, _value)| *key)
            .collect();

        // Propagate indexed vertices in polygons and sort them
        for (_conv_v, ordered_vertex) in ordered_vs.iter(){
            for (_poly_type, poly_index) in ordered_vertex.polygons.iter(){
                if let Some(polygon) = self.polygons.get_mut(poly_index){

                    let mut found_vertex: bool = false;
                    for vertex in polygon.vertices.iter_mut(){
                        if vertex.loc == ordered_vertex.loc {
                            vertex.index = ordered_vertex.index;
                            vertex.polygons = ordered_vertex.polygons.clone();
                            found_vertex = true;
                        }
                    }
                    
                    if found_vertex == false {
                        polygon.vertices.push(ordered_vertex.clone());
                    }
                }
            }
        }

        // Merge, cleanup and sort vertices in polygons
        for (_polygon_id, polygon) in self.polygons.iter_mut(){

            if polygon.vertices.len() > 4 {
                polygon.vertices.retain(|v| !(v.index == 0 && v.polygons.len() == 0)); // Remove empty vertex
                polygon.vertices.retain(|v| !duplicates.contains(&v.index));  // Remove duplicated vertex
            }

            // Update vertices to average height
            for vertex in polygon.vertices.iter_mut(){
                if let Some(data) = mapping_by_index.get(&vertex.index){
                    vertex.loc.y = *data.1;
                } 
            }

            polygon.sort_vertices();
        }


        let mut vertex_map: HashMap<usize, Vertex> = HashMap::with_capacity(ordered_vs.len());
        for (_loc, vertex) in ordered_vs.iter_mut(){

            // Ignore duplicated index
            if duplicates.contains(&vertex.index){
                continue;
            }

            // Update to average height if mapped
            if let Some(data) = mapping_by_index.get(&vertex.index){
                vertex.loc.y = *data.1;
            }

            // Insert vertex to 
            vertex_map.insert(vertex.index, vertex.clone());  
        } 

        self.vertices = vertex_map;

    }

    fn _get_vertices_mapping_by_height(
        &self, 
        ordered_vs: &HashMap<[OrderedFloat<f32>; 3], Vertex>) -> HashMap<usize,(usize, OrderedFloat<f32>)>{
        // Merge Ordered vertices by height (if the same x,z but different heights)
        // Mapping has XZ location as key, value is list of vertices and list of heights
        let mut mapping: HashMap<[i32; 2], (Vec<usize>, Vec<OrderedFloat<f32>>)> = HashMap::with_capacity(1000);
        for (loc, vertex) in ordered_vs.iter(){
            let loc_xz: [i32; 2] = [*loc[0] as i32, *loc[2] as i32];
            if let Some(v) = mapping.get_mut(&loc_xz){
                v.0.push(vertex.index);
                v.1.push(loc[1]);
            } else {
                mapping.insert(loc_xz, (vec![vertex.index], vec![loc[1]]));
            }
        }

        mapping.retain(|_key, v| v.0.len() > 1);
        let mapping_calculated: HashMap<[i32; 2], (Vec<usize>, usize, OrderedFloat<f32>)> = 
                            mapping.iter()
                                   .map(|(key, v)| (*key, (
                                        v.0.clone(), // List
                                        *v.0.iter().min().unwrap(), // Min Index
                                        // TODO: get different strategies based on type: Navigable should have always priority for example
                                        v.1.iter().copied().sum::<OrderedFloat<f32>>()/OrderedFloat::<f32>::from(v.1.len() as f32)  
                                    )))
                                   .collect();

        // Reorgenize data to be index focused
        let mut mapping_by_index: HashMap<usize,(usize, OrderedFloat<f32>)> = HashMap::new();
        for (_loc_xz, data) in mapping_calculated.iter(){
            for single_index in data.0.iter(){
                if mapping_by_index.get(single_index).is_some(){
                    // should not happen
                    panic!("Mapping by index already has: {}", single_index);
                }
                mapping_by_index.insert(*single_index, (data.1, data.2));
            }
        }

        return mapping_by_index;
    }

    pub fn ray_intersection(&self, ray: &NavRay) -> Option<(Vec3, f32, usize, NavType)>  {
        for (_polygon, polygon) in self.polygons.iter(){
            if let Some((world_pos, _dist, index)) = polygon.ray_intersection(&ray){
                return Some((world_pos, _dist, index, polygon.typ));
            }
        }
        return None;
    }

    // pub(crate) fn clear(&mut self) {
    //     self.polygons.clear();
    //     self.vertices.clear();
    // }


    pub fn get_polygon_height(
        &self, 
        loc:          Vec2,
        water_height: f32
    ) -> Option<(&Polygon, f32)> {

        if let Some(poly) = self.has_point(loc){
            let origin = Vec3::new(loc.x, ORIGIN_HEIGHT, loc.y);
            let ray = NavRay::new(origin, Vec3::NEG_Y);

            if let Some((_pos, dist, _index)) = poly.ray_intersection(&ray){
                let dist = dist.round() as i32;
                let height: i32 = ORIGIN_HEIGHT as i32 - dist;

                if poly.typ == NavType::Water {
                    return Some((poly, water_height))
                } else {
                    return Some((poly, height as f32));
                }
            } else {
                // TODO solve this
                warn!("Navmesh ray calculation went wrong for {} and {:?}", poly.index, ray);
            }
        }

        return None;
    }


}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Polygon {
    pub index:      usize,
    pub vertices:   Vec<Vertex>,
    pub aabb:       QuadAABB,
    pub typ:        NavType,
    pub(crate) neighbours: Neighbours
}
impl Polygon {
    fn from_navquad(q: &mut NavQuad) -> Self {
        let mut poly = Polygon {
            index: q.index,
            vertices: vec![
                Vertex::from_vec3a(q.aabb.min_x_min_z),
                Vertex::from_vec3a(q.aabb.min_x_max_z),
                Vertex::from_vec3a(q.aabb.max_x_min_z),
                Vertex::from_vec3a(q.aabb.max_x_max_z)
            ],            
            aabb: q.aabb,
            typ: q.typ,
            neighbours: q.neighbours.clone()
        };
        poly.sort_vertices();
        return poly;
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
            .map(|pair| [pair[0].index, pair[1].index])
            .chain(std::iter::once([
                self.vertices[self.vertices.len() - 1].index,
                self.vertices[0].index,
            ]))
    }
    fn convert_vertices(&self) -> [[OrderedFloat<f32>; 3]; 4] {
        if self.vertices.len() != 4 {
            panic!("On this stage there should be 4 vertices in {}", self.index);
        }

        return [
            self.vertices[0].convert(),
            self.vertices[1].convert(),
            self.vertices[2].convert(),
            self.vertices[3].convert()
        ]
    }

    fn has_point_int(&self, loc: Vec2) -> bool {
        self.aabb.has_point_int(loc)
    }

    fn sort_vertices(&mut self){
        let n = self.vertices.len() as f32;
        let centroid = self.vertices.iter().map(|v| v.loc.xz()).sum::<Vec2>()/n;
        self.vertices.sort_by(|a, b| {
            let angle_a = (a.loc.z - centroid.y).atan2(a.loc.x - centroid.x);
            let angle_b = (b.loc.z - centroid.y).atan2(b.loc.x - centroid.x);
            angle_b.partial_cmp(&angle_a).unwrap_or(std::cmp::Ordering::Equal)
        });
    }
    
    pub fn ray_intersection(&self, ray: &NavRay) -> Option<(Vec3, f32, usize)>  {
        if let Some(distance) = self.aabb.ray_intersection(ray) {
            if distance > 0.0 {
                let position = ray.position(distance);
                return Some((position, distance, self.index));
            }
        }
        return None;
    }

    pub fn display(&self, gizmos: &mut Gizmos, white: bool, offset_y: f32){
        let clr: Color;
        if white {
            clr = Color::from(WHITE);
        } else {
            clr = self.typ.color();
        }
        for pair in self.vertices.windows(2){
            let mut a: Vec3 = pair[0].loc.into();
            a.y += offset_y;

            let mut b: Vec3 = pair[1].loc.into();
            b.y += offset_y;

            gizmos.line(a, b, clr);
        }
        
        if self.vertices.len() == 4 {
            // Add last line
            let mut first: Vec3 = self.vertices[0].loc.into();
            let mut last: Vec3 = self.vertices[self.vertices.len()-1].loc.into();
            first.y += offset_y;
            last.y += offset_y;
            gizmos.line(last, first, clr);
        }

    }

    pub(crate) fn mindata(&self) -> (NavType, usize) {
        return (self.typ, self.index);
    }
}


#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Vertex {
    pub loc: Vec3A,
    pub(crate) polygons: Neighbours,
    pub index: usize
}
impl Vertex {
    pub fn convert(&self) -> [OrderedFloat<f32>; 3]{
        return [
            OrderedFloat(self.loc[0]),
            OrderedFloat(self.loc[1]),
            OrderedFloat(self.loc[2])
        ];
    }
    pub(crate) fn xz(&self) -> Vec2 {
        self.loc.xz()
    }
    pub fn from_vec3a(loc: Vec3A) -> Self {
        Vertex {
            loc,
            index: 0,
            polygons: Neighbours::default()
        }
    }

    // Joins with polygon of index
    fn joins(&self, index: usize) -> bool {
        self.polygons.iter().any(|(_n_typ, n_index)| n_index == &index)
    }

    // At least one of the polygons of this vertex is a blocker or the length is one
    pub fn is_corner(&self, blockers: &Vec<NavType>) -> bool {
        return self.polygons.iter().any(|(p_typ, _p_index)| blockers.contains(p_typ)) || self.polygons.len() == 1;
    }

    pub fn common(
        &self, 
        other: &Vertex, 
        except: &usize, 
        blockers: &Vec<NavType>
    ) -> Vec<&usize>{
        return 
            self.polygons.iter()
            .filter(|(p_typ, p_index)| !blockers.contains(p_typ) && other.joins(**p_index) && p_index != &except)
            .map(|(_x_typ, x_index)| x_index)
            .collect::<Vec<&usize>>();
    }

}



#[inline]
fn ordered_float_distance(
    a: &[OrderedFloat<f32>; 3], 
    b: &[OrderedFloat<f32>; 3]
) -> f32 {
    let e0 = a[0] - b[0];
    let e1 = a[1] - b[1];
    let e2 = a[2] - b[2];
    return *(e0*e0 + e1*e1 + e2*e2);
}
