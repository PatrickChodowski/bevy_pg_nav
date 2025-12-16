use bevy::color::palettes::css::{BLACK, WHITE};
use std::ops::RangeInclusive;
use bevy::prelude::*;
use bevy::platform::collections::{HashSet, HashMap};

use crate::pathfinding::{Path, SearchStep, PathFinder};
use crate::plugin::ORIGIN_HEIGHT;


#[derive(Clone, Debug)]
pub struct PGVertex {
    pub index:    usize,
    pub loc:      Vec3A,
    pub polygons: HashSet<usize>
}
impl PGVertex {
    pub fn is_corner(&self) -> bool {
        return true;
    }
    pub fn xz(&self) -> Vec2 {
        return Vec2::new(0.0, 0.0);
    }
    pub fn common(&self, other: &PGVertex, polygon: &usize) -> Vec<usize> {

        return Vec::new()
    }
}



#[derive(Clone, Debug)]
pub struct PGPolygon {
    pub index:      usize,
    pub vertices:   Vec<PGVertex>,
    pub neighbours: HashSet<usize>
}

impl PGPolygon {
    pub fn has_point(&self, loc: &Vec2) -> bool {

        return true;
    }

    pub fn ray_intersection(
        &self, 
        origin: &Vec3, 
        direction: &Vec3
    ) -> Option<(Vec3, f32, usize)> {


        return None;
    }

    pub fn ray_side_intersection(&self, origin: Vec3, direction: Vec3, len: f32) -> (usize, f32) {
        return self.aabb.ray_side_intersection(origin.into(), direction.into(), len);
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

}


#[derive(Resource, Clone, Debug, bevy::asset::Asset, bevy::reflect::TypePath)]
pub struct Navs {
    pub water: PGNavmesh,
    pub terrain: PGNavmesh
}

impl Default for Navs {
    fn default() -> Self {
        Navs { water: PGNavmesh::default(), terrain: PGNavmesh::default() }
    }
}

#[derive(Resource, Clone, Debug, bevy::asset::Asset, bevy::reflect::TypePath)]
pub struct PGNavmesh {
    pub polygons:     HashMap<usize, PGPolygon>,
    pub vertices:     HashMap<usize, PGVertex>,
    pub water_height: f32,
    pub search_limit: usize
}

impl Default for PGNavmesh {
    fn default() -> Self {
        PGNavmesh{
            polygons: HashMap::default(),
            vertices: HashMap::default(),
            water_height: 0.0,
            search_limit: 1000
        }
    }
}

impl PGNavmesh {

    pub fn get_polygon_height(
        &self, 
        loc:          Vec2
    ) -> Option<(&PGPolygon, f32)> {

        if let Some(poly) = self.has_point(loc){
            let origin = Vec3::new(loc.x, ORIGIN_HEIGHT, loc.y);
            let direction = Vec3::NEG_Y;

            if let Some((_pos, dist, _index)) = poly.ray_intersection(&origin, &direction){
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
    pub fn ray_intersection(&self, origin: Vec3, direction: Vec3) -> Option<(Vec3, f32, usize)>  {
        let direction = direction.normalize();
        for (_polygon, polygon) in self.polygons.iter(){
            if let Some((world_pos, _dist, index)) = polygon.ray_intersection(&origin, &direction){
                return Some((world_pos, _dist, index));
            }
        }
        return None;
    }

        pub fn has_point(&self, loc: Vec2) -> Option<&PGPolygon> {
        for (_polygon_id, polygon) in self.polygons.iter(){
            if polygon.has_point(&loc){
                return Some(polygon);
            }
        }
        return None;
    }

    pub fn path_points(
        &self, 
        from:     Vec2, 
        to:       Vec2,
        agent_radius: f32
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
                path: vec![to].into(),
            };
            return Some((path, starting_polygon.index, ending_polygon.index));
        }

        // println!("Need to find path between {}: ({}) and {}: ({})", starting_polygon.index, from, ending_polygon.index, to);
        let mut path_finder = PathFinder::setup(
            self,
            (from, starting_polygon.index),
            (to, ending_polygon.index)
        );

        for _s in 0..self.search_limit {

             match path_finder.search() {
                SearchStep::Found(path) => {
                    let offset_path = path.offset_inward(from, agent_radius);
                    return Some((offset_path, starting_polygon.index, ending_polygon.index));
                }
                SearchStep::NotFound => {
                    return None;
                }
                SearchStep::Continue => {}
            }
        }
        return None;
    }

    pub fn vertex(&self, id: &usize) -> Option<&PGVertex> {


        return None;
    }


    pub fn polygon(&self, id: &usize) -> Option<&PGPolygon> {

        return None;
    }


    // Iterate through polygon neighbours till we find end of raycast
    pub fn ray_intersection_blocker(
        &self, 
        origin: Vec3, 
        direction: Vec3, 
        len: f32,
        origin_polygon: usize,
        blockers: Vec<NavType>
    
    ) -> (bool, f32) {

        // info!("Inside ray intersection blocker");
        // Need to run till we find the polygon with the end of ray or the first blocker
        let mut polys_to_check: Vec<(&NavType, &usize)> = Vec::with_capacity(10);
        let mut polys_buffer: Vec<(&NavType, &usize)> = Vec::with_capacity(10);

        let safety: usize = 5;
        let mut i: usize = 0;

        let Some(origin_poly) = self.polygons.get(&origin_polygon) else {return (false, 0.0);};
        polys_to_check.extend(origin_poly.neighbours.iter());

        while i < safety {
            // info!("Polys to check len: {}", polys_to_check.len());
            for (typ, poly_id) in polys_to_check.iter(){
                let Some(poly) = self.polygons.get(*poly_id) else {continue;};
                let (intersections, min_dist) = poly.ray_side_intersection(origin, direction, len);
                match (intersections, blockers.contains(typ)) {
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

            // if i == safety {warn!("reached safety");}
        }

        return (false, 0.0);
    }

}
