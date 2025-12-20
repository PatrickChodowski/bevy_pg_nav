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
        if self.polygons.len() == 1 {
            return true;
        } else {
            return false;
        }
    }

    pub fn xz(&self) -> Vec2 {
        return self.loc.xz();
    }

    fn joins(&self, index: usize) -> bool {
        return self.polygons.contains(&index);
    }

    pub fn common(
        &self, 
        other: &PGVertex,
        except: &usize
    ) -> Vec<usize> {

        return self.polygons.iter()
            .filter(|p_index| other.joins(**p_index) && p_index != &except)
            .map(|x| *x)
            .collect::<Vec<usize>>();
    }
}



#[derive(Clone, Debug)]
pub struct PGPolygon {
    pub index:      usize,
    pub vertices:   Vec<PGVertex>,
    pub neighbours: HashSet<usize>
}

impl PGPolygon {
    pub fn has_point(
        &self, 
        loc: &Vec2
    ) -> bool {
        const EPSILON: f32 = 0.0000001;

        let a = self.vertices[0].xz();
        let b = self.vertices[1].xz();
        let c = self.vertices[2].xz();

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

    pub fn ray_intersection(
        &self, 
        origin:    &Vec3A, 
        direction: &Vec3A
    ) -> Option<(Vec3A, f32)> {

        const EPSILON: f32 = 0.0000001;
        let a = self.vertices[0].loc;
        let b = self.vertices[1].loc;
        let c = self.vertices[2].loc;

        let edge1 = b - a;
        let edge2 = c - a;
        let h = direction.cross(edge2);
        let a = edge1.dot(h);

        // Ray is parallel to triangle
        if a > -EPSILON && a < EPSILON {
            return None;
        }
        let f = 1.0 / a;
        let s = origin - a;
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

    fn edges(&self) -> [(Vec2, Vec2); 3] {
        let a = self.vertices[0].xz();
        let b = self.vertices[1].xz();
        let c = self.vertices[2].xz();
        return [(a,b),(b,c),(c,a)];
    }


    pub fn ray_side_intersection(&self, origin: Vec3, direction: Vec3, len: f32) -> (usize, f32) {
        let ray_segment = (origin.xz(), Vec2::new(origin.x + direction.x, origin.z + direction.z));
        let edges: [(Vec2, Vec2); 3] = self.edges();
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

// #[derive(Resource, Clone, Debug, bevy::asset::Asset, bevy::reflect::TypePath)]
// pub struct Navs {
//     pub water: PGNavmesh,
//     pub terrain: PGNavmesh
// }

// impl Default for Navs {
//     fn default() -> Self {
//         Navs { water: PGNavmesh::default(), terrain: PGNavmesh::default() }
//     }
// }

#[derive(Component, Clone, Debug, bevy::asset::Asset, bevy::reflect::TypePath)]
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
            let origin = Vec3A::new(loc.x, ORIGIN_HEIGHT, loc.y);
            let direction = Vec3A::NEG_Y;

            if let Some((_world_pos, dist)) = poly.ray_intersection(&origin, &direction){
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

    pub fn ray_intersection(&self, origin: &Vec3A, direction: &Vec3A) -> Option<(Vec3A, f32, usize)>  {
        let direction = direction.normalize();
        for (polygon_index, polygon) in self.polygons.iter(){
            if let Some((world_pos, _dist)) = polygon.ray_intersection(&origin, &direction){
                return Some((world_pos, _dist, *polygon_index));
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
        return self.vertices.get(id);
    }

    pub fn polygon(&self, id: &usize) -> Option<&PGPolygon> {
        return self.polygons.get(id);
    }

}
