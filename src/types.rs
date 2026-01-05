use std::ops::RangeInclusive;
use bevy::prelude::*;
use bevy::platform::collections::HashSet;
use serde::{Deserialize, Serialize};

use crate::pgnavmesh::PGNavmesh;

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
    pub fn locs(&self, pgn: &PGNavmesh) -> [Vec3; 3] {
        let a: Vec3 = pgn.vertex(&self.vertices[0]).loc;
        let b: Vec3 = pgn.vertex(&self.vertices[1]).loc;
        let c: Vec3 = pgn.vertex(&self.vertices[2]).loc;
        return [a,b,c];
    }

    pub fn locs_2d(&self, pgn: &PGNavmesh) -> [Vec2;3]{
        let [a3,b3,c3] = self.locs(pgn);
        let a = a3.xz();
        let b = b3.xz();
        let c = c3.xz();
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
    ) -> Option<Vec3> {

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
            return Some(intersection_point);
        } else {
            return None;
        }
    }

    fn edges(&self, pgn: &PGNavmesh) -> [(Vec2, Vec2); 3] {
        let [a,b,c] = self.locs_2d(pgn);
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

    pub(crate) fn ray_side_intersection(
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

    pub(crate) fn closest_point(&self, p: &Vec2, pgn: &PGNavmesh) -> Vec2 {

        const EPSILON: f32 = 5.0;
        let [a3, b3, c3] = self.locs(pgn);
        let a = a3.xz();
        let b = b3.xz();
        let c = c3.xz();
                
        // Calculate triangle center for offsetting
        let center = (a + b + c) / 3.0;
        
        // Check if P in vertex region outside A
        let ab = b - a;
        let ac = c - a;
        let ap = p - a;
        let d1 = ab.dot(ap);
        let d2 = ac.dot(ap);
        if d1 <= 0.0 && d2 <= 0.0 {
            return a + (center - a).normalize() * EPSILON;
        }
        
        // Check if P in vertex region outside B
        let bp = p - b;
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);
        if d3 >= 0.0 && d4 <= d3 {
            return b + (center - b).normalize() * EPSILON;
        }
        
        // Check if P in edge region of AB
        let vc = d1 * d4 - d3 * d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let v = d1 / (d1 - d3);
            let edge_point = a + ab * v;
            return edge_point + (center - edge_point).normalize() * EPSILON;
        }
        
        // Check if P in vertex region outside C
        let cp = p - c;
        let d5 = ab.dot(cp);
        let d6 = ac.dot(cp);
        if d6 >= 0.0 && d5 <= d6 {
            return c + (center - c).normalize() * EPSILON;
        }
        
        // Check if P in edge region of AC
        let vb = d5 * d2 - d1 * d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let w = d2 / (d2 - d6);
            let edge_point = a + ac * w;
            return edge_point + (center - edge_point).normalize() * EPSILON;
        }
        
        // Check if P in edge region of BC
        let va = d3 * d6 - d5 * d4;
        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            let edge_point = b + (c - b) * w;
            return edge_point + (center - edge_point).normalize() * EPSILON;
        }
        
        // P inside face region - use barycentric coordinates (no offset needed)
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
