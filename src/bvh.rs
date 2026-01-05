use bevy::prelude::*;
use bevy_pg_core::prelude::AABB;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct BVH {
    pub(crate) data: Vec<BVHNode>
}
impl BVH {
    pub(crate) fn empty() -> Self {
        BVH {data: Vec::new()}
    } 
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub(crate) enum BHVType {
    Branch(Vec<usize>), // Nodes
    Leaf(Vec<usize>)    // Navmesh Polygons
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct BVHNode {
    pub id:   u32,
    pub aabb: AABB, 
    pub typ:  BHVType
}

pub(crate) fn aabb_intersects_triangle(aabb: &AABB, p0: Vec2, p1: Vec2, p2: Vec2) -> bool {
    // First check: are any triangle vertices inside the AABB?
    if aabb.has_point(p0) || aabb.has_point(p1)  || aabb.has_point(p2)  {
        return true;
    }
    
    // Second check: does any triangle edge intersect the AABB edges?
    let aabb_edges = [
        (Vec2::new(aabb.min_x, aabb.min_z), Vec2::new(aabb.max_x, aabb.min_z)),
        (Vec2::new(aabb.max_x, aabb.min_z), Vec2::new(aabb.max_x, aabb.max_z)),
        (Vec2::new(aabb.max_x, aabb.max_z), Vec2::new(aabb.min_x, aabb.max_z)),
        (Vec2::new(aabb.min_x, aabb.max_z), Vec2::new(aabb.min_x, aabb.min_z)),
    ];
    
    let triangle_edges = [(p0, p1), (p1, p2), (p2, p0)];
    
    for &(a1, a2) in &aabb_edges {
        for &(t1, t2) in &triangle_edges {
            if segments_intersect(a1, a2, t1, t2) {
                return true;
            }
        }
    }
    
    // Third check: is the AABB center inside the triangle?
    let center = Vec2::new(
        (aabb.min_x + aabb.max_x) / 2.0,
        (aabb.min_z + aabb.max_z) / 2.0,
    );
    if point_in_triangle(center, p0, p1, p2) {
        return true;
    }
    
    false
}

fn segments_intersect(a1: Vec2, a2: Vec2, b1: Vec2, b2: Vec2) -> bool {
    let d1 = sign((b2 - b1).perp_dot(a1 - b1));
    let d2 = sign((b2 - b1).perp_dot(a2 - b1));
    let d3 = sign((a2 - a1).perp_dot(b1 - a1));
    let d4 = sign((a2 - a1).perp_dot(b2 - a1));
    
    if d1 != d2 && d3 != d4 {
        return true;
    }
    
    // Check for collinear overlapping segments
    if d1 == 0 && on_segment(b1, a1, b2) { return true; }
    if d2 == 0 && on_segment(b1, a2, b2) { return true; }
    if d3 == 0 && on_segment(a1, b1, a2) { return true; }
    if d4 == 0 && on_segment(a1, b2, a2) { return true; }
    
    false
}

fn sign(val: f32) -> i32 {
    if val > 1e-6 { 1 } else if val < -1e-6 { -1 } else { 0 }
}

fn on_segment(p: Vec2, q: Vec2, r: Vec2) -> bool {
    q.x <= p.x.max(r.x) && q.x >= p.x.min(r.x) &&
    q.y <= p.y.max(r.y) && q.y >= p.y.min(r.y)
}

fn point_in_triangle(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool {
    let d1 = sign((b - a).perp_dot(p - a));
    let d2 = sign((c - b).perp_dot(p - b));
    let d3 = sign((a - c).perp_dot(p - c));
    
    let has_neg = d1 < 0 || d2 < 0 || d3 < 0;
    let has_pos = d1 > 0 || d2 > 0 || d3 > 0;
    
    !(has_neg && has_pos)
}
