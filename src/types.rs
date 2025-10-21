use bevy::camera::primitives::Aabb;
use bevy::prelude::*;
use bevy::platform::collections::{HashSet, HashMap};
use bevy::color::palettes::tailwind::{BLUE_500, GREEN_500, YELLOW_500, RED_500};
use serde::{Serialize, Deserialize};
use std::cmp::Ordering;

use crate::tools::NavRay;
use crate::tools::AABB;


#[derive(Debug, PartialEq, Copy, Clone)]
pub enum NavStaticType {
    Navigable(f32), // Yoffset
    Blocker
}


#[derive(Component, Clone, Copy)]
pub struct NavStatic {
    pub typ:    NavStaticType,
    pub shape:  NavStaticShape
}
impl NavStatic {
    pub fn navigable_rect(
        x: f32, 
        y: f32,
        y_offset: f32
    ) -> Self {
        NavStatic {
            typ: NavStaticType::Navigable(y_offset),
            shape: NavStaticShape::rect(Vec2::new(x,y))
        }
    }

    pub fn navigable_circle(
        radius : f32,
        y_offset: f32
    ) -> Self {
        NavStatic{
            typ: NavStaticType::Navigable(y_offset),
            shape: NavStaticShape::circle(radius)
        }
    }

    pub fn blocker_rect(
        x: f32, y: f32
    ) -> Self {
        NavStatic{
            typ: NavStaticType::Blocker,
            shape: NavStaticShape::rect(Vec2::new(x,y))
        }
    }

    pub fn blocker_circle(
        radius : f32
    ) -> Self {
        NavStatic{
            typ: NavStaticType::Blocker,
            shape: NavStaticShape::circle(radius)
        }
    }

}

#[derive(Debug, Clone, Copy)]
pub enum NavStaticShape {
    Circle(f32), // radius
    Rect(Vec2)  // Dimensions
}

impl NavStaticShape {
    pub fn circle( 
        radius: f32
    ) -> Self {
        NavStaticShape::Circle(radius)
    }
    pub fn rect(
        dims: Vec2
    ) -> Self {
        NavStaticShape::Rect(dims)
    }
}




#[derive(Resource)]
pub struct NavDebug {
    pub hit_quad_id:   Option<usize>
}
impl Default for NavDebug {
    fn default() -> Self {
        NavDebug{
            hit_quad_id: None
        }
    }
}


#[derive(Debug)]
pub(crate) enum RayTargetMeshShape {
    Circle((Vec3, f32)), // Loc, radius
    Rect((Vec3, Vec3, Vec2, f32))  // Loc, normal, Dimensions, Z angle
}
impl RayTargetMeshShape{
    pub(crate) fn from_navstatic(
        navstatic:   NavStatic,
        transform:   Transform, 
        aabb:        &Aabb,
        // Shape, ray_hit_height, vertex_height
    ) -> (Self, f32, f32) {

        // Extents are not scaled on its won
        let aabb_dims: Vec3 = Vec3::from(aabb.half_extents)*transform.scale*2.0;

        // This is simply top of the box
        let ray_hit_height = transform.translation.y + aabb_dims.z;

        // this is actual navmesh vertex visible/walkable
        let vertex_height: f32 = match navstatic.typ {
            NavStaticType::Navigable(y_offset) => {
                transform.translation.y + y_offset*transform.scale.y
            }
            NavStaticType::Blocker => {
                transform.translation.y - aabb_dims.z
            }
        };

        match navstatic.shape {
            NavStaticShape::Rect(dims) => {
                let initial_normal = Vec3::Z;
                let rotated_normal = (transform.rotation * initial_normal).normalize();
                let custom_aabb = AABB::from_loc_dims(transform.translation, dims*transform.scale.xy());
                let shape = RayTargetMeshShape::Rect((
                    transform.translation, 
                    rotated_normal, 
                    custom_aabb.dims(), 
                    transform.rotation.to_euler(EulerRot::XYZ).2
                ));
                return (shape, ray_hit_height, vertex_height);
            }
            NavStaticShape::Circle(radius) => {
                let shape = RayTargetMeshShape::Circle((transform.translation, radius*transform.scale.x));
                return (shape, ray_hit_height, vertex_height);
            }
        }
    }
}



#[derive(Debug)]
pub(crate) struct RayTargetMesh {
    pub(crate) ray_hit_height: f32, // Its top value, and its sorted by it, because to sort out multiple blockers stacked
    pub(crate) vertex_height:  f32,
    pub(crate) shape:          RayTargetMeshShape,
    pub(crate) typ:            NavStaticType
}
impl RayTargetMesh {

    pub(crate) fn test(&self, ray: &NavRay) -> Option<NavType> {
        match &self.shape {    

            RayTargetMeshShape::Circle((loc, radius)) => {
                let distance: f32 = ray.origin.xz().distance_squared(loc.xz());
                if &distance <= radius {
                    match self.typ {
                        NavStaticType::Blocker => {return Some(NavType::Blocker)}
                        NavStaticType::Navigable(_) => {return Some(NavType::Navigable)}
                    };
                }
            }

            RayTargetMeshShape::Rect((loc, norm, dims, y_angle)) => {
                let plane_normal = norm.normalize();

                let denom = plane_normal.dot(ray.direction.into());
                if denom.abs() < 1e-6 {
                    return None; // Parallel
                }

                let t = (loc - Vec3::from(ray.origin)).dot(*norm)/denom;
                if t < 0.0 {
                    return None; // Plane is behind the ray
                }

                // Intersection point
                let hit_point = Vec3::from(ray.origin + ray.direction * t);

                // Transform hit point into rectangle local space
                let rot = Quat::from_rotation_y(-y_angle);
                let local_hit = rot * (hit_point - loc);
                let half_w = dims.x * 0.5;
                let half_d = dims.y * 0.5;

                if local_hit.x.abs() <= half_w && local_hit.z.abs() <= half_d {
                    match self.typ {
                        NavStaticType::Blocker => {return Some(NavType::Blocker)}
                        NavStaticType::Navigable(_) => {return Some(NavType::Navigable)}
                    };
                } else {
                    return None;
                }
            }
        }
    
        return None;
    }
}

impl PartialEq for RayTargetMesh {
    fn eq(&self, other: &Self) -> bool {
        self.ray_hit_height == other.ray_hit_height
    }
}

impl Eq for RayTargetMesh {}

impl PartialOrd for RayTargetMesh {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for RayTargetMesh {
    fn cmp(&self, other: &Self) -> Ordering {
        other.ray_hit_height.partial_cmp(&self.ray_hit_height).unwrap_or(Ordering::Equal)
    }
}

#[derive(PartialEq, Eq, Clone, Copy, Debug, Hash, Serialize, Deserialize, Reflect)]
pub enum NavType {
    Terrain,
    Water,
    // Slope,
    Navigable,
    Blocker
}
impl NavType {
    pub(crate) fn color(&self) ->  Color {
        let clr = match self {
            NavType::Blocker => {RED_500}
            NavType::Navigable => {YELLOW_500}
            NavType::Terrain => {GREEN_500}
            NavType::Water => {BLUE_500}
        };
        return Color::from(clr);
    }
    pub fn boats_blockers() -> Option<Vec<NavType>> {
        return Some(vec![NavType::Navigable, NavType::Terrain, NavType::Blocker]);
    }
}


#[derive(Clone, Debug, Serialize, Deserialize)]
pub (super) struct Neighbours {
    data: HashMap<NavType, HashSet<usize>>
}
impl Neighbours {
    pub (super) fn insert(&mut self, typ: NavType, index: usize) {
        self.data.get_mut(&typ).unwrap().insert(index);
    }
    pub (super) fn len(&self) -> usize {
        let mut total_count: usize = 0;
        for (_k,v) in self.data.iter(){
            total_count += v.len();
        }   
        return total_count;
    }
    pub (super) fn iter(&self) -> impl Iterator<Item = (&NavType, &usize)> {
        self.data.iter().flat_map(|(nav, set)| {
            set.iter().map(move |u| (nav, u))
        })
    }
    pub (super) fn from_pairs(v: &Vec<(NavType, usize)>) -> Self {
        let mut n = Neighbours::default();
        for (typ, index) in v.iter(){
            n.insert(*typ, *index);
        };
        return n;
    }
    
    pub (super) fn merge_into_new(
        &self, 
        other: &Neighbours,
    ) -> Neighbours {

        let mut n = Neighbours::default();

        for (typ, polys) in self.data.iter(){
            for poly in polys.iter(){
                n.data.get_mut(typ).unwrap().insert(*poly);
            }
        }

        for (typ, polys) in other.data.iter(){
            for poly in polys.iter(){
                n.data.get_mut(typ).unwrap().insert(*poly);
            }
        }

        return n;

    }

    pub (super) fn remove(&mut self, index: usize) -> bool {
        for (_typ, polys) in self.data.iter_mut(){
            if polys.contains(&index){
                return polys.remove(&index);
            }
        }
        return false;
    }
}


impl Default for Neighbours {
    fn default() -> Self {
        Self {
            data: HashMap::from([
                (NavType::Terrain, HashSet::new()),
                (NavType::Navigable, HashSet::new()),
                (NavType::Blocker, HashSet::new()),
                (NavType::Water, HashSet::new()),
            ])
        }
    }
}


#[derive(Clone, Debug)]
pub(crate) struct NavQuad {
    pub(crate) group_id:     usize, // Terrain quad/Blocker/Navigable ID, Water part ID?
    pub(crate) loc:          Vec3A,
    pub(crate) index:        usize,
    pub(crate) vertices:     Vec<Vec3A>,
    pub(crate) normal:       Vec3A,
    pub(crate) typ:          NavType,
    pub(crate) aabb:         QuadAABB,
    pub(crate) neighbours:   Neighbours
}

impl NavQuad {
    pub(crate) fn merge_by_aabb(
        &self, 
        other: &NavQuad,
        edge: [(f32, f32); 2]
    ) -> (usize, usize, NavQuad) {

        // Merge AABBs into new one
        let mut new_aabb = self.aabb;
        new_aabb.merge(&other.aabb);

        // Merge Vertices, remove a pair from the edge
        let mut new_vertices = self.vertices.clone();
        for vertex in other.vertices.iter() {
            if ((vertex.x == edge[0].0) & (vertex.z == edge[0].1)) |
               ((vertex.x == edge[1].0) & (vertex.z == edge[1].1)) {
                continue;
            }
            new_vertices.push(*vertex);
        }

        // Merge Neighbours:
        let mut new_neighbours: Neighbours = self.neighbours.merge_into_new(&other.neighbours);
        let keep_index = self.index.min(other.index);
        let remove_index = self.index.max(other.index);

        new_neighbours.remove(keep_index);
        new_neighbours.remove(remove_index);

        // Takes min index of 2, normal, type (should be the same), merged aabb, vertices and neighbours
        let mut new_nav_quad = NavQuad { 
            group_id: self.group_id,
            loc: new_aabb.center(),
            index: keep_index,
            normal: self.normal,
            typ: self.typ,
            aabb: new_aabb,
            vertices: new_vertices,
            neighbours: new_neighbours
        };

        new_nav_quad.sort_vertices();
        return (keep_index, remove_index, new_nav_quad);
    }

    pub(crate)fn sort_vertices(&mut self){
        let n = self.vertices.len() as f32;
        let centroid = self.vertices.iter().map(|v| v.xz()).sum::<Vec2>()/n;
        self.vertices.sort_by(|a, b| {
            let angle_a = (a.z - centroid.y).atan2(a.x - centroid.x);
            let angle_b = (b.z - centroid.y).atan2(b.x - centroid.x);
            angle_b.partial_cmp(&angle_a).unwrap_or(std::cmp::Ordering::Equal)
        });
    }
}



#[derive(Clone, Debug, Copy, Serialize, Deserialize)]
pub struct QuadAABB {
    // Edges
    pub min_x: f32,
    pub max_x: f32,
    pub min_z: f32,
    pub max_z: f32,

    // Corners
    pub min_x_min_z: Vec3A,
    pub min_x_max_z: Vec3A,
    pub max_x_min_z: Vec3A,
    pub max_x_max_z: Vec3A
}
impl Default for QuadAABB {
    fn default() -> Self {
        return QuadAABB{
            min_x: 0.0, 
            max_x: 0.0, 
            min_z: 0.0, 
            max_z: 0.0,
            min_x_min_z: Vec3A::ZERO, 
            min_x_max_z: Vec3A::ZERO, 
            max_x_min_z: Vec3A::ZERO, 
            max_x_max_z: Vec3A::ZERO
        };
    }
}


#[derive(Clone, Copy)]
pub(crate) enum Edge {
    // Top,
    Bottom,
    // Left,
    Right
}

impl QuadAABB {
    pub fn from_loc(
        loc: Vec3A, 
        extent: f32
    )  -> Self {

        let min_x: f32 = loc.x - extent;
        let max_x: f32 = loc.x + extent;
        let min_z: f32 = loc.z - extent;
        let max_z: f32 = loc.z + extent;
        
        QuadAABB {
            min_x,
            max_x,
            min_z,
            max_z,
            min_x_min_z: Vec3A::new(min_x, loc.y, min_z),
            min_x_max_z: Vec3A::new(min_x, loc.y, max_z),
            max_x_min_z: Vec3A::new(max_x, loc.y, min_z),
            max_x_max_z: Vec3A::new(max_x, loc.y, max_z),
        }
    } 

    pub fn corners(
        &self
    ) -> Vec<Vec3A> {
        return vec![
            self.min_x_min_z,
            self.min_x_max_z,
            self.max_x_min_z,
            self.max_x_max_z
        ];
    }

    pub fn has_point(&self, loc: Vec2) -> bool {
        loc.x >= self.min_x && loc.x <= self.max_x && loc.y >= self.min_z && loc.y <= self.max_z
    }

    pub fn has_point_int(&self, loc: Vec2) -> bool {
        loc.x  as i32 >= self.min_x as i32 
        && loc.x  as i32  <= self.max_x  as i32  
        && loc.y  as i32  >= self.min_z  as i32  
        && loc.y  as i32  <= self.max_z as i32 
    }

    fn corners_2d(&self) -> Vec<Vec2> {
        return vec![
            Vec2::new(self.min_x, self.min_z).round(),  // min xz
            Vec2::new(self.min_x, self.max_z).round(),  // min x max z 
            Vec2::new(self.max_x, self.min_z).round(),  // max x min z
            Vec2::new(self.max_x, self.max_z).round(),  // max_x max_z
        ];
    }

    // fn normal(&self) -> Vec3A {
    //     let e1 = self.max_x_min_z - self.min_x_min_z;
    //     let e2 = self.min_x_max_z - self.min_x_min_z;
    //     let normal = e1.cross(e2).normalize();
    //     return normal;
    // }


    pub fn ray_intersection(&self, origin: Vec3A, direction: Vec3A) -> Option<f32> {

        let min_corner = self.min_x_min_z;
        let max_corner = self.max_x_max_z;
    
        let inv_dir = direction.recip();
        
        let t1 = (min_corner - origin) * inv_dir;
        let t2 = (max_corner - origin) * inv_dir;
        
        let t_min = Vec3A::min(t1, t2);
        let t_max = Vec3A::max(t1, t2);
        
        let t_enter = t_min.max_element();
        let t_exit = t_max.min_element();
        
        let hit: bool = t_enter <= t_exit && t_exit >= 0.0;
        if hit {
            return Some(t_enter.max(0.0));
        } else {
            // error!("Issue for ray_intersection with {:?}, ray: {:?}", self, ray);
            return None;
        }
    }

    pub fn ray_side_intersection(&self, origin: Vec3A, direction: Vec3A, len: f32) -> (usize, f32) {
        let end = Vec2::new(origin.x + direction.x, origin.z + direction.z);
        let (r_x1, r_z1) = (origin.x, origin.z);
        let (r_x2, r_z2) = (end.x, end.y);
        let edges: [[(f32, f32); 2]; 4] = self.edges_f32();
        let mut distances = Vec::with_capacity(2);
        for &[(x1, z1), (x2, z2)] in edges.iter() {
            if let Some(t) = line_segments_intersect((r_x1, r_z1), (r_x2, r_z2), (x1, z1), (x2, z2)) {
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


    pub(crate) fn edges_corners_2d(&self) -> (Vec<[Vec2; 2]>, Vec<Vec2>) {
        let corners = self.corners_2d();
    
        let v0 = corners[0]; // min xz
        let v1 = corners[1]; // min x max z
        let v2 = corners[2]; // max x min z
        let v3 = corners[3]; // max xz

        let edges = vec![
            [v0, v1],  // min X
            [v2, v3],  // max X
            [v0, v2],  // min Z
            [v1, v3]   // max z
        ];
    
        return (edges, corners);
    }

    #[inline(always)]
    fn top_f32(&self) -> [(f32, f32); 2]{
        return [(self.min_x, self.min_z), (self.max_x, self.min_z)];
    }

    #[inline(always)]
    fn bottom_f32(&self) -> [(f32, f32); 2]{
        return [(self.min_x, self.max_z), (self.max_x, self.max_z)];
    }

    #[inline(always)]
    fn left_f32(&self) -> [(f32, f32); 2]{
        return [(self.min_x, self.min_z), (self.min_x, self.max_z)];
    }

    #[inline(always)]
    fn right_f32(&self) -> [(f32, f32); 2]{
        return [(self.max_x, self.min_z), (self.max_x, self.max_z)];
    }

    #[inline(always)]
    fn top_u32(&self) -> [(u32, u32); 2]{
        return [(self.min_x as u32, self.min_z as u32), (self.max_x as u32, self.min_z as u32)];
    }

    #[inline(always)]
    fn bottom_u32(&self) -> [(u32, u32); 2]{
        return [(self.min_x as u32, self.max_z as u32 ), (self.max_x as u32, self.max_z as u32)];
    }

    #[inline(always)]
    fn left_u32(&self) -> [(u32, u32); 2]{
        return [(self.min_x as u32, self.min_z as u32), (self.min_x as u32, self.max_z as u32)];
    }

    #[inline(always)]
    fn right_u32(&self) -> [(u32, u32); 2]{
        return [(self.max_x as u32, self.min_z as u32), (self.max_x as u32, self.max_z as u32)];
    }

    fn edges_u32(&self) -> [[(u32, u32); 2]; 4] {
        [
            self.top_u32(),
            self.right_u32(),
            self.left_u32(),
            self.bottom_u32()
        ]
    }

    fn edges_f32(&self) -> [[(f32, f32); 2]; 4] {
        [
            self.top_f32(),
            self.right_f32(),
            self.left_f32(),
            self.bottom_f32()
        ]
    }

    pub (super) fn check_edge(
        &self, 
        other: &QuadAABB, 
        edge:  Edge  // Edge from the perspective of self
    ) -> Option<[(u32, u32);2]> {

        match edge {
            Edge::Bottom => {
                let edge = self.bottom_u32();
                if edge == other.top_u32() {
                    return Some(edge)
                }
            }
            // Edge::Top => {
            //     let edge = self.top_u32();
            //     if edge == other.bottom_u32() {
            //         return Some(edge)
            //     }
            // }
            // Edge::Left => {
            //     let edge = self.left_u32();
            //     if edge == other.right_u32() {
            //         return Some(edge)
            //     }  
            // }
            Edge::Right => {
                let edge = self.right_u32();
                if edge == other.left_u32() {
                    return Some(edge)
                }  
            }
        }

        return None;
    }

    #[allow(dead_code)]
    pub(crate) fn get_identical_edge(&self, other: &QuadAABB) -> Option<[(u32, u32);2]> {
        for [a0, a1] in self.edges_u32() {
            for [b0, b1] in other.edges_u32() {
                if (a0 == b0 && a1 == b1) | (a0 == b1 && a1 == b0){
                    return Some([a0, a1]);
                }
            }
        }
        return None;
    }

    #[allow(dead_code)]
    fn from_vertices(vertices: Vec<Vec3A>) -> Self {
        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut min_z = f32::MAX;
        let mut max_z = f32::MIN;

        for v in vertices.iter(){
            if v.x < min_x {min_x = v.x;}
            if v.x > max_x {max_x = v.x;}
            if v.z < min_z {min_z = v.z;}
            if v.z > max_z {max_z = v.z;} 
        };

        let mut min_x_min_z = Vec3A::ZERO;
        let mut min_x_max_z = Vec3A::ZERO;
        let mut max_x_min_z = Vec3A::ZERO;
        let mut max_x_max_z = Vec3A::ZERO;

        for v in vertices.iter(){
            if (v.x == min_x) & (v.z == min_z) {
                min_x_min_z = *v;
            }
            if (v.x == min_x) & (v.z == max_z) {
                min_x_max_z = *v;
            }
            if (v.x == max_x) & (v.z == min_z) {
                max_x_min_z = *v;
            }
            if (v.x == max_x) & (v.z == max_z) {
                max_x_max_z = *v;
            }
        }

        if (min_x_min_z == Vec3A::ZERO) | (min_x_max_z == Vec3A::ZERO) | (max_x_min_z == Vec3A::ZERO) | (max_x_max_z == Vec3A::ZERO)  {
            panic!("sad vertices: {:?}", vertices);
        }

        return QuadAABB {
            min_x, max_x, min_z, max_z, 
            min_x_min_z, min_x_max_z, 
            max_x_min_z, max_x_max_z
        }
    }

    // fn dims(&self) -> Vec2 {
    //     return Vec2::new(self.max_x - self.min_x, self.max_z-self.min_z)
    // }

    // fn area(&self) -> f32 {
    //     let dims = self.dims();
    //     return dims.x*dims.y;
    // }

    fn merge(&mut self, other: &QuadAABB){

        if (other.min_x_min_z.x <= self.min_x_min_z.x) & (other.min_x_min_z.z <= self.min_x_min_z.z) {
            self.min_x_min_z = other.min_x_min_z;
        }

        if (other.min_x_max_z.x <= self.min_x_max_z.x) & (other.min_x_max_z.z >= self.min_x_max_z.z) {
            self.min_x_max_z = other.min_x_max_z;
        }

        if (other.max_x_min_z.x >= self.max_x_min_z.x) & (other.max_x_min_z.z <= self.max_x_min_z.z) {
            self.max_x_min_z = other.max_x_min_z;
        }

        if (other.max_x_max_z.x >= self.max_x_max_z.x) & (other.max_x_max_z.z >= self.max_x_max_z.z) {
            self.max_x_max_z = other.max_x_max_z;
        }

        self.min_x = self.min_x.min(other.min_x);
        self.min_z = self.min_z.min(other.min_z);
        self.max_x = self.max_x.max(other.max_x);
        self.max_z = self.max_z.max(other.max_z);
    }

    pub(crate) fn merge_many(aabbs: &Vec<QuadAABB>) -> QuadAABB {

        let mut min_x = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut min_z = f32::INFINITY;
        let mut max_z = f32::NEG_INFINITY;
        let mut y = 0.0_f32;

        for aabb in aabbs {
            min_x = min_x.min(aabb.min_x);
            max_x = max_x.max(aabb.max_x);
            min_z = min_z.min(aabb.min_z);
            max_z = max_z.max(aabb.max_z);

            // assuming all quads share same Y plane
            y = aabb.min_x_min_z.y;
        }

        return QuadAABB {
            min_x,
            max_x,
            min_z,
            max_z,
            min_x_min_z: Vec3A::new(min_x, y, min_z),
            min_x_max_z: Vec3A::new(min_x, y, max_z),
            max_x_min_z: Vec3A::new(max_x, y, min_z),
            max_x_max_z: Vec3A::new(max_x, y, max_z),
        };
    }

    pub(crate) fn center(&self) -> Vec3A {
        let center_x = (self.min_x + self.max_x) * 0.5;
        let center_z = (self.min_z + self.max_z) * 0.5;
        let center_y = self.min_x_min_z.y; // Assuming all Y's are the same
        return Vec3A::new(center_x, center_y, center_z);
    }

}


/// Check if two 2D line segments intersect
fn line_segments_intersect(
    (x1, z1): (f32, f32),
    (x2, z2): (f32, f32),
    (x3, z3): (f32, f32),
    (x4, z4): (f32, f32),
) -> Option<f32> {
    fn cross(ax: f32, az: f32, bx: f32, bz: f32) -> f32 {
        ax * bz - az * bx
    }

    let d1x = x2 - x1;
    let d1z = z2 - z1;
    let d2x = x4 - x3;
    let d2z = z4 - z3;

    let denom = cross(d1x, d1z, d2x, d2z);
    if denom.abs() < f32::EPSILON {
        return None; // parallel
    }

    let dx = x3 - x1;
    let dz = z3 - z1;

    let t = cross(dx, dz, d2x, d2z) / denom;
    let u = cross(dx, dz, d1x, d1z) / denom;

    if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
        Some(t)
    } else {
        None
    }
}
