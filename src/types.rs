use bevy::prelude::*;
use bevy::platform::collections::{HashSet, HashMap};

#[allow(unused_imports)]
use bevy::color::palettes::tailwind::{PURPLE_500, BLUE_500, GREEN_200, GREEN_500, YELLOW_500, RED_500};
use serde::{Serialize, Deserialize};
use std::cmp::Ordering;

use crate::tools::Ray;
use crate::tools::AABB;

use crate::plugin::NavConfig;


#[derive(Component)]
pub struct NavStatic;

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

#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct Navigable {
    pub y: f32,
    pub scale_width: f32
}
impl Navigable {
    pub fn new(y: f32, scale_width: f32) -> Self {
        return Navigable{y, scale_width};
    }
}

#[derive(Debug, Default, PartialEq, Eq, Copy, Clone)]
pub(crate) enum RayMeshType {
    Navigable,
    #[default]
    Blocker
}

#[derive(Debug)]
pub(crate) enum RayMeshShape {
    Circle((Vec3, f32, f32)), // Loc, radius, Vertex Height
    Rect((Vec3, Vec3, Vec2, f32))  // Loc, normal, Dimensions, Vertex Height
}
impl RayMeshShape{
    pub(crate) fn new(
        typ: RayMeshType, 
        tr:  Transform, 
        y: f32, 
        aabb: &AABB,
        navconfig: &Res<NavConfig>
    ) -> Self {
        match typ {
            RayMeshType::Navigable => {
                let mut loc = tr.translation;
                loc.y = y; // Actually 
                let initial_normal = Vec3::Z;
                let rotated_normal = (tr.rotation * initial_normal).normalize();
                return RayMeshShape::Rect(
                    (loc, rotated_normal, aabb.dims(), tr.translation.y)
                );
            }
            RayMeshType::Blocker => {
                let radius = aabb.max_edge()*navconfig.blocker_scale;
                let mut loc = tr.translation;
                loc.y = y;
                return RayMeshShape::Circle((loc, radius*radius, tr.translation.y));
            }
        }
    }
}



#[derive(Debug)]
pub(crate) struct RayMesh {
    pub y:         f32,
    pub name:      Name,
    pub shape:     RayMeshShape,
    pub typ:       RayMeshType
}
impl RayMesh {

    pub(crate) fn test(&self, ray: &Ray) -> Option<(f32, NavType)> {
        match &self.shape {    

            RayMeshShape::Circle((loc, radius, height)) => {
                let distance: f32 = ray.origin.xz().distance_squared(loc.xz());
                if &distance <= radius {
                    match self.typ {
                        RayMeshType::Blocker => {return Some((*height, NavType::Blocker))}
                        RayMeshType::Navigable => {return Some((loc.y, NavType::Navigable))}
                    };
                }
            }

            RayMeshShape::Rect((loc, norm, dims, height)) => {
                let denom = norm.dot(Vec3::from(ray.direction));
                // Check if the ray is parallel (denominator is too small)
                if denom.abs() < 1e-6 {
                    return None;
                }
                let t = (loc - Vec3::from(ray.origin)).dot(*norm)/denom;

                // If t is negative, the intersection is behind the ray
                if t < 0.0 {
                    return None;
                }
        
                // Compute intersection point
                let hit_point = Vec3::from(ray.origin + t * ray.direction);
        
                // Compute local axes of the rectangle (assuming normal is correct)
                let right = norm.cross(Vec3::Z).normalize();
                let up = right.cross(*norm).normalize();
        
                // Project hit_point onto rectangle's local axes
                let local_x = (hit_point - loc).dot(right);
                let local_z = (hit_point - loc).dot(up);

                if local_x.abs() <= dims.x / 2.0 && local_z.abs() <= dims.y / 2.0 {
                    match self.typ {
                        RayMeshType::Blocker => {return Some((*height, NavType::Blocker))}
                        RayMeshType::Navigable => {return Some((hit_point.y, NavType::Navigable))}
                    };
                } else {
                    return None;
                }
            }
        }
    
        return None;
    }
}

impl PartialEq for RayMesh {
    fn eq(&self, other: &Self) -> bool {
        self.y == other.y
    }
}

impl Eq for RayMesh {}

impl PartialOrd for RayMesh {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for RayMesh {
    fn cmp(&self, other: &Self) -> Ordering {
        other.y.partial_cmp(&self.y).unwrap_or(Ordering::Equal)
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

    // pub(crate) fn cleanup_vertices(&mut self) {
    //     if self.vertices.len() == 4 {
    //         self.sort_vertices();
    //         return;
    //     }
    //     self.vertices.retain(|v| 
    //         (
    //             (v.x == self.aabb.min_x) | 
    //             (v.x == self.aabb.max_x) 
    //         )&
    //         (
    //             (v.z == self.aabb.max_z) | 
    //             (v.z == self.aabb.min_z)
    //         )
    //     );

    //     self.sort_vertices();
    // }
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
    Top,
    Bottom,
    Left,
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


    fn default_big() -> Self {
        let mut aabb = QuadAABB::default();
        aabb.max_x = f32::MIN;
        aabb.min_x = f32::MAX;
        aabb.max_z = f32::MIN;
        aabb.min_z = f32::MAX;
        return aabb;
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

    fn normal(&self) -> Vec3A {
        let e1 = self.max_x_min_z - self.min_x_min_z;
        let e2 = self.min_x_max_z - self.min_x_min_z;
        let normal = e1.cross(e2).normalize();
        return normal;
    }


    pub fn ray_intersection(&self, ray: &Ray) -> Option<f32> {

        let min_corner = self.min_x_min_z;
        let max_corner = self.max_x_max_z;
    
        let inv_dir = ray.direction.recip();
        
        let t1 = (min_corner - ray.origin) * inv_dir;
        let t2 = (max_corner - ray.origin) * inv_dir;
        
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
    fn top(&self) -> [(f32, f32); 2]{
        return [(self.min_x, self.min_z), (self.max_x, self.min_z)];
    }

    #[inline(always)]
    fn bottom(&self) -> [(f32, f32); 2]{
        return [(self.min_x, self.max_z), (self.max_x, self.max_z)];
    }

    #[inline(always)]
    fn left(&self) -> [(f32, f32); 2]{
        return [(self.min_x, self.min_z), (self.min_x, self.max_z)];
    }

    #[inline(always)]
    fn right(&self) -> [(f32, f32); 2]{
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



    #[inline(always)]
    fn edges_u32(&self) -> [[(u32, u32); 2]; 4] {
        [
            self.top_u32(),
            self.right_u32(),
            self.left_u32(),
            self.bottom_u32()
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
            Edge::Top => {
                let edge = self.top_u32();
                if edge == other.bottom_u32() {
                    return Some(edge)
                }
            }
            Edge::Left => {
                let edge = self.left_u32();
                if edge == other.right_u32() {
                    return Some(edge)
                }  
            }
            Edge::Right => {
                let edge = self.right_u32();
                if edge == other.left_u32() {
                    return Some(edge)
                }  
            }
        }

        return None;
    }


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
