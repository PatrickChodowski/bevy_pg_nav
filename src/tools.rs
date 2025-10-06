use bevy::prelude::{Vec2, Vec3A, Vec3};
use bevy::math::Ray3d;
use std::f32::EPSILON;

#[derive(Debug, PartialEq, Copy, Clone, Default)]
pub struct NavRay {
    pub(crate) origin: Vec3A,
    pub(crate) direction: Vec3A,
}

impl NavRay {
    pub fn new(origin: Vec3, direction: Vec3) -> Self {
        NavRay {
            origin: origin.into(),
            direction: direction.normalize().into(),
        }
    }

    /// Position vector describing the ray origin
    pub(crate) fn origin(&self) -> Vec3 {
        self.origin.into()
    }

    /// Unit vector describing the ray direction
    pub(crate) fn direction(&self) -> Vec3 {
        self.direction.into()
    }

    pub(crate) fn position(&self, distance: f32) -> Vec3 {
        (self.origin + self.direction * distance).into()
    }
    pub(crate) fn down(x: f32, z: f32) -> Self {
        let origin = Vec3::new(x, 1000.0, z);
        let dir = Vec3::NEG_Y;
        NavRay {
            origin: origin.into(),
            direction: dir.normalize().into(),
        }
    }
}

impl From<Ray3d> for NavRay {
    fn from(ray: Ray3d) -> Self {
        NavRay::new(ray.origin, *ray.direction)
    }
}


#[inline(always)]
pub(crate) fn ray_triangle_intersection(
    ray: &NavRay,
    triangle: &[Vec3A; 3]
) -> Option<RayHit> {
    raycast_moller_trumbore(ray, triangle)
}


#[derive(Default, Debug)]
pub(crate) struct RayHit {
    pub(crate) distance: f32,
    pub(crate) uv_coords: (f32, f32),
}

impl RayHit {
    /// Get a reference to the intersection's uv coords.
    pub(crate) fn uv_coords(&self) -> &(f32, f32) {
        &self.uv_coords
    }

    /// Get a reference to the intersection's distance.
    pub(crate) fn distance(&self) -> &f32 {
        &self.distance
    }
}

/// Implementation of the Möller-Trumbore ray-triangle intersection test
fn raycast_moller_trumbore(
    ray: &NavRay,
    triangle: &[Vec3A; 3]
) -> Option<RayHit> {
    // Source: https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
    let vector_v0_to_v1: Vec3A = triangle[1] - triangle[0];
    let vector_v0_to_v2: Vec3A = triangle[2] - triangle[0];
    let p_vec: Vec3A = ray.direction.cross(vector_v0_to_v2);
    let determinant: f32 = vector_v0_to_v1.dot(p_vec);

    if determinant < EPSILON {
        return None;
    }

    let determinant_inverse = 1.0 / determinant;

    let t_vec = ray.origin - triangle[0];
    let u = t_vec.dot(p_vec) * determinant_inverse;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q_vec = t_vec.cross(vector_v0_to_v1);
    let v = ray.direction.dot(q_vec) * determinant_inverse;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    // The distance between ray origin and intersection is t.
    let t: f32 = vector_v0_to_v2.dot(q_vec) * determinant_inverse;

    Some(RayHit {
        distance: t,
        uv_coords: (u, v),
    })
}


#[derive(Debug, Clone)]
pub struct IntersectionData {
    normal: Vec3,
    distance: f32,
    triangle_index: usize
}

impl IntersectionData {
    pub fn new(
        normal: Vec3, 
        distance: f32,
        triangle_index: usize) -> Self {
        Self {
            normal,
            distance,
            triangle_index
        }
    }

    /// Get the intersection data's normal.
    #[must_use]
    pub fn normal(&self) -> Vec3 {
        self.normal
    }

    /// Get the intersection data's distance.
    #[must_use]
    pub fn distance(&self) -> f32 {
        self.distance
    }
    
    #[must_use]
    pub fn triangle_index(&self) -> usize {
        self.triangle_index
    }
}






#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct AABB {
    pub(crate) min_x: f32,
    pub(crate) max_x: f32,
    pub(crate) min_z: f32,
    pub(crate) max_z: f32,
}

impl AABB {

    pub(crate) fn dims(&self) -> Vec2 {
        return Vec2::new(self.max_x - self.min_x, self.max_z-self.min_z)
    }
    pub (crate) fn max_edge(&self) -> f32 {
        let dims = self.dims()*0.5;
        return dims.x.max(dims.y);
    }
}