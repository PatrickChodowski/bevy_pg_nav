//! Automatic navmesh obstacles from scene objects.
//!
//! Every scene object (`EditorAsset::Scene`) gets a collider built from its own render mesh, in
//! its own transform, so rotation and scale are always right. Recast then decides walkability
//! from the geometry: a dock or path deck within step height of the ground extends the terrain
//! navmesh, walls and trunks block it, unreachable tops (roofs) end up as islands that are
//! removed. For the water navmesh every object blocks boats where its triangles come down near
//! the waterline, so a dock blocks boats while a high bridge does not.
//!
//! Objects lower than the terrain's `walkable_climb` (grass, flowers, pebbles) are skipped:
//! people step over them anyway, and it keeps generation fast.
//!
//! Exceptions go in an optional JSON file (`PGNavPlugin::nav_overrides`, e.g.
//! `terrain/nav.overrides.json` in the game's assets). The first rule whose `contains` is part of
//! the object's name wins:
//!
//! ```json
//! { "rules": [
//!     { "contains": "SM_Env_Reeds", "water": false },
//!     { "contains": "SM_Env_Tree",  "shape": "hull" },
//!     { "contains": "SM_Prop_Flag", "shape": "none" }
//! ] }
//! ```
//!
//! `shape`: `"mesh"` (default, exact), `"hull"` (convex hull: faster, fills concave parts),
//! `"box"` (bounding box) or `"none"` (ignored). `terrain` / `water` (default true): whether the
//! object takes part in that navmesh. A rule also overrides the low-object skip.

use avian3d::prelude::{Collider, Rotation};
use bevy::mesh::{Indices, Mesh, VertexAttributeValues};
use bevy::prelude::*;
use serde::{Deserialize, Serialize};

/// Collider shape for a scene object.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum NavShape {
    /// The render mesh's triangles: exact.
    #[default]
    Mesh,
    /// Convex hull of the render mesh: cheaper, but fills concave parts (arches, U shapes).
    Hull,
    /// The mesh's bounding box.
    Box,
    /// Not an obstacle at all.
    None,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct NavOverrideRule {
    /// Applies to objects whose name contains this text.
    pub contains: String,
    #[serde(default)]
    pub shape: NavShape,
    #[serde(default = "default_true")]
    pub terrain: bool,
    #[serde(default = "default_true")]
    pub water: bool,
}

fn default_true() -> bool { true }

#[derive(Serialize, Deserialize, Clone, Debug, Default, Asset, TypePath)]
pub struct NavOverrides {
    pub rules: Vec<NavOverrideRule>,
}

impl NavOverrides {
    pub fn rule_for(&self, name: &str) -> Option<&NavOverrideRule> {
        self.rules.iter().find(|r| !r.contains.is_empty() && name.contains(&r.contains))
    }
}

/// What a scene object becomes for navmesh generation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavObjectPlan {
    pub shape: NavShape,
    pub terrain: bool,
    pub water: bool,
}

/// Decides how an object takes part. `height` is its world-space height; objects not taller
/// than `min_height` (the step height) are skipped unless a rule mentions them.
pub fn plan_object(name: &str, height: f32, min_height: f32, overrides: &NavOverrides) -> Option<NavObjectPlan> {
    let plan = match overrides.rule_for(name) {
        Some(rule) => NavObjectPlan { shape: rule.shape, terrain: rule.terrain, water: rule.water },
        None if height <= min_height => return None,
        None => NavObjectPlan { shape: NavShape::Mesh, terrain: true, water: true },
    };
    (plan.shape != NavShape::None && (plan.terrain || plan.water)).then_some(plan)
}

/// The collider for `shape`, in the mesh's own (local) space.
pub fn collider_for(mesh: &Mesh, shape: NavShape) -> Option<Collider> {
    match shape {
        NavShape::Mesh => Collider::trimesh_from_mesh(mesh).or_else(|| Collider::convex_hull_from_mesh(mesh)),
        NavShape::Hull => Collider::convex_hull_from_mesh(mesh),
        NavShape::Box => {
            let (min, max) = local_bounds(mesh)?;
            let size = (max - min).max(Vec3::splat(0.01));
            Some(Collider::compound(vec![(
                (min + max) * 0.5,
                Rotation::IDENTITY,
                Collider::cuboid(size.x, size.y, size.z),
            )]))
        }
        NavShape::None => None,
    }
}

fn local_bounds(mesh: &Mesh) -> Option<(Vec3, Vec3)> {
    let Some(VertexAttributeValues::Float32x3(positions)) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else { return None };
    let first = Vec3::from(*positions.first()?);
    Some(positions.iter().fold((first, first), |(lo, hi), p| (lo.min(Vec3::from(*p)), hi.max(Vec3::from(*p)))))
}

/// The mesh's triangles in world space.
pub fn world_triangles(mesh: &Mesh, transform: &GlobalTransform) -> Vec<[Vec3; 3]> {
    let Some(VertexAttributeValues::Float32x3(positions)) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else { return Vec::new() };
    let world: Vec<Vec3> = positions.iter().map(|p| transform.transform_point(Vec3::from(*p))).collect();
    let indices: Vec<usize> = match mesh.indices() {
        Some(Indices::U16(i)) => i.iter().map(|&v| v as usize).collect(),
        Some(Indices::U32(i)) => i.iter().map(|&v| v as usize).collect(),
        None => (0..world.len()).collect(),
    };
    indices
        .chunks_exact(3)
        .filter_map(|t| Some([*world.get(t[0])?, *world.get(t[1])?, *world.get(t[2])?]))
        .collect()
}

/// World-space height (max y - min y) of a mesh.
pub fn world_height(mesh: &Mesh, transform: &GlobalTransform) -> f32 {
    let Some(VertexAttributeValues::Float32x3(positions)) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else { return 0.0 };
    let (lo, hi) = positions.iter().fold((f32::MAX, f32::MIN), |(lo, hi), p| {
        let y = transform.transform_point(Vec3::from(*p)).y;
        (lo.min(y), hi.max(y))
    });
    if hi >= lo { hi - lo } else { 0.0 }
}

/// Marks as blocked (false) every mask cell a triangle's ground footprint touches: cells whose
/// centre is inside it, plus cells its edges pass through, so thin walls and posts (whose
/// footprint is a sliver) still block. The mask covers `origin + (c, r) * cell`, `cols` wide.
pub fn stamp_footprint(mask: &mut [bool], cols: usize, rows: usize, origin: Vec2, cell: f32, tri: [Vec2; 3]) {
    let to_cell = |p: Vec2| ((p - origin) / cell).floor();
    let mut mark = |c: f32, r: f32| {
        if c >= 0.0 && r >= 0.0 && (c as usize) < cols && (r as usize) < rows {
            mask[r as usize * cols + c as usize] = false;
        }
    };

    // Edges, sampled finer than the cells.
    for i in 0..3 {
        let (a, b) = (tri[i], tri[(i + 1) % 3]);
        let steps = ((b - a).length() / (cell * 0.5)).ceil().max(1.0) as usize;
        for s in 0..=steps {
            let p = to_cell(a.lerp(b, s as f32 / steps as f32));
            mark(p.x, p.y);
        }
    }

    // Interior: cell centres inside the triangle.
    let lo = to_cell(tri[0].min(tri[1]).min(tri[2])).max(Vec2::ZERO);
    let hi = to_cell(tri[0].max(tri[1]).max(tri[2])).min(Vec2::new(cols as f32 - 1.0, rows as f32 - 1.0));
    let cross = |o: Vec2, a: Vec2, b: Vec2| (a - o).perp_dot(b - o);
    let mut r = lo.y;
    while r <= hi.y {
        let mut c = lo.x;
        while c <= hi.x {
            let p = origin + (Vec2::new(c, r) + 0.5) * cell;
            let (d0, d1, d2) = (cross(tri[0], tri[1], p), cross(tri[1], tri[2], p), cross(tri[2], tri[0], p));
            let inside = (d0 >= 0.0 && d1 >= 0.0 && d2 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0 && d2 <= 0.0);
            if inside {
                mark(c, r);
            }
            c += 1.0;
        }
        r += 1.0;
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn low_objects_are_skipped_unless_a_rule_names_them() {
        let none = NavOverrides::default();
        assert_eq!(plan_object("SM_Env_Grass_01", 0.3, 0.5, &none), None);
        assert_eq!(
            plan_object("SM_Bld_Base_01", 6.0, 0.5, &none),
            Some(NavObjectPlan { shape: NavShape::Mesh, terrain: true, water: true })
        );

        let rules: NavOverrides = serde_json::from_str(
            r#"{ "rules": [
                { "contains": "Reeds", "water": false },
                { "contains": "Flag", "shape": "none" },
                { "contains": "Grass_Large", "shape": "hull" }
            ] }"#,
        )
        .unwrap();
        assert_eq!(plan_object("SM_Env_Reeds_01", 1.0, 0.5, &rules).unwrap().water, false);
        assert_eq!(plan_object("SM_Prop_Flag_01", 4.0, 0.5, &rules), None);
        assert_eq!(
            plan_object("SM_Env_Grass_Large_01", 0.3, 0.5, &rules).unwrap().shape,
            NavShape::Hull,
            "a rule includes even a low object"
        );
    }

    #[test]
    fn thin_wall_footprint_still_blocks_cells() {
        // A wall seen from above: a sliver along x, almost no depth.
        let (cols, rows) = (10, 10);
        let mut mask = vec![true; cols * rows];
        let tri = [Vec2::new(1.0, 5.0), Vec2::new(8.0, 5.0), Vec2::new(8.0, 5.001)];
        stamp_footprint(&mut mask, cols, rows, Vec2::ZERO, 1.0, tri);
        for c in 1..=8 {
            assert!(!mask[5 * cols + c], "cell {c} along the wall is blocked");
        }
        assert!(mask[2 * cols + 4], "cells away from the wall stay water");
    }

    #[test]
    fn big_triangle_blocks_its_interior() {
        let (cols, rows) = (10, 10);
        let mut mask = vec![true; cols * rows];
        stamp_footprint(&mut mask, cols, rows, Vec2::ZERO, 1.0, [Vec2::new(0.0, 0.0), Vec2::new(9.0, 0.0), Vec2::new(0.0, 9.0)]);
        assert!(!mask[2 * cols + 2], "inside");
        assert!(mask[8 * cols + 8], "outside the hypotenuse");
    }

    #[test]
    fn box_collider_is_built_from_mesh_bounds() {
        let mesh = Cuboid::new(2.0, 4.0, 6.0).mesh().build();
        assert!(collider_for(&mesh, NavShape::Box).is_some());
        assert!(collider_for(&mesh, NavShape::Mesh).is_some());
        assert!(collider_for(&mesh, NavShape::None).is_none());
    }
}
