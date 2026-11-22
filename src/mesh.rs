use bevy::prelude::*;
use bevy::{asset::RenderAssetUsages, mesh::PrimitiveTopology};
use crate::bvh::BVH;

use crate::pgnavmesh::PGNavmesh;

// Converts PGNavmesh to mesh

/// Orientation of a polygon around the Y axis (Newell's method, Y component).
/// Positive means counter-clockwise seen from above, which is what Bevy treats as front-facing.
fn winding_y(pts: &[Vec3]) -> f32 {
    let mut sum = 0.0;
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        sum += (a.z - b.z) * (a.x + b.x);
    }
    sum
}

impl PGNavmesh {
    /// Converts the navmesh polygons to a triangle mesh (one fan per polygon).
    pub fn to_mesh(&self) -> Option<Mesh> {
        self.to_mesh_lifted(0.0)
    }

    /// Same, but moved up by `lift` world units. Useful for a debug overlay on top of terrain
    /// (something like 0.05) so it doesn't z-fight with the ground.
    pub fn to_mesh_lifted(&self, lift: f32) -> Option<Mesh> {
        // HashMap iteration order is random; sort so the output is stable between runs.
        let mut ids: Vec<u32> = self.polygons.keys().copied().collect();
        ids.sort_unstable();

        let mut positions: Vec<[f32; 3]> = Vec::new();
        let mut normals: Vec<[f32; 3]> = Vec::new();

        for id in ids {
            let polygon = &self.polygons[&id];

            let mut pts: Vec<Vec3> = polygon
                .vertices
                .iter()
                .filter_map(|v_index| self.vertices.get(v_index))
                .map(|v| v.loc + Vec3::Y * lift)
                .collect();

            if pts.len() < 3 {
                continue;
            }

            // Make every polygon face up, whatever winding the navmesh was built with.
            if winding_y(&pts) < 0.0 {
                pts.reverse();
            }

            // Navmesh polygons are convex, so a fan from the first vertex is enough.
            for i in 1..pts.len() - 1 {
                let (a, b, c) = (pts[0], pts[i], pts[i + 1]);

                let normal = (b - a).cross(c - a).normalize_or_zero();
                if normal == Vec3::ZERO {
                    continue; // degenerate triangle
                }

                positions.extend([a.to_array(), b.to_array(), c.to_array()]);
                normals.extend([normal.to_array(); 3]);
            }
        }

        if positions.is_empty() {
            return None;
        }

        let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        Some(mesh)
    }

    pub fn remove_polygon(&mut self, id: u32) -> bool {
        let Some(polygon) = self.polygons.remove(&id) else {
            return false;
        };

        // 1. No polygon may list it as a neighbour any more. Scanning all of them (instead of
        //    only polygon.neighbours) also covers one-directional neighbour sets.
        for other in self.polygons.values_mut() {
            other.neighbours.remove(&id);
        }

        // 2. Vertices forget the polygon; vertices that no polygon uses are dropped.
        let mut orphaned: Vec<u32> = Vec::new();
        for vertex_id in &polygon.vertices {
            if let Some(vertex) = self.vertices.get_mut(vertex_id) {
                vertex.polygons.retain(|p| *p != id);
                if vertex.polygons.is_empty() {
                    orphaned.push(*vertex_id);
                }
            }
        }
        for vertex_id in orphaned {
            self.vertices.remove(&vertex_id);
        }

        // 3. The BVH still contains the polygon.
        self.bvh.remove_polygon(id);
        true
    }

    pub fn move_vertex(&mut self, id: u32, loc: Vec3) -> bool {
        let Some(vertex) = self.vertices.get_mut(&id) else {
            return false;
        };
        if vertex.loc == loc {
            return false;
        }
        vertex.loc = loc;
        let polygons = vertex.polygons.clone();

        for polygon_id in polygons {
            let Some(polygon) = self.polygons.get(&polygon_id) else { continue };
            let outline: Vec<Vec2> = polygon
                .vertices
                .iter()
                .filter_map(|v| self.vertices.get(v))
                .map(|v| v.loc.xz())
                .collect();
            self.bvh.reinsert_polygon(polygon_id, &outline);
        }
        true
    }

    // pub fn to_mesh_indexed(&self, lift: f32) -> Option<Mesh> {
    //     use std::collections::HashMap;

    //     let mut ids: Vec<u32> = self.polygons.keys().copied().collect();
    //     ids.sort_unstable();

    //     let mut positions: Vec<[f32; 3]> = Vec::new();
    //     let mut remap: HashMap<u32, u32> = HashMap::new(); // vertex id -> mesh index
    //     let mut indices: Vec<u32> = Vec::new();

    //     for id in ids {
    //         let polygon = &self.polygons[&id];

    //         let mut corners: Vec<(u32, Vec3)> = polygon
    //             .vertices
    //             .iter()
    //             .filter_map(|i| self.vertices.get(i).map(|v| (*i, v.loc + Vec3::Y * lift)))
    //             .collect();
    //         if corners.len() < 3 {
    //             continue;
    //         }

    //         let pts: Vec<Vec3> = corners.iter().map(|c| c.1).collect();
    //         if winding_y(&pts) < 0.0 {
    //             corners.reverse();
    //         }

    //         let mut mesh_index = |vid: u32, p: Vec3| -> u32 {
    //             *remap.entry(vid).or_insert_with(|| {
    //                 positions.push(p.to_array());
    //                 (positions.len() - 1) as u32
    //             })
    //         };

    //         let first = mesh_index(corners[0].0, corners[0].1);
    //         for i in 1..corners.len() - 1 {
    //             let b = mesh_index(corners[i].0, corners[i].1);
    //             let c = mesh_index(corners[i + 1].0, corners[i + 1].1);
    //             indices.extend([first, b, c]);
    //         }
    //     }

    //     if indices.is_empty() {
    //         return None;
    //     }

    //     let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    //     mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    //     mesh.insert_indices(bevy::mesh::Indices::U32(indices));
    //     mesh.compute_smooth_normals(); // or compute_flat_normals(), which un-indexes the mesh again
    //     Some(mesh)
    // }

}


#[derive(Component, Clone, Default)]
pub struct PGNavmeshMesh;

pub fn spawn_navmesh_mesh(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    navmeshes: Query<(Entity, &PGNavmesh), Added<PGNavmesh>>,
) {
    for (entity, navmesh) in &navmeshes {
        let Some(mesh) = navmesh.to_mesh_lifted(0.05) else { continue };
        commands.entity(entity).with_child((
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.1, 0.8, 0.3, 0.4),
                alpha_mode: AlphaMode::Blend,
                unlit: true,
                ..default()
            })),
        ));
    }
}