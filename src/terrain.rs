use std::f32::EPSILON;

use bevy::prelude::*;
use bevy::render::{
    mesh::{Indices, Mesh, VertexAttributeValues},
    render_resource::PrimitiveTopology,
};
use bevy::platform::collections::HashMap;

use crate::tools::{Ray, IntersectionData, ray_triangle_intersection};

// Generate Optimized data structure for raycast testing
#[derive(Debug, Clone)]
pub(crate) struct TerrainRayMeshData {
    mesh_transform:   Mat4,
    vertex_positions: Vec<[Vec3A; 3]>,
    vertex_normals:   Vec<Vec3A>,
    triangle_count:   usize,
    quads_mapping:    HashMap<usize, usize> // Pairs of triangles that are making a quad
}
impl TerrainRayMeshData {
    pub(crate)  fn triangle(&self, triangle_id: usize) -> Option<[Vec3A; 3]> {
        if let Some(vpos) = self.vertex_positions.get(triangle_id){
            let mt = self.mesh_transform.inverse();
            return Some([
                mt.transform_point3a(vpos[0]),
                mt.transform_point3a(vpos[1]),
                mt.transform_point3a(vpos[2]),
            ]);
        }
        return None;
    }

    pub(crate) fn test(&self, ray: &Ray) -> Option<(f32, usize, Vec3)>{      
        if let Some(intersection_data) = self.ray_intersection(ray){
            let dist = intersection_data.distance().round() as i32;
            let height: f32 = (ray.origin.y as i32 - dist) as f32;

            let mut group_id = intersection_data.triangle_index();
            if let Some(mapping) = self.quads_mapping.get(&group_id){
                group_id = *mapping;
            }

            return Some((height, group_id, intersection_data.normal()));
        } else {
            return None;
        }
    }

    pub(crate) fn from_mesh(mesh: &Mesh, mesh_transform: &Mat4) -> Self {
        if mesh.primitive_topology() != PrimitiveTopology::TriangleList {
            panic!("Wrong Topology for Terrain {:?}", mesh.primitive_topology());
        };
        let vertex_positions: &Vec<[f32; 3]> = match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            None => panic!("Mesh does not contain vertex positions"),
            Some(vertex_values) => match &vertex_values {
                VertexAttributeValues::Float32x3(positions) => positions,
                _ => panic!("Unexpected types in {:?}", Mesh::ATTRIBUTE_POSITION),
            },
        };
        let vertex_normals: Vec<[f32; 3]> = 
            if let Some(normal_values) = mesh.attribute(Mesh::ATTRIBUTE_NORMAL) {
                match &normal_values {
                    VertexAttributeValues::Float32x3(normals) => normals.to_vec(),
                    _ => panic!("wrong normals"),
                }
            } else {
                panic!("No normals");
            };

        let mut tri_pos: Vec<[Vec3A; 3]> = Vec::new();
        let mut tri_norm: Vec<Vec3A> = Vec::new();

        if let Some(indices) = &mesh.indices() {
            match indices {
                Indices::U16(vertex_indices) => {
                    for index in vertex_indices.chunks(3) {
                        let tri_vertex_positions = [
                            Vec3A::from(vertex_positions[index[0] as usize]),
                            Vec3A::from(vertex_positions[index[1] as usize]),
                            Vec3A::from(vertex_positions[index[2] as usize]),
                        ];
                        let tvn = [
                            Vec3A::from(vertex_normals[index[0] as usize]),
                            Vec3A::from(vertex_normals[index[1] as usize]),
                            Vec3A::from(vertex_normals[index[2] as usize]),
                        ];
                        tri_pos.push(tri_vertex_positions);

                        if (tvn[0] == tvn[1]) & (tvn[0] == tvn[2]){
                            tri_norm.push(tvn[0]);
                        } else{
                            panic!("TRI FAILS on normals");
                        }

                    }
                }
                Indices::U32(vertex_indices) => {
                    for index in vertex_indices.chunks(3) {
                        let tri_vertex_positions = [
                            Vec3A::from(vertex_positions[index[0] as usize]),
                            Vec3A::from(vertex_positions[index[1] as usize]),
                            Vec3A::from(vertex_positions[index[2] as usize]),
                        ];
                        let tvn = [
                            Vec3A::from(vertex_normals[index[0] as usize]),
                            Vec3A::from(vertex_normals[index[1] as usize]),
                            Vec3A::from(vertex_normals[index[2] as usize]),
                        ];
                        tri_pos.push(tri_vertex_positions);
                        if (tvn[0] == tvn[1]) & (tvn[0] == tvn[2]){
                            tri_norm.push(tvn[0]);
                        } else{
                            panic!("TRI FAILS on normals");
                        }

                    }
                }
            }
        } else {
            panic!("No indices in terrain");
        }
        let triangle_count = tri_pos.len();
        let mut trmd =  TerrainRayMeshData{
            mesh_transform: mesh_transform.inverse(),
            vertex_positions: tri_pos,
            vertex_normals: tri_norm,
            triangle_count,
            quads_mapping: HashMap::new()
        };

        trmd.map_quads();

        info!("[NAVMESH][TERRAIN] Count of Mapped triangles: {}, groups: {}", 
            trmd.quads_mapping.len(),
            trmd.quads_mapping.len()/2
        );

        return trmd;

    }

    fn map_quads(&mut self){
        let mut quads_mapping: HashMap<usize, usize> = HashMap::with_capacity(self.triangle_count);

        for a_tri in 0..self.triangle_count{

            if quads_mapping.contains_key(&a_tri){
                continue;
            }
            if !self.is_right(a_tri){
                continue;
            }
            let a_points: [Vec3A; 2] = self.get_longest_side_points(a_tri);

            for b_tri in 0..self.triangle_count{

                if a_tri == b_tri {
                    continue;
                }
                if quads_mapping.contains_key(&b_tri){
                    continue;
                }
                if !self.is_right(b_tri){
                    continue;
                }
                let b_points: [Vec3A; 2] = self.get_longest_side_points(b_tri);

                if self.normal(a_tri) != self.normal(b_tri) {
                    continue;
                }

                if ((a_points[0] == b_points[0]) & (a_points[1] == b_points[1])) |
                   ((a_points[0] == b_points[1]) & (a_points[1] == b_points[0])) {

                    let index = a_tri.min(b_tri);
                    quads_mapping.insert(a_tri, index);
                    quads_mapping.insert(b_tri, index);

                }
            }
        }
        self.quads_mapping = quads_mapping;
    }

    fn normal(&self, triangle_id: usize) -> Vec3A {
        return self.vertex_normals[triangle_id];
    }

    fn get_longest_side_points(&self, triangle_id: usize) -> [Vec3A; 2] {

        let tri_pos = self.triangle(triangle_id).unwrap();
        
        let v0 = tri_pos[0];
        let v1 = tri_pos[1];
        let v2 = tri_pos[2];

        let edges: [(Vec3A, Vec3A, f32); 3] = [
            (v0, v1, (v1[0] - v0[0]).powi(2) + (v1[2] - v0[2]).powi(2)),
            (v1, v2, (v2[0] - v1[0]).powi(2) + (v2[2] - v1[2]).powi(2)),
            (v2, v0, (v0[0] - v2[0]).powi(2) + (v0[2] - v2[2]).powi(2)),
        ];

        let longest_edge: [Vec3A; 2] = edges.into_iter()
            .max_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(v1, v2, _)| [v1, v2])
            .unwrap();

        return longest_edge;
    }

    fn is_right(&self, triangle_id: usize) -> bool{

        let tri_pos = self.triangle(triangle_id).unwrap();
        let v0 = tri_pos[0];
        let v1 = tri_pos[1];
        let v2 = tri_pos[2];

        let mut sides: [f32; 3] = [
            (v1[0] - v0[0]).powi(2) + (v1[2] - v0[2]).powi(2),
            (v2[0] - v1[0]).powi(2) + (v2[2] - v1[2]).powi(2),
            (v0[0] - v2[0]).powi(2) + (v0[2] - v2[2]).powi(2),
        ];

        sides.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let s0 = sides[0];
        let s1 = sides[1];
        let s2 = sides[2];

        return (s0 + s1) - s2 <= EPSILON;
    }





    fn ray_intersection(
        &self, 
        ray: &Ray
    ) -> Option<IntersectionData> {

        let mesh_space_ray = Ray::new(
            self.mesh_transform.transform_point3(ray.origin()),
            self.mesh_transform.transform_vector3(ray.direction()),
        );

        for t in 0..self.triangle_count{

            let tri_vertices = self.vertex_positions[t];
            let tri_normals = self.vertex_normals[t];

            if let Some(ray_hit) = ray_triangle_intersection(&mesh_space_ray, &tri_vertices) {
                let distance = *ray_hit.distance();
                // info!("distance: {}", distance);
                if distance > 0.0 {
                    let position = mesh_space_ray.position(distance);
                    let u = ray_hit.uv_coords().0;
                    let v = ray_hit.uv_coords().1;
                    let w = 1.0 - u - v;
                    let normal: Vec3 = (tri_normals * u + tri_normals * v + tri_normals * w).into();
                    let intersection = IntersectionData::new(
                        self.mesh_transform.transform_point3(position),
                        self.mesh_transform.transform_vector3(normal),
                        self.mesh_transform
                            .transform_vector3(mesh_space_ray.direction() * distance)
                            .length(),
                        t
                    );
                    return Some(intersection);
                }
            }
        }

        return None;
    }
}