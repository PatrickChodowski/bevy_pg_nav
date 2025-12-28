use std::f32::EPSILON;

use bevy::prelude::*;
use bevy::mesh::{Indices, Mesh, VertexAttributeValues};
use bevy::render::render_resource::PrimitiveTopology;
use bevy::platform::collections::{HashMap, HashSet};

use crate::tools::{NavRay, IntersectionData, ray_triangle_intersection};
// use crate::triangles::NavPolygon;
// use crate::types::NavType;
// use crate::types::Neighbours;

// Generate Optimized data structure for raycast testing
#[derive(Debug, Clone)]
pub struct TerrainRayMeshData {
    pub mesh_transform:            Mat4,
    triangle_vertex_positions: Vec<[Vec3A; 3]>,
    triangle_normals:          Vec<Vec3A>,
    triangle_count:            usize,
    pub vertices:              Vec<Vec3A>,
    pub edges:                 HashSet<(usize, usize)>,
    // quads_mapping:             HashMap<usize, usize> // Pairs of triangles that are making a quad
}
impl TerrainRayMeshData {
    pub(crate)  fn triangle(&self, triangle_id: usize) -> Option<[Vec3A; 3]> {
        if let Some(vpos) = self.triangle_vertex_positions.get(triangle_id){
            let mt = self.mesh_transform.inverse();
            return Some([
                mt.transform_point3a(vpos[0]),
                mt.transform_point3a(vpos[1]),
                mt.transform_point3a(vpos[2]),
            ]);
        }
        return None;
    }

    pub fn test(&self, ray: &NavRay) -> Option<(f32, usize, Vec3)>{      
        if let Some(intersection_data) = self.ray_intersection(ray){
            let dist = intersection_data.distance.round() as i32;
            let height: f32 = (ray.origin.y as i32 - dist) as f32;
            return Some((height, intersection_data.triangle_index, intersection_data.normal));
        } else {
            return None;
        }
    }

    pub fn from_mesh(mesh: &Mesh, mesh_transform: &Mat4) -> Self {
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


        let mut edges: HashSet<(usize, usize)> = HashSet::new();

        if let Some(indices) = &mesh.indices() {
            match indices {
                Indices::U16(vertex_indices) => {
                    for index in vertex_indices.chunks(3) {

                        edges.insert((index[0].into(), index[1].into()));
                        edges.insert((index[1].into(), index[2].into()));
                        edges.insert((index[2].into(), index[0].into()));

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
                        tri_norm.push(triangle_normal(&tvn[0], &tvn[1], &tvn[2]));
                    }
                }
                Indices::U32(vertex_indices) => {

                    for index in vertex_indices.chunks(3) {
                        edges.insert((index[0] as usize, index[1] as usize));
                        edges.insert((index[1] as usize, index[2] as usize));
                        edges.insert((index[2] as usize, index[0] as usize));

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
                        tri_norm.push(triangle_normal(&tvn[0], &tvn[1], &tvn[2]));
                    }
                }
            }
        } else {
            panic!("No indices in terrain");
        }


        let vertices: Vec<Vec3A> = vertex_positions.iter().map(|v| Vec3A::new(v[0], v[1], v[2])).collect::<Vec<Vec3A>>();

        let triangle_count = tri_pos.len();
        let mut trmd =  TerrainRayMeshData{
            mesh_transform: mesh_transform.inverse(),
            triangle_vertex_positions: tri_pos,
            triangle_normals: tri_norm,
            triangle_count,
            vertices,
            edges,
            // quads_mapping: HashMap::new()
        };

        // trmd.map_quads();

        // for a_tri in 0..trmd.triangle_count {
        //     let positions = trmd.vertex_positions[a_tri];
        //     for pos in positions.iter(){
        //         if pos.y >= 210.0 {
        //             info!("Above 210: {} normal: {} mapping: {:?} positions: {:?}", a_tri, trmd.normal(a_tri), trmd.quads_mapping.get(&a_tri), positions);
        //             break;
        //         }
        //     }
        // }

        // Looks like its only an issue for edges of the mesh?
        // for a_tri in 0..trmd.triangle_count {
        //     if !trmd.quads_mapping.contains_key(&a_tri) {
        //         // info!("Triangle not in mapping: {}, norma: {:?}, pos: {:?}, is_right: {}", a_tri, trmd.normal(a_tri), trmd.positions(a_tri), trmd.is_right(a_tri));
        //         // hmmm
        //         trmd.quads_mapping.insert(a_tri, a_tri);

        //     }
        // }

        // info!("[NAVMESH][TERRAIN] Count of Mapped triangles: {}, groups: {}, mesh triangle count: {}", 
        //     trmd.quads_mapping.len(),
        //     trmd.quads_mapping.len()/2,
        //     triangle_count
        // );

        return trmd;

    }

    // pub fn vertices_to_navpolygons(&self, start_index: usize) -> Vec<NavPolygon> {
    //     let mut polys: Vec<NavPolygon> = Vec::with_capacity(self.vertices.len());
    //     let mut index = start_index;
    //     let invert = Vec3A::new(-1.0, 1.0, -1.0);
    //     for t in 0..self.triangle_count{
    //         index += 1;
    //         let tri_vertices = self.triangle_positions(t);
    //         let tri_normal = self.triangle_normal(t);

    //         for vertex in tri_vertices.iter(){
    //             let t_vertex = vec![self.mesh_transform.transform_point3a(*vertex)*invert];
    //             // info!("{:?}", t_vertex);

    //             let np = NavPolygon{
    //                 group_id: t,
    //                 loc: *vertex,
    //                 index,
    //                 vertices: t_vertex,
    //                 normal: tri_normal,
    //                 typ: NavType::Terrain,
    //                 neighbours: Neighbours::default()
    //             };
    //             polys.push(np);
    //         }
    //     }
    //     return polys;
    // }


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

                if self.triangle_normal(a_tri) != self.triangle_normal(b_tri) {
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
        // self.quads_mapping = quads_mapping;
    }

    fn triangle_normal(&self, triangle_id: usize) -> Vec3A {
        return self.triangle_normals[triangle_id]
    }


    fn triangle_positions(&self, triangle_id: usize) -> [Vec3A;3] {
        return self.triangle_vertex_positions[triangle_id]
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




    // Tests against all triangles of the mesh
    fn ray_intersection(
        &self, 
        ray: &NavRay
    ) -> Option<IntersectionData> {

        let mesh_space_ray = NavRay::new(
            self.mesh_transform.transform_point3(ray.origin()),
            self.mesh_transform.transform_vector3(ray.direction()),
        );

        for triangle_index in 0..self.triangle_count{

            let tri_vertices = self.triangle_positions(triangle_index);
            let tri_normal = self.triangle_normal(triangle_index);

            if let Some(ray_hit) = ray_triangle_intersection(&mesh_space_ray, &tri_vertices) {
                let distance = *ray_hit.distance();
                // info!("distance: {}", distance);
                if distance > 0.0 {
                    // let u = ray_hit.uv_coords().0;
                    // let v = ray_hit.uv_coords().1;
                    // let w = 1.0 - u - v;
                    // let normal: Vec3 = (tri_normals * u + tri_normals * v + tri_normals * w).into();
                    let intersection = IntersectionData::new(
                        self.mesh_transform.transform_vector3(tri_normal.into()),
                        self.mesh_transform
                            .transform_vector3(mesh_space_ray.direction() * distance)
                            .length(),
                        triangle_index
                    );
                    return Some(intersection);
                }
            }
        }

        return None;
    }
}


fn triangle_normal(
    n_a: &Vec3A, n_b: &Vec3A, n_c: &Vec3A
) -> Vec3A {
    (n_a + n_b + n_c).normalize()
}