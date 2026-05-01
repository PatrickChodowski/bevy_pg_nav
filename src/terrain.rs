
use bevy::prelude::*;
use bevy::mesh::{Indices, Mesh, VertexAttributeValues};
use bevy::render::render_resource::PrimitiveTopology;
use bevy::platform::collections::HashSet;
use avian3d::prelude::{Collider, RigidBody};
use bevy_rerecast::debug::DetailNavmeshGizmo;
use bevy_rerecast::generator::NavmeshGenerator;
use bevy_rerecast::NavmeshSettings;

use bevy_pg_core::prelude::TerrainChunk;
use crate::pgnavmesh::PGNavmeshType;
use crate::plugin::NavmeshGenerationData;
use crate::tools::{NavRay, IntersectionData, ray_triangle_intersection};


pub struct PGTerrainNavPlugin;

impl Plugin for PGTerrainNavPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_observer(prepare_terrain_colliders)
        .add_observer(generate_terrain_navmesh_on_colliders_ready)
        .add_systems(Update, wait_for_colliders.run_if(resource_exists::<WaitColliders>))
        ;
    }
}


#[derive(Event)]
pub(crate) struct GenerateTerrainNavmesh;

fn prepare_terrain_colliders(
    _trigger:       On<GenerateTerrainNavmesh>,
    terrains:       Query<(&Transform, &TerrainChunk, &Mesh3d)>,
    meshes:         Res<Assets<Mesh>>,
    mut commands:   Commands,
    navgendata:     Res<NavmeshGenerationData>
){
    let Ok((_terrain_transform, terrain, terrain_mesh)) = terrains.get(navgendata.plane_entity) else {return};
    let Some(mesh) = meshes.get(&terrain_mesh.0) else {return};
    info!("[NAV] Generate terrain navmesh for entity: {}", navgendata.plane_entity);
    let heightfield = extract_heightfield(&mesh);
    let collider = Collider::heightfield(heightfield, Vec3::new(terrain.dims.x, 1.0, terrain.dims.y));
    commands.entity(navgendata.plane_entity).insert((collider, RigidBody::Static));
    commands.insert_resource(WaitColliders::new());
}

#[derive(Resource)]
struct WaitColliders {
    timer: Timer
}
impl WaitColliders {
    fn new() -> Self {
        return WaitColliders{timer: Timer::from_seconds(0.5, TimerMode::Once)};
    }
}

#[derive(Event)]
struct CollidersReady;

fn wait_for_colliders(
    time: Res<Time>,
    mut commands: Commands,
    mut wait: ResMut<WaitColliders>
){
    wait.timer.tick(time.delta());

    if wait.timer.is_finished(){
        commands.trigger(CollidersReady);
        commands.remove_resource::<WaitColliders>();
    }
}



fn generate_terrain_navmesh_on_colliders_ready(
    _trigger:        On<CollidersReady>,
    mut generator:  NavmeshGenerator,
    mut commands:   Commands,
    mut navgendata: ResMut<NavmeshGenerationData>,

){

    info!("[NAV] Generate terrain navmesh for entity: {}", navgendata.plane_entity);
    let mut hs: HashSet<Entity> = HashSet::new();
    hs.insert(navgendata.plane_entity);


    // for (entity, transform, nstatic, name) in statics.iter(){

    //     if !terrain_aabb.has_point(transform.translation.xz()){
    //         continue;
    //     }

    //     if (nstatic.typ == NavStaticType::Blocker) & ((name.contains("Bld")) | name.contains("Prop")) {
    //         hs.insert(entity);
    //     }
    //     if let NavStaticType::Navigable(a) = nstatic.typ {
    //         hs.insert(entity);
    //     }

    // }

    // let mut count_blockers: usize = 0;
    // for (entity, nstatic, name) in statics.iter(){
    //     if (nstatic.typ == NavStaticType::Blocker) & (name.contains("Tree")) {
    //         hs.insert(entity);
    //         count_blockers += 1;
    //     }
    //     if count_blockers >= 1600 {
    //         info!("Blockers limit reached");
    //         break;
    //     }
    // }

    let settings = NavmeshSettings {
        filter: Some(hs),
        walkable_climb: 1.0,
        walkable_slope_angle: 55.0_f32.to_radians(),
        agent_radius: 0.5,
        agent_height: 0.0,
        // min_region_size: 10,
        max_vertices_per_polygon: 20,
        max_simplification_error: 1.3,
        // cell_size_fraction: 2.0,
        // cell_height_fraction: 4.0,
        up: Vec3::Y,
        // aabb: Some(Aabb3d::new(terrain_transform.translation, Vec3::new(terrain.dims.x * 0.5, 1.0, terrain.dims.y * 0.5))),
        // merge_region_size: 100,

        // min_region_size: // def: 8,
        // merge_region_size: // def: 20,
        // detail_sample_dist: 12.0, //def: 6.0
        // detail_sample_max_error: 10.0, //def 1.0
        // contour_flags: BuildContoursFlags::TESSELLATE_AREA_EDGES,
        ..default()
    };
    let navmesh = generator.generate(settings);
    commands.spawn(DetailNavmeshGizmo::new(&navmesh));
    navgendata.add_handle(PGNavmeshType::Terrain, navmesh);
}







// Generate Optimized data structure for raycast testing
#[derive(Debug, Clone, Component)]
pub(crate) struct TerrainRayMeshData {
    pub mesh_transform:            Mat4,
    triangle_vertex_positions: Vec<[Vec3A; 3]>,
    triangle_normals:          Vec<Vec3A>,
    triangle_count:            usize,
    pub (crate) vertices:              Vec<Vec3A>,
    pub (crate) edges:                 HashSet<(usize, usize)>,
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

    pub (crate) fn test(&self, ray: &NavRay) -> Option<(f32, usize, Vec3)>{      
        if let Some(intersection_data) = self.ray_intersection(&ray.origin(), &ray.direction()){
            let dist = intersection_data.distance.round() as i32;
            let height: f32 = (ray.origin.y as i32 - dist) as f32;
            return Some((height, intersection_data.triangle_index, intersection_data.normal));
        } else {
            return None;
        }
    }

    pub (crate) fn from_mesh(mesh: &Mesh, mesh_transform: &Mat4) -> Self {
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
        let trmd =  TerrainRayMeshData{
            mesh_transform: mesh_transform.inverse(),
            triangle_vertex_positions: tri_pos,
            triangle_normals: tri_norm,
            triangle_count,
            vertices,
            edges,
        };

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

    fn triangle_normal(&self, triangle_id: usize) -> Vec3A {
        return self.triangle_normals[triangle_id]
    }


    fn triangle_positions(&self, triangle_id: usize) -> [Vec3A;3] {
        return self.triangle_vertex_positions[triangle_id]
    }

    // fn get_longest_side_points(&self, triangle_id: usize) -> [Vec3A; 2] {

    //     let tri_pos = self.triangle(triangle_id).unwrap();
        
    //     let v0 = tri_pos[0];
    //     let v1 = tri_pos[1];
    //     let v2 = tri_pos[2];

    //     let edges: [(Vec3A, Vec3A, f32); 3] = [
    //         (v0, v1, (v1[0] - v0[0]).powi(2) + (v1[2] - v0[2]).powi(2)),
    //         (v1, v2, (v2[0] - v1[0]).powi(2) + (v2[2] - v1[2]).powi(2)),
    //         (v2, v0, (v0[0] - v2[0]).powi(2) + (v0[2] - v2[2]).powi(2)),
    //     ];

    //     let longest_edge: [Vec3A; 2] = edges.into_iter()
    //         .max_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
    //         .map(|(v1, v2, _)| [v1, v2])
    //         .unwrap();

    //     return longest_edge;
    // }

    // fn is_right(&self, triangle_id: usize) -> bool{

    //     let tri_pos = self.triangle(triangle_id).unwrap();
    //     let v0 = tri_pos[0];
    //     let v1 = tri_pos[1];
    //     let v2 = tri_pos[2];

    //     let mut sides: [f32; 3] = [
    //         (v1[0] - v0[0]).powi(2) + (v1[2] - v0[2]).powi(2),
    //         (v2[0] - v1[0]).powi(2) + (v2[2] - v1[2]).powi(2),
    //         (v0[0] - v2[0]).powi(2) + (v0[2] - v2[2]).powi(2),
    //     ];

    //     sides.sort_by(|a, b| a.partial_cmp(b).unwrap());

    //     let s0 = sides[0];
    //     let s1 = sides[1];
    //     let s2 = sides[2];

    //     return (s0 + s1) - s2 <= EPSILON;
    // }

    // Tests against all triangles of the mesh
    pub (crate) fn ray_intersection(
        &self, 
        ray_origin: &Vec3,
        ray_direction: &Vec3
    ) -> Option<IntersectionData> {

        let mesh_space_ray = NavRay::new(
            self.mesh_transform.transform_point3(*ray_origin),
            self.mesh_transform.transform_vector3(*ray_direction),
        );

        for triangle_index in 0..self.triangle_count{

            let tri_vertices = self.triangle_positions(triangle_index);
            let tri_normal = self.triangle_normal(triangle_index);

            if let Some(ray_hit) = ray_triangle_intersection(&mesh_space_ray, &tri_vertices) {

                if ray_hit.distance > 0.0 {
                    let intersection = IntersectionData::new(
                        self.mesh_transform.transform_point3(ray_hit.position),
                        self.mesh_transform.transform_vector3(tri_normal.into()),
                        self.mesh_transform
                            .transform_vector3(mesh_space_ray.direction() * ray_hit.distance)
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


#[allow(dead_code)]
fn extract_heightfield(mesh: &Mesh) -> Vec<Vec<f32>> {
    // Get vertex positions from the mesh
    let Some(VertexAttributeValues::Float32x3(positions)) = 
        mesh.attribute(Mesh::ATTRIBUTE_POSITION) 
    else {
        return vec![];
    };

    // Collect unique X and Z values to determine grid dimensions
    let mut x_coords: Vec<f32> = positions.iter().map(|p| p[0]).collect();
    let mut z_coords: Vec<f32> = positions.iter().map(|p| p[2]).collect();

    x_coords.sort_by(|a, b| a.partial_cmp(b).unwrap());
    x_coords.dedup_by(|a, b| (*a - *b).abs() < 1e-4);

    z_coords.sort_by(|a, b| a.partial_cmp(b).unwrap());
    z_coords.dedup_by(|a, b| (*a - *b).abs() < 1e-4);

    let rows = x_coords.len();
    let cols = z_coords.len();

    // Build a lookup: (x_index, z_index) -> y
    let mut heights = vec![vec![0.0f32; cols]; rows];

    for pos in positions.iter() {
        let x = pos[0];
        let y = pos[1];
        let z = pos[2];

        let xi = x_coords.partition_point(|&v| v < x - 1e-4);
        let zi = z_coords.partition_point(|&v| v < z - 1e-4);

        if xi < rows && zi < cols {
            heights[xi][zi] = y;
        }
    }

    heights
}
