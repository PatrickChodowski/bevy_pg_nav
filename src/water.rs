use bevy::prelude::*;
use bevy::mesh::Mesh;
use bevy::platform::collections::{HashSet, HashMap};
use bevy::mesh::{VertexAttributeValues, Indices};
use bevy::color::palettes::css::WHITE;
use dashmap::DashMap;
use rayon::prelude::*;
use avian3d::prelude::{Collider, RigidBody};
use bevy_rerecast::generator::NavmeshGenerator;

use bevy_rerecast::NavmeshSettings;
use bevy_rerecast::debug::DetailNavmeshGizmo;

use bevy_pg_core::prelude::{AABB, TerrainChunk, WaterChunk};
use crate::plugin::RecastNavmeshHandles;
use crate::prelude::{GenerateNavMesh,PGNavmeshType};
use crate::terrain::TerrainRayMeshData;
use crate::tools::NavRay;

pub struct PGWaterNavPlugin;

impl Plugin for PGWaterNavPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_observer(generate_water_navmesh)
        .add_observer(on_water_navmesh_sources_ready)
        ;
    }
}

fn generate_water_navmesh(
    trigger:        On<GenerateNavMesh>,
    terrains:       Query<(&Transform, &Mesh3d, Option<&Name>, &TerrainChunk)>,
    water_query:    Query<(&Transform, &WaterChunk)>,
    mut commands:   Commands,
    mut meshes:     ResMut<Assets<Mesh>>,
    mut materials:  ResMut<Assets<StandardMaterial>>,
    // navconfig:      Res<NavConfig>
){

    let Ok((terrain_transform, mesh3d, maybe_name, chunk)) = terrains.get(trigger.plane_entity) else {return};

    if maybe_name.is_none(){
        warn!("Plane needs a name before generating water navmesh");
        return;
    }

    info!("Generate Water Navmesh for entity {}", trigger.plane_entity);
    // let raycast_step = navconfig.raycast_step as usize;
    let raycast_step = 1;
    let Some(mesh) = meshes.get(&mesh3d.0) else {return};

    let loc = terrain_transform.translation;
    info!("terrain chunk dims: {} terrain loc: {}", chunk.dims, loc);
    let chunk_half_dims: Vec2 = chunk.dims*0.5;
    let trmd = TerrainRayMeshData::from_mesh(mesh, &terrain_transform.to_matrix());

    let min_x = (loc.x - chunk_half_dims.x) as i32;
    let max_x = (loc.x + chunk_half_dims.x) as i32;
    let min_z = (loc.z - chunk_half_dims.y) as i32;
    let max_z = (loc.z + chunk_half_dims.y) as i32;

    let xs_u: Vec<i32> = (min_x..=max_x).step_by(raycast_step).collect();
    let zs_u: Vec<i32> = (min_z..=max_z).step_by(raycast_step).collect();

    let xs = xs_u.iter().map(|n| *n as f32).collect::<Vec<f32>>();
    let zs = zs_u.iter().map(|n| *n as f32).collect::<Vec<f32>>();

    let mut water_aabbs: Vec<(AABB, f32)> = Vec::new();
    for (water_transform, water_chunk) in water_query.iter(){
        let water_aabb: AABB = AABB::from_loc_dims(water_transform.translation.xz(), water_chunk.dims);
        water_aabbs.push((water_aabb, water_transform.translation.y));
    }


    let mut water_hits = raycasts_rain(&xs, &zs, &trmd, &water_aabbs);
    // Group waters already picks only boundary points
    let groups_map = group_waters(&mut water_hits, xs_u.len() as i32 - 1, zs_u.len() as i32 - 1);

    // Order groups
    let mut hm_groups: HashMap<usize, Vec<WaterVertex>> = HashMap::new();
    for (_tile, vertex) in groups_map.iter(){
        hm_groups.entry(vertex.group).or_insert(Vec::new()).push(*vertex);
    }

    // info!("{:?}", hm_groups);

    let mut triangle_map: HashMap<usize, Vec<[Vec3A; 3]>> = HashMap::new();
    for (group, vertices) in hm_groups.iter_mut(){
        // info!("group {} vertices count: {:?}", group, vertices.len());
        let mut vlocs: Vec<Vec3A> = order_boundary(vertices, xs_u.len() as i32 - 1, zs_u.len() as i32 - 1);

        // info!(" Group: {} Vertices count after order_boundary: {}", group, vlocs.len());
        vlocs = remove_collinear(&mut vlocs);
        // info!(" Group: {} Vertices count after remove_collinear: {}", group, vlocs.len());


        // for v in vlocs.iter(){
        //     commands.spawn(
        //         (
        //             Mesh3d(meshes.add(Sphere::new(0.5))),
        //             NotShadowReceiver,
        //             MeshMaterial3d(materials.add(StandardMaterial {
        //                 base_color: Color::WHITE,
        //                 ..default()
        //             })),
        //             Transform::from_translation(Vec3::new(v.x, v.y, v.z)).with_scale(Vec3::splat(3.0)),
        //         )
        //     );
        // }

        let triangles = triangulate(&mut vlocs);
        // info!("triangles: {:?}", triangles);

        let mesh = mesh_from_triangles(&triangles);
        triangle_map.insert(*group, triangles);
        // info!("mesh: {:?}", mesh);

        let trimesh_data = extract_trimesh(&mesh);
        // info!("{:?}", trimesh_data);
        let collider = Collider::trimesh(trimesh_data.0, trimesh_data.1);

        // info!("collider: {:?}", collider);

        commands.spawn(
            (
                Mesh3d(meshes.add(mesh)),
                MeshMaterial3d(materials.add(StandardMaterial::from(Color::from(WHITE)))),
                WaterNavmeshSource,
                Visibility::Hidden,
                collider,
                RigidBody::Static
            )
        );
    }

    commands.trigger(WaterNavmeshesReady);
    
}



fn on_water_navmesh_sources_ready(
    _trigger:            On<WaterNavmeshesReady>,
    mut generator:       NavmeshGenerator,
    mut commands:        Commands,
    water_sources:       Query<Entity, With<WaterNavmeshSource>>,
    mut navmesh_handles: ResMut<RecastNavmeshHandles>
){
    let mut hs = HashSet::new();
    for water_mesh_entity in water_sources.iter(){
        hs.insert(water_mesh_entity);
    }

    let settings = NavmeshSettings {
        filter: Some(hs),
        walkable_climb: 10.0,
        walkable_slope_angle: 0.1_f32.to_radians(),
        agent_radius: 1.0,
        agent_height: 2.0,
        max_vertices_per_polygon: 20,
        max_simplification_error: 1.3,
        min_region_size: 1,
        up: Vec3::Y,
        ..default()
    };
    let navmesh = generator.generate(settings);
    commands.spawn(DetailNavmeshGizmo::new(&navmesh));
    navmesh_handles.data.insert(PGNavmeshType::Water, Some(navmesh));
}






#[derive(Component)]
pub(crate) struct WaterNavmeshSource;

#[derive(Event)]
pub(crate) struct WaterNavmeshesReady;

#[derive(Clone, Copy, Debug)]
pub(crate) struct WaterVertex {
   pub(crate) x_u: usize,
   pub(crate) z_u: usize,
   pub(crate) loc: Vec3A,
   pub(crate) group: usize
}

impl PartialEq for WaterVertex {
    fn eq(&self, other: &Self) -> bool {
        self.x_u == other.x_u && self.z_u == other.z_u
    }
}

impl Eq for WaterVertex {}

impl PartialOrd for WaterVertex {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for WaterVertex {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Sort by x_u first, then by z_u
        match self.x_u.cmp(&other.x_u) {
            std::cmp::Ordering::Equal => self.z_u.cmp(&other.z_u),
            other => other,
        }
    }
}

pub(crate) fn raycasts_rain(
    xs:             &Vec<f32>,
    zs:             &Vec<f32>,
    // ray_target_meshes:     &Vec<RayTargetMesh>,
    trmd:           &TerrainRayMeshData,
    waters:         &Vec<(AABB, f32)>
) ->  Vec<WaterVertex> {

    let raycast_map: DashMap<(usize, usize), Vec3A> = DashMap::new();
    xs.par_iter()
      .enumerate()
      .flat_map(|(x_index, &x)| 
            zs.par_iter()
              .enumerate()
              .map(move |(z_index, &z)| (x_index, x, z_index, z)))
              .for_each(|(x_index, x, z_index, z)|{

        let ray: NavRay = NavRay::down(x, z);

        // Check against the terrain
        if let Some((terrain_height, _group_id, _normal)) = trmd.test(&ray){

            let mut maybe_water_height: Option<f32> = None;
            for (water_aabb, water_height) in waters.iter(){
                if water_aabb.has_point(Vec2::new(x,z)){
                    maybe_water_height = Some(*water_height);
                    break;
                }
            }

            if let Some(water_height) = maybe_water_height {
                if terrain_height <= water_height {
                    let tile = (x_index, z_index);
                    let loc = Vec3A::new(x, water_height, z);
                    raycast_map.insert(tile, loc);
                }
            }
        }
    });

    let mut v_waters = raycast_map.clone().into_iter().map(|(tile, loc)| WaterVertex{x_u: tile.0, z_u: tile.1, loc: loc, group: 0}).collect::<Vec<WaterVertex>>();
    v_waters.sort_by(|a, b| {
        a.x_u
            .partial_cmp(&b.x_u)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(
                a.z_u
                    .partial_cmp(&b.z_u)
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
    });
    return v_waters;
}


pub(crate) fn group_waters(
    v_waters: &mut Vec<WaterVertex>, 
    maxx: i32, 
    maxz: i32
) -> HashMap<(usize, usize), WaterVertex> {

    let adjs: [(isize, isize); 8] = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1,-1), (1,1), (-1, 1), (1, -1)];
    let max_x = maxx as isize;
    let max_z = maxz as isize;

    let mut group_map: HashMap<(usize, usize), WaterVertex> = v_waters.iter().map(|v| ((v.x_u, v.z_u), *v)).collect::<HashMap<(usize, usize), WaterVertex>>();
    let mut global_group: usize = 1;

    for vertex in v_waters.iter(){
        if group_map.get(&(vertex.x_u, vertex.z_u)).map_or(false, |v| v.group != 0) {
            continue;
        }

        let mut local_group: Option<usize> = None;
        for adj in adjs.iter() {
            let key:(isize, isize) = (adj.0 + vertex.x_u as isize, adj.1 + vertex.z_u as isize);
            if (key.0 < 0) | (key.1 < 0) | (key.0 > max_x) | (key.1 > max_z) {
                continue;
            }
            let u_key = (key.0 as usize, key.1 as usize);

            if let Some(vertex) = group_map.get(&u_key){
                if vertex.group != 0 {
                    local_group = Some(vertex.group);
                    break;
                }
            }
        }

        if local_group.is_none(){
            local_group = Some(global_group);
            global_group += 1;
        }

        if let Some(local_group) = local_group {

            for adj in adjs.iter() {
                let key:(isize, isize) = (adj.0 + vertex.x_u as isize, adj.1 + vertex.z_u as isize);
                if (key.0 < 0) | (key.1 < 0) | (key.0 > max_x) | (key.1 > max_z) {
                    continue;
                }

                let u_key = (key.0 as usize, key.1 as usize);
                if let Some(vertex) = group_map.get_mut(&u_key){
                    vertex.group = local_group;
                }
            }

            if let Some(vertex) = group_map.get_mut(&(vertex.x_u, vertex.z_u)){
                vertex.group = local_group;
            }
        }
    }

    // 2nd pass
    loop {
        let mut group_pairs: HashSet<(usize, usize)> = HashSet::new();
        let mut used: HashSet<usize> = HashSet::new();

        for (_tile, vertex) in group_map.iter(){
            if used.contains(&vertex.group){
                continue;
            }

            for adj in adjs.iter() {
                let key:(isize, isize) = (adj.0 + vertex.x_u as isize, adj.1 + vertex.z_u as isize);
                if (key.0 < 0) | (key.1 < 0) | (key.0 > max_x) | (key.1 > max_z) {
                    continue;
                }
                let u_key = (key.0 as usize, key.1 as usize);
                if let Some(vertex2) = group_map.get(&u_key){
                    if used.contains(&vertex2.group){
                        continue;
                    }

                    if vertex2.group != vertex.group {
                        let ming = vertex.group.min(vertex2.group);
                        let maxg = vertex.group.max(vertex2.group);
                        group_pairs.insert((ming, maxg));
                        used.insert(ming);
                        used.insert(maxg);
                    }
                }
            }
        }
        if group_pairs.len() == 0 {
            break;
        }

        for pair in group_pairs.iter(){
            for (_tile, vertex) in group_map.iter_mut(){
                if vertex.group == pair.1 {
                    vertex.group = pair.0;
                }
            }
        }
    }

    let unique_groups = group_map.iter().map(|v| v.1.group).collect::<HashSet<usize>>();

    // 3rd pass - keep only boundary points
    let mut to_rm: Vec<(usize, usize)> = Vec::new();
    for (tile, vertex) in group_map.iter(){
        let mut adj_count: usize = 0;

        for adj in adjs.iter() {
            let key:(isize, isize) = (adj.0 + vertex.x_u as isize, adj.1 + vertex.z_u as isize);
            if (key.0 < 0) | (key.1 < 0) | (key.0 > max_x) | (key.1 > max_z) {
                continue;
            }
            let u_key = (key.0 as usize, key.1 as usize);
            if let Some(_vertex2) = group_map.get(&u_key){
                adj_count += 1;
            }
        }
        if adj_count == 8 {
            to_rm.push(*tile);
        }
    }

    for tile in to_rm.iter(){
        group_map.remove(tile);
    }
    return group_map;
}


pub(crate) fn mesh_from_triangles(
    triangles: &Vec<[Vec3A; 3]>
) -> Mesh {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();
    for (tri_idx, triangle) in triangles.iter().enumerate() {
        let [v0, v1, v2] = triangle;
        positions.push([v0.x, v0.y, v0.z]);
        positions.push([v1.x, v1.y, v1.z]);
        positions.push([v2.x, v2.y, v2.z]);

        normals.push([0.0, 1.0, 0.0]);
        normals.push([0.0, 1.0, 0.0]);
        normals.push([0.0, 1.0, 0.0]);
        let base_idx = (tri_idx * 3) as u16;
        indices.push(base_idx);
        indices.push(base_idx + 2);
        indices.push(base_idx + 1);
    }

    let water_simple_mesh = Mesh::new(
        bevy::render::render_resource::PrimitiveTopology::TriangleList,
        bevy::asset::RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_indices(bevy::mesh::Indices::U16(indices));

    return water_simple_mesh;
}


pub(crate) fn triangulate(
    vertices: &mut Vec<Vec3A>
) -> Vec<[Vec3A; 3]> {
    let flat_vertices: Vec<f64> = vertices
        .iter()
        .flat_map(|v| [v.x as f64, v.z as f64])
        .collect();
    let triangle_indices = earcutr::earcut(&flat_vertices, &[], 2).unwrap();
    let triangles: Vec<[Vec3A; 3]> = triangle_indices.chunks(3).map(|chunk| {
        [
            vertices[chunk[0]],
            vertices[chunk[1]],
            vertices[chunk[2]],
        ]
    })
    .collect();
    return triangles;
}


pub(crate) fn remove_collinear(
    vlocs: &mut Vec<Vec3A>
) -> Vec<Vec3A> {
    let threshold: f32 = 0.01;
    let mut result = vec![vlocs[0]];

    for i in 1..vlocs.len() {

        if i == vlocs.len() - 1 {
            result.push(vlocs[i]);
            break;
        }

        let p1 = result[result.len() - 1];
        let p2 = vlocs[i];
        let p3 = vlocs[i + 1];
        let v1x = p2.x - p1.x;
        let v1y = p2.z - p1.z;
        let v2x = p3.x - p1.x;
        let v2y = p3.z - p1.z;
        let cross = (v1x * v2y - v1y * v2x).abs();
        let len_a = (v1x * v1x + v1y * v1y).sqrt();
        let len_b = (v2x * v2x + v2y * v2y).sqrt();
        if len_a < 1e-6 || len_b < 1e-6 {
            continue; // degenerate segment, skip
        }

        let normalized_cross = cross / (len_a * len_b);
        if normalized_cross > threshold {
            result.push(p2);
        }

        // info!("cross: {}", cross);
        // if cross > threshold {
        //     result.push(p2);
        // }
    }
    result.push(vlocs[vlocs.len() - 1]);

    return result;
}


pub(crate) fn order_boundary(
    vertices: &mut Vec<WaterVertex>, 
    maxx: i32, 
    maxz: i32
) -> Vec<Vec3A>{
    let max_x: isize = maxx as isize;
    let max_z: isize = maxz as isize;

    let mut new_order: Vec<Vec3A> = Vec::with_capacity(vertices.len());
    let mut used: HashSet<(usize, usize)> = HashSet::new();
    vertices.sort();

    let mut current = vertices[0];
    new_order.push(current.loc);
    used.insert((current.x_u, current.z_u));
    
    let madjs: [(isize, isize); 4] = [(1, 0), (0, 1), (0, -1), (-1, 0)];
    let dadjs: [(isize, isize); 4] = [(-1, -1), (1, 1), (1, -1), (-1, 1)];

    loop {
        let mut next: Option<WaterVertex> = None;
        for adj in madjs.iter(){
            let adj_x = current.x_u as isize + adj.0;
            let adj_z = current.z_u as isize + adj.1;

            if (adj_x >= 0) & (adj_x <= max_x) & (adj_z >= 0) & (adj_z <= max_z){
                for v in vertices.iter(){
                    if used.contains(&(v.x_u, v.z_u)){
                        continue;
                    }

                    if (adj_x as usize == v.x_u) & (adj_z as usize == v.z_u){
                        next = Some(*v);
                        break; 
                    }
                }
            }
        }

        if let Some(next) = next {
            used.insert((current.x_u, current.z_u));
            current = next;
            new_order.push(current.loc);
        } else {
            for adj in dadjs.iter(){
                let adj_x = current.x_u as isize + adj.0;
                let adj_z = current.z_u as isize + adj.1;

                if (adj_x > 0) & (adj_x <= max_x) & (adj_z > 0) & (adj_z <= max_z){
                    for v in vertices.iter(){
                        if used.contains(&(v.x_u, v.z_u)){
                            continue;
                        }

                        if (adj_x as usize == v.x_u) & (adj_z as usize == v.z_u){
                            next = Some(*v);
                            break;
                        }
                    }
                }
            }

            if let Some(next) = next {
                used.insert((current.x_u, current.z_u));
                current = next;
                new_order.push(current.loc);
            } else {
                break;
            }

        }

    }

    return new_order;
}



#[allow(dead_code)]
fn extract_trimesh(mesh: &Mesh) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    // Extract vertices
    let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute(Mesh::ATTRIBUTE_POSITION)
    else {
        return (vec![], vec![]);
    };

    let vertices: Vec<Vec3> = positions
        .iter()
        .map(|p| Vec3::new(p[0], p[1], p[2]))
        .collect();

    // Extract indices
    let indices: Vec<[u32; 3]> = match mesh.indices() {
        Some(Indices::U32(idx)) => idx.chunks_exact(3)
            .map(|t| [t[0], t[1], t[2]])
            .collect(),
        Some(Indices::U16(idx)) => idx.chunks_exact(3)
            .map(|t| [t[0] as u32, t[1] as u32, t[2] as u32])
            .collect(),
        None => return (vec![], vec![]),
    };

    (vertices, indices)
}
