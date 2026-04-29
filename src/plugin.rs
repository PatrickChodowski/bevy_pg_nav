use avian_rerecast::ColliderToTriMesh;
use avian3d::prelude::{Collider, RigidBody};
use bevy::light::NotShadowReceiver;
use bevy::mesh::{VertexAttributeValues, Indices};
use bevy::color::palettes::css::WHITE;
use bevy::math::bounding::Aabb3d;
use serde::{Serialize, Deserialize};
use bevy::tasks::IoTaskPool;
use std::fs::File;
use std::io::{BufWriter, Write};
use bevy_common_assets::json::JsonAssetPlugin;
use bevy::prelude::*;
use bevy::platform::collections::{HashSet, HashMap};
use bevy_rerecast::{debug::DetailNavmeshGizmo, prelude::*};
// use bevy_rerecast::Mesh3dBackendPlugin;
use avian_rerecast::AvianBackendPlugin;
use bevy_pg_core::prelude::{AABB, GameState, TerrainChunk, WaterChunk};
// use avian3d::collision::collider::ColliderConstructor;

use crate::debug::PGNavDebugPlugin;
use crate::water::{
    raycasts_rain, remove_collinear, order_boundary, 
    group_waters, triangulate, mesh_from_triangles,
    WaterVertex, WaterNavmeshSource, WaterNavmeshesReady
};
use crate::terrain::TerrainRayMeshData;
use crate::recast_convert::convert_rerecast;
use crate::pgnavmesh::{PGNavmesh, PGNavmeshType};

pub struct PGNavPlugin;

impl Plugin for PGNavPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_plugins((
            NavmeshPlugins::default(),
            // Mesh3dBackendPlugin::default(),
            AvianBackendPlugin::default()
        ))
        // .insert_resource(NavConfig::default())
        .init_asset::<PGNavmesh>()
        .add_plugins(JsonAssetPlugin::<PGNavmesh>::new(&["nav.json"]))
        .add_plugins(PGNavDebugPlugin)
        .insert_resource(RecastNavmeshHandles::default())

        .add_observer(prepare_colliders)
        .add_observer(generate_terrain_navmesh)
        .add_observer(generate_water_navmesh)
        .add_observer(on_water_navmesh_sources_ready)
        .add_observer(on_ready_navmesh)
        .add_observer(on_spawn_navmesh)
        .add_systems(OnExit(GameState::Play), clear)

        .add_systems(Update, wait_for_colliders.run_if(resource_exists::<WaitColliders>))
        ;
    }
}

#[derive(Resource)]
struct WaitColliders {
    entity: Entity,
    timer: Timer
}

#[derive(Event)]
struct CollidersReady {
    entity: Entity
}

fn wait_for_colliders(
    time: Res<Time>,
    mut commands: Commands,
    mut wait: ResMut<WaitColliders>
){
    wait.timer.tick(time.delta());

    if wait.timer.is_finished(){
        commands.trigger(CollidersReady{entity: wait.entity});
        commands.remove_resource::<WaitColliders>();
    }

}

fn clear(
    mut commands: Commands,
    navs:         Query<Entity, With<PGNavmesh>>
){
    for entity in navs.iter(){
        commands.entity(entity).despawn();
    }

}


pub(crate) const ORIGIN_HEIGHT: f32 = 1000.0;

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


#[derive(
    Resource, Debug, bevy::asset::Asset, bevy::reflect::TypePath, Clone, Deserialize, Serialize
)]
pub struct NavConfig {
    pub raycast_step: u32,
    pub offset_y: f32, // Offset on Display only
    pub iter_count_limit: usize,
    pub serialize: bool,
    pub debug: bool
}


#[derive(Component)]
pub struct NavmeshWater;

#[derive(Component)]
pub struct NavmeshTerrain;

#[derive(Event)]
pub struct GenerateNavMesh {
    pub plane_entity: Entity
}

#[derive(Resource, Debug)]
struct RecastNavmeshHandles {
    data: HashMap<PGNavmeshType, Option<Handle<Navmesh>>>
}
impl Default for RecastNavmeshHandles {
    fn default() -> Self {
        RecastNavmeshHandles {
            data: HashMap::from(
                [
                    (PGNavmeshType::Terrain, None),
                    (PGNavmeshType::Water, None)
                ]
            )
        }
    }
}


fn on_water_navmesh_sources_ready(
    _trigger:            On<WaterNavmeshesReady>,
    mut generator:       NavmeshGenerator,
    mut commands:        Commands,
    water_sources:       Query<Entity, With<WaterNavmeshSource>>,
    // statics:          Query<(Entity, &NavStatic, &Name)>,
    mut navmesh_handles: ResMut<RecastNavmeshHandles>
){
    let mut hs = HashSet::new();
    for water_mesh_entity in water_sources.iter(){
        hs.insert(water_mesh_entity);
    }

    // for (entity, nstatic, name) in statics.iter(){
    //     if (nstatic.typ == NavStaticType::Blocker) & ((name.contains("Bld")) | name.contains("Prop")) {
    //         hs.insert(entity);
    //     }
    // }

    // for (entity, nstatic, _name) in statics.iter(){
    //     if let NavStaticType::Navigable(a) = nstatic.typ {
    //         hs.insert(entity);
    //     }
    // }

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


fn generate_water_navmesh(
    trigger:        On<GenerateNavMesh>,
    terrains:       Query<(&Transform, &Mesh3d, Option<&Name>, &TerrainChunk)>,
    water_query:    Query<(&Transform, &WaterChunk)>,
    mut commands:   Commands,
    mut meshes:     ResMut<Assets<Mesh>>,
    mut materials:  ResMut<Assets<StandardMaterial>>,
    navconfig:      Res<NavConfig>
){

    let Ok((terrain_transform, mesh3d, maybe_name, chunk)) = terrains.get(trigger.plane_entity) else {return};

    if maybe_name.is_none(){
        warn!("Plane needs a name before generating water navmesh");
        return;
    }

    info!("Generate Water Navmesh for entity {} raycast step: {}", trigger.plane_entity, navconfig.raycast_step);
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
        info!("group {} vertices count: {:?}", group, vertices.len());
        let mut vlocs: Vec<Vec3A> = order_boundary(vertices, xs_u.len() as i32 - 1, zs_u.len() as i32 - 1);

        info!(" Group: {} Vertices count after order_boundary: {}", group, vlocs.len());
        vlocs = remove_collinear(&mut vlocs);
        info!(" Group: {} Vertices count after remove_collinear: {}", group, vlocs.len());


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
        info!("triangles: {:?}", triangles);

        let mesh = mesh_from_triangles(&triangles);
        triangle_map.insert(*group, triangles);
        info!("mesh: {:?}", mesh);

        let trimesh_data = extract_trimesh(&mesh);
        info!("{:?}", trimesh_data);
        let collider = Collider::trimesh(trimesh_data.0, trimesh_data.1);

        info!("collider: {:?}", collider);

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

fn prepare_colliders(
    trigger:             On<GenerateNavMesh>,
    terrains:            Query<(&Transform, &TerrainChunk, &Mesh3d, Option<&Name>)>,
    meshes:              Res<Assets<Mesh>>,
    mut commands:        Commands,
){
    info!("[NAV] Generate terrain navmesh for entity: {}", trigger.plane_entity);

    let Ok((_terrain_transform, terrain, terrain_mesh, maybe_name)) = terrains.get(trigger.plane_entity) else {return};

    if maybe_name.is_none(){
        warn!("Plane needs a name before generating navmesh");
        return;
    }

    let Some(mesh) = meshes.get(&terrain_mesh.0) else {return};
    let heightfield = extract_heightfield(&mesh);
    // let terrain_aabb = AABB::from_loc_dims(terrain_transform.translation.xz(), terrain.dims);
    let collider = Collider::heightfield(heightfield, Vec3::new(terrain.dims.x, 1.0, terrain.dims.y));

    // let (vertices, indices) = extract_trimesh(&mesh);
    // let collider = Collider::trimesh(vertices, indices);

    // info!("Collider: {:?}", collider.shape_scaled());
    commands.entity(trigger.plane_entity).insert((collider, RigidBody::Static));

    commands.insert_resource(WaitColliders{entity: trigger.plane_entity, timer: Timer::from_seconds(0.5, TimerMode::Once)});
}




fn generate_terrain_navmesh(
    // trigger:             On<GenerateNavMesh>,
    // trigger: On<Add, Collider>,
    trigger: On<CollidersReady>,
    mut generator:       NavmeshGenerator,
    mut commands:        Commands,
    statics:             Query<(Entity, &Transform, &NavStatic, &Name)>,
    terrains:            Query<(&Transform, &TerrainChunk, &Mesh3d)>,
    meshes:              Res<Assets<Mesh>>,
    mut navmesh_handles: ResMut<RecastNavmeshHandles>
){

    // info!("[NAV] Generate terrain navmesh for entity: {}", trigger.plane_entity);
pub(crate) fn group_waters(
    v_waters: &mut Vec<WaterVertex>, 
    maxx: i32, 
    maxz: i32
) -> HashMap<(usize, usize), WaterVertex> {

    let adjs: [(isize, isize); 8] = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1,-1), (1,1), (-1, 1), (1, -1)];
    let max_x = maxx as isize;
    let max_z = maxz as isize;

    let mut group_map: HashMap<(usize, usize), WaterVertex> = v_waters.iter().map(|v| ((v.x_u, v.z_u), *v)).collect::<HashMap<(usize, usize), WaterVertex>>();
    // let mut group: usize = 0;
    let mut global_group: usize = 1;

    for vertex in v_waters.iter(){
        // info!("vertex: ({}, {})", vertex.x_u, vertex.z_u)

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

    // info!("global_group: {}", global_group);


    // 2nd pass
    loop {
        // let mut group_pairs: HashSet<(usize, usize)> = HashSet::new();
        let mut group_pairs: HashMap<usize, usize> = HashMap::new(); // max_group -> min_group

        // let mut used: HashSet<usize> = HashSet::new();

        for (_tile, vertex) in group_map.iter(){
            // if used.contains(&vertex.group){
            //     continue;
            // }

            for adj in adjs.iter() {
                let key:(isize, isize) = (adj.0 + vertex.x_u as isize, adj.1 + vertex.z_u as isize);
                if (key.0 < 0) | (key.1 < 0) | (key.0 > max_x) | (key.1 > max_z) {
                    continue;
                }
                let u_key = (key.0 as usize, key.1 as usize);
                if let Some(vertex2) = group_map.get(&u_key){
                    // if used.contains(&vertex2.group){
                    //     continue;
                    // }

                    if vertex2.group != vertex.group {
                        let ming = vertex.group.min(vertex2.group);
                        let maxg = vertex.group.max(vertex2.group);
                        // group_pairs.insert((ming, maxg));
                        group_pairs.entry(maxg).and_modify(|v| *v = (*v).min(ming)).or_insert(ming);
                        // used.insert(ming);
                        // used.insert(maxg);
                    }
                }
            }
        }
        info!("2nd pass: group pairs len: {}", group_pairs.len());
        // if group_pairs.len() == 0 {
        //     break;
        // }

        if group_pairs.is_empty() {
            break;
        }
        // for pair in group_pairs.iter(){
        //     for (_tile, vertex) in group_map.iter_mut(){
        //         if vertex.group == pair.1 {
        //             vertex.group = pair.0;
        //         }
        //     }
        // }
        for (_tile, vertex) in group_map.iter_mut(){
            if let Some(&target) = group_pairs.get(&vertex.group) {
                vertex.group = target;
            }
        }
    }

    let unique_groups = group_map.iter().map(|v| v.1.group).collect::<HashSet<usize>>();
    info!("unique_groups: {:?}", unique_groups);

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

    // let Ok((terrain_transform, terrain, terrain_mesh)) = terrains.get(trigger.plane_entity) else {return};
    // let Some(mesh) = meshes.get(&terrain_mesh.0) else {return};
    // let heightfield = extract_heightfield(&mesh);
    // let terrain_aabb = AABB::from_loc_dims(terrain_transform.translation.xz(), terrain.dims);

    // // commands.entity(trigger.plane_entity).insert(ColliderConstructor::TrimeshFromMesh)
    // // let collider_constructor = ColliderConstructor::Heightfield { heights: heightfield, scale: Vec3::ONE };
    // let collider = Collider::heightfield(heightfield, terrain_transform.scale);
    // // let trimesh = collider.to_trimesh();

    // // Collider::from(mesh);

    // commands.entity(trigger.plane_entity).insert(collider);

    info!("[NAV] Generate terrain navmesh for entity2: {}", trigger.entity);
    let Ok((terrain_transform, terrain, _terrain_mesh)) = terrains.get(trigger.entity) else {return};
    let terrain_aabb = AABB::from_loc_dims(terrain_transform.translation.xz(), terrain.dims);

    // Start using plane entity
    let mut hs: HashSet<Entity> = HashSet::new();
    hs.insert(trigger.entity);


    for (entity, transform, nstatic, name) in statics.iter(){

        if !terrain_aabb.has_point(transform.translation.xz()){
            continue;
        }

        if (nstatic.typ == NavStaticType::Blocker) & ((name.contains("Bld")) | name.contains("Prop")) {
            hs.insert(entity);
        }
        if let NavStaticType::Navigable(a) = nstatic.typ {
            hs.insert(entity);
        }

    }

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
        // aabb: Some(Aabb3d::new(terrain_transform.translation, Vec3::new(5000.0, 300.0, 5000.0))),
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
    navmesh_handles.data.insert(PGNavmeshType::Terrain, Some(navmesh));
}

fn on_ready_navmesh(
    trigger:             On<NavmeshReady>,
    mut commands:        Commands,
    ass_nav:             Res<Assets<Navmesh>>,
    navconfig:           Res<NavConfig>,
    navmesh_handles:     Res<RecastNavmeshHandles>,
){

    let Some(recast_navmesh) = ass_nav.get(trigger.0) else {return;};

    for (navmesh_type, maybe_navmesh_handle) in navmesh_handles.data.iter(){
        if let Some(navmesh_handle) = maybe_navmesh_handle {
            if navmesh_handle.id() == trigger.0 {
                
                let mut pgn: PGNavmesh = convert_rerecast(
                    recast_navmesh,
                    navmesh_type
                );

                // pgn.cleanup_lower();
                pgn.reorder_vertex_polygons();
                pgn.islands_removal();
                pgn.bake();
                
                commands.spawn(pgn.clone());

                if navconfig.serialize {
                    info!("Serializing navmesh to {}", pgn.filename());
                    IoTaskPool::get().spawn(async move {
                        let f = File::create(&pgn.filename()).ok().unwrap();
                        let mut writer = BufWriter::new(f);
                        let _res = serde_json::to_writer(&mut writer, &pgn);
                        let _res = writer.flush();
                    })
                    .detach();
                }
            }
        }
    }
}

fn on_spawn_navmesh(
   trigger:       On<Add, PGNavmesh>,
   mut commands:  Commands,
   navs:          Query<&PGNavmesh>
){
    if let Ok(navmesh) = navs.get(trigger.entity){
        match navmesh.typ {
            PGNavmeshType::Terrain => {
                commands.entity(trigger.entity).insert(NavmeshTerrain);
            }
            PGNavmeshType::Water => {
                commands.entity(trigger.entity).insert(NavmeshWater);
            }
        }
    }
}


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
