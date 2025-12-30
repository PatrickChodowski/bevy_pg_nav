use bevy::color::palettes::css::{BLACK, WHITE};
use serde::{Serialize, Deserialize};
use bevy::tasks::IoTaskPool;
use std::fs::File;
use std::io::{BufWriter, Write};
use bevy_common_assets::json::JsonAssetPlugin;
use bevy::{input::common_conditions::input_just_pressed, prelude::*};
use bevy::platform::collections::{HashSet, HashMap};
use bevy_rerecast::{debug::DetailNavmeshGizmo, prelude::*};
use bevy_rerecast::Mesh3dBackendPlugin;
use bevy_pg_core::prelude::TerrainChunk;

use crate::debug::PGNavDebugPlugin;
use crate::water::{
    raycasts_rain, remove_collinear, order_boundary, 
    group_waters, triangulate, mesh_from_triangles,
    WaterVertex, WaterNavmeshSource, WaterNavmeshesReady
};
use crate::terrain::TerrainRayMeshData;
use crate::recast_convert::convert_rerecast;
use crate::types::PGNavmesh;

pub struct PGNavPlugin;

impl Plugin for PGNavPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_plugins((
            NavmeshPlugins::default(),
            Mesh3dBackendPlugin::default(),
            // AvianBackendPlugin::default()
        ))
        .insert_resource(NavConfig::default())
        .init_asset::<PGNavmesh>()
        .add_message::<GenerateNavMesh>()
        .add_plugins(JsonAssetPlugin::<PGNavmesh>::new(&["navmesh.json"]))
        // .add_plugins(PGNavDebugPlugin)
        .add_systems(Update, trigger_navmesh.run_if(input_just_pressed(KeyCode::KeyG)))
        .insert_resource(RecastNavmeshHandles::default())

        .add_observer(generate_terrain_navmesh)
        .add_observer(generate_water_navmesh)
        .add_observer(on_water_navmesh_sources_ready)
        .add_observer(on_ready_navmesh)
        .add_observer(on_spawn_navmesh)
        ;
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
    pub water_height: f32,
    pub raycast_step: u32,
    pub offset_y: f32, // Offset on Display only
    pub iter_count_limit: usize,
    pub serialize: bool,
    pub debug: bool
}
impl Default for NavConfig {
    fn default() -> Self {
        NavConfig{
            water_height: 180.0,
            raycast_step: 10,
            offset_y: 20.0,
            iter_count_limit: 20,
            serialize: true,
            debug: true
        }
    }
}

fn trigger_navmesh(
    mut commands:  Commands
){
    commands.trigger(GenerateNavMesh::default());
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug, Reflect, Serialize, Deserialize)]
pub enum PGNavmeshType {
    Terrain,
    Water
}

#[derive(Component)]
pub struct NavmeshWater;

#[derive(Component)]
pub struct NavmeshTerrain;



#[derive(Event, Message)]
pub struct GenerateNavMesh {
    pub name:     String,
    pub map_name: String,
    pub chunk_id: String,
    pub chunk_size: f32
}
impl Default for GenerateNavMesh {
    fn default() -> Self {
        GenerateNavMesh{
            name: "".to_string(), 
            map_name: "".to_string(), 
            chunk_id: "".to_string(), 
            chunk_size: 0.0
        }
    }
}

impl GenerateNavMesh {
    pub fn new(
        name: String,
        map_name: &str, 
        chunk_id: &str,
        chunk_size: f32
    
    ) -> Self{
        GenerateNavMesh {
            name,
            map_name: map_name.to_string(), 
            chunk_id: chunk_id.to_string(),
            chunk_size
        }
    }
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
        walkable_climb: 0.5,
        walkable_slope_angle: 30.0_f32.to_radians(),
        agent_radius: 5.0,
        agent_height: 10.0,
        max_vertices_per_polygon: 20,
        max_simplification_error: 1.3,
        min_region_size: 100,
        up: Vec3::Y,
        ..default()
    };
    let navmesh = generator.generate(settings);
    commands.spawn(DetailNavmeshGizmo::new(&navmesh));
    navmesh_handles.data.insert(PGNavmeshType::Water, Some(navmesh));
}

fn generate_water_navmesh(
    _trigger:       On<GenerateNavMesh>,
    terrains:       Query<(&Transform, &Mesh3d, &Name), With<TerrainChunk>>,
    mut commands:   Commands,
    mut meshes:     ResMut<Assets<Mesh>>,
    mut materials:  ResMut<Assets<StandardMaterial>>,
    navconfig:      Res<NavConfig>
){
    let raycast_step = navconfig.raycast_step as usize;
    let water_height: f32 = navconfig.water_height;
    let extent: f32 = navconfig.raycast_step as f32 * 0.5;
    let half_chunk_size: f32 = 2500.0;

    for (terrain_transform, mesh3d, terrain_name) in terrains.iter(){

        let Some(mesh) = meshes.get(&mesh3d.0) else {continue;};
        let safety_offset: f32 = navconfig.raycast_step as f32 *2.0;
        let trmd = TerrainRayMeshData::from_mesh(mesh, &terrain_transform.to_matrix());
        let loc = terrain_transform.translation;

        let min_x = (loc.x - half_chunk_size - safety_offset - extent) as u32; // For Some reason in X I need to do it
        let max_x = (loc.x + half_chunk_size + safety_offset) as u32;
        let min_z = (loc.z - half_chunk_size - safety_offset) as u32;
        let max_z = (loc.z + half_chunk_size + safety_offset) as u32;

        let xs_u: Vec<u32> = (min_x..=max_x).step_by(raycast_step).collect();
        let zs_u: Vec<u32> = (min_z..=max_z).step_by(raycast_step).collect();

        let xs = xs_u.iter().map(|n| *n as f32).collect::<Vec<f32>>();
        let zs = zs_u.iter().map(|n| *n as f32).collect::<Vec<f32>>();

        let mut water_hits = raycasts_rain(&xs, &zs, &trmd, water_height);
        let groups_map = group_waters(&mut water_hits, max_x, max_z);

        // Order groups
        let mut hm_groups: HashMap<usize, Vec<WaterVertex>> = HashMap::new();
        for (_tile, vertex) in groups_map.iter(){
            hm_groups.entry(vertex.group).or_insert(Vec::new()).push(*vertex);
        }

        let mut triangle_map: HashMap<usize, Vec<[Vec3A; 3]>> = HashMap::new();
        for (group, vertices) in hm_groups.iter_mut(){
            let mut vlocs: Vec<Vec3A> = order_boundary(vertices, max_x, max_z);
            info!(" Group: {} Vertices count after order_boundary: {}", group, vertices.len());
            vlocs = remove_collinear(&mut vlocs);
            info!(" Group: {} Vertices count after remove_collinear: {}", group, vertices.len());
            let triangles = triangulate(&mut vlocs);
            let mesh = mesh_from_triangles(&triangles);
            triangle_map.insert(*group, triangles);

            commands.spawn(
                (
                    Mesh3d(meshes.add(mesh)),
                    MeshMaterial3d(materials.add(StandardMaterial::from(Color::from(WHITE)))),
                    WaterNavmeshSource,
                    Visibility::Hidden
                )
            );
        }

        commands.trigger(WaterNavmeshesReady);
    }
}

fn generate_terrain_navmesh(
    _trigger:            On<GenerateNavMesh>,
    mut generator:       NavmeshGenerator,
    mut commands:        Commands,
    terrain_entity:      Single<Entity, With<TerrainChunk>>,
    statics:             Query<(Entity, &NavStatic, &Name)>,
    mut navmesh_handles: ResMut<RecastNavmeshHandles>
){
    info!("[NAV] Generate terrain navmesh");

    let mut hs = HashSet::new();
    hs.insert(*terrain_entity);

    for (entity, nstatic, name) in statics.iter(){
        if (nstatic.typ == NavStaticType::Blocker) & ((name.contains("Bld")) | name.contains("Prop")) {
            hs.insert(entity);
        }
    }

    for (entity, nstatic, _name) in statics.iter(){
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
        walkable_climb: 8.0,
        walkable_slope_angle: 55.0_f32.to_radians(),
        agent_radius: 5.0,
        agent_height: 10.0,
        // min_region_size: 10,
        max_vertices_per_polygon: 20,
        max_simplification_error: 1.3,
        // cell_size_fraction: 2.0,
        // cell_height_fraction: 4.0,
        up: Vec3::Y,
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
                    navconfig.water_height,
                    navmesh_type
                );

                pgn.cleanup_lower();
                pgn.reorder_vertex_polygons();
                commands.spawn(pgn.clone());

                if navconfig.serialize {
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
   navs:          Query<&PGNavmesh>,
   navconfig:     Res<NavConfig>
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

        // if navconfig.debug {
        //     commands.spawn(DetailNavmeshGizmo::new(&navmesh));
        // }
    }
}
