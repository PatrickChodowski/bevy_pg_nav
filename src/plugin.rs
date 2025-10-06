
use bevy::prelude::*;
use serde::{Serialize, Deserialize};
use bevy::render::primitives::Aabb;
use bevy::platform::collections::HashMap;
use dashmap::DashMap;
use bevy::tasks::IoTaskPool;
use std::fs::File;
use std::io::{BufWriter, Write};
use bevy_common_assets::json::JsonAssetPlugin;

use crate::maps::scenes::{MapsData, Static, TerrainChunk};

use crate::functions::{
    find_neighbours, 
    merge_by_groups,
    get_ray_meshes, 
    raycasts_rain,
    loop_merge_quads_directional
};

use crate::terrain::TerrainRayMeshData;
use crate::types::{RayMesh, NavQuad, Navigable, NavDebug};
use crate::navmesh::NavMesh;

pub struct PGNavPlugin;

impl Plugin for PGNavPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_event::<GenerateNavMesh>()
        .add_plugins(JsonAssetPlugin::<NavMesh>::new(&["navmesh.json"]))
        .insert_resource(NavConfig::default())
        .insert_resource(NavMesh::default())
        .insert_resource(NavDebug::default())
        .add_systems(PreUpdate, generate_navmesh.run_if(on_event::<GenerateNavMesh>))
        .add_systems(Update, debug)
        ;
    }
}

#[derive(Event)]
pub struct GenerateNavMesh {
    pub map_name: String,
    pub chunk_id: String
}
impl GenerateNavMesh {
    pub fn new(map_name: &str, chunk_id: &str) -> Self{
        GenerateNavMesh {
            map_name: map_name.to_string(), 
            chunk_id: chunk_id.to_string()
        }
    }
}

fn generate_navmesh(
    mut events:     EventReader<GenerateNavMesh>,
    mut commands:   Commands,
    meshes:         Res<Assets<Mesh>>,
    mesh_query:     Query<(&Transform, &Name, &Aabb, Option<&Navigable>), With<Static>>,
    trmd_query:     Query<(&Transform, &Mesh3d, &TerrainChunk)>,
    mapsdata:       Res<MapsData>,
    navconfig:      Res<NavConfig>,
    settings:       Res<Settings>
){
    for ev in events.read(){

        let raycast_step = navconfig.raycast_step as usize;
        let water_height: f32 = settings.water_height as f32;
        let extent: f32 = navconfig.raycast_step as f32 * 0.5;
        let mapdata = mapsdata.get_map(&ev.map_name);
        let half_chunk_size: f32 = mapdata.chunk_size*0.5;

        let mut navmesh_done: bool = false;

        for (terrain_transform, mesh3d, terrain_chunk) in trmd_query.iter(){

            if (terrain_chunk.map_name != ev.map_name) ||
               (terrain_chunk.chunk_id != ev.chunk_id) {
                continue;
            } 

            let Some(mesh) = meshes.get(&mesh3d.0) else {continue;};
            info!("[NAVMESH] Generate Navmesh for {} {}", ev.map_name, ev.chunk_id);
            info!("[NAVMESH] RayCast Step: {}", raycast_step);
            info!("[NAVMESH] Extent: {}", extent);

            let safety_offset: f32 = navconfig.raycast_step as f32 *2.0;
            let trmd = TerrainRayMeshData::from_mesh(mesh, &terrain_transform.compute_matrix());
            let loc = terrain_transform.translation;
            let min_x = (loc.x - half_chunk_size - safety_offset - extent) as u32; // For Some reason in X I need to do it
            let max_x = (loc.x + half_chunk_size + safety_offset) as u32;
            let min_z = (loc.z - half_chunk_size - safety_offset) as u32;
            let max_z = (loc.z + half_chunk_size + safety_offset) as u32;

            info!("[NAVMESH][GENERATE] LOC: ({},{})", loc.x, loc.z);
            info!("[NAVMESH][GENERATE] x: ({},{})", min_x, max_x);
            info!("[NAVMESH][GENERATE] z: ({},{})", min_z, max_z);
            info!("[NAVMESH][GENERATE] DimX: ({})", max_x-min_x);
            info!("[NAVMESH][GENERATE] DimZ: ({})", max_z-min_z);

            let ray_meshes: Vec<RayMesh> = get_ray_meshes(&mesh_query, &navconfig);
            info!("[NAVMESH][GENERATE] after generating ray meshes");

            let xs_u: Vec<u32> = (min_x..=max_x).step_by(raycast_step).collect();
            let zs_u: Vec<u32> = (min_z..=max_z).step_by(raycast_step).collect();

            let xs = xs_u.iter().map(|n| *n as f32).collect::<Vec<f32>>();
            let zs = zs_u.iter().map(|n| *n as f32).collect::<Vec<f32>>();

            let mut dash_nav_quads: DashMap<usize, NavQuad> = raycasts_rain(
                &xs,
                &zs, 
                &ray_meshes, 
                &trmd,
                water_height,
                extent
            );

            info!("[NAVMESH][GENERATE] after raycasts_rain: {}", dash_nav_quads.len());

            merge_by_groups(&mut dash_nav_quads);

            info!("[NAVMESH][GENERATE] after merge_by_groups: {}", dash_nav_quads.len());
            let mut quads_count: usize = dash_nav_quads.len();

            let mut nav_quads: HashMap<usize, NavQuad> = dash_nav_quads.clone().into_iter().map(|(_tile, quad)| (quad.index, quad)).collect();

            loop_merge_quads_directional(&mut nav_quads, &mut quads_count, navconfig.iter_count_limit);
            info!("[NAVMESH][GENERATE] after loop_merge_quads_directional: {}", nav_quads.len());

            find_neighbours(&mut nav_quads);
            info!("[NAVMESH][GENERATE] after find_neighbours");

            navmesh_done = true;
            let navmesh = NavMesh::from_hash_navquads(&mut nav_quads);
            info!("[NAVMESH][GENERATE] NavMesh Polygon count: {} ", navmesh.polygons.len());
            info!("[NAVMESH][GENERATE] NavMesh Vertex count: {} ", navmesh.vertices.len());

            commands.insert_resource(NavDebug{hit_quad_id: None});
            commands.insert_resource(navmesh.clone());

            if navconfig.serialize {
                info!("[NAVMESH][GENERATE] Saving Navmesh to json for {} {}", terrain_chunk.map_name, terrain_chunk.chunk_id);
                let filename = format!("./assets/navmesh/{}_{}.navmesh.json", terrain_chunk.map_name, terrain_chunk.chunk_id);
                IoTaskPool::get().spawn(async move {
                    let f = File::create(&filename).ok().unwrap();
                    let mut writer = BufWriter::new(f);
                    let _res = serde_json::to_writer(&mut writer, &navmesh);
                    let _res = writer.flush();
                })
                .detach();
            }

        }

        if !navmesh_done {
            // info!("[NAVMESH][GENERATE] NavMesh was not created, sending event again for {} {}", ev.map_name, ev.chunk_id);
            commands.send_event(GenerateNavMesh::new(&ev.map_name, &ev.chunk_id));
        }
    }
}

fn debug(
    nav_mesh:   Res<NavMesh>,
    mut gizmos: Gizmos,
    nav_config: Res<NavConfig>
){
    if nav_config.debug {
        for (_k, polygon) in nav_mesh.polygons.iter(){
            polygon.display(&mut gizmos, false, nav_config.offset_y);
        }
    }
}


#[derive(
    Resource, Serialize, Deserialize, Debug, bevy::asset::Asset, bevy::reflect::TypePath, Clone,
)]
pub struct NavConfig{
    pub raycast_step: u32,
    pub blocker_scale: f32,
    pub adjust_y: f32,
    pub offset_y: f32, // Offset on Display only
    pub iter_count_limit: usize,
    pub serialize: bool,
    pub debug: bool
}
impl Default for NavConfig {
    fn default() -> Self {
        NavConfig{
            raycast_step: 10,
            blocker_scale: 1.5,
            adjust_y: 1.0,
            offset_y: 20.0,
            iter_count_limit: 20,
            serialize: true,
            debug: false
        }
    }
}
