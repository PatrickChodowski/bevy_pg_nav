
use bevy::prelude::*;
use itertools::Itertools;
use serde::{Serialize, Deserialize};
use bevy::camera::primitives::Aabb;
use bevy::platform::collections::HashMap;
use dashmap::DashMap;
use bevy::tasks::IoTaskPool;
use spade::handles::FixedVertexHandle;
use std::fs::File;
use std::io::{BufWriter, Write};
use bevy_common_assets::json::JsonAssetPlugin;
use geo::{Polygon, LineString, Coord, MultiPolygon};
use geo::algorithm::intersects::Intersects;
use geo::BooleanOps;

use geo_booleanop::boolean::BooleanOp;

use crate::triangles::NavPolygon;
use crate::functions::get_target_ray_meshes;

use crate::triangles::{
    // find_neighbours, 
    merge_by_groups,
    raycasts_rain,
    // loop_merge_quads_directional
};

use crate::terrain::TerrainRayMeshData;
use crate::types::{NavDebug, NavQuad, NavStatic, NavStaticType, RayTargetMesh, RayTargetMeshShape};
use crate::navmesh::NavMesh;

pub struct PGNavPlugin;

impl Plugin for PGNavPlugin {
    fn build(&self, app: &mut App) {
        app
        .init_asset::<NavMesh>()
        .add_message::<GenerateNavMesh>()
        .add_plugins(JsonAssetPlugin::<NavMesh>::new(&["navmesh.json"]))
        .insert_resource(NavConfig::default())
        .insert_resource(NavMesh::default())
        .insert_resource(NavDebug::default())
        .add_systems(PreUpdate, generate_navmesh.run_if(on_message::<GenerateNavMesh>))
        // .add_systems(Update, debug)

        .add_systems(Update, debug_triangulation.run_if(resource_exists::<DebugTriangulation1>))
        ;
    }
}

#[derive(Message)]
pub struct GenerateNavMesh {
    pub name: String,
    pub map_name: String,
    pub chunk_id: String,
    pub chunk_size: f32
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

use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
use ordered_float::OrderedFloat;

#[derive(Resource)]
pub struct DebugTriangulation1 {
    pub data: HashMap<usize, NavPolygon>
}


#[derive(Resource)]
pub struct DebugTriangulation {
    pub faces: Vec<[Vec3;4]>
}
use bevy::color::palettes::css::{WHITE, RED};
fn debug_triangulation(
    data: Res<DebugTriangulation1>,
    mut gizmos: Gizmos,
){
    let clr: Color= Color::from(WHITE);
    let center_clr = Color::from(RED);

    for (key, nav_polygon) in data.data.iter(){


        for pair in nav_polygon.vertices.windows(2){
            gizmos.line(pair[0].into(), pair[1].into(), clr);
        }


        // let a = face[0];
        // let b = face[1];
        // let c = face[2];
        // let center = face[3];

        // gizmos.line(a, b, clr);
        // gizmos.line(b, c, clr);
        // gizmos.line(c, a, clr);

        // gizmos.circle(Isometry3d::from_translation(center), 3.0, center_clr);

    }
}

fn generate_navmesh(
    mut events:     MessageReader<GenerateNavMesh>,
    mut commands:   Commands,
    meshes:         Res<Assets<Mesh>>,
    mesh_query:     Query<(&Transform, &Aabb, &NavStatic)>,
    terrains:       Query<(&Transform, &Mesh3d, &Name)>,
    navconfig:      Res<NavConfig>
){
    for ev in events.read(){

        let raycast_step = navconfig.raycast_step as usize;
        let water_height: f32 = navconfig.water_height;
        let extent: f32 = navconfig.raycast_step as f32 * 0.5;
        let half_chunk_size: f32 = ev.chunk_size*0.5;

        let mut navmesh_done: bool = false;

        for (terrain_transform, mesh3d, terrain_name) in terrains.iter(){

            if ev.name != terrain_name.to_string() {
                continue;
            } 

            let Some(mesh) = meshes.get(&mesh3d.0) else {continue;};
            info!("[NAVMESH] Generate Navmesh for {} {}", ev.map_name, ev.chunk_id);
            info!("[NAVMESH] RayCast Step: {}", raycast_step);
            info!("[NAVMESH] Extent: {}", extent);

            let safety_offset: f32 = navconfig.raycast_step as f32 *2.0;
            let trmd = TerrainRayMeshData::from_mesh(mesh, &terrain_transform.to_matrix());
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

            let ray_meshes: Vec<RayTargetMesh> = get_target_ray_meshes(&mesh_query);
            info!("[NAVMESH][GENERATE] after generating ray target meshes");

            // SPADE APPROACH 

            // let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f64>>::new();

            // for (index, vertex) in trmd.vertices.iter().enumerate(){
            //     let tvex = trmd.mesh_transform.transform_point3a(*vertex);
            //     let point = Point2::new(tvex.x as f64*-1.0, tvex.z as f64*-1.0);
            //     let handle = cdt.insert(point).unwrap();
            // }

            // let mut polygons: Vec<Polygon> = Vec::new();
            // for ray_mesh in ray_meshes.iter(){
            //     if ray_mesh.typ == NavStaticType::Blocker {
            //         let polygon = ray_mesh.shape.to_geo_polygon();
            //         polygons.push(polygon)
            //     }
            // }
            // // info!("Initial Polygons count: {}", polygons.len());

            // loop {
            //     let mut pairs: Vec<(Polygon, Polygon)> = Vec::new();
            //     let mut used: Vec<usize> = Vec::new();

            //     for (index1, poly1) in polygons.iter().enumerate(){
            //         if used.contains(&index1){
            //             continue;
            //         }
            //         for (index2, poly2) in polygons.iter().enumerate(){

            //             if used.contains(&index2){
            //                 continue;
            //             }

            //             if index1 == index2 {
            //                 continue;
            //             }

            //             if poly1.intersects(poly2){
            //                 pairs.push((poly1.clone(), poly2.clone()));
            //                 used.push(index1);
            //                 used.push(index2);
            //             }
            //         }
            //     }
            //     // info!("Pairs count: {}", pairs.len());
            //     if pairs.len() == 0 {
            //         break; // Exit loop if no more intersections
            //     }
            //     // Merge pairs
            //     for (poly1, poly2) in pairs.iter(){

            //         let unioned_multi = poly1.union(poly2);
            //         if unioned_multi.0.len() > 1 {
            //             info!("unioned multi has more than one polygon: {}", unioned_multi.0.len());
            //         }

            //         let unioned_polygon: Polygon = unioned_multi.0[0].clone();
            //         polygons.retain(|p| (p != poly1) & (p != poly2));
            //         polygons.push(unioned_polygon);
            //     }
            //     // info!("Polygons count: {}", polygons.len());
            // }


            // for polygon in polygons.iter(){
            //     if let Ok(ace)  = cdt.add_constraint_edges(polygon.exterior().0.iter().map(|p| Point2::new(p.x, p.y)), true) {
            //         // info!("Correctly added edges: {:?}", ace);
            //     } else {
            //         info!("ISSUE ADDING EDGES");
            //     }
            
            // }

            // let num_faces = cdt.num_all_faces();
            // info!("NUM FACES: {}", num_faces);

            // let mut faces: Vec<[Vec3; 4]> = Vec::new();

            // let outer_face = cdt.outer_face();

            // // outer_face.


            // for face in cdt.inner_faces(){
            //     let vertices = face.vertices();
            //     let mut face_points: [Vec3; 4] = [Default::default(); 4];
            //     for (index, point) in vertices.iter().enumerate(){
            //         let pos = point.position();
            //         face_points[index] = Vec3::new(pos.x as f32, 250.0, pos.y as f32);
            //     }

            //     face_points[3] = Vec3::new(face.center().x as f32, 250.0, face.center().y as f32);

            //     faces.push(face_points);
            // }

            // commands.insert_resource(DebugTriangulation{faces});


            let xs_u: Vec<u32> = (min_x..=max_x).step_by(raycast_step).collect();
            let zs_u: Vec<u32> = (min_z..=max_z).step_by(raycast_step).collect();

            let xs = xs_u.iter().map(|n| *n as f32).collect::<Vec<f32>>();
            let zs = zs_u.iter().map(|n| *n as f32).collect::<Vec<f32>>();

            let mut dash_nav_quads: DashMap<usize, NavPolygon> = raycasts_rain(
                &xs,
                &zs, 
                &ray_meshes, 
                &trmd,
                water_height,
                // extent
            );

            let max_key = dash_nav_quads.iter().map(|entry| *entry.key()).max().unwrap();
            let terrain_vertices_navpolys = trmd.vertices_to_navpolygons(max_key);
            for np in terrain_vertices_navpolys.iter(){
                dash_nav_quads.insert(np.index, np.clone());
            }

            info!("[NAVMESH][GENERATE] after raycasts_rain: {}", dash_nav_quads.len());
            merge_by_groups(&mut dash_nav_quads);

            info!("[NAVMESH][GENERATE] after merge_by_groups: {}", dash_nav_quads.len());
            let mut quads_count: usize = dash_nav_quads.len();
            let mut nav_quads: HashMap<usize, NavPolygon> = dash_nav_quads.clone().into_iter().map(|(_tile, quad)| (quad.index, quad)).collect();

            commands.insert_resource(DebugTriangulation1{data: nav_quads.clone()});

            // loop_merge_quads_directional(&mut nav_quads, &mut quads_count, navconfig.iter_count_limit);
            // info!("[NAVMESH][GENERATE] after loop_merge_quads_directional: {}", nav_quads.len());

            // find_neighbours(&mut nav_quads);
            info!("[NAVMESH][GENERATE] after find_neighbours");

            navmesh_done = true;
            // let mut navmesh = NavMesh::from_hash_navpolygons(&mut nav_quads);
            // navmesh.water_height = navconfig.water_height;
            // info!("[NAVMESH][GENERATE] NavMesh Polygon count: {} ", navmesh.polygons.len());
            // info!("[NAVMESH][GENERATE] NavMesh Vertex count: {} ", navmesh.vertices.len());

            // commands.insert_resource(NavDebug{hit_quad_id: None});
            // commands.insert_resource(navmesh.clone());

            // if navconfig.serialize {
            //     info!("[NAVMESH][GENERATE] Saving Navmesh to json for {} {}", ev.map_name, ev.chunk_id);
            //     let filename = format!("./assets/navmesh/{}_{}.navmesh.json", ev.map_name, ev.chunk_id);
            //     IoTaskPool::get().spawn(async move {
            //         let f = File::create(&filename).ok().unwrap();
            //         let mut writer = BufWriter::new(f);
            //         let _res = serde_json::to_writer(&mut writer, &navmesh);
            //         let _res = writer.flush();
            //     })
            //     .detach();
            // }

        }

        if !navmesh_done {
            // info!("[NAVMESH][GENERATE] NavMesh was not created, sending event again for {} {}", ev.map_name, ev.chunk_id);
            commands.write_message(GenerateNavMesh::new(ev.name.clone(), &ev.map_name, &ev.chunk_id, ev.chunk_size));
        }
        // }
    }
}

fn debug(
    navmesh:    Option<Res<NavMesh>>,
    mut gizmos: Gizmos,
    nav_config: Res<NavConfig>
){
    if let Some(navmesh) = navmesh {
        if nav_config.debug {
            for (_k, polygon) in navmesh.polygons.iter(){
                polygon.display(&mut gizmos, false, nav_config.offset_y);
            }
        }
    }
}


#[derive(
    Resource, Serialize, Deserialize, Debug, bevy::asset::Asset, bevy::reflect::TypePath, Clone,
)]
pub struct NavConfig{
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
            debug: false
        }
    }
}
