
use bevy::tasks::IoTaskPool;
use avian3d::prelude::{Collider, RigidBody};
use std::fs::File;
use std::io::{BufWriter, Write};
use bevy_common_assets::json::JsonAssetPlugin;
use bevy::prelude::*;
use bevy::platform::collections::HashMap;
use bevy_rerecast::prelude::*;
use avian_rerecast::AvianBackendPlugin;
use bevy_pg_core::prelude::{GameState, TerrainChunk};

use crate::water::{PGWaterNavPlugin, WaterNavmeshSource, GenerateWaterNavmesh};
use crate::terrain::{PGTerrainNavPlugin, GenerateTerrainNavmesh};
use crate::recast_convert::convert_rerecast;
use crate::pgnavmesh::{PGNavmesh, PGNavmeshType};

pub struct PGNavPlugin{
    pub colliders_mapping: fn(object_name: String) -> Option<(Collider, NavStatic)>
}

#[derive(Resource)]
pub(crate) struct NavResources {
    pub(crate) colliders_mapping: fn(object_name: String) -> Option<(Collider, NavStatic)>
}

impl Plugin for PGNavPlugin {
    fn build(&self, app: &mut App) {
        app
        .init_asset::<PGNavmesh>()
        .add_plugins((
            NavmeshPlugins::default(),
            AvianBackendPlugin::default(),
            JsonAssetPlugin::<PGNavmesh>::new(&["nav.json"]),
            PGTerrainNavPlugin,
            PGWaterNavPlugin
        ))
        .insert_resource(NavResources{colliders_mapping: self.colliders_mapping})
        .add_observer(on_generate_navmesh)
        .add_observer(on_ready_navmesh)
        .add_observer(on_spawn_navmesh)
        .add_systems(Update, check_navgendata.run_if(resource_exists_and_changed::<NavmeshGenerationData>))
        .add_systems(OnExit(GameState::Play), clear)
        ;
    }
}

// Entry point for external systems
fn on_generate_navmesh(
    trigger:      On<GenerateNavMesh>,
    terrains:     Query<Option<&Name>, (With<TerrainChunk>, With<Transform>, With<Mesh3d>)>,
    mut commands: Commands
){
    let Ok(maybe_name) = terrains.get(trigger.plane_entity) else {return};

    if maybe_name.is_none(){
        warn!("Plane needs a name before generating navmesh");
        return;
    }
    let plane_name = maybe_name.unwrap().to_string();

    info!("Generate Navmesh for plane entity: {} name: {}", trigger.plane_entity, plane_name);
    commands.insert_resource(NavmeshGenerationData::new(trigger.plane_entity, plane_name));
    commands.trigger(GenerateTerrainNavmesh);
    commands.trigger(GenerateWaterNavmesh);
}

fn check_navgendata(
    navgendata:     Res<NavmeshGenerationData>,
    mut commands:   Commands,
    nav_colliders:  Query<Entity, Or<(With<TerrainChunk>, With<NavStatic>)>>,
    water_sources:   Query<Entity, With<WaterNavmeshSource>>
){

    let mut dones: usize = 0;

    for (_navtype, navstate) in navgendata.recast_handles.iter(){
        if navstate.1 {
            dones += 1;
        }
    }
    if dones == navgendata.recast_handles.len(){
        info!("Cleaning up after navmesh generation");

        // commands.remove_resource::<NavmeshGenerationData>();

        for entity in nav_colliders.iter(){
            commands.entity(entity).remove::<Collider>();
            commands.entity(entity).remove::<RigidBody>();
            commands.entity(entity).try_remove::<NavStatic>();
        }

        for entity in water_sources.iter(){
            commands.entity(entity).despawn();
        }
    }

}



fn clear(
    mut commands: Commands,
    navs:         Query<Entity, With<PGNavmesh>>
){
    for entity in navs.iter(){
        commands.entity(entity).despawn();
    }

    commands.remove_resource::<NavmeshGenerationData>();

}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum NavStaticType {
    Navigable(f32), // Yoffset
    Blocker
}

#[derive(Component, Clone, Copy)]
pub struct NavStatic {
    pub typ:    NavStaticType
}
impl NavStatic {
    pub fn blocker() -> Self {
        NavStatic{typ: NavStaticType::Blocker}
    }
    pub fn navigable() -> Self {
        NavStatic{typ: NavStaticType::Navigable(0.0)}
    }
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
pub (crate) struct NavmeshGenerationData {
    recast_handles: HashMap<PGNavmeshType, (Option<Handle<Navmesh>>, bool)>,
    name: String,
    pub(crate) plane_entity: Entity
}
impl NavmeshGenerationData {
    pub(crate) fn new(
        plane_entity: Entity, 
        name: String
    ) -> Self {
        let ngd = NavmeshGenerationData {
            recast_handles: HashMap::from(
                [
                    (PGNavmeshType::Terrain, (None, false)),
                    (PGNavmeshType::Water, (None, false))
                ]
            ),
            name: name.clone(),
            plane_entity
        };
        return ngd;
    }
    pub(crate) fn add_handle(
        &mut self, 
        typ: PGNavmeshType, 
        handle: Handle<Navmesh>
    ){
        self.recast_handles.insert(typ, (Some(handle), false));
    }
}



fn on_ready_navmesh(
    trigger:        On<NavmeshReady>,
    mut commands:   Commands,
    ass_nav:        Res<Assets<Navmesh>>,
    mut navgendata: ResMut<NavmeshGenerationData>
){

    let Some(recast_navmesh) = ass_nav.get(trigger.0) else {return;};
    let navname = navgendata.name.clone();

    for (navmesh_type, ref mut nav_state) in navgendata.recast_handles.iter_mut(){
        if nav_state.1 {continue};
        let Some(ref navmesh_handle) = nav_state.0  else {continue};
        if navmesh_handle.id() != trigger.0 {continue};

        let mut pgn: PGNavmesh = convert_rerecast(
            recast_navmesh,
            navmesh_type
        );
        pgn.name = navname.clone();

        // pgn.cleanup_lower();
        pgn.reorder_vertex_polygons();
        pgn.islands_removal();
        pgn.bake();
        
        commands.spawn(pgn.clone());

        info!("Serializing navmesh to {}", pgn.filename());
        IoTaskPool::get().spawn(async move {
            let f = File::create(&pgn.filename()).ok().unwrap();
            let mut writer = BufWriter::new(f);
            let _res = serde_json::to_writer(&mut writer, &pgn);
            let _res = writer.flush();
        })
        .detach();
        
        nav_state.1 = true;
    }
}

// Inserts components of type. Separate system as it can also run when spawning navmesh from file
fn on_spawn_navmesh(
   trigger:       On<Add, PGNavmesh>,
   mut commands:  Commands,
   navs:          Query<&PGNavmesh>
){
    if let Ok(navmesh) = navs.get(trigger.entity){
        info!("Spawned navmesh of type: {:?} name: {}", navmesh.typ, navmesh.name);
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
