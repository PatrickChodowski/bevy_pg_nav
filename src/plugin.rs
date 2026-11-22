
use bevy::tasks::IoTaskPool;
use avian3d::prelude::{Collider, RigidBody, PhysicsSchedulePlugin, PhysicsPlugins};
use std::fs::File;
use std::io::{BufWriter, Write};
use bevy_common_assets::json::JsonAssetPlugin;
use bevy::prelude::*;
use bevy::platform::collections::HashMap;
use bevy_rerecast::prelude::*;
use bevy_rerecast::debug::DetailNavmeshGizmo;
use avian_rerecast::AvianBackendPlugin;
use bevy_pg_core::prelude::TerrainChunk;
use crate::multi_nav::PGMultiNavPlugin;
use crate::water::{PGWaterNavPlugin, WaterNavmeshSource, GenerateWaterNavmesh};
use crate::terrain::{PGTerrainNavPlugin, GenerateTerrainNavmesh};
use crate::recast_convert::convert_rerecast;
use crate::pgnavmesh::{PGNavmesh, PGNavmeshType};

pub struct PGNavPlugin{
    pub colliders_mapping: fn(object_name: String) -> Option<(Collider, NavStatic)>
}

#[derive(Resource)]
pub struct NavResources {
    pub(crate) colliders_mapping: fn(object_name: String) -> Option<(Collider, NavStatic)>,
    pub visible: bool
}

impl Plugin for PGNavPlugin {
    fn build(&self, app: &mut App) {
        app
        .init_asset::<PGNavmesh>();

        if !app.is_plugin_added::<PhysicsSchedulePlugin>() {
            app.add_plugins(PhysicsPlugins::default());
        }

        app.add_plugins((
            NavmeshPlugins::default(),
            AvianBackendPlugin::default(),
            JsonAssetPlugin::<PGNavmesh>::new(&["nav.json"]),
            PGTerrainNavPlugin,
            PGWaterNavPlugin,
            PGMultiNavPlugin
        ))
        .insert_resource(NavResources{
            colliders_mapping: self.colliders_mapping,
            visible: false
        })
        .insert_resource(NavHandlesHolders::new())
        .add_observer(on_generate_navmesh)
        .add_observer(on_ready_navmesh)
        .add_observer(on_spawn_navmesh)
        .add_systems(Update, check_navgendata.run_if(resource_exists_and_changed::<NavmeshGenerationData>))
        .add_systems(Update, toggle_visibility) //.run_if(resource_changed::<NavResources>))
        ;
    }
}

// Entry point for external systems
fn on_generate_navmesh(
    trigger:      On<GenerateNavMesh>,
    mut commands: Commands
){
    info!("Generate Navmesh for plane entity: {}", trigger.plane_entity);
    let ngd = NavmeshGenerationData::from_event(&trigger.event());
    commands.insert_resource(ngd);
    commands.trigger(GenerateTerrainNavmesh);
    commands.trigger(GenerateWaterNavmesh);
}

fn check_navgendata(
    navgendata:     Res<NavmeshGenerationData>,
    mut commands:   Commands,
    nav_colliders:  Query<Entity, Or<(With<TerrainChunk>, With<NavStatic>)>>,
    water_sources:  Query<Entity, With<WaterNavmeshSource>>,
    mut handles:    ResMut<NavHandlesHolders>
){

    let mut dones: usize = 0;
    for (_navtype, navstate) in navgendata.recast_handles.iter(){
        if navstate.1 {
            dones += 1;
        }
    }
    if dones == navgendata.recast_handles.len(){
        info!("Cleaning up after navmesh generation");

        handles.clear();
        for (_typ, recast_handle) in navgendata.recast_handles.iter(){
            handles.add(recast_handle.0.as_ref().unwrap().clone());
        }

        commands.remove_resource::<NavmeshGenerationData>();

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
    pub plane_entity: Entity,
    pub name: String,
    pub terrain_settings: Option<NavmeshSettings>,
    pub water_settings: Option<NavmeshSettings>
}

// Resource existing only during navmesh creation. Removed after finish
#[derive(Resource, Debug)]
pub (crate) struct NavmeshGenerationData {
    recast_handles: HashMap<PGNavmeshType, (Option<Handle<Navmesh>>, bool)>,
    name: String,
    pub(crate) plane_entity: Entity,
    pub(crate) terrain_settings: Option<NavmeshSettings>,
    pub(crate) water_settings: Option<NavmeshSettings>,
}
impl NavmeshGenerationData {
    // pub(crate) fn new(
    //     plane_entity: Entity, 
    //     name: String
    // ) -> Self {
    //     info!("New Navmesh Generation Data");
    //     let ngd = NavmeshGenerationData {
    //         recast_handles: HashMap::from(
    //             [
    //                 (PGNavmeshType::Terrain, (None, false)),
    //                 (PGNavmeshType::Water, (None, false))
    //             ]
    //         ),
    //         name: name,
    //         plane_entity,
    //         terrain_settings: None,
    //         water_settings: None
    //     };
    //     return ngd;
    // }
    pub(crate) fn add_handle(
        &mut self, 
        typ: PGNavmeshType, 
        handle: Handle<Navmesh>
    ){
        self.recast_handles.insert(typ, (Some(handle), false));
    }


    pub(crate) fn from_event(event: &GenerateNavMesh) -> Self {
        let ngd = NavmeshGenerationData {
            recast_handles: HashMap::from(
                [
                    (PGNavmeshType::Terrain, (None, false)),
                    (PGNavmeshType::Water, (None, false))
                ]
            ),
            name: event.name.clone(),
            plane_entity: event.plane_entity,
            terrain_settings: event.terrain_settings.clone(),
            water_settings: event.water_settings.clone()
        };

        return ngd;
    }
}


#[derive(Resource, Debug)]
pub (crate) struct NavHandlesHolders {
   pub (crate) data: Vec<Handle<Navmesh>>
}
impl NavHandlesHolders {
    fn new() -> Self {
        NavHandlesHolders{data: Vec::new()}
    }
    fn add(&mut self, handle: Handle<Navmesh>){
        self.data.push(handle);
    }
    fn clear(&mut self){
        self.data.clear();
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
   trigger:       On<Add<PGNavmesh>>,
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


fn toggle_visibility(
    navres:       Res<NavResources>,
    mut query:    Query<(Entity, &mut Visibility), With<DetailNavmeshGizmo>>
){

    if navres.visible {
        for (_entity, mut vis) in query.iter_mut(){
            *vis = Visibility::Inherited;
        }
    } else {
        for (_entity, mut vis) in query.iter_mut(){
            *vis = Visibility::Hidden;
        }
    }
}
