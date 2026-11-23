
use bevy::tasks::IoTaskPool;
use avian3d::prelude::{Collider, RigidBody, PhysicsSchedulePlugin, PhysicsPlugins};
use std::fs::File;
use std::io::{BufWriter, Write};
use bevy_common_assets::json::JsonAssetPlugin;
use bevy::prelude::*;
use bevy::platform::collections::HashMap;
use bevy_rerecast::prelude::*;
// use bevy_rerecast::debug::DetailNavmeshGizmo;
use avian_rerecast::AvianBackendPlugin;
use bevy_pg_core::prelude::TerrainChunk;
use crate::multi_nav::PGMultiNavPlugin;
use crate::water::{PGWaterNavPlugin, WaterNavmeshSource, UnderwaterCeiling};
use crate::terrain::{PGTerrainNavPlugin, GenerateTerrainNavmesh};
use crate::recast_convert::convert_rerecast;
use crate::pgnavmesh::{PGNavmesh, PGNavmeshType};
use crate::statics::{NavOverrides, NavShape};

/// Navmesh generation and loading. Scene objects become obstacles automatically from their
/// meshes (see statics.rs); `nav_overrides` optionally names a JSON asset with exceptions.
#[derive(Default)]
pub struct PGNavPlugin {
    /// Asset path of the obstacle overrides (`*.overrides.json`), e.g.
    /// "terrain/nav.overrides.json". None = no overrides.
    pub nav_overrides: Option<String>,
}

#[derive(Resource)]
pub struct NavResources {
    pub(crate) overrides: Option<Handle<NavOverrides>>,
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
            JsonAssetPlugin::<NavOverrides>::new(&["overrides.json"]),
            PGTerrainNavPlugin,
            PGWaterNavPlugin,
            PGMultiNavPlugin
        ))
        .insert_resource(NavResources{
            overrides: None,
            visible: false
        })
        .insert_resource(NavOverridesPath(self.nav_overrides.clone()))
        .add_systems(Startup, load_nav_overrides)
        .insert_resource(NavHandlesHolders::new())
        .add_observer(on_generate_navmesh)
        .add_observer(on_ready_navmesh)
        .add_observer(on_spawn_navmesh)
        .add_systems(Update, check_navgendata.run_if(resource_exists_and_changed::<NavmeshGenerationData>))
        // .add_systems(Update, toggle_visibility) //.run_if(resource_changed::<NavResources>))
        ;
    }
}

#[derive(Resource)]
struct NavOverridesPath(Option<String>);

fn load_nav_overrides(path: Res<NavOverridesPath>, ass: Res<AssetServer>, mut navres: ResMut<NavResources>) {
    if let Some(path) = &path.0 {
        navres.overrides = Some(ass.load(path.clone()));
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
    // Water follows once the terrain/static colliders are ready (water::spawn_water_sources).
    commands.trigger(GenerateTerrainNavmesh);
}

fn check_navgendata(
    navgendata:     Res<NavmeshGenerationData>,
    mut commands:   Commands,
    nav_colliders:  Query<Entity, Or<(With<TerrainChunk>, With<NavStatic>)>>,
    water_sources:  Query<Entity, Or<(With<WaterNavmeshSource>, With<UnderwaterCeiling>)>>,
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
        commands.trigger(NavmeshGenerationFinished { name: navgendata.name.clone(), plane_entity: navgendata.plane_entity });

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

/// On a scene object while navmeshes are generated: how it takes part (inserted with its
/// collider by terrain::prepare_terrain_colliders, removed afterwards).
#[derive(Component, Clone, Copy, Debug)]
pub struct NavStatic {
    pub shape:   NavShape,
    /// Part of the terrain navmesh's geometry (Recast decides walkable vs blocking).
    pub terrain: bool,
    /// Blocks boats where it comes down near the waterline.
    pub water:   bool,
}

#[derive(Component)]
pub struct NavmeshWater;

#[derive(Component)]
pub struct NavmeshTerrain;

/// Sent when every navmesh requested by a [`GenerateNavMesh`] has been built and spawned
/// (as `PGNavmesh` entities named `name`).
#[derive(Event, Clone, Debug)]
pub struct NavmeshGenerationFinished {
    pub name: String,
    pub plane_entity: Entity,
}

#[derive(Event)]
pub struct GenerateNavMesh {
    pub plane_entity: Entity,
    pub name: String,
    pub terrain_settings: Option<NavmeshSettings>,
    /// None = no water navmesh.
    pub water_settings: Option<NavmeshSettings>,
    /// Minimum water depth for boats (world units): shallower water counts as land.
    pub water_min_depth: f32,
    /// Leave ground under water deeper than this out of the terrain navmesh (people can wade
    /// up to this depth). None = keep all underwater ground walkable.
    pub terrain_max_water_depth: Option<f32>,
}

// Resource existing only during navmesh creation. Removed after finish
#[derive(Resource, Debug)]
pub (crate) struct NavmeshGenerationData {
    recast_handles: HashMap<PGNavmeshType, (Option<Handle<Navmesh>>, bool)>,
    name: String,
    pub(crate) plane_entity: Entity,
    pub(crate) terrain_settings: Option<NavmeshSettings>,
    pub(crate) water_settings: Option<NavmeshSettings>,
    pub(crate) water_min_depth: f32,
    pub(crate) terrain_max_water_depth: Option<f32>,
    /// Heights of the water surfaces used for the water navmesh (see water::spawn_water_sources).
    pub(crate) water_heights: Vec<f32>,
    /// World XZ bounds (min, max) of the plane the navmeshes are for. Set by
    /// terrain::prepare_terrain_colliders; the navmeshes are built to exactly these edges.
    pub(crate) tile: Option<(Vec2, Vec2)>,
    /// Lowest and highest terrain (plane and neighbours), for the build box.
    pub(crate) height_range: (f32, f32),
    /// Neighbouring terrain planes given colliders for this generation (see [`build_box`]).
    pub(crate) neighbours: Vec<Entity>,
}

/// Width (world units) of the border rerecast leaves out around its build box:
/// walkable radius + 3 cells, as rerecast's ConfigBuilder computes it.
pub(crate) fn border_width(settings: &NavmeshSettings) -> f32 {
    let cell = settings.agent_radius / settings.cell_size_fraction.max(1e-3);
    ((settings.agent_radius / cell.max(1e-6)).ceil() + 3.0) * cell
}

/// The box Recast builds in: the plane's tile grown by exactly the border rerecast leaves out.
///
/// Recast drops that border and shrinks walkable areas by the agent radius next to the box's
/// edges. With the neighbouring planes' terrain inside the border, all of that happens outside
/// the tile, so the navmesh reaches the tile's edges exactly, on the same cell grid as the
/// neighbour's navmesh: the two meet along the seam and get connected (multi_nav).
pub(crate) fn build_box(tile: (Vec2, Vec2), height_range: (f32, f32), settings: &NavmeshSettings) -> bevy::shape::Aabb3d {
    let border = border_width(settings);
    let margin = settings.agent_height + 1.0;
    bevy::shape::Aabb3d {
        min: Vec3::new(tile.0.x - border, height_range.0 - margin, tile.0.y - border).into(),
        max: Vec3::new(tile.1.x + border, height_range.1 + margin, tile.1.y + border).into(),
    }
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

    /// This navmesh type won't be generated (e.g. no navigable water): stop waiting for it.
    pub(crate) fn skip(&mut self, typ: PGNavmeshType) {
        self.recast_handles.remove(&typ);
    }


    pub(crate) fn from_event(event: &GenerateNavMesh) -> Self {
        let mut recast_handles = HashMap::from([(PGNavmeshType::Terrain, (None, false))]);
        if event.water_settings.is_some() {
            recast_handles.insert(PGNavmeshType::Water, (None, false));
        }
        let ngd = NavmeshGenerationData {
            recast_handles,
            name: event.name.clone(),
            plane_entity: event.plane_entity,
            terrain_settings: event.terrain_settings.clone(),
            water_settings: event.water_settings.clone(),
            water_min_depth: event.water_min_depth.max(0.0),
            terrain_max_water_depth: event.terrain_max_water_depth.map(|d| d.max(0.0)),
            water_heights: Vec::new(),
            tile: None,
            height_range: (0.0, 0.0),
            neighbours: Vec::new(),
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
    let water_heights = navgendata.water_heights.clone();

    for (navmesh_type, ref mut nav_state) in navgendata.recast_handles.iter_mut(){
        if nav_state.1 {continue};
        let Some(ref navmesh_handle) = nav_state.0  else {continue};
        if navmesh_handle.id() != trigger.0 {continue};

        let mut pgn: PGNavmesh = convert_rerecast(
            recast_navmesh,
            navmesh_type
        );
        pgn.name = navname.clone();

        // Recast quantises heights to its voxel cell height (derived from agent_height), which
        // can put the surface a fraction of a unit above the water. Water is flat: snap exactly.
        if *navmesh_type == PGNavmeshType::Water && !water_heights.is_empty() {
            snap_to_heights(&mut pgn, &water_heights);
        }

        // pgn.cleanup_lower();
        pgn.reorder_vertex_polygons();
        pgn.islands_removal();
        // Island removal can leave nothing (e.g. only a tiny patch of deep water). An empty
        // navmesh would be saved with infinite bounds (null in JSON) and fail to load.
        if pgn.polygons.is_empty() {
            info!("{navmesh_type:?} navmesh for {navname} is empty after island removal; not spawned");
            nav_state.1 = true;
            continue;
        }
        pgn.bake();
        
        commands.spawn(pgn.clone());

        // info!("Serializing navmesh to {}", pgn.filename());
        // IoTaskPool::get().spawn(async move {
        //     let f = File::create(&pgn.filename()).ok().unwrap();
        //     let mut writer = BufWriter::new(f);
        //     let _res = serde_json::to_writer(&mut writer, &pgn);
        //     let _res = writer.flush();
        // })
        // .detach();
        
        nav_state.1 = true;
    }
}

/// Moves every vertex to the closest of `heights`.
fn snap_to_heights(pgn: &mut PGNavmesh, heights: &[f32]) {
    for vertex in pgn.vertices.values_mut() {
        if let Some(h) = heights.iter().min_by(|a, b| (*a - vertex.loc.y).abs().total_cmp(&(*b - vertex.loc.y).abs())) {
            vertex.loc.y = *h;
        }
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


// fn toggle_visibility(
//     navres:       Res<NavResources>,
//     mut query:    Query<(Entity, &mut Visibility), With<DetailNavmeshGizmo>>
// ){

//     if navres.visible {
//         for (_entity, mut vis) in query.iter_mut(){
//             *vis = Visibility::Inherited;
//         }
//     } else {
//         for (_entity, mut vis) in query.iter_mut(){
//             *vis = Visibility::Hidden;
//         }
//     }
// }
