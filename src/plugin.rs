
use bevy::tasks::IoTaskPool;
use std::fs::File;
use std::io::{BufWriter, Write};
use bevy_common_assets::json::JsonAssetPlugin;
use bevy::prelude::*;
use bevy::platform::collections::HashMap;
use bevy_rerecast::prelude::*;
use avian_rerecast::AvianBackendPlugin;
use bevy_pg_core::prelude::GameState;

use crate::water::PGWaterNavPlugin;
use crate::terrain::PGTerrainNavPlugin;
use crate::recast_convert::convert_rerecast;
use crate::pgnavmesh::{PGNavmesh, PGNavmeshType};

pub struct PGNavPlugin;

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
        .insert_resource(RecastNavmeshHandles::default())
        .add_observer(on_ready_navmesh)
        .add_observer(on_spawn_navmesh)
        .add_systems(OnExit(GameState::Play), clear)
        .add_systems(Update, wait_for_colliders.run_if(resource_exists::<WaitColliders>))
        ;
    }
}

#[derive(Resource)]
pub (crate) struct WaitColliders {
    pub (crate) entity: Entity,
    pub (crate) timer: Timer
}

#[derive(Event)]
pub (crate) struct CollidersReady {
    pub (crate) entity: Entity
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

// #[derive(Debug, PartialEq, Copy, Clone)]
// pub enum NavStaticType {
//     Navigable(f32), // Yoffset
//     Blocker
// }

// #[derive(Component, Clone, Copy)]
// pub struct NavStatic {
//     pub typ:    NavStaticType,
//     pub shape:  NavStaticShape
// }
// impl NavStatic {
//     pub fn navigable_rect(
//         x: f32, 
//         y: f32,
//         y_offset: f32
//     ) -> Self {
//         NavStatic {
//             typ: NavStaticType::Navigable(y_offset),
//             shape: NavStaticShape::rect(Vec2::new(x,y))
//         }
//     }

//     pub fn navigable_circle(
//         radius : f32,
//         y_offset: f32
//     ) -> Self {
//         NavStatic{
//             typ: NavStaticType::Navigable(y_offset),
//             shape: NavStaticShape::circle(radius)
//         }
//     }

//     pub fn blocker_rect(
//         x: f32, y: f32
//     ) -> Self {
//         NavStatic{
//             typ: NavStaticType::Blocker,
//             shape: NavStaticShape::rect(Vec2::new(x,y))
//         }
//     }

//     pub fn blocker_circle(
//         radius : f32
//     ) -> Self {
//         NavStatic{
//             typ: NavStaticType::Blocker,
//             shape: NavStaticShape::circle(radius)
//         }
//     }

// }

// #[derive(Debug, Clone, Copy)]
// pub enum NavStaticShape {
//     Circle(f32), // radius
//     Rect(Vec2)  // Dimensions
// }

// impl NavStaticShape {
//     pub fn circle( 
//         radius: f32
//     ) -> Self {
//         NavStaticShape::Circle(radius)
//     }
//     pub fn rect(
//         dims: Vec2
//     ) -> Self {
//         NavStaticShape::Rect(dims)
//     }
// }

#[derive(Component)]
pub struct NavmeshWater;

#[derive(Component)]
pub struct NavmeshTerrain;

#[derive(Event)]
pub struct GenerateNavMesh {
    pub plane_entity: Entity
}

#[derive(Resource, Debug)]
pub (crate) struct RecastNavmeshHandles {
    pub(crate) data: HashMap<PGNavmeshType, Option<Handle<Navmesh>>>,
    pub(crate) name: String // needs to be passed here
}
impl Default for RecastNavmeshHandles {
    fn default() -> Self {
        RecastNavmeshHandles {
            data: HashMap::from(
                [
                    (PGNavmeshType::Terrain, None),
                    (PGNavmeshType::Water, None)
                ]
            ),
            name: "test".to_string()
        }
    }
}


fn on_ready_navmesh(
    trigger:             On<NavmeshReady>,
    mut commands:        Commands,
    ass_nav:             Res<Assets<Navmesh>>,
    navmesh_handles:     Res<RecastNavmeshHandles>,
){

    let Some(recast_navmesh) = ass_nav.get(trigger.0) else {return;};
    info!("On ready navmesh...");

    for (navmesh_type, maybe_navmesh_handle) in navmesh_handles.data.iter(){
        if let Some(navmesh_handle) = maybe_navmesh_handle {
            if navmesh_handle.id() == trigger.0 {
                
                let mut pgn: PGNavmesh = convert_rerecast(
                    recast_navmesh,
                    navmesh_type
                );

                pgn.name = navmesh_handles.name.clone();

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
