use bevy::color::palettes::css::WHITE_SMOKE;
use bevy::color::palettes::tailwind::ORANGE_500;
use bevy::platform::collections::HashSet;
use bevy::prelude::*;
use bevy_pg_core::prelude::{GameState, PointerData};

use crate::plugin::{NavConfig, PGNavmeshType};
use crate::types::PGNavmesh;


pub struct PGNavDebugPlugin;

impl Plugin for PGNavDebugPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_systems(OnEnter(GameState::Play), init)
        .add_systems(Update, display_pointer)
        .add_systems(Update, display_all)
        ;
    }
}

#[derive(Component)]
struct PGNavDebugText;


fn init(
    mut commands: Commands
){
    commands.spawn((
        Node {
            left: percent(50.0),
            top: percent(0.0),
            display: Display::Flex,
            width: percent(50.0),
            height: percent(100.0),
            ..default()
        },
        PGNavDebugText,
        Text::new(""),
        TextFont{
            font_size: 12.0,
            ..default()
        },
        Pickable::IGNORE
    ));
}

fn display_all(
    navconfig:  Res<NavConfig>,
    navmeshes:  Query<&PGNavmesh>,
    mut gizmos: Gizmos,
){
    if !navconfig.debug {
        return
    }

    for pgn in navmeshes.iter(){
        if pgn.typ != PGNavmeshType::Water {
            continue;
        }
        for (polygon_id, polygon) in pgn.polygons.iter(){
            let mut clr = Color::from(WHITE_SMOKE).with_alpha(0.2);
            let [a,b,c] = polygon.locs(pgn);
            gizmos.linestrip([a,b,c,a], clr);
            for vertex_id in polygon.vertices.iter(){
                let vertex = pgn.vertex(vertex_id).unwrap();
                gizmos.sphere(Isometry3d::from_translation(vertex.loc), 3.0, clr);
            }
        }
    }
}



fn display_pointer(
    navconfig:  Res<NavConfig>,
    navmeshes:  Query<&PGNavmesh>,
    mut gizmos: Gizmos,
    pointer:    Res<PointerData>,
    mut text:   Single<&mut Text, With<PGNavDebugText>>
){

    if !navconfig.debug {
        return
    }

    for pgn in navmeshes.iter(){

        if pgn.typ != PGNavmeshType::Water {
            continue;
        }

        let mut pointer_polygon_id: Option<usize> = None;
        if let Some(world_pos) = pointer.world_pos {
            if let Some(pointer_polygon) = pgn.has_point(world_pos.xz()){
                pointer_polygon_id = Some(pointer_polygon.index);
            }
        }

        let mut display_neighbours: HashSet<usize> = HashSet::new();
        for (polygon_id, polygon) in pgn.polygons.iter(){
            let mut clr = Color::from(WHITE_SMOKE).with_alpha(0.2);

            if let Some(pointer_polygon_id) = pointer_polygon_id {
                if pointer_polygon_id == *polygon_id {
                    clr = Color::from(ORANGE_500);

                    text.0 = format!("{:?}", polygon);
                    for pv in polygon.vertices.iter(){
                        text.0 += &format!(" \n {:?}", pgn.vertex(pv).unwrap());
                    }

                    let [a,b,c] = polygon.locs(pgn);
                    gizmos.linestrip([a,b,c,a], clr);
                    for vertex_id in polygon.vertices.iter(){
                        let vertex = pgn.vertex(vertex_id).unwrap();
                        gizmos.sphere(Isometry3d::from_translation(vertex.loc), 3.0, clr);
                    }

                    display_neighbours = polygon.neighbours.clone();
                    break;
                }
            }
        }

        for npoly in display_neighbours.iter(){
            let [a,b,c] = pgn.polygon(npoly).unwrap().locs(pgn);
            gizmos.linestrip([a,b,c,a], Color::from(WHITE_SMOKE).with_alpha(0.2));
        }

    }
}
