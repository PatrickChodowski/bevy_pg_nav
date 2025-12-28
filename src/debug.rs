use bevy::color::palettes::css::WHITE_SMOKE;
use bevy::prelude::*;

use crate::plugin::{NavConfig, PGNavmeshType};
use crate::types::PGNavmesh;


pub struct PGNavDebugPlugin;

impl Plugin for PGNavDebugPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_systems(Update, display)
        ;
    }
}



fn display(
    navconfig:  Res<NavConfig>,
    navmeshes:  Query<&PGNavmesh>,
    mut gizmos: Gizmos
){

    if !navconfig.debug {
        return
    }

    for pgn in navmeshes.iter(){

        if pgn.typ != PGNavmeshType::Terrain {
            continue;
        }

        for (_polygon_id, polygon) in pgn.polygons.iter(){
            let [a,b,c] = polygon.locs(pgn);
            gizmos.linestrip([a,b,c, a], Color::from(WHITE_SMOKE));
        }
    }
}
