use bevy::color::palettes::css::WHITE_SMOKE;
use bevy::prelude::*;

use crate::plugin::NavConfig;
use crate::types::{PGNavmesh, PGPolygon, PGVertex};


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
        for (polygon_id, polygon) in pgn.polygons.iter(){
            let [a,b,c] = polygon.locs(pgn);
            gizmos.linestrip([a,b,c], Color::from(WHITE_SMOKE));
        }
    }
}
