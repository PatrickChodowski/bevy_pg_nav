use bevy::prelude::*;
use bevy::platform::collections::{HashSet};
use std::collections::VecDeque;

use crate::prelude::{PGNavmesh, PGNavmeshType};
use crate::prelude::Path;
use crate::types::PGPolygon;


pub (crate) struct PGMultiNavPlugin;

impl Plugin for PGMultiNavPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_message::<ConnectNavs>()
        .add_observer(on_spawn_nav)
        .add_observer(on_despawn_nav)
        .add_systems(Update, connect_navs.run_if(on_message::<ConnectNavs>))
        ;
    }
}

fn on_spawn_nav(
    trigger:    On<Add, PGNavmesh>,
    navs:       Query<(Entity, &PGNavmesh)>,
    mut writer: MessageWriter<ConnectNavs>,
){

    let Ok((a_entity, a_nav)) = navs.get(trigger.entity) else {return};

    for (entity, nav) in navs.iter(){

        if entity == a_entity {
            continue;
        }

        let mut poly_pairs: Vec<(u32, u32)> = Vec::new();
        for (a_poly_index, a_poly) in a_nav.polygons.iter(){
            for (poly_index, poly) in nav.polygons.iter(){
                if a_poly.intersects(poly, a_nav, nav){
                    poly_pairs.push((*a_poly_index, *poly_index));
                }
            }
        }

        if poly_pairs.len() > 0 {
            writer.write(ConnectNavs{
                nav_entity1: entity,
                nav_entity2: entity,
                polygon_pairs: poly_pairs
            });
        }
    }

}

fn on_despawn_nav(
    trigger:    On<Remove, PGNavmesh>,
    mut navs:   Query<(Entity, &mut PGNavmesh)>
){
    for (entity, mut nav) in navs.iter_mut(){
        if entity == trigger.entity {
            continue;
        }
        if nav.conns.contains_key(&trigger.entity) {
           nav.conns.remove(&trigger.entity);
        }
    }
}

fn connect_navs(
    mut reader: MessageReader<ConnectNavs>,
    mut navs:   Query<&mut PGNavmesh>,
){
    for msg in reader.read(){
        let Ok([mut nav1, mut nav2]) = navs.get_many_mut([msg.nav_entity1, msg.nav_entity2]) else { continue };
        let reversed: Vec<(u32, u32)> = msg.polygon_pairs.iter().map(|&(a, b)| (b, a)).collect();
        nav1.conns.insert(msg.nav_entity2, msg.polygon_pairs.clone());
        nav2.conns.insert(msg.nav_entity1, reversed);
    }
}


// Helpers for managing search paths when there is multiple navigation meshes, types, etc. (Navigation mesh per tile.)

// Current usages:

// clamp_move()

/* 
    let move_direction = Vec2::new(trigger.value.x, -trigger.value.y).normalize();
    let move_data = move_direction*player_movement.speed*time.delta_secs();
    let potential_new_pos = player_transform.translation.xz() + move_data;

    let mut maybe_nav: Option<&PGNavmesh> = None;
    for nav in navs.iter(){
        if nav.typ == player_movement.navmesh_type {
            maybe_nav = Some(nav);
            break;
        }
    }

    let Some(nav) = maybe_nav else {return};
    if player_movement.last_polygon.is_none(){
        if let Some((poly, _loc)) = nav.has_point(&player_transform.translation.xz()){
            player_movement.last_polygon = Some(poly.index);
        }
    }

    let Some(last_polygon) = player_movement.last_polygon else {panic!("dupa")};
    let (new_pos, new_poly) = nav.clamp_move(potential_new_pos, last_polygon);
    player_movement.last_polygon = Some(new_poly);

    // // Need new move data if the position was adjusted
    let new_move_data = new_pos.xz() - player_transform.translation.xz();

    // Look to based if we are in combat
    let look_to = if let Some(combat_look_to) = maybe_combat_look_to {
        combat_look_to - player_transform.translation.xz()
    } else {
        new_move_data
    };

    player_movement.smooth_rotate_move(new_move_data, look_to, &mut player_transform);
    player_transform.translation.y = new_pos.y - 1.75;
    commands.entity(player_entity).insert(Moved{finished: false, last_dir: new_move_data});
*/

// Path Points:
/*
    if let Some(path) = navmesh.path_points(&find_path.from, &find_path.to, movement.agent_radius){
        let move_task = MoveTask::new(path.0, find_path.to, path.1, find_path.limit_distance, find_path.target_entity, find_path.peripheral); 
        commands.entity(entity).remove::<FindPathTask>();    
        commands.entity(entity).insert(move_task);
    }

*/


// Scenarios:
// start and end on the same nav
// start and end on different nav
// start and end on different nav type 
// we may want to find the last point, check navmesh connection

// Does Path need this info? 
/*
pub struct Path {
    pub length: f32,
    pub path: SmallVec<[Vec2;10]>,
}
*/
// Probably not, but its needed during path finding




fn multi_nav_path_points(
    navs:         &Query<(Entity, &PGNavmesh)>,
    start:        &Vec2, 
    end:          &Vec2,
    agent_radius: f32,
    agent_types:  &Vec<PGNavmeshType>
) -> Option<(Path, u32, u32)> {

    // Navmesh Entity, navmesh, polygon, loc
    let mut maybe_start_data: Option<(Entity, &PGNavmesh, &PGPolygon, Vec3)> = None;
    let mut maybe_end_data:  Option<(Entity, &PGNavmesh, &PGPolygon, Vec3)> = None;

    // Get the Navmeshes and polygons for start and end of the path
    for (nav_entity, nav) in navs.iter(){
        if !agent_types.contains(&nav.typ){
            continue;
        }

        if maybe_start_data.is_none(){
            if let Some((poly, loc)) = nav.has_point(start) {
                maybe_start_data = Some((nav_entity, nav, poly, loc));
            }
        }

        if maybe_end_data.is_none(){
            if let Some((poly, loc)) = nav.has_point(end) {
                maybe_end_data = Some((nav_entity, nav, poly, loc));
            }
        }

        if maybe_start_data.is_some() & maybe_end_data.is_some(){
            break;
        }
    }

    // No start_data is a problem
    let Some(start_data) = maybe_start_data else {
        warn!("Couldnt find start data for {}", start);
        return None;
    };

    let Some(end_data) = maybe_end_data else {
        warn!("Couldnt find end data for {}", end);
        return None;
    };


    // Scenario 0: same navmesh for start and end:
    if start_data.0 == end_data.0 {
        return start_data.1.path_between_polygons(
            start, 
            end, 
            start_data.2.index, 
            end_data.2.index, 
            agent_radius
        );
    } else {

        start_data.1.path_multi_nav(
            end_data.1, 
            start, 
            end,
            start_data.0,
            end_data.0,
            &navs
        );



    }


    return None;
}

#[derive(Message)]
struct ConnectNavs {
    nav_entity1: Entity,
    nav_entity2: Entity,
    polygon_pairs: Vec<(u32, u32)> // polygon from entity1 and polygon from entity2
}


// Graph search for connections between navmesh A and navmesh B
pub (crate) fn search_navs_path(
    start: Entity,
    end:   Entity,
    navs:  &Query<(Entity, &PGNavmesh)>
) -> Option<Vec<(Entity, Vec<(u32, u32)>)>> {
    if start == end { return Some(vec![(start, vec![])]); }

    let mut visited: HashSet<Entity> = HashSet::new();
    let mut queue:   VecDeque<(Entity, Vec<(Entity, Vec<(u32, u32)>)>)> = VecDeque::new();

    queue.push_back((start, vec![(start, vec![])]));
    visited.insert(start);

    while let Some((current, path)) = queue.pop_front() {
        let Ok((_nav_entity, nav)) = navs.get(current) else { continue };

        for (neighbour, pairs) in &nav.conns {
            if !visited.insert(*neighbour) { continue; }

            let mut new_path = path.clone();
            new_path.push((*neighbour, pairs.clone()));

            if *neighbour == end {
                return Some(new_path);
            }

            queue.push_back((*neighbour, new_path));
        }
    }

    None
}
