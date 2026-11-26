use bevy::prelude::*;
use bevy::platform::collections::{HashSet};
use std::collections::VecDeque;

use crate::pgnavmesh::{PGNavmesh, PGNavmeshType, closest_point_on_triangle_2d};
use crate::pathfinding::Path;
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
    trigger:    On<Add<PGNavmesh>>,
    navs:       Query<(Entity, &PGNavmesh)>,
    mut writer: MessageWriter<ConnectNavs>,
){

    let Ok((a_entity, a_nav)) = navs.get(trigger.entity) else {return};

    for (entity, nav) in navs.iter(){

        // Only neighbouring navmeshes of the same kind (terrain-terrain across map chunks,
        // water-water). Terrain and water overlap in the shallows but mustn't be joined.
        if entity == a_entity || nav.typ != a_nav.typ {
            continue;
        }

        let poly_pairs = polygon_pairs(a_nav, nav);

        if poly_pairs.len() > 0 {
            // polygon_pairs are (a_nav polygon, nav polygon): entity1 must be a_entity.
            writer.write(ConnectNavs{
                nav_entity1: a_entity,
                nav_entity2: entity,
                polygon_pairs: poly_pairs
            });
        }
    }

}

/// How far apart (world units) neighbouring navmeshes' triangles may be and still connect. The
/// navmeshes of neighbouring map tiles don't always meet: the bake can lose a sample cell along a
/// tile's edge (1.33 for water, 0.6 for terrain), leaving a strip of nothing between them that
/// no boat or walker would cross. Within this, they connect, and a move across jumps the strip
/// (multi_nav_clamp_move).
pub const SEAM_GAP: f32 = 1.5;

/// XZ bounds of a navmesh triangle.
fn triangle_bounds(poly: &PGPolygon, nav: &PGNavmesh) -> (Vec2, Vec2) {
    let [a, b, c] = poly.locs(nav);
    (a.xz().min(b.xz()).min(c.xz()), a.xz().max(b.xz()).max(c.xz()))
}

/// Pairs (polygon of `a`, polygon of `b`) whose triangles overlap or touch: where the two
/// navmeshes connect. Only triangles in the strip where the navmeshes' bounds overlap can
/// touch, and those are bucketed on a grid, so neighbouring map tiles with tens of thousands
/// of polygons each cost only their seam, not every pair (which froze the game).
pub(crate) fn polygon_pairs(a: &PGNavmesh, b: &PGNavmesh) -> Vec<(u32, u32)> {
    // Touching, or within the seam gap.
    const EPS: f32 = SEAM_GAP;
    let bounds = |nav: &PGNavmesh| -> Vec<(u32, Vec2, Vec2)> {
        nav.polygons.iter().map(|(id, p)| { let (lo, hi) = triangle_bounds(p, nav); (*id, lo, hi) }).collect()
    };
    let (ba, bb) = (bounds(a), bounds(b));
    let extent = |v: &[(u32, Vec2, Vec2)]| v.iter().fold((Vec2::MAX, Vec2::MIN), |(lo, hi), (_, l, h)| (lo.min(*l), hi.max(*h)));
    let ((alo, ahi), (blo, bhi)) = (extent(&ba), extent(&bb));
    let (lo, hi) = (alo.max(blo) - EPS, ahi.min(bhi) + EPS);
    if lo.x > hi.x || lo.y > hi.y {
        return Vec::new(); // the navmeshes don't touch
    }
    let touches = |l: Vec2, h: Vec2, lo: Vec2, hi: Vec2| l.x <= hi.x && h.x >= lo.x && l.y <= hi.y && h.y >= lo.y;
    let ca: Vec<_> = ba.into_iter().filter(|(_, l, h)| touches(*l, *h, lo, hi)).collect();
    let cb: Vec<_> = bb.into_iter().filter(|(_, l, h)| touches(*l, *h, lo, hi)).collect();
    if ca.is_empty() || cb.is_empty() {
        return Vec::new();
    }

    // Grid over the overlap, cells about the size of a candidate triangle.
    let mean = cb.iter().map(|(_, l, h)| (*h - *l).max_element()).sum::<f32>() / cb.len() as f32;
    let cell = mean.max(0.05);
    let size = hi - lo;
    let (cols, rows) = (((size.x / cell).ceil() as usize).clamp(1, 4096), ((size.y / cell).ceil() as usize).clamp(1, 4096));
    let to_cell = |p: Vec2| {
        let g = ((p - lo) / size.max(Vec2::splat(1e-6)) * Vec2::new(cols as f32, rows as f32)).floor();
        (g.x.clamp(0.0, (cols - 1) as f32) as usize, g.y.clamp(0.0, (rows - 1) as f32) as usize)
    };
    let mut grid: Vec<Vec<usize>> = vec![Vec::new(); cols * rows];
    for (k, (_, l, h)) in cb.iter().enumerate() {
        let ((c0, r0), (c1, r1)) = (to_cell(*l - EPS), to_cell(*h + EPS));
        for r in r0..=r1 {
            for c in c0..=c1 {
                grid[r * cols + c].push(k);
            }
        }
    }

    let mut pairs = Vec::new();
    let mut seen = vec![usize::MAX; cb.len()];
    for (i, (a_id, l, h)) in ca.iter().enumerate() {
        let a_poly = &a.polygons[a_id];
        let ((c0, r0), (c1, r1)) = (to_cell(*l - EPS), to_cell(*h + EPS));
        for r in r0..=r1 {
            for c in c0..=c1 {
                for &k in &grid[r * cols + c] {
                    if seen[k] == i {
                        continue; // already tested from another cell
                    }
                    seen[k] = i;
                    let (b_id, bl, bh) = &cb[k];
                    if touches(*l - EPS, *h + EPS, *bl, *bh) && a_poly.touches(&b.polygons[b_id], a, b, EPS) {
                        pairs.push((*a_id, *b_id));
                    }
                }
            }
        }
    }
    pairs
}

fn on_despawn_nav(
    trigger:    On<Remove<PGNavmesh>>,
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


pub fn multi_nav_clamp_move(
    current_nav_entity: Entity,
    current_polygon_id: u32,
    target:             Vec2,
    navs:               &Query<(Entity, &PGNavmesh)>,
) -> (Entity, Vec3, u32) {

    let Ok((_nav_entity, current_nav)) = navs.get(current_nav_entity) else {panic!("Missing navmesh")};
    let current_poly = current_nav.polygon(&current_polygon_id);

    // If same polygon
    if let Some(pos) = current_poly.has_point(target, &current_nav){
        return (current_nav_entity, pos, current_polygon_id)
    };

    let mut candidates: Vec<(Entity, &PGNavmesh, u32)> = Vec::new();
    
    // Check neighbouring polygons from the same navmesh
    for n in current_poly.neighbours.iter(){
        candidates.push((current_nav_entity, current_nav, *n));
    }

    // Check if this polygon is in the navmesh connections to other navmesh
    for (other_nav_entity, poly_pairs) in current_nav.conns.iter(){
        let Ok((_entity, other_nav)) = navs.get(*other_nav_entity) else {continue};
        for poly_pair in poly_pairs.iter(){
            if poly_pair.0 == current_polygon_id {
                candidates.push((*other_nav_entity, other_nav, poly_pair.1));
            }
        }
    }

    let mut closest_point: Vec2 = target;
    let mut closest_point_height: f32 = 0.0;
    let mut min_dist_sq: f32 = f32::MAX;
    let mut closest_poly_index: u32 = current_polygon_id;
    let mut closest_point_navmesh_entity = current_nav_entity;

    // Search for the nearest point across all candidates. Another navmesh's polygon counts as
    // the seam gap nearer (SEAM_GAP): a move into the strip between two tiles' navmeshes goes
    // on across it (ties keep the current navmesh: candidates of this one come first).
    for (candidate_nav_entity, candidate_nav, candidate_polygon_id) in &candidates {
        let candidate_polygon = candidate_nav.polygon(&candidate_polygon_id);
        let [a, b, c] = candidate_polygon.locs_2d(candidate_nav);
        let clamped_2d = closest_point_on_triangle_2d(target, a,b,c);
        let mut dist = clamped_2d.distance(target);
        if *candidate_nav_entity != current_nav_entity {
            dist = (dist - SEAM_GAP).max(0.0);
        }
        let dist_sq = dist * dist;
        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
            closest_point = clamped_2d;
            closest_poly_index = *candidate_polygon_id;
            closest_point_height = candidate_polygon.get_height(closest_point, &candidate_nav);
            closest_point_navmesh_entity = *candidate_nav_entity;
        }
    }

    return (
        closest_point_navmesh_entity, 
        Vec3::new(closest_point.x, closest_point_height, closest_point.y), 
        closest_poly_index
    );
}


pub fn multi_nav_path_points(
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

    // Different navmeshes needing connections
        return start_data.1.path_multi_nav(
            end_data.1, 
            start, 
            end,
            start_data.0,
            end_data.0,
            &navs,
            agent_radius,
            agent_types
        );

    }

}

#[derive(Message)]
struct ConnectNavs {
    nav_entity1: Entity,
    nav_entity2: Entity,
    polygon_pairs: Vec<(u32, u32)> // polygon from entity1 and polygon from entity2
}


// Graph search for connections between navmesh A and navmesh B
pub (crate) fn search_navs_path(
    start:       Entity,
    end:         Entity,
    navs:        &Query<(Entity, &PGNavmesh)>,
    agent_types: &Vec<PGNavmeshType>
) -> Option<Vec<(Entity, Vec<(u32, u32)>)>> {
    if start == end { return Some(vec![(start, vec![])]); }

    let mut visited: HashSet<Entity> = HashSet::new();
    let mut queue:   VecDeque<(Entity, Vec<(Entity, Vec<(u32, u32)>)>)> = VecDeque::new();

    queue.push_back((start, vec![(start, vec![])]));
    visited.insert(start);

    while let Some((current, path)) = queue.pop_front() {
        let Ok((_nav_entity, nav)) = navs.get(current) else { continue };

        if !agent_types.contains(&nav.typ){
            continue;
        }

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


#[cfg(test)]
mod pair_tests {
    use super::*;
    use crate::types::PGVertex;
    use bevy::platform::collections::HashSet;

    /// A navmesh of 2 triangles per cell over `cells` x `cells` unit cells starting at `x0`.
    fn grid_nav(x0: f32, cells: u32) -> PGNavmesh {
        let mut nav = PGNavmesh::default();
        let n = cells + 1;
        for j in 0..n {
            for i in 0..n {
                let id = j * n + i;
                nav.vertices.insert(id, PGVertex { index: id, loc: Vec3::new(x0 + i as f32, 0.0, j as f32), polygons: vec![] });
            }
        }
        let mut id = 0;
        for j in 0..cells {
            for i in 0..cells {
                let v = j * n + i;
                for tri in [[v, v + n, v + 1], [v + 1, v + n, v + n + 1]] {
                    nav.polygons.insert(id, PGPolygon { index: id, vertices: tri.to_vec(), neighbours: HashSet::new() });
                    id += 1;
                }
            }
        }
        nav
    }

    fn brute(a: &PGNavmesh, b: &PGNavmesh) -> Vec<(u32, u32)> {
        let mut out = vec![];
        for (ia, pa) in &a.polygons {
            for (ib, pb) in &b.polygons {
                if pa.touches(pb, a, b, SEAM_GAP) { out.push((*ia, *ib)); }
            }
        }
        out.sort();
        out
    }

    #[test]
    fn seam_pairs_match_the_brute_force_ones() {
        let (a, b) = (grid_nav(0.0, 6), grid_nav(6.0, 6)); // edge to edge at x = 6
        let mut fast = polygon_pairs(&a, &b);
        fast.sort();
        let slow = brute(&a, &b);
        assert!(!slow.is_empty());
        assert_eq!(fast, slow);
    }

    #[test]
    fn a_gap_along_the_seam_still_connects() {
        let a = grid_nav(0.0, 4);
        assert!(!polygon_pairs(&a, &grid_nav(4.00001, 4)).is_empty(), "rounding");
        // The bake's lost cell along a tile edge (water: 1.33).
        assert!(!polygon_pairs(&a, &grid_nav(4.0 + 1.33, 4)).is_empty(), "a lost sample cell");
        assert!(polygon_pairs(&a, &grid_nav(4.0 + 2.0, 4)).is_empty(), "a real gap doesn't");
    }

    /// West of Bergen (north_europe_11_9 into the open sea of 10_9): 11_9's water navmesh stops
    /// 1.33 short of the tile edge at x = 4400, where 10_9's starts. A boat at 11_9's edge
    /// steering west goes on into 10_9, and back. (Skipped without the game's maps.)
    #[test]
    fn a_boat_crosses_the_seam_west_of_bergen() {
        use bevy::ecs::system::RunSystemOnce;
        let load = |name: &str| -> Option<PGNavmesh> {
            let path = format!("../doomhalla/assets/game_map/{name}/navmesh/{name}_water.nav.json");
            serde_json::from_str(&std::fs::read_to_string(path).ok()?).ok()
        };
        let (Some(mut east), Some(mut west)) = (load("north_europe_11_9"), load("north_europe_10_9")) else { return };
        let mut world = World::new();
        let (e, w) = (world.spawn_empty().id(), world.spawn_empty().id());
        let pairs = polygon_pairs(&east, &west);
        assert!(!pairs.is_empty(), "the tiles connect");
        east.conns.insert(w, pairs.clone());
        west.conns.insert(e, pairs.iter().map(|&(a, b)| (b, a)).collect());
        world.entity_mut(e).insert(east);
        world.entity_mut(w).insert(west);
        world.run_system_once(move |navs: Query<(Entity, &PGNavmesh)>| {
            let (_, east) = navs.get(e).unwrap();
            let start = Vec2::new(4401.8, 3875.0);
            let (poly, _) = east.has_point(&start).expect("on 11_9's water");
            // A few steps west (a boat moves ~0.3 a frame).
            let (mut nav, mut at, mut poly) = (e, start, poly.index);
            for _ in 0..12 {
                let (n, p, pid) = multi_nav_clamp_move(nav, poly, at - Vec2::new(0.3, 0.0), &navs);
                (nav, at, poly) = (n, p.xz(), pid);
            }
            assert_eq!(nav, w, "crossed into 10_9, at {at}");
            assert!(at.x < 4399.0, "and on west: {at}");
            // And back east.
            for _ in 0..12 {
                let (n, p, pid) = multi_nav_clamp_move(nav, poly, at + Vec2::new(0.3, 0.0), &navs);
                (nav, at, poly) = (n, p.xz(), pid);
            }
            assert_eq!(nav, e, "back into 11_9, at {at}");
        }).unwrap();
    }

    #[test]
    fn far_apart_navmeshes_have_no_pairs() {
        assert!(polygon_pairs(&grid_nav(0.0, 3), &grid_nav(10.0, 3)).is_empty());
    }
}



