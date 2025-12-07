
use bevy::{prelude::*, camera::primitives::Aabb};
use bevy::platform::collections::{HashSet, HashMap};
use core::f32;
use std::f32::EPSILON;
use std::usize;
use rayon::prelude::*;
use dashmap::DashMap;

use crate::tools::NavRay;
use crate::types::{Edge, NavQuad, NavStatic, NavType, Neighbours, QuadAABB, RayTargetMesh, RayTargetMeshShape};
use crate::terrain::TerrainRayMeshData;


const NORMAL_EPSILON_DIFF: f32 = 0.000001;

// Should only be used in merge_by_groups
struct QuadsGroupToMerge {
    group_id: usize,
    aabbs:    Vec<QuadAABB>,
    normals:  Vec<Vec3A>,
    types:    HashSet<NavType>,
    indexes:  HashSet<usize>
}


impl QuadsGroupToMerge {
    fn is_heterogonus(&self) -> bool {

        // Easiest is type check:
        if self.types.len() != 1 {
            return false;
        }

        let typ = self.types.iter().next().unwrap();
        if typ == &NavType::Terrain {
            // Check normals:
            let first = self.normals[0];
            // let normals_eq = self.normals.iter().all(|&v| v == first);//.length_squared() <= NORMAL_EPSILON_DIFF);
            let normals_eq = self.normals.iter().all(|&v| (v - first).length_squared() <= EPSILON);
            if !normals_eq {
                return false;
            } 
        }

        // Maybe check for heights? Dont know
        return true;
    }

    fn merge(&self) -> (usize, NavQuad) {
        let aabb = QuadAABB::merge_many(&self.aabbs);
        let vertices = aabb.corners();
        let new_index = self.indexes.iter().min().unwrap();

        let mut nq = NavQuad {
            group_id: self.group_id,
            loc: aabb.center(),
            index: *new_index,
            normal: self.normals[0],
            typ: *self.types.iter().next().unwrap(),
            aabb,
            vertices,
            neighbours: Neighbours::default()
        };
        nq.sort_vertices();
        return (*new_index, nq);

    } 
}


// Should properly reduce number of NavQuads if we merge them by group_id (terrain face quad), normal, height, type etc.
pub(crate) fn merge_by_groups(
    nav_quads: &mut  DashMap<usize, NavQuad>
){

    info!("[NAVMESH] Number of NavQuds before merging by groups: {}", nav_quads.len());
    // let reference_quads = nav_quads.clone();
    let groups: DashMap<usize, QuadsGroupToMerge> = DashMap::with_capacity(100000);

    // Group NavQuads by group_id:
    nav_quads.par_iter_mut().for_each(|nv_quads_entry| {
        let quad = nv_quads_entry.value();
        if let Some(mut gtm) = groups.get_mut(&quad.group_id){
            gtm.aabbs.push(quad.aabb);
            gtm.types.insert(quad.typ);
            gtm.normals.push(quad.normal);
            gtm.indexes.insert(quad.index);
        } else {
            groups.insert(quad.group_id, QuadsGroupToMerge{
                aabbs: vec![quad.aabb],
                types: HashSet::from([quad.typ]),
                group_id: quad.group_id,
                normals: vec![quad.normal],
                indexes: HashSet::from([quad.index])
            });
        }
    });

    info!("[NAVMESH] Number of groups: {}", groups.len());
    info!("[NAVMESH] Number of heterogonous groups: {}", groups.iter().map(|entry| if entry.value().is_heterogonus() {1} else {0}).sum::<i32>());

    // Check which groups can be merged and merge them
    groups.par_iter_mut().for_each(|group_entry| {

        // info!("checking group entry: {}", group_entry.group_id);
        let gtm = group_entry.value();

        if gtm.is_heterogonus() == false{
            // info!("group {} is NOT heterogonus, returning", group_entry.group_id);
            return;
        }

        let (new_index, new_quad) = gtm.merge();
        for old_index in gtm.indexes.iter(){
            nav_quads.remove(old_index);
        }
        // info!("group {} is heterogonus, merging old indexes ({}) into {}", group_entry.group_id, gtm.indexes.len(), new_index);
        nav_quads.insert(new_index, new_quad);

    });

}


// Function for finding neighbours by AABB edges. Parametrized as it can be used in different scenarios

pub(crate) fn find_neighbours(
    nav_quads: &mut  HashMap<usize, NavQuad>
){

    let mut reference_quads = nav_quads.clone()
                               .iter()
                               .map(|(_quad_id, quad)| quad.clone())
                               .collect::<Vec<NavQuad>>();

    reference_quads.sort_by(|a, b| {a.index.partial_cmp(&b.index).unwrap()});

    for a_quad in reference_quads.iter(){ 
        let (a_edges, a_corners) = a_quad.aabb.edges_corners_2d();
        let quad_ref: &mut NavQuad = nav_quads.get_mut(&a_quad.index).unwrap();
        for b_quad in reference_quads.iter(){
            if a_quad.index == b_quad.index {
                continue;
            }            
            let (b_edges, b_corners) = b_quad.aabb.edges_corners_2d();
            if test_neighbours_quads(
                &a_edges, &b_edges,
                &a_corners, &b_corners
            ) {
                quad_ref.neighbours.insert(b_quad.typ, b_quad.index);
            }
        }
    }
}


pub(crate) fn raycasts_rain(
    xs:             &Vec<f32>,
    zs:             &Vec<f32>,
    ray_target_meshes:     &Vec<RayTargetMesh>,
    trmd:           &TerrainRayMeshData,
    water_height:   f32,
    extent:         f32
) -> DashMap<usize, NavQuad> {

    let raycast_map: DashMap<(usize, usize), NavQuad> = DashMap::new();
    xs.par_iter()
      .enumerate()
      .flat_map(|(x_index, &x)| 
            zs.par_iter()
              .enumerate()
              .map(move |(z_index, &z)| (x_index, x, z_index, z)))
              .for_each(|(x_index, x, z_index, z)|{

        let ray: NavRay = NavRay::down(x as f32, z as f32);

        // Check against the terrain
        if let Some((terrain_height, group_id, normal)) = trmd.test(&ray){
            let mut height: f32 = terrain_height;
            let mut vertex_type = NavType::Terrain;
            let mut quad_normal: Vec3A = normal.into();

            // Check against blockers and navigables
            for rm in ray_target_meshes.iter(){
                if let Some(nvt) = rm.test(&ray){
                    if rm.vertex_height > height {
                        height = rm.vertex_height; // Update, Only if its above the terrain height in that point
                    }
                    vertex_type = nvt;
                    quad_normal = Vec3A::Y;
                    break;
                }
            }
            
            // Check for water
            if height <= water_height { 
                height = water_height;
                vertex_type = NavType::Water;
                quad_normal = Vec3A::Y;
            }

            // Create NavQuad object
            let tile = (x_index, z_index);
            let loc = Vec3A::new(x, height, z);
            let aabb = QuadAABB::from_loc(loc, extent);
            let vertices = aabb.corners();

            let mut quad = NavQuad {
                group_id,
                loc,
                index: 0, // Temporary
                vertices,
                normal: quad_normal,
                typ: vertex_type,
                aabb,
                neighbours: Neighbours::default()
            };
            quad.sort_vertices();
            raycast_map.insert(tile, quad);
        }
    });

    // Sort Quads, apply index on sorted. Create DashMap<usize, NavQuad>
    let mut v_navquads: Vec<NavQuad> = raycast_map.iter().map(|entry| entry.value().clone()).collect::<Vec<NavQuad>>();
    v_navquads.sort_by(|a, b| {
        a.loc.x
            .partial_cmp(&b.loc.x)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(
                a.loc.z
                    .partial_cmp(&b.loc.z)
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
    });

    let nvmap = v_navquads.iter_mut().enumerate().map(|(index, quad)| {quad.index = index; (index, quad.clone())}).collect::<DashMap<usize, NavQuad>>();
    return nvmap;
}


pub(crate) fn get_target_ray_meshes(
    query:     &Query<(&Transform, &Aabb, &NavStatic)>
) -> Vec<RayTargetMesh> {

    let mut ray_meshes: Vec<RayTargetMesh> = Vec::with_capacity(query.iter().len());
    for (transform, aabb, navstatic) in query.iter(){
        let (shape, ray_hit_height, vertex_height) = RayTargetMeshShape::from_navstatic(*navstatic, *transform, aabb);
        let rm = RayTargetMesh{ray_hit_height, vertex_height, shape, typ: navstatic.typ};
        ray_meshes.push(rm);
    }
    ray_meshes.sort();
    info!("[NAVMESH] RayMeshes count: {}", ray_meshes.len());
    return ray_meshes;
}



pub(crate) fn loop_merge_quads_directional(
    nav_quads:      &mut  HashMap<usize, NavQuad>,
    quads_count:    &mut usize,
    limit:          usize
){
    let mut i: usize = 0;
    loop {
        *quads_count = nav_quads.len();
        info!("Merge Quad Directional Iteration: {} Quad Count: {}", i, quads_count);
        merge_quads_directional(nav_quads, i);
        let new_count = nav_quads.len();

        i += 1;
        if i >= limit {
            info!("Merge Quad Directional ITER_COUNT limit reached {} count: {}", i, new_count);
            break;
        }
        
        if *quads_count == new_count{
            info!("Merge Quad Directional Convergence {}", new_count);
            break;
        }
    }

}

fn merge_quads_directional(
    nav_quads: &mut  HashMap<usize, NavQuad>,
    iteration: usize
){
    let mut merge_results: Vec<(usize, usize, NavQuad)> = Vec::with_capacity(20000); // (keep, remove, new)
    let mut v_quads = nav_quads.clone().iter().map(|(_quad_id, quad)| quad.clone()).collect::<Vec<NavQuad>>();
    let mut used_quads: HashSet<usize> = HashSet::with_capacity(20000);
    v_quads.sort_by(|a, b| {
        a.loc.x
            .partial_cmp(&b.loc.x)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(
                a.loc.z
                    .partial_cmp(&b.loc.z)
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
    });

    let edge = match iteration % 2 == 0 {
        true => {Edge::Right}
        false => {Edge::Bottom}
    };

    for a_quad in v_quads.iter(){


        if used_quads.contains(&a_quad.index){
            continue;
        }

        for b_quad in v_quads.iter(){

            if a_quad.index == b_quad.index {
                continue;
            }

            if used_quads.contains(&b_quad.index){
                continue;
            }

            if a_quad.typ != b_quad.typ {
                continue;
            }

            if a_quad.normal != b_quad.normal {
                continue;
            }
            // if a_quad.normal.distance(b_quad.normal) > NORMAL_EPSILON_DIFF {
            //     continue;
            // }

            // if let Some(edge) = a_quad.aabb.get_identical_edge(&b_quad.aabb){
            if let Some(edge) = a_quad.aabb.check_edge(&b_quad.aabb, edge){
                let edge_f32 = [(edge[0].0 as f32, edge[0].1 as f32), (edge[1].0 as f32, edge[1].1 as f32)];
                let merge_result: (usize, usize, NavQuad) = a_quad.merge_by_aabb(&b_quad, edge_f32);
                used_quads.insert(merge_result.0);
                used_quads.insert(merge_result.1);
                merge_results.push((merge_result.0, merge_result.1, merge_result.2));
                break; // Need to do one at a time 
            }
        }
    }

    // Step2: Process merge results
    for entry in merge_results.iter(){
        let (keep_index, remove_index) = (entry.0, entry.1);
        let new_quad = entry.2.clone();
        nav_quads.insert(keep_index, new_quad);
        nav_quads.remove(&remove_index);
    }
}



pub(crate) fn test_neighbours_quads(
    a_edges:   &Vec<[Vec2; 2]>,
    b_edges:   &Vec<[Vec2; 2]>,
    a_corners: &Vec<Vec2>,
    b_corners: &Vec<Vec2>
) -> bool {

    // Check edges first
    for (_edge_index, a) in a_edges.iter().enumerate(){
        for b in b_edges.iter(){
            let li = lines_intersection(a, b);
            if li {
                return true;
            }

        }
    }

    // Check corners next
    for (_corner_index, a) in a_corners.iter().enumerate(){
        for b in b_corners.iter(){
            if a == b {
                return true;
            }
        }
    }
    return false;
}


#[inline(always)]
fn lines_intersection(
    line1: &[Vec2; 2], 
    line2: &[Vec2; 2], 
) -> bool {
    let a0 = line1[0];
    let a1 = line1[1];
    let b0 = line2[0];
    let b1 = line2[1];

    let d1 = a1 - a0;
    let d2 = b1 - b0;
    let det = _cross(d1, d2);
    
    if det.abs() < f32::EPSILON {
        return false; // Parallel lines
    }

    let t = _cross(b0 - a0, d2) / det;
    let u = _cross(b0 - a0, d1) / det;
    
    return t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0;
   
}

#[inline(always)]
fn _cross(v1: Vec2, v2: Vec2) -> f32 {
    v1.x * v2.y - v1.y * v2.x
}
