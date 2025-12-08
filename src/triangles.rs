use bevy::prelude::*;
use bevy::platform::collections::{HashMap, HashSet};
use dashmap::DashMap;
use geo::{Centroid, ConvexHull, CoordsIter};
use rayon::prelude::*;
use std::f32::EPSILON;

use crate::tools::NavRay;
use crate::types::{Edge, NavStatic, NavType, Neighbours, RayTargetMesh, RayTargetMeshShape, NavQuad, QuadAABB};
use crate::terrain::TerrainRayMeshData;

#[derive(Clone, Debug)]
pub struct NavPolygon{
    pub group_id:     usize, // Terrain quad/Blocker/Navigable ID, Water part ID?
    pub loc:          Vec3A,
    pub index:        usize,
    pub vertices:     Vec<Vec3A>,
    pub normal:       Vec3A,
    pub typ:          NavType,
    pub neighbours:   Neighbours
    
}

pub(crate) fn raycasts_rain(
    xs:             &Vec<f32>,
    zs:             &Vec<f32>,
    ray_target_meshes:     &Vec<RayTargetMesh>,
    trmd:           &TerrainRayMeshData,
    water_height:   f32
) -> DashMap<usize, NavPolygon> {

    let raycast_map: DashMap<(usize, usize), NavPolygon> = DashMap::new();
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
            let mut normal: Vec3A = normal.into();

            // Check against blockers and navigables
            for rm in ray_target_meshes.iter(){
                if let Some(nvt) = rm.test(&ray){
                    if rm.vertex_height > height {
                        height = rm.vertex_height; // Update, Only if its above the terrain height in that point
                    }
                    vertex_type = nvt;
                    normal = Vec3A::Y;
                    break;
                }
            }
            
            // Check for water
            if height <= water_height { 
                height = water_height;
                vertex_type = NavType::Water;
                normal = Vec3A::Y;
            }

            // Create NavQuad object
            let tile = (x_index, z_index);
            let loc = Vec3A::new(x, height, z);

            let quad = NavPolygon {
                group_id,
                loc,
                index: 0, // Temporary
                vertices: vec![loc],
                normal,
                typ: vertex_type,
                neighbours: Neighbours::default()
            };
            raycast_map.insert(tile, quad);
        }
    });

    // Sort Quads, apply index on sorted. Create DashMap<usize, NavPolygon>
    let mut v_navquads: Vec<NavPolygon> = raycast_map.iter().map(|entry| entry.value().clone()).collect::<Vec<NavPolygon>>();
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

    let nvmap = v_navquads.iter_mut().enumerate().map(|(index, quad)| {quad.index = index; (index, quad.clone())}).collect::<DashMap<usize, NavPolygon>>();
    return nvmap;
}




struct PolygonsGroupToMerge {
    vertices: Vec<Vec3A>,
    group_id: usize,
    normals:  Vec<Vec3A>,
    types:    HashSet<NavType>,
    indexes:  HashSet<usize>
}


impl PolygonsGroupToMerge {
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

    // Get only external boundary vertices;
    fn merge(&self) -> (usize, NavPolygon) {
        use geo::{Point, ConvexHull};
        let new_index = self.indexes.iter().min().unwrap();
        let points: Vec<Point<f32>> = self.vertices.iter().map(|v| Point::new(v.x, v.z)).collect();
        let multi_point: geo::MultiPoint<f32> = points.into();
        let convex_hull = multi_point.convex_hull();
        let center = convex_hull.centroid().unwrap();
        let vertices = convex_hull.exterior_coords_iter().map(|x| Vec3A::new(x.x, 200.0, x.y)).collect::<Vec<Vec3A>>();
        let nq = NavPolygon {
            group_id: self.group_id,
            loc: Vec3A::new(center.x(), 200.0, center.y()),
            index: *new_index,
            normal: self.normals[0],
            typ: *self.types.iter().next().unwrap(),
            vertices,
            neighbours: Neighbours::default()
        };
        return (*new_index, nq);

    } 
}



// Should properly reduce number of NavQuads if we merge them by group_id (terrain face quad), normal, height, type etc.
pub(crate) fn merge_by_groups(
    nav_quads: &mut  DashMap<usize, NavPolygon>
){

    info!("[NAVMESH] Number of NavPolygons before merging by groups: {}", nav_quads.len());
    let groups: DashMap<usize, PolygonsGroupToMerge> = DashMap::with_capacity(100000);

    // Group NavQuads by group_id:
    nav_quads.par_iter_mut().for_each(|nv_quads_entry| {
        let quad = nv_quads_entry.value();
        if let Some(mut gtm) = groups.get_mut(&quad.group_id){
            gtm.vertices.extend(quad.vertices.clone());
            gtm.types.insert(quad.typ);
            gtm.normals.push(quad.normal);
            gtm.indexes.insert(quad.index);
        } else {
            groups.insert(quad.group_id, PolygonsGroupToMerge{
                vertices: quad.vertices.clone(),
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
