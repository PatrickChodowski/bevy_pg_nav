use bevy::prelude::{info, Component, Event, Vec3A};
use bevy::mesh::Mesh;
use bevy::platform::collections::{HashSet, HashMap};
use dashmap::DashMap;
use rayon::prelude::*;

use crate::terrain::TerrainRayMeshData;
use crate::tools::NavRay;


#[derive(Component)]
pub(crate) struct WaterNavmeshSource;

#[derive(Event)]
pub(crate) struct WaterNavmeshesReady;

#[derive(Clone, Copy, Debug)]
pub(crate) struct WaterVertex {
   pub(crate) x_u: usize,
   pub(crate) z_u: usize,
   pub(crate) loc: Vec3A,
   pub(crate) group: usize
}

impl PartialEq for WaterVertex {
    fn eq(&self, other: &Self) -> bool {
        self.x_u == other.x_u && self.z_u == other.z_u
    }
}

impl Eq for WaterVertex {}

impl PartialOrd for WaterVertex {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for WaterVertex {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Sort by x_u first, then by z_u
        match self.x_u.cmp(&other.x_u) {
            std::cmp::Ordering::Equal => self.z_u.cmp(&other.z_u),
            other => other,
        }
    }
}

pub(crate) fn raycasts_rain(
    xs:             &Vec<f32>,
    zs:             &Vec<f32>,
    // ray_target_meshes:     &Vec<RayTargetMesh>,
    trmd:           &TerrainRayMeshData,
    water_height:   f32
) ->  Vec<WaterVertex> {

    let raycast_map: DashMap<(usize, usize), Vec3A> = DashMap::new();
    xs.par_iter()
      .enumerate()
      .flat_map(|(x_index, &x)| 
            zs.par_iter()
              .enumerate()
              .map(move |(z_index, &z)| (x_index, x, z_index, z)))
              .for_each(|(x_index, x, z_index, z)|{

        let ray: NavRay = NavRay::down(x as f32, z as f32);

        // Check against the terrain
        if let Some((terrain_height, _group_id, normal)) = trmd.test(&ray){
            let mut height: f32 = terrain_height;


            // Check against blockers and navigables
            // for rm in ray_target_meshes.iter(){
            //     if let Some(nvt) = rm.test(&ray){
            //         if rm.vertex_height > height {
            //             height = rm.vertex_height; // Update, Only if its above the terrain height in that point
            //         }
            //         vertex_type = nvt;
            //         normal = Vec3A::Y;
            //         break;
            //     }
            // }
            
            // Check for water
            if height <= water_height { 
                height = water_height;
                let tile = (x_index, z_index);
                let loc = Vec3A::new(x, water_height, z);
                raycast_map.insert(tile, loc);
            }
        }
    });

    // return raycast_map.clone().into_iter().map(|(tile, loc)| (tile, loc)).collect();

    let mut v_waters = raycast_map.clone().into_iter().map(|(tile, loc)| WaterVertex{x_u: tile.0, z_u: tile.1, loc: loc, group: 0}).collect::<Vec<WaterVertex>>();
    v_waters.sort_by(|a, b| {
        a.x_u
            .partial_cmp(&b.x_u)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(
                a.z_u
                    .partial_cmp(&b.z_u)
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
    });
    return v_waters;
}



pub(crate) fn group_waters(
    v_waters: &mut Vec<WaterVertex>, 
    maxx: u32, 
    maxz: u32
) -> HashMap<(usize, usize), WaterVertex> {

    let adjs: [(isize, isize); 8] = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1,-1), (1,1), (-1, 1), (1, -1)];
    let max_x = maxx as isize;
    let max_z = maxz as isize;

    let mut group_map: HashMap<(usize, usize), WaterVertex> = v_waters.iter().map(|v| ((v.x_u, v.z_u), *v)).collect::<HashMap<(usize, usize), WaterVertex>>();
    // let mut group: usize = 0;
    let mut global_group: usize = 1;

    for vertex in v_waters.iter(){
        // info!("vertex: ({}, {})", vertex.x_u, vertex.z_u)

        if vertex.group != 0 {
            continue;
        }

        let mut local_group: Option<usize> = None;
        for adj in adjs.iter() {
            let key:(isize, isize) = (adj.0 + vertex.x_u as isize, adj.1 + vertex.z_u as isize);
            if (key.0 < 0) | (key.1 < 0) | (key.0 > max_x) | (key.1 > max_z) {
                continue;
            }
            let u_key = (key.0 as usize, key.1 as usize);

            if let Some(vertex) = group_map.get(&u_key){
                if vertex.group != 0 {
                    local_group = Some(vertex.group);
                    break;
                }
            }
        }

        if local_group.is_none(){
            local_group = Some(global_group);
            global_group += 1;
        }

        if let Some(local_group) = local_group {

            for adj in adjs.iter() {
                let key:(isize, isize) = (adj.0 + vertex.x_u as isize, adj.1 + vertex.z_u as isize);
                if (key.0 < 0) | (key.1 < 0) | (key.0 > max_x) | (key.1 > max_z) {
                    continue;
                }

                let u_key = (key.0 as usize, key.1 as usize);
                if let Some(vertex) = group_map.get_mut(&u_key){
                    vertex.group = local_group;
                }
            }

            if let Some(vertex) = group_map.get_mut(&(vertex.x_u, vertex.z_u)){
                vertex.group = local_group;
            }
        }
    }

    // info!("global_group: {}", global_group);


    // 2nd pass
    loop {
        let mut group_pairs: HashSet<(usize, usize)> = HashSet::new();
        let mut used: HashSet<usize> = HashSet::new();

        for (_tile, vertex) in group_map.iter(){
            if used.contains(&vertex.group){
                continue;
            }

            for adj in adjs.iter() {
                let key:(isize, isize) = (adj.0 + vertex.x_u as isize, adj.1 + vertex.z_u as isize);
                if (key.0 < 0) | (key.1 < 0) | (key.0 > max_x) | (key.1 > max_z) {
                    continue;
                }
                let u_key = (key.0 as usize, key.1 as usize);
                if let Some(vertex2) = group_map.get(&u_key){
                    if used.contains(&vertex2.group){
                        continue;
                    }

                    if vertex2.group != vertex.group {
                        let ming = vertex.group.min(vertex2.group);
                        let maxg = vertex.group.max(vertex2.group);
                        group_pairs.insert((ming, maxg));
                        used.insert(ming);
                        used.insert(maxg);
                    }
                }
            }
        }
        info!("2nd pass: group pairs len: {}", group_pairs.len());
        if group_pairs.len() == 0 {
            break;
        }
        for pair in group_pairs.iter(){
            for (_tile, vertex) in group_map.iter_mut(){
                if vertex.group == pair.1 {
                    vertex.group = pair.0;
                }
            }
        }
    }

    let unique_groups = group_map.iter().map(|v| v.1.group).collect::<HashSet<usize>>();
    info!("unique_groups: {:?}", unique_groups);

    // 3rd pass - keep only boundary points
    let mut to_rm: Vec<(usize, usize)> = Vec::new();
    for (tile, vertex) in group_map.iter(){
        let mut adj_count: usize = 0;

        for adj in adjs.iter() {
            let key:(isize, isize) = (adj.0 + vertex.x_u as isize, adj.1 + vertex.z_u as isize);
            if (key.0 < 0) | (key.1 < 0) | (key.0 > max_x) | (key.1 > max_z) {
                continue;
            }
            let u_key = (key.0 as usize, key.1 as usize);
            if let Some(_vertex2) = group_map.get(&u_key){
                adj_count += 1;
            }
        }
        if adj_count == 8 {
            to_rm.push(*tile);
        }
    }

    for tile in to_rm.iter(){
        group_map.remove(tile);
    }
    return group_map;
}


pub(crate) fn mesh_from_triangles(
    triangles: &Vec<[Vec3A; 3]>
) -> Mesh {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();
    for (tri_idx, triangle) in triangles.iter().enumerate() {
        let [v0, v1, v2] = triangle;
        positions.push([v0.x, v0.y, v0.z]);
        positions.push([v1.x, v1.y, v1.z]);
        positions.push([v2.x, v2.y, v2.z]);

        normals.push([0.0, 1.0, 0.0]);
        normals.push([0.0, 1.0, 0.0]);
        normals.push([0.0, 1.0, 0.0]);
        let base_idx = (tri_idx * 3) as u16;
        indices.push(base_idx);
        indices.push(base_idx + 2);
        indices.push(base_idx + 1);
    }

    let water_simple_mesh = Mesh::new(
        bevy::render::render_resource::PrimitiveTopology::TriangleList,
        bevy::asset::RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_indices(bevy::mesh::Indices::U16(indices));

    return water_simple_mesh;
}


pub(crate) fn triangulate(
    vertices: &mut Vec<Vec3A>
) -> Vec<[Vec3A; 3]> {
    let flat_vertices: Vec<f64> = vertices
        .iter()
        .flat_map(|v| [v.x as f64, v.z as f64])
        .collect();
    let triangle_indices = earcutr::earcut(&flat_vertices, &[], 2).unwrap();
    let triangles: Vec<[Vec3A; 3]> = triangle_indices.chunks(3).map(|chunk| {
        [
            vertices[chunk[0]],
            vertices[chunk[1]],
            vertices[chunk[2]],
        ]
    })
    .collect();
    return triangles;
}


pub(crate) fn remove_collinear(
    vlocs: &mut Vec<Vec3A>
) -> Vec<Vec3A> {
    // 200, 400 looks good
    let threshold: f32 = 1200.0; // Up to 1400 I saw
    let mut result = vec![vlocs[0]];

    for i in 1..vlocs.len() {

        if i == vlocs.len() - 1 {
            result.push(vlocs[i]);
            break;
        }

        let p1 = result[result.len() - 1];
        let p2 = vlocs[i];
        let p3 = vlocs[i + 1];
        let v1x = p2.x - p1.x;
        let v1y = p2.z - p1.z;
        let v2x = p3.x - p1.x;
        let v2y = p3.z - p1.z;
        let cross = (v1x * v2y - v1y * v2x).abs();

        if cross > threshold {
            result.push(p2);
        }
    }
    result.push(vlocs[vlocs.len() - 1]);

    return result;
}


pub(crate) fn order_boundary(
    vertices: &mut Vec<WaterVertex>, 
    maxx: u32, 
    maxz: u32
) -> Vec<Vec3A>{
    let max_x: isize = maxx as isize;
    let max_z: isize = maxz as isize;

    let mut new_order: Vec<Vec3A> = Vec::with_capacity(vertices.len());
    let mut used: HashSet<(usize, usize)> = HashSet::new();
    vertices.sort();

    let mut current = vertices[0];
    new_order.push(current.loc);
    
    let madjs: [(isize, isize); 4] = [(1, 0), (0, 1), (0, -1), (-1, 0)];
    let dadjs: [(isize, isize); 4] = [(-1, -1), (1, 1), (1, -1), (-1, 1)];

    loop {
        let mut next: Option<WaterVertex> = None;
        for adj in madjs.iter(){
            let adj_x = current.x_u as isize + adj.0;
            let adj_z = current.z_u as isize + adj.1;

            if (adj_x > 0) & (adj_x <= max_x) & (adj_z > 0) & (adj_z <= max_z){
                for v in vertices.iter(){
                    if used.contains(&(v.x_u, v.z_u)){
                        continue;
                    }

                    if (adj_x as usize == v.x_u) & (adj_z as usize == v.z_u){
                        next = Some(*v);
                        break;
                    }
                }
            }
        }

        if let Some(next) = next {
            used.insert((current.x_u, current.z_u));
            current = next;
            new_order.push(current.loc);
        } else {
            for adj in dadjs.iter(){
                let adj_x = current.x_u as isize + adj.0;
                let adj_z = current.z_u as isize + adj.1;

                if (adj_x > 0) & (adj_x <= max_x) & (adj_z > 0) & (adj_z <= max_z){
                    for v in vertices.iter(){
                        if used.contains(&(v.x_u, v.z_u)){
                            continue;
                        }

                        if (adj_x as usize == v.x_u) & (adj_z as usize == v.z_u){
                            next = Some(*v);
                            break;
                        }
                    }
                }
            }
        }
        if let Some(next) = next {
            used.insert((current.x_u, current.z_u));
            current = next;
            new_order.push(current.loc);
        } else {
            break;
        }

    }
    return new_order;
}
