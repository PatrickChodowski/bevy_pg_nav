use bevy::platform::collections::{HashSet, HashMap};
use rerecast::{PolygonNavmesh, DetailNavmesh};
use bevy_rerecast::Navmesh;
use bevy::prelude::{info, default};

use crate::{plugin::PGNavmeshType, types::{PGNavmesh, PGPolygon, PGVertex}};

trait RecastPolyMeshDetailExt {
    fn common_vertices(&self) -> HashMap<u32, Vec<u32>>;
}

impl RecastPolyMeshDetailExt for DetailNavmesh {
    fn common_vertices(&self) -> HashMap<u32, Vec<u32>> {
        self.vertices
            .iter()
            .enumerate()
            .map(|(i, v)| {
                (
                    i as u32,
                    self.vertices
                        .iter()
                        .enumerate()
                        .filter_map(|(i2, v2)| (v == v2).then_some(i2 as u32))
                        .collect(),
                )
            })
            .collect()
    }
}

pub(crate) fn convert_rerecast(
    renav: &Navmesh,
    water_height: f32,
    typ:   &PGNavmeshType
) -> PGNavmesh {

    info!("About to convert navmesh!");
    let triangles_with_mesh_info = triangles_with_mesh_info(&renav);
    let areas = areas(&renav.polygon);

    let mut vertex_map: HashMap<usize, PGVertex> = renav.detail.vertices.iter().enumerate()
        .map(|(vertex_index, vloc)| {
            (
                vertex_index, 
                PGVertex {
                    index: vertex_index,
                    loc: *vloc,
                    polygons: Vec::new()
                }
            )
        }).collect();

    // deduplicate vertex map

    let threshold = 0.001;
    let index_mapping: HashMap<usize, usize> = deduplicate(&mut vertex_map, threshold);



    // Create polygons
    let mut polygons: Vec<PGPolygon> = Vec::new();
    for area in areas.iter(){
        let area_polygons: Vec<PGPolygon> = triangles_with_mesh_info
            .iter()
            .enumerate()
            .filter(|polygon| polygon.1.mesh_area == *area)
            .map(|(polygon_index, polygon)| {
                // reversed on purpose
                let v0_id = polygon.vertices[2];
                let v1_id = polygon.vertices[1];
                let v2_id = polygon.vertices[0];

                let v0 = index_mapping.get(&v0_id).unwrap();
                let v1 = index_mapping.get(&v1_id).unwrap();
                let v2 = index_mapping.get(&v2_id).unwrap();

                PGPolygon {
                    index: polygon_index,
                    vertices: vec![*v0, *v1, *v2],
                    neighbours: HashSet::new()
                }
            })
            .collect();

        polygons.extend(area_polygons);
    }

    // Assign polygons to vertices
    for (vertex_index, vertex) in vertex_map.iter_mut(){
        for polygon in polygons.iter(){
            for v in polygon.vertices.iter(){
                if v == vertex_index {
                    vertex.polygons.push(polygon.index);
                }
            }
        }
    }

    // Assign neighbours inside polygons
    for polygon in polygons.iter_mut(){
        let mut neighbours: HashSet<usize> = HashSet::new();
        for vertex_index in polygon.vertices.iter(){
            let vertex = vertex_map.get(vertex_index).unwrap();
            for v_polygon in vertex.polygons.iter(){
                if &polygon.index != v_polygon {
                    neighbours.insert(*v_polygon);
                }
            }
        }
        polygon.neighbours = neighbours;
    }

    info!("polygons length: {}", polygons.len());
    let polygon_map = polygons.iter().map(|p| (p.index, p.clone())).collect::<HashMap<usize, PGPolygon>>();

    let pgn = PGNavmesh {
        polygons: polygon_map,
        vertices: vertex_map,
        water_height,
        typ: *typ,
        ..default()
    };

    return pgn;
}

struct PolygonWithMeshInfo {
    vertices: [usize; 3],
    mesh_area: u8,
}


fn triangles_with_mesh_info(renav: &Navmesh) -> Vec<PolygonWithMeshInfo> {
    renav.detail
        .meshes
        .iter()
        .zip(renav.polygon.areas.iter())
        .flat_map(|(mesh, mesh_area)| {
            renav.detail
                .triangles
                .iter()
                .skip(mesh.base_triangle_index as usize)
                .take(mesh.triangle_count as usize)
                .map(|[a, b, c]| PolygonWithMeshInfo {
                    vertices: [
                        *a as usize + mesh.base_vertex_index as usize,
                        *b as usize + mesh.base_vertex_index as usize,
                        *c as usize + mesh.base_vertex_index as usize,
                    ],
                    mesh_area: mesh_area.0,
                })
        })
        .collect()
}



fn areas(renav: &PolygonNavmesh) -> Vec<u8> {
    let mut areas: Vec<u8> = renav
        .areas
        .iter()
        .map(|area| area.0)
        .collect::<HashSet<_>>()
        .into_iter()
        .collect();
    areas.sort_unstable();
    if let Some(255) = areas.last() {
        areas.pop();
        areas.insert(0, 255);
    }
    areas
}




fn deduplicate(
    vertex_map: &mut HashMap<usize, PGVertex>,
    threshold: f32,
) -> HashMap<usize, usize> {
    let threshold_sq = threshold * threshold;

    info!("vertex map size before: {}", vertex_map.len());

    let mut index_mapping: HashMap<usize, HashSet<usize>> = HashMap::new();
    let mut processed: HashSet<usize> = HashSet::new();

    for (k1, v1) in vertex_map.iter(){

        if processed.contains(k1) {
            continue;
        }

        let mut cluster: HashSet<usize> = HashSet::new();
        cluster.insert(*k1);


        for (k2, v2) in vertex_map.iter(){

            if k1 == k2 {
                continue;
            }

            if v1.xz().distance_squared(v2.xz()) > threshold_sq {
                continue;
            }

            cluster.insert(*k2);

        }

        let min_index = *cluster.iter().min().unwrap();
        
        // Mark all indices in this cluster as processed
        for &idx in &cluster {
            processed.insert(idx);
        }

        index_mapping.insert(min_index, cluster);
    }

    info!("index map size: {}", index_mapping.len());

    // for (k, v) in index_mapping.iter(){
    //     info!("k: {} v: {:?}", k, v);
    // }

    let mut reverse_map: HashMap<usize, usize> = HashMap::new();
    for (min_idx, cluster) in index_mapping.iter() {
        for &idx in cluster {
            reverse_map.insert(idx, *min_idx);
        }
    }

    for (min_idx, cluster) in index_mapping.iter() {

        if cluster.len() == 1 {
            continue;
        }

        let mut polygons: Vec<usize> = Vec::new();
        for mv in cluster.iter(){
            for p in vertex_map.get(mv).unwrap().polygons.iter(){
                polygons.push(*p);
            }
        }
        polygons.sort_unstable();
        polygons.dedup();

        let pgv = PGVertex {
            index: *min_idx,
            loc: vertex_map.get(min_idx).unwrap().loc,
            polygons
        };

        for mv in cluster.iter(){
            vertex_map.remove(mv);
        }

        vertex_map.insert(pgv.index, pgv.clone());
    }


    info!("index map size: {}", reverse_map.len());
    info!("vertex map size after: {}", vertex_map.len());

    reverse_map
}