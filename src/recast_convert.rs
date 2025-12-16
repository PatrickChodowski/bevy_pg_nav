use bevy::platform::collections::{HashSet, HashMap};
use rerecast::{PolygonNavmesh, DetailNavmesh};
use bevy_rerecast::Navmesh;
use bevy::prelude::{info, Vec3A, default};

use crate::types::{PGVertex, PGPolygon, PGNavmesh};

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
    water_height: f32
) -> PGNavmesh {

    info!("About to convert navmesh!");
    let common_vertices = renav.detail.common_vertices();
    let triangles_with_mesh_info = triangles_with_mesh_info(&renav);
    let areas = areas(&renav.polygon);

    let reindexed_polygons: HashMap<u8, HashSet<usize>> = areas.iter()
        .map(|area| {(
            *area,
            triangles_with_mesh_info
                .iter()
                .enumerate()
                .filter_map(|(original_index, polygon)| {
                    (*area == polygon.mesh_area).then_some(original_index)
                })
                .enumerate()
                .map(|(_layer_index, original_index)| original_index)
                .collect::<HashSet<usize>>(),
        )}).collect();

    let vertices: Vec<PGVertex> = renav.detail.vertices
        .iter()
        .enumerate()
        .map(|(vertex_index, vloc)| {
            PGVertex {
                index: vertex_index,
                loc: Vec3A::from(*vloc),
                polygons: triangles_with_mesh_info
                    .iter()
                    .enumerate()
                    .filter_map(|(polygon_index, polygon)| {
                        common_vertices.get(&(vertex_index as u32)).unwrap().iter().find_map(
                                    |common_vertex_index| {
                                        polygon
                                            .vertices
                                            .contains(&(*common_vertex_index as usize))
                                            .then(|| {
                                                reindexed_polygons
                                                    .get(&polygon.mesh_area)
                                                    .unwrap()
                                                    .get(&polygon_index)
                                                    .cloned()
                                            })
                                            .flatten()
                                    },
                    )}).collect(),
        }}).collect();

    let vertex_map = vertices.iter().map(|v| (v.index, v.clone())).collect::<HashMap<usize, PGVertex>>();
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

                let mut neighbours: HashSet<usize> = HashSet::new();
                let v0 = vertex_map.get(&v0_id).unwrap();
                let v1 = vertex_map.get(&v1_id).unwrap();
                let v2 = vertex_map.get(&v2_id).unwrap();

                for n0 in v0.polygons.iter(){
                    if n0 != &polygon_index {
                        neighbours.insert(*n0);
                    }
                }
                for n1 in v1.polygons.iter(){
                    if n1 != &polygon_index {
                        neighbours.insert(*n1);
                    }
                }
                for n2 in v2.polygons.iter(){
                    if n2 != &polygon_index {
                        neighbours.insert(*n2);
                    }
                }

                PGPolygon {
                    index: polygon_index,
                    vertices: vec![v0.clone(), v1.clone(), v2.clone()],
                    neighbours
                }
            })
            .collect();

        polygons.extend(area_polygons);

    }

    info!("polygons length: {}", polygons.len());
    let polygon_map = polygons.iter().map(|p| (p.index, p.clone())).collect::<HashMap<usize, PGPolygon>>();
    let vertex_map = vertices.iter().map(|v| (v.index, v.clone())).collect::<HashMap<usize, PGVertex>>();

    let pgn = PGNavmesh {
        polygons: polygon_map,
        vertices: vertex_map,
        water_height,
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
