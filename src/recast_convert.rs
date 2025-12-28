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
                    polygons: HashSet::new()
                }
            )
        }).collect();

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
                PGPolygon {
                    index: polygon_index,
                    vertices: vec![v0_id, v1_id, v2_id],
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
                    vertex.polygons.insert(polygon.index);
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
