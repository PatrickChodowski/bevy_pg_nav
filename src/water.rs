use bevy::prelude::*;
use bevy::platform::collections::HashSet;
use rayon::prelude::*;
use avian3d::prelude::{Collider, ColliderOf, CollisionLayers, RigidBody};
use bevy_rerecast::generator::NavmeshGenerator;

use bevy_rerecast::NavmeshSettings;
use bevy_rerecast::debug::DetailNavmeshGizmo;

use bevy_pg_core::prelude::{AABB, TerrainChunk, WaterChunk};
use crate::plugin::NavmeshGenerationData;
use crate::pgnavmesh::PGNavmeshType;
use crate::terrain::TerrainRayMeshData;
use crate::tools::NavRay;

/// Distance between height samples. Smaller = finer shoreline, more triangles.
/// Compare against the navmesh cell size (derived from agent radius).
const WATER_GRID_STEP: f32 = 1.0;

/// Minimum water depth (surface - ground) for a point to count as navigable.
/// Set this to the boat's draft. 0.0 reproduces the old "ground <= water" test.
const MIN_WATER_DEPTH: f32 = 0.0;

/// Also test the terrain height at each cell centre, so a narrow rock between
/// four wet corners doesn't get a water quad drawn over it.
const CHECK_CELL_CENTER: bool = true;

/// Two water levels closer than this are treated as the same level.
const LEVEL_EPSILON: f32 = 1e-3;

pub struct PGWaterNavPlugin;

impl Plugin for PGWaterNavPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_observer(generate_water_navmesh)
        .add_observer(on_water_navmesh_sources_ready)
        .add_systems(Update, await_water_colliders)
        ;
    }
}

#[derive(Event)]
pub(crate) struct GenerateWaterNavmesh;

#[derive(Component)]
pub(crate) struct WaterNavmeshSource;

#[derive(Event)]
pub(crate) struct WaterNavmeshesReady;

/// Present while we wait for Avian to finish setting up the water colliders.
#[derive(Resource)]
struct PendingWaterNavmesh;


fn generate_water_navmesh(
    _trigger:       On<GenerateWaterNavmesh>,
    terrains:       Query<(&Transform, &Mesh3d, &TerrainChunk)>,
    water_query:    Query<(&Transform, &WaterChunk)>,
    old_sources:    Query<Entity, With<WaterNavmeshSource>>,
    mut commands:   Commands,
    meshes:         Res<Assets<Mesh>>,
    navgendata:     Res<NavmeshGenerationData>,
){
    let Ok((terrain_transform, mesh3d, chunk)) = terrains.get(navgendata.plane_entity) else {
        warn!("[NAV] water: plane_entity {} is not a terrain chunk", navgendata.plane_entity);
        return;
    };
    let Some(mesh) = meshes.get(&mesh3d.0) else {
        warn!("[NAV] water: terrain mesh asset not loaded yet");
        return;
    };

    info!("Generate Water Navmesh for entity {}", navgendata.plane_entity);
    let loc = terrain_transform.translation;
    let half = chunk.dims * 0.5;
    info!("terrain chunk dims: {} terrain loc: {}", chunk.dims, loc);

    let trmd = TerrainRayMeshData::from_mesh(mesh, &terrain_transform.to_matrix());

    let xs = grid_axis(loc.x, half.x, WATER_GRID_STEP);
    let zs = grid_axis(loc.z, half.y, WATER_GRID_STEP);

    let water_aabbs: Vec<(AABB, f32)> = water_query
        .iter()
        .map(|(t, w)| (AABB::from_loc_dims(t.translation.xz(), w.dims), t.translation.y))
        .collect();

    let (vertices, indices) = build_water_trimesh(&xs, &zs, &trmd, &water_aabbs);

    // Remove water sources from a previous run
    for e in old_sources.iter() {
        commands.entity(e).despawn();
    }

    if indices.is_empty() {
        warn!("[NAV] water: no navigable water found, skipping water navmesh");
        return;
    }

    info!("[NAV] water: {} vertices, {} triangles", vertices.len(), indices.len());

    commands.spawn((
        WaterNavmeshSource,
        Collider::trimesh(vertices, indices),
        RigidBody::Static,
        // This is a navmesh proxy, not physical water. Remove this line if your
        // Avian version has no CollisionLayers::NONE or the backend ignores
        // colliders on empty layers.
        CollisionLayers::NONE,
        Transform::default(),
        Visibility::Hidden,
    ));

    // WaterNavmeshesReady is triggered by await_water_colliders once Avian has
    // linked the collider to its body.
    commands.insert_resource(PendingWaterNavmesh);
}


fn await_water_colliders(
    pending:  Option<Res<PendingWaterNavmesh>>,
    sources:  Query<Has<ColliderOf>, With<WaterNavmeshSource>>,
    mut commands: Commands,
){
    if pending.is_none() || sources.is_empty() {
        return;
    }
    if sources.iter().all(|ready| ready) {
        commands.remove_resource::<PendingWaterNavmesh>();
        commands.trigger(WaterNavmeshesReady);
    }
}


fn on_water_navmesh_sources_ready(
    _trigger:       On<WaterNavmeshesReady>,
    mut generator:  NavmeshGenerator,
    mut commands:   Commands,
    water_sources:  Query<Entity, With<WaterNavmeshSource>>,
    mut navgendata: ResMut<NavmeshGenerationData>,
){
    let hs: HashSet<Entity> = water_sources.iter().collect();
    if hs.is_empty() {
        warn!("[NAV] water: no water sources, skipping navmesh generation");
        return;
    }

    let settings = NavmeshSettings {
        filter: Some(hs),
        walkable_climb: 10.0,
        walkable_slope_angle: 0.1_f32.to_radians(),
        agent_radius: 1.0,
        agent_height: 2.0,
        max_vertices_per_polygon: 20,
        max_simplification_error: 1.3,
        min_region_size: 1,
        up: Vec3::Y,
        ..default()
    };
    let navmesh = generator.generate(settings);
    commands.spawn(DetailNavmeshGizmo::new(&navmesh));
    navgendata.add_handle(PGNavmeshType::Water, navmesh);
}


/// Sample positions along one axis, from `center - half` to `center + half`.
/// The last sample is clamped to the edge so chunks never overlap their neighbours.
fn grid_axis(center: f32, half: f32, step: f32) -> Vec<f32> {
    let min = center - half;
    let max = center + half;
    let n = ((max - min) / step).ceil().max(1.0) as usize;
    (0..=n).map(|i| (min + i as f32 * step).min(max)).collect()
}


/// Highest water surface covering (x, z), if any.
fn water_level_at(waters: &[(AABB, f32)], x: f32, z: f32) -> Option<f32> {
    let p = Vec2::new(x, z);
    waters
        .iter()
        .filter(|(aabb, _)| aabb.has_point(p))
        .map(|(_, y)| *y)
        .fold(None, |acc: Option<f32>, y| Some(acc.map_or(y, |a| a.max(y))))
}


fn ground_height(trmd: &TerrainRayMeshData, x: f32, z: f32) -> Option<f32> {
    trmd.test(&NavRay::down(x, z)).map(|(h, _, _)| h)
}


/// Builds a triangle soup covering all navigable water.
///
/// A sample point is "wet" if a water AABB covers it and the terrain below it is
/// at least `MIN_WATER_DEPTH` under the surface. A grid cell becomes two upward
/// facing triangles when all four corners are wet at the same level.
fn build_water_trimesh(
    xs:     &[f32],
    zs:     &[f32],
    trmd:   &TerrainRayMeshData,
    waters: &[(AABB, f32)],
) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let (nx, nz) = (xs.len(), zs.len());
    if nx < 2 || nz < 2 || waters.is_empty() {
        return (vec![], vec![]);
    }

    // Water surface height per sample point, None if dry / too shallow / off terrain.
    // Index = xi * nz + zi
    let wet: Vec<Option<f32>> = (0..nx * nz)
        .into_par_iter()
        .map(|i| {
            let (x, z) = (xs[i / nz], zs[i % nz]);
            let water_y = water_level_at(waters, x, z)?;
            let ground = ground_height(trmd, x, z)?;
            (ground + MIN_WATER_DEPTH <= water_y).then_some(water_y)
        })
        .collect();

    // Per cell: Some(level) if the cell should be filled. Index = xi * (nz-1) + zi
    let cell_count = (nx - 1) * (nz - 1);
    let cells: Vec<Option<f32>> = (0..cell_count)
        .into_par_iter()
        .map(|ci| {
            let (xi, zi) = (ci / (nz - 1), ci % (nz - 1));
            let y0 = wet[xi * nz + zi]?;
            let y1 = wet[xi * nz + zi + 1]?;
            let y2 = wet[(xi + 1) * nz + zi + 1]?;
            let y3 = wet[(xi + 1) * nz + zi]?;

            let same_level = [y1, y2, y3].iter().all(|y| (y - y0).abs() < LEVEL_EPSILON);
            if !same_level {
                return None;
            }

            if CHECK_CELL_CENTER {
                let cx = (xs[xi] + xs[xi + 1]) * 0.5;
                let cz = (zs[zi] + zs[zi + 1]) * 0.5;
                let ground = ground_height(trmd, cx, cz)?;
                if ground + MIN_WATER_DEPTH > y0 {
                    return None;
                }
            }
            Some(y0)
        })
        .collect();

    let mut vertices: Vec<Vec3> = Vec::new();
    let mut indices: Vec<[u32; 3]> = Vec::new();
    let mut vertex_of_sample: Vec<u32> = vec![u32::MAX; nx * nz];

    for (ci, level) in cells.iter().enumerate() {
        if level.is_none() {
            continue;
        }
        let (xi, zi) = (ci / (nz - 1), ci % (nz - 1));

        // Counter-clockwise seen from above, so both triangles face +Y
        let corners = [(xi, zi), (xi, zi + 1), (xi + 1, zi + 1), (xi + 1, zi)];
        let [a, b, c, d] = corners.map(|(cx, cz)| {
            let i = cx * nz + cz;
            if vertex_of_sample[i] == u32::MAX {
                vertex_of_sample[i] = vertices.len() as u32;
                vertices.push(Vec3::new(xs[cx], wet[i].unwrap_or(0.0), zs[cz]));
            }
            vertex_of_sample[i]
        });

        indices.push([a, b, c]);
        indices.push([a, c, d]);
    }

    #[cfg(debug_assertions)]
    for tri in indices.iter() {
        let [p0, p1, p2] = tri.map(|i| vertices[i as usize]);
        let normal = (p1 - p0).cross(p2 - p0);
        debug_assert!(normal.y > 0.0, "water triangle is not facing +Y: {:?}", tri);
    }

    (vertices, indices)
}