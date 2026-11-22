//! Water navmesh (boats).
//!
//! The navigable water surface is decided in 2D first, then handed to Recast on its own:
//!
//! 1. For every water plane (entities with `WaterChunk`), its real extent (mesh bounds x world
//!    transform) is rasterised into a grid of `WATER_CELL` cells. A cell is water when the
//!    terrain under it is at least `water_min_depth` below the surface (or there is no terrain,
//!    e.g. open sea past the plane's edge) and no nav static (dock, pier, building...) covers it.
//! 2. Water cells become a flat mesh at the water height (row runs merged into strips). Islands
//!    and shorelines are just holes and edges in that mesh: no boundary tracing or polygon
//!    triangulation needed, so any shape works, including lakes with islands.
//! 3. Recast generates the navmesh from only those meshes, eroding the shores by the boat radius
//!    (`water_settings.agent_radius`) and building clean polygons.
//!
//! Terrain heights are sampled from the plane's vertex grid (bilinear, constant time per cell)
//! instead of raycasting every triangle per cell.

use avian3d::prelude::{Collider, RigidBody, Rotation, SimpleCollider};
use bevy::mesh::{Mesh, VertexAttributeValues};
use bevy::platform::collections::HashSet;
use bevy::prelude::*;
use bevy_rerecast::generator::NavmeshGenerator;

use bevy_pg_core::prelude::WaterChunk;
use crate::pgnavmesh::PGNavmeshType;
use crate::plugin::{NavStatic, NavmeshGenerationData};
use crate::terrain::CollidersReady;

/// Size of the water mask cells (world units). Recast smooths the staircase edges afterwards.
const WATER_CELL: f32 = 0.5;
/// Colliders need a moment to be registered by physics before Recast can see them.
const WATER_SOURCES_WAIT_SECS: f32 = 0.5;

pub struct PGWaterNavPlugin;

impl Plugin for PGWaterNavPlugin {
    fn build(&self, app: &mut App) {
        app
        .add_observer(spawn_water_sources)
        .add_systems(Update, generate_when_sources_ready.run_if(resource_exists::<WaitWaterSources>))
        ;
    }
}

#[derive(Component)]
pub(crate) struct WaterNavmeshSource;

#[derive(Resource)]
struct WaitWaterSources(Timer);

/// A water plane's area and height, in world space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct WaterArea {
    pub(crate) min: Vec2,
    pub(crate) max: Vec2,
    pub(crate) height: f32,
}

/// Terrain heights on the plane's regular vertex grid, in world space.
pub(crate) struct HeightGrid {
    xs: Vec<f32>,
    zs: Vec<f32>,
    heights: Vec<f32>, // [zi * xs.len() + xi]
}

impl HeightGrid {
    /// From a plane mesh (a regular grid of vertices, as Plane3d and the editor make) and its
    /// world transform. Returns None for meshes that aren't such a grid.
    pub(crate) fn from_mesh(mesh: &Mesh, transform: &GlobalTransform) -> Option<Self> {
        let Some(VertexAttributeValues::Float32x3(positions)) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else { return None };
        let world: Vec<Vec3> = positions.iter().map(|p| transform.transform_point(Vec3::from(*p))).collect();
        let mut xs: Vec<f32> = world.iter().map(|p| p.x).collect();
        let mut zs: Vec<f32> = world.iter().map(|p| p.z).collect();
        xs.sort_by(f32::total_cmp);
        zs.sort_by(f32::total_cmp);
        xs.dedup_by(|a, b| (*a - *b).abs() < 1e-3);
        zs.dedup_by(|a, b| (*a - *b).abs() < 1e-3);
        if xs.len() < 2 || zs.len() < 2 || xs.len() * zs.len() != world.len() {
            return None;
        }
        let mut heights = vec![0.0; xs.len() * zs.len()];
        for p in &world {
            let xi = xs.partition_point(|&v| v < p.x - 1e-3);
            let zi = zs.partition_point(|&v| v < p.z - 1e-3);
            heights[zi * xs.len() + xi] = p.y;
        }
        Some(Self { xs, zs, heights })
    }

    /// Terrain height at (x, z), bilinear between grid vertices. None outside the terrain.
    pub(crate) fn height_at(&self, p: Vec2) -> Option<f32> {
        let (x0, x1) = (*self.xs.first()?, *self.xs.last()?);
        let (z0, z1) = (*self.zs.first()?, *self.zs.last()?);
        if p.x < x0 || p.x > x1 || p.y < z0 || p.y > z1 {
            return None;
        }
        let cell = |coords: &[f32], v: f32| {
            let i = coords.partition_point(|&c| c <= v).clamp(1, coords.len() - 1) - 1;
            let t = (v - coords[i]) / (coords[i + 1] - coords[i]).max(1e-6);
            (i, t.clamp(0.0, 1.0))
        };
        let (xi, tx) = cell(&self.xs, p.x);
        let (zi, tz) = cell(&self.zs, p.y);
        let w = self.xs.len();
        let h = |xi: usize, zi: usize| self.heights[zi * w + xi];
        let top = h(xi, zi) + (h(xi + 1, zi) - h(xi, zi)) * tx;
        let bottom = h(xi, zi + 1) + (h(xi + 1, zi + 1) - h(xi, zi + 1)) * tx;
        Some(top + (bottom - top) * tz)
    }
}

/// Which cells of `area` (cell size `cell`) boats can use. Returns (columns, rows, mask) with
/// mask[row * columns + column]; cell centres are `area.min + (c + 0.5, r + 0.5) * cell`.
pub(crate) fn water_mask(
    area: &WaterArea,
    cell: f32,
    min_depth: f32,
    terrain_height: impl Fn(Vec2) -> Option<f32>,
    blocked: impl Fn(Vec2) -> bool,
) -> (usize, usize, Vec<bool>) {
    let size = area.max - area.min;
    let cols = (size.x / cell).ceil().max(0.0) as usize;
    let rows = (size.y / cell).ceil().max(0.0) as usize;
    let mut mask = vec![false; cols * rows];
    for r in 0..rows {
        for c in 0..cols {
            let p = area.min + Vec2::new(c as f32 + 0.5, r as f32 + 0.5) * cell;
            let p = p.min(area.max); // last, partial cells: sample inside the plane
            let deep_enough = terrain_height(p).is_none_or(|ground| ground <= area.height - min_depth);
            mask[r * cols + c] = deep_enough && !blocked(p);
        }
    }
    (cols, rows, mask)
}

/// Flat triangles at the water height covering the mask's water cells. Each row's runs of water
/// cells become one quad, so a simple lake is a few hundred triangles, not one pair per cell.
pub(crate) fn mask_to_mesh(area: &WaterArea, cell: f32, cols: usize, rows: usize, mask: &[bool]) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for r in 0..rows {
        let mut c = 0;
        while c < cols {
            if !mask[r * cols + c] {
                c += 1;
                continue;
            }
            let start = c;
            while c < cols && mask[r * cols + c] {
                c += 1;
            }
            let x0 = area.min.x + start as f32 * cell;
            let x1 = (area.min.x + c as f32 * cell).min(area.max.x);
            let z0 = area.min.y + r as f32 * cell;
            let z1 = (area.min.y + (r + 1) as f32 * cell).min(area.max.y);
            let base = vertices.len() as u32;
            vertices.extend([
                Vec3::new(x0, area.height, z0),
                Vec3::new(x1, area.height, z0),
                Vec3::new(x1, area.height, z1),
                Vec3::new(x0, area.height, z1),
            ]);
            // Counter-clockwise seen from above (+Y), so the surface faces up.
            indices.push([base, base + 2, base + 1]);
            indices.push([base, base + 3, base + 2]);
        }
    }
    (vertices, indices)
}

/// World-space area of a water plane from its mesh and transform.
fn water_area(mesh: &Mesh, transform: &GlobalTransform) -> Option<WaterArea> {
    let Some(VertexAttributeValues::Float32x3(positions)) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else { return None };
    let world = positions.iter().map(|p| transform.transform_point(Vec3::from(*p)));
    let (min, max) = world.fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(lo, hi), p| (lo.min(p), hi.max(p)));
    (max.x > min.x && max.z > min.z).then(|| WaterArea { min: min.xz(), max: max.xz(), height: (min.y + max.y) * 0.5 })
}

/// Builds the boat-usable water surfaces once the terrain and nav-static colliders exist.
fn spawn_water_sources(
    _trigger:     On<CollidersReady>,
    mut commands: Commands,
    meshes:       Res<Assets<Mesh>>,
    mut navgendata: ResMut<NavmeshGenerationData>,
    terrains:     Query<(&Mesh3d, &GlobalTransform)>,
    waters:       Query<(&Mesh3d, &GlobalTransform), With<WaterChunk>>,
    statics:      Query<(&Collider, &GlobalTransform), With<NavStatic>>,
) {
    if navgendata.water_settings.is_none() {
        return;
    }

    let terrain = terrains
        .get(navgendata.plane_entity)
        .ok()
        .and_then(|(mesh3d, transform)| HeightGrid::from_mesh(meshes.get(&mesh3d.0)?, transform));
    if terrain.is_none() {
        warn!("[NAV] [water] Terrain plane isn't a regular vertex grid; treating all water as deep");
    }

    // Docks, piers, buildings... block boats where they stand (their collider's footprint).
    let obstacles: Vec<(Vec2, Vec2)> = statics
        .iter()
        .map(|(collider, transform)| {
            let (_, rotation, translation) = transform.to_scale_rotation_translation();
            let aabb = collider.aabb(translation, Rotation(rotation), 0.0);
            (aabb.min.xz(), aabb.max.xz())
        })
        .collect();
    let blocked = |p: Vec2| obstacles.iter().any(|(lo, hi)| p.cmpge(*lo).all() && p.cmple(*hi).all());

    let mut sources = 0;
    for (mesh3d, transform) in &waters {
        let Some(area) = meshes.get(&mesh3d.0).and_then(|m| water_area(m, transform)) else { continue };
        let (cols, rows, mask) = water_mask(
            &area,
            WATER_CELL,
            navgendata.water_min_depth,
            |p| terrain.as_ref().and_then(|t| t.height_at(p)),
            blocked,
        );
        let (vertices, indices) = mask_to_mesh(&area, WATER_CELL, cols, rows, &mask);
        if indices.is_empty() {
            info!("[NAV] [water] Water plane at {:?} has no water deep enough for boats", area.min);
            continue;
        }
        navgendata.water_heights.push(area.height);
        commands.spawn((
            Name::new("Water navmesh source"),
            WaterNavmeshSource,
            Collider::trimesh(vertices, indices),
            RigidBody::Static,
            Transform::IDENTITY,
        ));
        sources += 1;
    }
    info!("[NAV] [water] {sources} water surface(s) prepared");
    commands.insert_resource(WaitWaterSources(Timer::from_seconds(WATER_SOURCES_WAIT_SECS, TimerMode::Once)));
}

fn generate_when_sources_ready(
    time:           Res<Time>,
    mut wait:       ResMut<WaitWaterSources>,
    mut commands:   Commands,
    mut generator:  NavmeshGenerator,
    sources:        Query<Entity, With<WaterNavmeshSource>>,
    mut navgendata: ResMut<NavmeshGenerationData>,
) {
    if !wait.0.tick(time.delta()).is_finished() {
        return;
    }
    commands.remove_resource::<WaitWaterSources>();

    if sources.is_empty() {
        info!("[NAV] [water] No navigable water: skipping the water navmesh");
        navgendata.skip(PGNavmeshType::Water);
        return;
    }
    let Some(mut settings) = navgendata.water_settings.clone() else { return };
    // Only the prepared water surfaces: terrain, statics and everything else stay out.
    settings.filter = Some(sources.iter().collect::<HashSet<Entity>>());
    let navmesh = generator.generate(settings);
    navgendata.add_handle(PGNavmeshType::Water, navmesh);
}


#[cfg(test)]
mod tests {
    use super::*;

    fn lake() -> WaterArea {
        WaterArea { min: Vec2::new(-10.0, -10.0), max: Vec2::new(10.0, 10.0), height: 0.0 }
    }

    #[test]
    fn island_becomes_a_hole_and_open_sea_stays_water() {
        // Island: terrain above water within 3 units of the centre; deep bed elsewhere.
        // Past x = 6 there is no terrain at all (open sea beyond the plane's edge).
        let terrain = |p: Vec2| if p.x > 6.0 { None } else if p.length() < 3.0 { Some(1.0) } else { Some(-2.0) };
        let (cols, rows, mask) = water_mask(&lake(), 1.0, 0.5, terrain, |_| false);
        assert_eq!((cols, rows), (20, 20));
        let at = |x: f32, z: f32| {
            let c = (x + 10.0) as usize;
            let r = (z + 10.0) as usize;
            mask[r * cols + c]
        };
        assert!(!at(0.0, 0.0), "island is not water");
        assert!(at(-8.0, -8.0), "deep lake bed is water");
        assert!(at(9.0, 0.0), "no terrain = open water");
    }

    #[test]
    fn shallow_water_and_obstacles_are_excluded() {
        let shallow = |_: Vec2| Some(-0.2); // 0.2 deep, boats need 0.5
        let (_, _, mask) = water_mask(&lake(), 1.0, 0.5, shallow, |_| false);
        assert!(mask.iter().all(|w| !w));

        let deep = |_: Vec2| Some(-5.0);
        let dock = |p: Vec2| p.x.abs() < 2.0 && p.y.abs() < 2.0;
        let (cols, _, mask) = water_mask(&lake(), 1.0, 0.5, deep, dock);
        assert!(!mask[10 * cols + 10], "under the dock");
        assert!(mask[0]);
    }

    #[test]
    fn mesh_merges_rows_and_faces_up() {
        let (cols, rows, mask) = water_mask(&lake(), 1.0, 0.5, |_| Some(-5.0), |_| false);
        let (vertices, indices) = mask_to_mesh(&lake(), 1.0, cols, rows, &mask);
        assert_eq!(indices.len(), rows * 2, "one quad per full row");
        for tri in &indices {
            let [a, b, c] = tri.map(|i| vertices[i as usize]);
            assert!((b - a).cross(c - a).y > 0.0, "triangle faces up");
        }
        let total_area: f32 = indices
            .iter()
            .map(|t| {
                let [a, b, c] = t.map(|i| vertices[i as usize]);
                (b - a).cross(c - a).length() * 0.5
            })
            .sum();
        assert!((total_area - 400.0).abs() < 1e-3, "covers the whole lake: {total_area}");
    }

    #[test]
    fn height_grid_samples_a_sculpted_plane() {
        let mut mesh = Plane3d::default().mesh().size(10.0, 10.0).subdivisions(1).build();
        if let Some(VertexAttributeValues::Float32x3(p)) = mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION) {
            for v in p.iter_mut() {
                v[1] = v[0]; // height = x: a ramp from -5 to 5
            }
        }
        let grid = HeightGrid::from_mesh(&mesh, &GlobalTransform::from_xyz(100.0, 1.0, 0.0)).unwrap();
        // World x = 100 is the plane's centre: ramp height 0, plus the plane's y offset of 1.
        assert!((grid.height_at(Vec2::new(100.0, 0.0)).unwrap() - 1.0).abs() < 1e-4);
        assert!((grid.height_at(Vec2::new(102.5, 3.0)).unwrap() - 3.5).abs() < 1e-4);
        assert!(grid.height_at(Vec2::new(0.0, 0.0)).is_none(), "outside the terrain");
    }
}
