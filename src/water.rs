//! Water navmesh (boats).
//!
//! The navigable water surface is decided in 2D first, then handed to Recast on its own:
//!
//! 1. For every water plane (entities with `WaterChunk`), its real extent (mesh bounds x world
//!    transform) is rasterised into a grid of `WATER_CELL` cells. A cell is water when the
//!    terrain under it is at least `water_min_depth` below the surface (or there is no terrain,
//!    e.g. open sea past the plane's edge) and no scene object (dock, pier, building...) comes
//!    down there to within a boat's height of the water (statics.rs).
//! 2. Water cells become a flat mesh at the water height (row runs merged into strips). Islands
//!    and shorelines are just holes and edges in that mesh: no boundary tracing or polygon
//!    triangulation needed, so any shape works, including lakes with islands.
//! 3. Recast generates the navmesh from only those meshes, eroding the shores by the boat radius
//!    (`water_settings.agent_radius`) and building clean polygons.
//!
//! Terrain heights are sampled from the plane's vertex grid (bilinear, constant time per cell)
//! instead of raycasting every triangle per cell.
//!
//! Optionally (`terrain_max_water_depth`) the same mask cuts the *terrain* navmesh: where the
//! water is deeper than that, a downward-facing "ceiling" is placed just above the ground for the
//! terrain generation only. Recast sees too little headroom under it and leaves that ground out
//! (eroding the edge by the agent radius), so people can wade in the shallows but not walk along
//! the bottom of a lake.

use avian3d::prelude::{Collider, RigidBody};
use bevy::mesh::{Mesh, VertexAttributeValues};
use bevy::platform::collections::HashSet;
use bevy::prelude::*;
use bevy_rerecast::generator::NavmeshGenerator;
use bevy_rerecast::NavmeshSettings;

use bevy_pg_core::prelude::WaterChunk;
use crate::pgnavmesh::PGNavmeshType;
use crate::plugin::{NavStatic, NavmeshGenerationData};
use crate::terrain::CollidersReady;
use crate::statics::{stamp_footprint, world_triangles};

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

/// Temporary obstacle above deep-water ground, used only by the terrain navmesh generation.
#[derive(Component)]
pub(crate) struct UnderwaterCeiling;

/// All temporary navmesh sources (water surfaces, underwater ceilings) are registered by physics:
/// the terrain and water navmeshes can be generated.
#[derive(Event)]
pub(crate) struct NavSourcesReady;

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

    /// The terrain's XZ extent (world space): min and max corner.
    pub(crate) fn bounds(&self) -> (Vec2, Vec2) {
        let (x0, x1) = (self.xs[0], self.xs[self.xs.len() - 1]);
        let (z0, z1) = (self.zs[0], self.zs[self.zs.len() - 1]);
        (Vec2::new(x0, z0), Vec2::new(x1, z1))
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

/// Downward-facing quads `lift` above the ground over the mask's cells (cells without ground are
/// skipped). Facing down makes them unwalkable themselves; their low height leaves the ground
/// under them without headroom, so Recast drops it from the terrain navmesh.
pub(crate) fn ceiling_mesh(
    area: &WaterArea,
    cell: f32,
    cols: usize,
    rows: usize,
    mask: &[bool],
    ground: impl Fn(Vec2) -> Option<f32>,
    lift: f32,
) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for r in 0..rows {
        for c in 0..cols {
            if !mask[r * cols + c] {
                continue;
            }
            let lo = area.min + Vec2::new(c as f32, r as f32) * cell;
            let hi = (lo + Vec2::splat(cell)).min(area.max);
            let Some(centre) = ground((lo + hi) * 0.5) else { continue };
            let corner = |x: f32, z: f32| Vec3::new(x, ground(Vec2::new(x, z)).unwrap_or(centre) + lift, z);
            let base = vertices.len() as u32;
            vertices.extend([corner(lo.x, lo.y), corner(hi.x, lo.y), corner(hi.x, hi.y), corner(lo.x, hi.y)]);
            // Clockwise seen from above: the surface faces down.
            indices.push([base, base + 1, base + 2]);
            indices.push([base, base + 2, base + 3]);
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

/// Smallest input extent (world units) Recast can build a navmesh from with these settings:
/// border = walkable radius + 3 cells on each side, plus room for a few cells in between.
pub(crate) fn min_recast_extent(settings: &NavmeshSettings) -> f32 {
    let cell = settings.agent_radius / settings.cell_size_fraction.max(1e-3);
    let walkable_radius = (settings.agent_radius / cell.max(1e-6)).ceil();
    (2.0 * (walkable_radius + 3.0) + 4.0) * cell
}

/// Builds the boat-usable water surfaces once the terrain and nav-static colliders exist.
fn spawn_water_sources(
    _trigger:     On<CollidersReady>,
    mut commands: Commands,
    meshes:       Res<Assets<Mesh>>,
    mut navgendata: ResMut<NavmeshGenerationData>,
    terrains:     Query<(&Mesh3d, &GlobalTransform)>,
    waters:       Query<(&Mesh3d, &GlobalTransform), With<WaterChunk>>,
    statics:      Query<(&Mesh3d, &GlobalTransform, &NavStatic)>,
) {
    let make_water = navgendata.water_settings.is_some();
    let cut_depth = navgendata.terrain_max_water_depth;
    // Ceiling height above deep-water ground: above the climb (so Recast keeps it a separate span
    // from the ground) but below the agent height (so the ground has too little headroom).
    let terrain_settings = navgendata.terrain_settings.clone().unwrap_or_default();
    let ceiling_lift = (terrain_settings.walkable_climb + terrain_settings.agent_height) * 0.5;

    let terrain = terrains
        .get(navgendata.plane_entity)
        .ok()
        .and_then(|(mesh3d, transform)| HeightGrid::from_mesh(meshes.get(&mesh3d.0)?, transform));
    if terrain.is_none() {
        warn!("[NAV] [water] Terrain plane isn't a regular vertex grid; treating all water as deep");
    }
    // Ground of the neighbouring planes too: the water navmesh is built a border beyond the
    // plane's edges (plugin::build_box), and that border must see their shores.
    let neighbour_grounds: Vec<HeightGrid> = navgendata
        .neighbours
        .iter()
        .filter_map(|e| terrains.get(*e).ok())
        .filter_map(|(mesh3d, transform)| HeightGrid::from_mesh(meshes.get(&mesh3d.0)?, transform))
        .collect();
    let water_border = navgendata.water_settings.as_ref().map(crate::plugin::border_width).unwrap_or(0.0);

    // Scene objects block boats with their own triangles (statics.rs): docks, piers, buildings
    // standing in water... Only the parts within a boat's height above the water count, so a
    // high bridge doesn't block.
    let boat_height = navgendata.water_settings.as_ref().map(|s| s.agent_height).unwrap_or(2.0);
    let min_water_extent = navgendata.water_settings.as_ref().map(min_recast_extent).unwrap_or(0.0);
    let obstacle_triangles: Vec<[Vec3; 3]> = statics
        .iter()
        .filter(|(_, _, s)| s.water)
        .filter_map(|(mesh3d, transform, _)| Some(world_triangles(meshes.get(&mesh3d.0)?, transform)))
        .flatten()
        .collect();

    let ground = |p: Vec2| {
        terrain.as_ref().and_then(|t| t.height_at(p)).or_else(|| neighbour_grounds.iter().find_map(|g| g.height_at(p)))
    };
    let mut sources = 0;
    let mut ceilings = 0;
    for (mesh3d, transform) in &waters {
        let Some(mut area) = meshes.get(&mesh3d.0).and_then(|m| water_area(m, transform)) else { continue };
        // Only this plane's part of the water: with several terrain tiles in the scene, the
        // other tiles' sea would otherwise count as deep water (no ground known there).
        // Only this plane's water plus the border beyond its edges: with several terrain tiles
        // in the scene, the other tiles' sea would otherwise all end up in this navmesh.
        if let Some(terrain) = &terrain {
            let (lo, hi) = terrain.bounds();
            area.min = area.min.max(lo - water_border);
            area.max = area.max.min(hi + water_border);
            if area.min.x >= area.max.x || area.min.y >= area.max.y {
                continue; // water elsewhere
            }
        }

        if let Some(max_depth) = cut_depth {
            let (cols, rows, mask) = water_mask(&area, WATER_CELL, max_depth, ground, |_| false);
            let (vertices, indices) = ceiling_mesh(&area, WATER_CELL, cols, rows, &mask, ground, ceiling_lift);
            if !indices.is_empty() {
                commands.spawn((
                    Name::new("Underwater navmesh ceiling"),
                    UnderwaterCeiling,
                    Collider::trimesh(vertices, indices),
                    RigidBody::Static,
                    Transform::IDENTITY,
                ));
                ceilings += 1;
            }
        }

        if !make_water {
            continue;
        }
        let (cols, rows, mut mask) = water_mask(
            &area,
            WATER_CELL,
            navgendata.water_min_depth,
            ground,
            |_| false,
        );
        for tri in &obstacle_triangles {
            let low = tri[0].y.min(tri[1].y).min(tri[2].y);
            let (lo, hi) = (tri[0].xz().min(tri[1].xz()).min(tri[2].xz()), tri[0].xz().max(tri[1].xz()).max(tri[2].xz()));
            let outside = hi.x < area.min.x || hi.y < area.min.y || lo.x > area.max.x || lo.y > area.max.y;
            if outside || low > area.height + boat_height {
                continue;
            }
            stamp_footprint(&mut mask, cols, rows, area.min, WATER_CELL, tri.map(|v| v.xz()));
        }
        let (vertices, indices) = mask_to_mesh(&area, WATER_CELL, cols, rows, &mask);
        if indices.is_empty() {
            info!("[NAV] [water] Water plane at {:?} has no water deep enough for boats", area.min);
            continue;
        }
        // Recast (non-tiled) needs the input wider than twice its border, or it underflows
        // (rerecast contours: `width - border_size * 2`). Such a patch is too small for boats anyway.
        let extent = vertices.iter().fold((Vec2::splat(f32::MAX), Vec2::splat(f32::MIN)), |(lo, hi), v| (lo.min(v.xz()), hi.max(v.xz())));
        let extent = extent.1 - extent.0;
        // With a tile, the build box is fixed (plugin::build_box) and the small strips of the
        // neighbours' water in its border are needed to carry the navmesh up to the tile edge.
        if navgendata.tile.is_none() && (extent.x < min_water_extent || extent.y < min_water_extent) {
            info!("[NAV] [water] Deep water at {:?} is only {:.1} x {:.1}: too small for boats, skipped", area.min, extent.x, extent.y);
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
    info!("[NAV] [water] {sources} water surface(s), {ceilings} underwater ceiling(s) prepared");
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
    commands.trigger(NavSourcesReady); // terrain navmesh (terrain.rs)

    if navgendata.water_settings.is_none() {
        return;
    }
    if sources.is_empty() {
        info!("[NAV] [water] No navigable water: skipping the water navmesh");
        navgendata.skip(PGNavmeshType::Water);
        return;
    }
    let Some(mut settings) = navgendata.water_settings.clone() else { return };
    // Only the prepared water surfaces: terrain, statics and everything else stay out.
    settings.filter = Some(sources.iter().collect::<HashSet<Entity>>());
    if let Some(tile) = navgendata.tile {
        let (low, high) = navgendata.water_heights.iter().fold(navgendata.height_range, |(l, h), w| (l.min(*w), h.max(*w)));
        settings.aabb = Some(crate::plugin::build_box(tile, (low, high), &settings));
    }
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
    fn ceiling_covers_deep_water_ground_only_and_faces_down() {
        // Ground at -3 (deep) for x < 0, -0.2 (wading depth) for x >= 0.
        let ground = |p: Vec2| Some(if p.x < 0.0 { -3.0 } else { -0.2 });
        let (cols, rows, mask) = water_mask(&lake(), 1.0, 0.6, ground, |_| false);
        let (vertices, indices) = ceiling_mesh(&lake(), 1.0, cols, rows, &mask, ground, 1.0);
        assert_eq!(indices.len(), 10 * 20 * 2, "only the deep half");
        assert!(vertices.iter().all(|v| v.x <= 0.0), "no ceiling over the wading area");
        // 1 above the deep ground (corners on the x = 0 boundary follow the ground there).
        assert!(vertices.iter().filter(|v| v.x < -0.5).all(|v| (v.y - -2.0).abs() < 1e-4));
        for tri in &indices {
            let [a, b, c] = tri.map(|i| vertices[i as usize]);
            assert!((b - a).cross(c - a).y < 0.0, "faces down");
        }
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
