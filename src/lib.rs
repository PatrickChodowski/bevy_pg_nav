mod pathfinding;
mod pgnavmesh;
mod plugin;
mod recast_convert;
mod water;
mod terrain;
mod tools;
mod types;
mod bvh;
mod multi_nav;
mod mesh;
mod statics;

pub mod prelude {
    pub use crate::pathfinding::{Path, PathFinder, SearchStep};
    pub use crate::plugin::{
        GenerateNavMesh, PGNavPlugin, NavmeshTerrain, NavmeshWater, NavStatic, NavResources
    };
    pub use crate::statics::{NavOverrides, NavOverrideRule, NavShape};
    pub use crate::mesh::{PGNavmeshMesh, spawn_navmesh_mesh};
    pub use crate::terrain::TerrainRayMeshData;
    pub use crate::types::{PGPolygon, PGVertex};
    pub use crate::pgnavmesh::{PGNavmesh, PGNavmeshType, find_point};
    pub use crate::multi_nav::{multi_nav_path_points, multi_nav_clamp_move};
}