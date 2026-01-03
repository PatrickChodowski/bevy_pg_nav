mod debug;
mod pathfinding;
mod plugin;
mod recast_convert;
mod water;
mod terrain;
mod tools;
mod types;

pub mod prelude {
    pub use crate::debug::display_polygon;
    pub use crate::pathfinding::{Path, PathFinder, SearchStep};
    pub use crate::plugin::{
        GenerateNavMesh, NavConfig, PGNavPlugin, NavStatic, 
        NavStaticShape, NavStaticType, NavmeshTerrain, NavmeshWater, PGNavmeshType
    };
    pub use crate::terrain::TerrainRayMeshData;
    pub use crate::types::{PGNavmesh, PGPolygon, PGVertex, find_point};
}