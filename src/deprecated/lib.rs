
mod functions;
mod navmesh;
mod pathfinding;
mod plugin;
mod terrain;
mod tools;
mod types;
mod triangles;

pub mod prelude {
    pub use crate::plugin::{GenerateNavMesh, NavConfig, PGNavPlugin};
    pub use crate::types::{NavType, NavStaticType, NavDebug, NavStatic, NavStaticShape};
    pub use crate::terrain::TerrainRayMeshData;
    pub use crate::pathfinding::Path;
    pub use crate::navmesh::{NavMesh, Polygon};
    pub use crate::tools::NavRay;

}