
mod functions;
mod navmesh;
mod pathfinding;
mod plugin;
mod terrain;
mod tools;
mod types;


pub mod prelude {
    pub use crate::plugin::{GenerateNavMesh, NavConfig, PGNavPlugin};
    pub use crate::types::{NavType, NavDebug, NavStatic, NavStaticShape};

    pub use crate::pathfinding::Path;
    pub use crate::navmesh::{NavMesh, Polygon};

}