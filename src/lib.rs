#![forbid(unsafe_code)]

pub mod error;
pub mod fk;
pub mod manual_simulation;
pub mod simulation;
pub mod types;

mod math;

pub use error::VvcmError;
pub use fk::VvcmFk;
pub use manual_simulation::VvcmManualSimulation;
pub use simulation::VvcmSimulation;
pub use types::{FkSolution, FkSolutions, Point2, Point3, RobotFormation, Scalar, SheetShape};
