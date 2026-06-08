#![forbid(unsafe_code)]
#![warn(missing_docs)]

//! Forward kinematics and simulation utilities for the Virtual Variable Cables
//! Model (VVCM).
//!
//! The crate exposes small domain types such as [`Point2`],
//! [`RobotFormation`], [`SheetShape`], and [`FkSolution`] while keeping
//! `nalgebra` as an internal numerical backend. Length values are unitless to
//! the type system, but the sample data and examples use millimeters.
//!
//! # Basic usage
//!
//! ```rust
//! use vvcm_rs::{Point2, RobotFormation, SheetShape, VvcmFk};
//!
//! let formation = RobotFormation::new(vec![
//!     Point2::new(213.7, 122.7),
//!     Point2::new(804.6, 37.2),
//!     Point2::new(904.0, 550.0),
//!     Point2::new(439.3, 715.9),
//! ])?;
//!
//! let sheet = SheetShape::new(vec![
//!     Point2::new(-316.1, -421.9),
//!     Point2::new(803.4, -384.1),
//!     Point2::new(746.1, 712.8),
//!     Point2::new(-367.3, 664.2),
//! ])?;
//!
//! let mut fk = VvcmFk::new(4, 1000.0, sheet)?;
//! let solutions = fk.update_stable_solutions(formation)?;
//!
//! assert!(solutions.stable_count() > 0);
//! # Ok::<(), vvcm_rs::VvcmError>(())
//! ```

pub mod error;
pub mod fk;
pub mod manual_simulation;
pub mod simulation;
pub mod types;

mod math;
mod python;

pub use error::VvcmError;
pub use fk::VvcmFk;
pub use manual_simulation::VvcmManualSimulation;
pub use simulation::VvcmSimulation;
pub use types::{FkSolution, FkSolutions, Point2, Point3, RobotFormation, Scalar, SheetShape};
