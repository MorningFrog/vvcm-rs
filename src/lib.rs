#![deny(unsafe_code)]
#![warn(missing_docs)]

//! Forward kinematics and simulation utilities for the Virtual Variable Cables
//! Model (VVCM).
//!
//! The crate exposes `nalgebra` point and vector types directly. Length values
//! are unitless to the type system and must be consistent across each solve.
//! The FK engine normalizes coordinates internally for numerical stability and
//! maps results back to the caller's coordinate frames.
//!
//! # Basic usage
//!
//! ```rust
//! use vvcm_rs::{point2, Point2, VvcmFk};
//!
//! let formation: Vec<Point2> = vec![
//!     point2((213.7, 122.7)),
//!     point2((804.6, 37.2)),
//!     point2((904.0, 550.0)),
//!     point2((439.3, 715.9)),
//! ];
//!
//! let sheet: Vec<Point2> = vec![
//!     point2((-316.1, -421.9)),
//!     point2((803.4, -384.1)),
//!     point2((746.1, 712.8)),
//!     point2((-367.3, 664.2)),
//! ];
//!
//! let mut fk = VvcmFk::new(1000.0, sheet)?;
//! let solutions = fk.update_stable_solutions(&formation)?;
//!
//! assert!(solutions.stable_count() > 0);
//! # Ok::<(), vvcm_rs::VvcmError>(())
//! ```

pub mod error;
#[doc(hidden)]
#[cfg(not(all(feature = "wasm", target_arch = "wasm32")))]
#[allow(unsafe_code)]
pub mod ffi;
pub mod fk;
pub mod manual_simulation;
pub mod simulation;
pub mod types;

mod math;
#[cfg(feature = "python-extension")]
mod python;
#[cfg(all(feature = "wasm", target_arch = "wasm32"))]
pub mod wasm;

pub use error::VvcmError;
pub use fk::VvcmFk;
pub use manual_simulation::VvcmManualSimulation;
pub use simulation::VvcmSimulation;
pub use types::{
    FkSolution, FkSolutions, IntoPoint2, IntoPoint3, Point2, Point3, Scalar, Vector2, Vector3,
    distance2, distance3, point2, point2_zero, point3, point3_zero, relative_xy_to,
    translated_xy_by,
};
