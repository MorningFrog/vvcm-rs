//! Error types returned by VVCM forward-kinematics and simulation APIs.

use thiserror::Error;

/// Error returned when a VVCM operation cannot produce a valid result.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum VvcmError {
    /// No candidate forward-kinematics solution could be constructed.
    #[error("no VVCM solution found")]
    NoSolution,

    /// Candidate solutions were found, but none passed the stability test.
    #[error("no stable VVCM solution found")]
    NoStableSolution,

    /// The robot formation cannot be realized by the supplied sheet geometry.
    #[error("robot formation is infeasible for the sheet shape")]
    InfeasibleFormation,

    /// An input collection has the wrong number of elements for the operation.
    #[error("dimension mismatch for {context}: expected {expected}, got {actual}")]
    DimensionMismatch {
        /// Human-readable name of the input being checked.
        context: &'static str,
        /// Required number of elements.
        expected: usize,
        /// Supplied number of elements.
        actual: usize,
    },
}
