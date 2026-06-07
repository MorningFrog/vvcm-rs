use thiserror::Error;

#[derive(Debug, Clone, PartialEq, Error)]
pub enum VvcmError {
    #[error("no VVCM solution found")]
    NoSolution,

    #[error("no stable VVCM solution found")]
    NoStableSolution,

    #[error("robot formation is infeasible for the sheet shape")]
    InfeasibleFormation,

    #[error("dimension mismatch for {context}: expected {expected}, got {actual}")]
    DimensionMismatch {
        context: &'static str,
        expected: usize,
        actual: usize,
    },

    #[error("VVCM algorithm is not implemented yet")]
    NotImplemented,
}
