//! Internal matrix conversion helpers.
//!
//! These adapters keep nalgebra-specific conversions outside the public domain
//! types. They are currently retained for porting work and future numerical
//! comparisons against matrix-oriented implementations.

#![allow(dead_code)]

use nalgebra::DMatrix;

use crate::types::{Point2, RobotFormation, Scalar, SheetShape};

/// Converts a robot formation into an `n x 2` matrix of XY coordinates.
pub(crate) fn formation_to_matrix(formation: &RobotFormation) -> DMatrix<Scalar> {
    points_to_matrix(formation.points())
}

/// Converts a sheet shape into an `n x 2` matrix of XY coordinates.
pub(crate) fn sheet_to_matrix(sheet: &SheetShape) -> DMatrix<Scalar> {
    points_to_matrix(sheet.vertices())
}

/// Converts an ordered point slice into an `n x 2` matrix.
fn points_to_matrix(points: &[Point2]) -> DMatrix<Scalar> {
    DMatrix::from_fn(points.len(), 2, |row, col| match col {
        0 => points[row].x,
        1 => points[row].y,
        _ => unreachable!("matrix has exactly two columns"),
    })
}
