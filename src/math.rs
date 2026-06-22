//! Internal matrix conversion helpers retained for tests and experiments.

#![allow(dead_code)]

use nalgebra::DMatrix;

use crate::types::{Point2, Scalar};

/// Converts an ordered point slice into an `n x 2` matrix of XY coordinates.
pub(crate) fn points_to_matrix(points: &[Point2]) -> DMatrix<Scalar> {
    DMatrix::from_fn(points.len(), 2, |row, col| match col {
        0 => points[row].x,
        1 => points[row].y,
        _ => unreachable!("matrix has exactly two columns"),
    })
}
