use nalgebra::DMatrix;

use crate::types::{Point2, RobotFormation, Scalar, SheetShape};

pub(crate) fn formation_to_matrix(formation: &RobotFormation) -> DMatrix<Scalar> {
    points_to_matrix(formation.points())
}

pub(crate) fn sheet_to_matrix(sheet: &SheetShape) -> DMatrix<Scalar> {
    points_to_matrix(sheet.vertices())
}

fn points_to_matrix(points: &[Point2]) -> DMatrix<Scalar> {
    DMatrix::from_fn(points.len(), 2, |row, col| match col {
        0 => points[row].x,
        1 => points[row].y,
        _ => unreachable!("matrix has exactly two columns"),
    })
}
