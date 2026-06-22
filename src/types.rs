//! Public geometry and solution types used by the VVCM API.
//!
//! The Rust API exposes `nalgebra` point and vector types directly. Foreign
//! language bindings adapt their matrix or typed-array inputs at the boundary
//! instead of shaping the Rust core around language-neutral wrapper types.

use nalgebra::{
    Point2 as NalgebraPoint2, Point3 as NalgebraPoint3, Vector2 as NalgebraVector2,
    Vector3 as NalgebraVector3,
};

/// Floating-point scalar used by the solver and public geometry types.
///
/// The current implementation uses `f32` for compact numeric data and
/// compatibility with the bundled fixtures and regression data.
pub type Scalar = f32;

/// A two-dimensional point in the XY plane.
pub type Point2 = NalgebraPoint2<Scalar>;

/// A three-dimensional point.
pub type Point3 = NalgebraPoint3<Scalar>;

/// A two-dimensional vector in the XY plane.
pub type Vector2 = NalgebraVector2<Scalar>;

/// A three-dimensional vector.
pub type Vector3 = NalgebraVector3<Scalar>;

/// Converts common row-like values into a [`Point2`].
///
/// This local trait exists because the crate cannot implement standard
/// conversion traits for `nalgebra` point types it does not own.
pub trait IntoPoint2 {
    /// Converts this value into a `nalgebra` 2D point.
    fn into_point2(self) -> Point2;
}

impl IntoPoint2 for Point2 {
    fn into_point2(self) -> Point2 {
        self
    }
}

impl IntoPoint2 for &Point2 {
    fn into_point2(self) -> Point2 {
        *self
    }
}

impl IntoPoint2 for [Scalar; 2] {
    fn into_point2(self) -> Point2 {
        Point2::new(self[0], self[1])
    }
}

impl IntoPoint2 for &[Scalar; 2] {
    fn into_point2(self) -> Point2 {
        Point2::new(self[0], self[1])
    }
}

impl IntoPoint2 for (Scalar, Scalar) {
    fn into_point2(self) -> Point2 {
        Point2::new(self.0, self.1)
    }
}

impl IntoPoint2 for &(Scalar, Scalar) {
    fn into_point2(self) -> Point2 {
        Point2::new(self.0, self.1)
    }
}

/// Converts common row-like values into a [`Point3`].
pub trait IntoPoint3 {
    /// Converts this value into a `nalgebra` 3D point.
    fn into_point3(self) -> Point3;
}

impl IntoPoint3 for Point3 {
    fn into_point3(self) -> Point3 {
        self
    }
}

impl IntoPoint3 for &Point3 {
    fn into_point3(self) -> Point3 {
        *self
    }
}

impl IntoPoint3 for [Scalar; 3] {
    fn into_point3(self) -> Point3 {
        Point3::new(self[0], self[1], self[2])
    }
}

impl IntoPoint3 for &[Scalar; 3] {
    fn into_point3(self) -> Point3 {
        Point3::new(self[0], self[1], self[2])
    }
}

impl IntoPoint3 for (Scalar, Scalar, Scalar) {
    fn into_point3(self) -> Point3 {
        Point3::new(self.0, self.1, self.2)
    }
}

impl IntoPoint3 for &(Scalar, Scalar, Scalar) {
    fn into_point3(self) -> Point3 {
        Point3::new(self.0, self.1, self.2)
    }
}

/// Converts a row-like value into a [`Point2`].
pub fn point2(value: impl IntoPoint2) -> Point2 {
    value.into_point2()
}

/// Converts a row-like value into a [`Point3`].
pub fn point3(value: impl IntoPoint3) -> Point3 {
    value.into_point3()
}

/// Returns the origin `(0, 0)`.
pub fn point2_zero() -> Point2 {
    Point2::new(0.0, 0.0)
}

/// Returns the origin `(0, 0, 0)`.
pub fn point3_zero() -> Point3 {
    Point3::new(0.0, 0.0, 0.0)
}

/// Computes the Euclidean distance between two 2D points.
pub fn distance2(left: Point2, right: Point2) -> Scalar {
    (left - right).norm()
}

/// Computes the Euclidean distance between two 3D points.
pub fn distance3(left: Point3, right: Point3) -> Scalar {
    (left - right).norm()
}

/// Returns `point` shifted by `origin`, preserving the Z coordinate.
pub fn relative_xy_to(point: Point3, origin: Point2) -> Point3 {
    Point3::new(point.x - origin.x, point.y - origin.y, point.z)
}

/// Returns `point` translated by an XY offset, preserving the Z coordinate.
pub fn translated_xy_by(point: Point3, offset: Point2) -> Point3 {
    Point3::new(point.x + offset.x, point.y + offset.y, point.z)
}

/// A single forward-kinematics candidate solution.
///
/// `po` is the object position in the current formation frame, `vo` is the
/// corresponding virtual object point in the sheet-local frame, and
/// `taut_cables` contains the robot indices whose virtual cables are taut for
/// this solution. `lambda_values` stores the corresponding Lagrange multiplier
/// coefficients in the same order as `taut_cables`.
#[derive(Debug, Clone, PartialEq)]
pub struct FkSolution {
    /// Whether the candidate is locally stable according to the VVCM stability
    /// test.
    pub stable: bool,
    /// Object position `Po` in the formation-local frame.
    pub po: Point3,
    /// Virtual object point `Vo` in the sheet-local XY frame.
    pub vo: Point2,
    /// Indices of the taut virtual cables for this candidate.
    pub taut_cables: Vec<usize>,
    /// Lagrange multiplier coefficients for the taut virtual cables.
    ///
    /// This is a taut-only vector: `lambda_values[i]` corresponds to
    /// `taut_cables[i]`. Slack cables are omitted rather than represented by
    /// zero-valued placeholders.
    pub lambda_values: Vec<Scalar>,
}

impl Default for FkSolution {
    fn default() -> Self {
        Self {
            stable: false,
            po: point3_zero(),
            vo: point2_zero(),
            taut_cables: Vec::new(),
            lambda_values: Vec::new(),
        }
    }
}

impl FkSolution {
    /// Creates a forward-kinematics solution value.
    pub fn new(stable: bool, po: Point3, vo: Point2, taut_cables: Vec<usize>) -> Self {
        Self::new_with_lambda_values(stable, po, vo, taut_cables, Vec::new())
    }

    /// Creates a forward-kinematics solution value with taut-cable lambda
    /// coefficients.
    pub fn new_with_lambda_values(
        stable: bool,
        po: Point3,
        vo: Point2,
        taut_cables: Vec<usize>,
        lambda_values: Vec<Scalar>,
    ) -> Self {
        Self {
            stable,
            po,
            vo,
            taut_cables,
            lambda_values,
        }
    }
}

/// Collection of forward-kinematics candidates returned by [`crate::VvcmFk`].
///
/// Stable and unstable candidates are kept in one ordered list; inspect each
/// [`FkSolution::stable`] flag or use [`FkSolutions::stable`] to filter stable
/// branches.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FkSolutions {
    /// All candidate solutions found during the most recent FK update.
    pub solutions: Vec<FkSolution>,
}

impl FkSolutions {
    /// Creates a solution collection from an ordered list of candidates.
    pub fn new(solutions: Vec<FkSolution>) -> Self {
        Self { solutions }
    }

    /// Returns `true` when no candidates are stored.
    pub fn is_empty(&self) -> bool {
        self.solutions.is_empty()
    }

    /// Iterates over all candidate solutions.
    pub fn iter(&self) -> impl Iterator<Item = &FkSolution> {
        self.solutions.iter()
    }

    /// Iterates over locally stable candidate solutions.
    pub fn stable(&self) -> impl Iterator<Item = &FkSolution> {
        self.solutions.iter().filter(|solution| solution.stable)
    }

    /// Finds the stable solution whose object position is closest to
    /// `reference`.
    ///
    /// Returns the solution index in [`FkSolutions::solutions`] together with a
    /// shared reference to the solution.
    pub fn closest_stable_to(&self, reference: Point3) -> Option<(usize, &FkSolution)> {
        self.iter()
            .enumerate()
            .filter(|(_, solution)| solution.stable)
            .min_by(|(_, left), (_, right)| {
                distance3(left.po, reference).total_cmp(&distance3(right.po, reference))
            })
    }

    /// Counts locally stable candidate solutions.
    pub fn stable_count(&self) -> usize {
        self.stable().count()
    }

    /// Counts all candidate solutions, stable and unstable.
    pub fn all_count(&self) -> usize {
        self.solutions.len()
    }
}
