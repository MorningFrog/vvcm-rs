//! Public domain types used by the VVCM forward-kinematics API.
//!
//! These types intentionally keep the public surface independent from the
//! internal `nalgebra` matrices used by the solver. All coordinates and lengths
//! are represented with [`Scalar`] and must use a consistent length unit.

use crate::VvcmError;

/// Floating-point scalar used by the solver and public geometry types.
///
/// The current implementation uses `f32` to match the precision of the
/// original C++ fixtures and regression data.
pub type Scalar = f32;

/// A two-dimensional point or vector in the XY plane.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point2 {
    /// X coordinate.
    pub x: Scalar,
    /// Y coordinate.
    pub y: Scalar,
}

impl Point2 {
    /// Creates a point from explicit XY coordinates.
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Self { x, y }
    }

    /// Returns the origin `(0, 0)`.
    pub fn zero() -> Self {
        Self::default()
    }

    /// Multiplies both coordinates by `factor`.
    pub fn scaled_by(self, factor: Scalar) -> Self {
        Self::new(self.x * factor, self.y * factor)
    }

    /// Returns this point translated by the XY `offset`.
    pub fn translated_by(self, offset: Point2) -> Self {
        Self::new(self.x + offset.x, self.y + offset.y)
    }

    /// Returns this point expressed relative to `origin`.
    pub fn relative_to(self, origin: Point2) -> Self {
        Self::new(self.x - origin.x, self.y - origin.y)
    }

    /// Computes the Euclidean distance to another 2D point.
    pub fn distance_to(self, other: Point2) -> Scalar {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

/// A three-dimensional point or vector.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point3 {
    /// X coordinate.
    pub x: Scalar,
    /// Y coordinate.
    pub y: Scalar,
    /// Z coordinate.
    pub z: Scalar,
}

impl Point3 {
    /// Creates a point from explicit XYZ coordinates.
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self { x, y, z }
    }

    /// Returns the origin `(0, 0, 0)`.
    pub fn zero() -> Self {
        Self::default()
    }

    /// Returns this point translated in the XY plane, leaving Z unchanged.
    pub fn translated_xy_by(self, offset: Point2) -> Self {
        Self::new(self.x + offset.x, self.y + offset.y, self.z)
    }

    /// Returns this point expressed relative to an XY `origin`, leaving Z
    /// unchanged.
    pub fn relative_xy_to(self, origin: Point2) -> Self {
        Self::new(self.x - origin.x, self.y - origin.y, self.z)
    }

    /// Computes the Euclidean distance to another 3D point.
    pub fn distance_to(self, other: Point3) -> Scalar {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// Ordered XY positions for the robots or their velocities.
///
/// The point order must match the order of the sheet vertices supplied through
/// [`SheetShape`]. In solver calls, each point represents the robot endpoint of
/// the corresponding virtual cable.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotFormation {
    points: Vec<Point2>,
}

impl RobotFormation {
    /// Creates a non-empty robot formation.
    ///
    /// Returns [`VvcmError::DimensionMismatch`] when `points` is empty.
    pub fn new(points: Vec<Point2>) -> Result<Self, VvcmError> {
        if points.is_empty() {
            return Err(VvcmError::DimensionMismatch {
                context: "robot formation",
                expected: 1,
                actual: 0,
            });
        }

        Ok(Self { points })
    }

    /// Creates a zero-valued formation with one point per robot.
    ///
    /// This is primarily useful for velocity inputs where every robot is
    /// initially stationary. Returns [`VvcmError::DimensionMismatch`] when
    /// `robot_count` is zero.
    pub fn zeros(robot_count: usize) -> Result<Self, VvcmError> {
        if robot_count == 0 {
            return Err(VvcmError::DimensionMismatch {
                context: "robot formation",
                expected: 1,
                actual: 0,
            });
        }

        Ok(Self {
            points: vec![Point2::zero(); robot_count],
        })
    }

    /// Returns the number of points in the formation.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Returns `true` when the formation contains no points.
    ///
    /// Values constructed through [`RobotFormation::new`] are never empty; this
    /// method is provided for conventional collection-like access.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Borrows the ordered robot points.
    pub fn points(&self) -> &[Point2] {
        &self.points
    }

    /// Consumes the formation and returns the underlying ordered points.
    pub fn into_points(self) -> Vec<Point2> {
        self.points
    }

    /// Computes the arithmetic mean of all points.
    pub fn centroid(&self) -> Point2 {
        let sum = self.points.iter().fold(Point2::zero(), |acc, point| {
            Point2::new(acc.x + point.x, acc.y + point.y)
        });
        sum.scaled_by(1.0 / self.points.len() as Scalar)
    }

    /// Returns a new formation translated by `offset`.
    pub fn translated_by(&self, offset: Point2) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|point| point.translated_by(offset))
                .collect(),
        }
    }

    /// Returns a new formation expressed relative to `origin`.
    pub fn relative_to(&self, origin: Point2) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|point| point.relative_to(origin))
                .collect(),
        }
    }

    /// Returns `true` when every point is exactly `(0, 0)`.
    pub fn all_zero(&self) -> bool {
        self.points
            .iter()
            .all(|point| point.x == 0.0 && point.y == 0.0)
    }
}

/// Ordered sheet attachment vertices in the sheet-local XY frame.
///
/// The vertex order must match the robot order used by [`RobotFormation`].
#[derive(Debug, Clone, PartialEq)]
pub struct SheetShape {
    vertices: Vec<Point2>,
}

impl SheetShape {
    /// Creates a sheet shape with at least three vertices.
    ///
    /// Returns [`VvcmError::DimensionMismatch`] when fewer than three vertices
    /// are supplied.
    pub fn new(vertices: Vec<Point2>) -> Result<Self, VvcmError> {
        if vertices.len() < 3 {
            return Err(VvcmError::DimensionMismatch {
                context: "sheet shape",
                expected: 3,
                actual: vertices.len(),
            });
        }

        Ok(Self { vertices })
    }

    /// Returns the number of sheet vertices.
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    /// Returns `true` when the sheet contains no vertices.
    ///
    /// Values constructed through [`SheetShape::new`] are never empty; this
    /// method is provided for conventional collection-like access.
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }

    /// Borrows the ordered sheet vertices.
    pub fn vertices(&self) -> &[Point2] {
        &self.vertices
    }
}

/// A single forward-kinematics candidate solution.
///
/// `po` is the object position in the current formation frame, `vo` is the
/// corresponding virtual object point in the sheet-local frame, and
/// `taut_cables` contains the robot indices whose virtual cables are taut for
/// this solution.
#[derive(Debug, Clone, PartialEq, Default)]
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
}

impl FkSolution {
    /// Creates a forward-kinematics solution value.
    pub fn new(stable: bool, po: Point3, vo: Point2, taut_cables: Vec<usize>) -> Self {
        Self {
            stable,
            po,
            vo,
            taut_cables,
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
                left.po
                    .distance_to(reference)
                    .total_cmp(&right.po.distance_to(reference))
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
