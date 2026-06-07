use crate::VvcmError;

pub type Scalar = f32;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point2 {
    pub x: Scalar,
    pub y: Scalar,
}

impl Point2 {
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Self { x, y }
    }

    pub fn zero() -> Self {
        Self::default()
    }

    pub fn scaled_by(self, factor: Scalar) -> Self {
        Self::new(self.x * factor, self.y * factor)
    }

    pub fn translated_by(self, offset: Point2) -> Self {
        Self::new(self.x + offset.x, self.y + offset.y)
    }

    pub fn relative_to(self, origin: Point2) -> Self {
        Self::new(self.x - origin.x, self.y - origin.y)
    }

    pub fn distance_to(self, other: Point2) -> Scalar {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point3 {
    pub x: Scalar,
    pub y: Scalar,
    pub z: Scalar,
}

impl Point3 {
    pub fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self::default()
    }

    pub fn translated_xy_by(self, offset: Point2) -> Self {
        Self::new(self.x + offset.x, self.y + offset.y, self.z)
    }

    pub fn relative_xy_to(self, origin: Point2) -> Self {
        Self::new(self.x - origin.x, self.y - origin.y, self.z)
    }

    pub fn distance_to(self, other: Point3) -> Scalar {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RobotFormation {
    points: Vec<Point2>,
}

impl RobotFormation {
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

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn points(&self) -> &[Point2] {
        &self.points
    }

    pub fn into_points(self) -> Vec<Point2> {
        self.points
    }

    pub fn centroid(&self) -> Point2 {
        let sum = self.points.iter().fold(Point2::zero(), |acc, point| {
            Point2::new(acc.x + point.x, acc.y + point.y)
        });
        sum.scaled_by(1.0 / self.points.len() as Scalar)
    }

    pub fn translated_by(&self, offset: Point2) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|point| point.translated_by(offset))
                .collect(),
        }
    }

    pub fn relative_to(&self, origin: Point2) -> Self {
        Self {
            points: self
                .points
                .iter()
                .map(|point| point.relative_to(origin))
                .collect(),
        }
    }

    pub fn all_zero(&self) -> bool {
        self.points
            .iter()
            .all(|point| point.x == 0.0 && point.y == 0.0)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SheetShape {
    vertices: Vec<Point2>,
}

impl SheetShape {
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

    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }

    pub fn vertices(&self) -> &[Point2] {
        &self.vertices
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct FkSolution {
    pub stable: bool,
    pub po: Point3,
    pub vo: Point2,
    pub taut_cables: Vec<usize>,
}

impl FkSolution {
    pub fn new(stable: bool, po: Point3, vo: Point2, taut_cables: Vec<usize>) -> Self {
        Self {
            stable,
            po,
            vo,
            taut_cables,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct FkSolutions {
    pub solutions: Vec<FkSolution>,
}

impl FkSolutions {
    pub fn new(solutions: Vec<FkSolution>) -> Self {
        Self { solutions }
    }

    pub fn is_empty(&self) -> bool {
        self.solutions.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &FkSolution> {
        self.solutions.iter()
    }

    pub fn stable(&self) -> impl Iterator<Item = &FkSolution> {
        self.solutions.iter().filter(|solution| solution.stable)
    }

    pub fn stable_count(&self) -> usize {
        self.stable().count()
    }

    pub fn all_count(&self) -> usize {
        self.solutions.len()
    }
}
