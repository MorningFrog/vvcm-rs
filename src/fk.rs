//! Forward-kinematics solver for the Virtual Variable Cables Model.
//!
//! A [`VvcmFk`] instance owns the fixed sheet geometry and solves repeated robot
//! formations against it. Each update enumerates candidate taut cable sets,
//! solves the corresponding constrained system, and marks locally stable
//! branches in the returned [`FkSolutions`] collection.

use crate::error::VvcmError;
use crate::types::{FkSolution, FkSolutions, Point2, Point3, RobotFormation, Scalar, SheetShape};
use nalgebra::{DMatrix, DVector};

// Numerical thresholds are intentionally local to the solver because changing
// them affects branch enumeration and stability classification.
const RANK_EPS: Scalar = 1.0e-4;
const STABILITY_EPS: Scalar = 1.0e-8;
const SLACK_EPS: Scalar = 1.0e-8;
const NORMALIZED_COORDINATE_UPPER_BOUND: Scalar = 1000.0;

/// Stateful forward-kinematics engine for a fixed deformable sheet.
///
/// The sheet geometry and robot count are fixed at construction time. Calling
/// [`VvcmFk::update_stable_solutions`] replaces the current formation and
/// solution cache with the result for the supplied formation.
#[derive(Debug, Clone)]
pub struct VvcmFk {
    robot_count: usize,
    hold_height: Scalar,
    sheet: SheetShape,
    formation: Option<RobotFormation>,
    solutions: FkSolutions,
    last_normalization: Option<NormalizationTransform>,
}

impl VvcmFk {
    /// Creates a solver for `robot_count` robots holding `sheet` at
    /// `hold_height`.
    ///
    /// `sheet` must contain exactly one vertex per robot, and each vertex must
    /// be ordered to match the robot formation points passed to
    /// [`VvcmFk::update_stable_solutions`].
    ///
    /// # Errors
    ///
    /// Returns [`VvcmError::DimensionMismatch`] when `robot_count` is less than
    /// three or when the sheet vertex count does not match `robot_count`.
    ///
    /// # Units
    ///
    /// Length units are not encoded in the type system. Inputs only need to use
    /// one consistent length unit; the solver normalizes coordinates
    /// internally and maps results back to the caller's original frame.
    pub fn new(
        robot_count: usize,
        hold_height: Scalar,
        sheet: SheetShape,
    ) -> Result<Self, VvcmError> {
        if robot_count < 3 {
            return Err(VvcmError::DimensionMismatch {
                context: "robot count",
                expected: 3,
                actual: robot_count,
            });
        }

        if sheet.len() != robot_count {
            return Err(VvcmError::DimensionMismatch {
                context: "sheet vertex count",
                expected: robot_count,
                actual: sheet.len(),
            });
        }

        Ok(Self {
            robot_count,
            hold_height,
            sheet,
            formation: None,
            solutions: FkSolutions::default(),
            last_normalization: None,
        })
    }

    /// Solves and stores the stable forward-kinematics branches for
    /// `formation`.
    ///
    /// The formation must contain exactly one point per robot. This method
    /// clears the previous solution cache before solving. If candidate
    /// solutions are found but none are stable, the candidate list remains
    /// available through [`VvcmFk::solutions`] with every
    /// [`FkSolution::stable`] flag set to `false`.
    ///
    /// # Errors
    ///
    /// Returns [`VvcmError::DimensionMismatch`] for a wrong formation size,
    /// [`VvcmError::InfeasibleFormation`] when the formation stretches the
    /// sheet beyond its pairwise distances, [`VvcmError::NoSolution`] when no
    /// candidate branch can be constructed, or
    /// [`VvcmError::NoStableSolution`] when every candidate is unstable.
    pub fn update_stable_solutions(
        &mut self,
        formation: RobotFormation,
    ) -> Result<&FkSolutions, VvcmError> {
        self.validate_formation(&formation)?;

        self.solutions = FkSolutions::default();
        self.formation = Some(formation.clone());

        let normalized = NormalizedProblem::new(&formation, &self.sheet, self.hold_height);
        self.last_normalization = Some(normalized.transform);

        let mut candidates = self.find_candidate_solutions(
            &normalized.formation,
            &normalized.sheet,
            normalized.hold_height,
        )?;
        mark_stable_solutions(&mut candidates);

        let transform = self
            .last_normalization
            .expect("normalization transform is set before solving");
        self.solutions = FkSolutions::new(
            candidates
                .into_iter()
                .map(|candidate| denormalize_solution(candidate.solution, transform))
                .collect(),
        );

        if self.solutions.stable_count() == 0 {
            Err(VvcmError::NoStableSolution)
        } else {
            Ok(&self.solutions)
        }
    }

    /// Returns the fixed number of robots solved by this engine.
    pub fn robot_count(&self) -> usize {
        self.robot_count
    }

    /// Returns the fixed robot holding height used to recover the object Z
    /// coordinate.
    pub fn hold_height(&self) -> Scalar {
        self.hold_height
    }

    /// Borrows the fixed sheet geometry.
    pub fn sheet(&self) -> &SheetShape {
        &self.sheet
    }

    /// Borrows the most recent formation passed to
    /// [`VvcmFk::update_stable_solutions`].
    pub fn current_formation(&self) -> Option<&RobotFormation> {
        self.formation.as_ref()
    }

    /// Borrows the most recent solution cache.
    pub fn solutions(&self) -> &FkSolutions {
        &self.solutions
    }

    /// Checks that a formation-like value has one point per robot.
    pub(crate) fn validate_formation(&self, formation: &RobotFormation) -> Result<(), VvcmError> {
        if formation.len() != self.robot_count {
            return Err(VvcmError::DimensionMismatch {
                context: "robot formation point count",
                expected: self.robot_count,
                actual: formation.len(),
            });
        }

        Ok(())
    }

    /// Enumerates taut cable subsets and solves every feasible candidate branch.
    fn find_candidate_solutions(
        &self,
        formation: &RobotFormation,
        sheet: &SheetShape,
        hold_height: Scalar,
    ) -> Result<Vec<CandidateSolution>, VvcmError> {
        let problem = ProblemData::new(formation, sheet);

        if !formation_feasible(&problem) {
            return Err(VvcmError::InfeasibleFormation);
        }

        let mut candidates = Vec::new();
        // At least three taut cables are required to locate the object, while
        // more than five taut cables are not needed for the supported cases.
        let max_taut_count = self.robot_count.min(5);

        for taut_count in 3..=max_taut_count {
            enumerate_combinations(self.robot_count, taut_count, |taut_cables| {
                if let Some(candidate) = self.solve_for_taut_set(&problem, hold_height, taut_cables)
                {
                    candidates.push(candidate);
                }
            });
        }

        if candidates.is_empty() {
            Err(VvcmError::NoSolution)
        } else {
            Ok(candidates)
        }
    }

    /// Solves one candidate branch for a chosen taut cable set.
    fn solve_for_taut_set(
        &self,
        problem: &ProblemData,
        hold_height: Scalar,
        taut_cables: &[usize],
    ) -> Option<CandidateSolution> {
        let taut_count = taut_cables.len();
        let id1 = taut_cables[0];

        // Every cable length equation is expressed relative to the first taut
        // cable. Taut rows form the equality system; non-taut rows are reused
        // later as strict slack checks.
        let row_count = self.robot_count - 1;
        let mut a = DMatrix::<Scalar>::zeros(row_count, 4);
        let mut b = DVector::<Scalar>::zeros(row_count);

        for (row, &id) in taut_cables[1..].iter().enumerate() {
            fill_constraint_row(problem, id1, id, row, &mut a, &mut b);
        }

        let mut slack_row = 0;
        for id in 0..self.robot_count {
            if taut_cables.contains(&id) {
                continue;
            }

            fill_constraint_row(problem, id1, id, taut_count - 1 + slack_row, &mut a, &mut b);
            slack_row += 1;
        }

        let constraint_count = taut_count - 1;
        let a1 = a.rows(0, constraint_count).into_owned();
        let b1 = b.rows(0, constraint_count).into_owned();
        let a1_bar = append_column(&a1, &b1);

        // Reject inconsistent taut equality systems before building the
        // Lagrange system.
        if matrix_rank(&a1) != matrix_rank(&a1_bar) {
            return None;
        }

        let independent_taut_count = constraint_count + 1;

        let c = [
            -2.0 * problem.formation_x[id1],
            -2.0 * problem.formation_y[id1],
            2.0 * problem.sheet_x[id1],
            2.0 * problem.sheet_y[id1],
        ];
        let f0 = problem.formation_norm_squared[id1] - problem.sheet_norm_squared[id1];

        // The Lagrange solve returns the stationary point for the four planar
        // unknowns: object XY and virtual-object XY.
        let solution = solve_lagrange_system(c, &a1, &b1)?;
        let x_bar = solution.rows(0, 4).into_owned();
        let lambda_raw = solution
            .rows(solution.len() - constraint_count, constraint_count)
            .into_owned();
        let lambda = expand_lambda(&lambda_raw);

        let term1 =
            x_bar[0] * x_bar[0] + x_bar[1] * x_bar[1] - x_bar[2] * x_bar[2] - x_bar[3] * x_bar[3];
        let term2 = c[0] * x_bar[0] + c[1] * x_bar[1] + c[2] * x_bar[2] + c[3] * x_bar[3];
        let tmp = -(term1 + term2 + f0);

        // `tmp` is the squared vertical drop from the robot holding plane. A
        // negative value has no real-valued object height.
        if tmp < 0.0 {
            return None;
        }

        let x_o = x_bar[0];
        let y_o = x_bar[1];
        let x_vo = x_bar[2];
        let y_vo = x_bar[3];
        let z_o = hold_height - tmp.sqrt();

        let taut_polygon: Vec<Point2> = taut_cables
            .iter()
            .map(|&idx| Point2::new(problem.formation_x[idx], problem.formation_y[idx]))
            .collect();
        if !in_polygon(Point2::new(x_o, y_o), &taut_polygon) {
            return None;
        }

        // Slack cables must remain strictly slack for this taut-set hypothesis.
        for row in constraint_count..row_count {
            let residual = b[row]
                - (a[(row, 0)] * x_bar[0]
                    + a[(row, 1)] * x_bar[1]
                    + a[(row, 2)] * x_bar[2]
                    + a[(row, 3)] * x_bar[3]);
            if residual <= SLACK_EPS {
                return None;
            }
        }

        Some(CandidateSolution {
            solution: FkSolution::new(
                false,
                Point3::new(x_o, y_o, z_o),
                Point2::new(x_vo, y_vo),
                taut_cables.to_vec(),
            ),
            taut_count,
            independent_taut_count,
            lambda,
            omega: None,
        })
    }
}

/// Candidate solution plus intermediate values needed by stability filtering.
#[derive(Debug, Clone)]
struct CandidateSolution {
    solution: FkSolution,
    taut_count: usize,
    independent_taut_count: usize,
    lambda: DVector<Scalar>,
    omega: Option<DMatrix<Scalar>>,
}

/// Solver inputs after shifting to positive coordinates and uniform scaling.
#[derive(Debug, Clone)]
struct NormalizedProblem {
    formation: RobotFormation,
    sheet: SheetShape,
    hold_height: Scalar,
    transform: NormalizationTransform,
}

impl NormalizedProblem {
    /// Builds normalized inputs while preserving the transform needed to map
    /// solutions back to the caller's original coordinate frames.
    fn new(formation: &RobotFormation, sheet: &SheetShape, hold_height: Scalar) -> Self {
        let transform = NormalizationTransform::new(formation, sheet);

        Self {
            formation: transform.normalize_formation(formation),
            sheet: transform.normalize_sheet(sheet),
            hold_height: hold_height * transform.scale,
            transform,
        }
    }
}

/// Affine transform used only inside one FK update.
#[derive(Debug, Clone, Copy)]
struct NormalizationTransform {
    formation_origin: Point2,
    sheet_origin: Point2,
    scale: Scalar,
}

impl NormalizationTransform {
    /// Chooses independent positive-coordinate origins and one shared scale.
    fn new(formation: &RobotFormation, sheet: &SheetShape) -> Self {
        let formation_bounds = PointBounds::from_points(formation.points());
        let sheet_bounds = PointBounds::from_points(sheet.vertices());
        let upper = formation_bounds.upper().max(sheet_bounds.upper());

        Self {
            formation_origin: formation_bounds.origin,
            sheet_origin: sheet_bounds.origin,
            scale: normalization_scale(upper),
        }
    }

    /// Returns the formation expressed in the normalized solve frame.
    fn normalize_formation(self, formation: &RobotFormation) -> RobotFormation {
        RobotFormation::new(
            formation
                .points()
                .iter()
                .map(|point| self.normalize_point(*point, self.formation_origin))
                .collect(),
        )
        .expect("validated formation cannot be empty")
    }

    /// Returns the sheet expressed in the normalized solve frame.
    fn normalize_sheet(self, sheet: &SheetShape) -> SheetShape {
        SheetShape::new(
            sheet
                .vertices()
                .iter()
                .map(|point| self.normalize_point(*point, self.sheet_origin))
                .collect(),
        )
        .expect("validated sheet cannot have fewer than three vertices")
    }

    /// Shifts a point to a positive-coordinate origin and applies the shared
    /// scale.
    fn normalize_point(self, point: Point2, origin: Point2) -> Point2 {
        point.relative_to(origin).scaled_by(self.scale)
    }

    /// Maps a normalized object position back to the formation frame supplied
    /// by the caller.
    fn denormalize_object_position(self, point: Point3) -> Point3 {
        let inverse_scale = 1.0 / self.scale;

        Point3::new(
            point.x * inverse_scale + self.formation_origin.x,
            point.y * inverse_scale + self.formation_origin.y,
            point.z * inverse_scale,
        )
    }

    /// Maps a normalized virtual object point back to the caller's sheet frame.
    fn denormalize_virtual_object(self, point: Point2) -> Point2 {
        let inverse_scale = 1.0 / self.scale;

        Point2::new(
            point.x * inverse_scale + self.sheet_origin.x,
            point.y * inverse_scale + self.sheet_origin.y,
        )
    }
}

/// Min corner and positive coordinate span for one XY point set.
#[derive(Debug, Clone, Copy)]
struct PointBounds {
    origin: Point2,
    max_x: Scalar,
    max_y: Scalar,
}

impl PointBounds {
    /// Measures the bounding box that results after shifting by the min corner.
    fn from_points(points: &[Point2]) -> Self {
        debug_assert!(!points.is_empty());

        let mut min_x = points[0].x;
        let mut min_y = points[0].y;
        let mut max_x = points[0].x;
        let mut max_y = points[0].y;

        for point in &points[1..] {
            if point.x < min_x {
                min_x = point.x;
            }
            if point.y < min_y {
                min_y = point.y;
            }
            if point.x > max_x {
                max_x = point.x;
            }
            if point.y > max_y {
                max_y = point.y;
            }
        }

        Self {
            origin: Point2::new(min_x, min_y),
            max_x: max_x - min_x,
            max_y: max_y - min_y,
        }
    }

    /// Returns this point set's largest shifted XY coordinate.
    fn upper(self) -> Scalar {
        self.max_x.max(self.max_y)
    }
}

/// Chooses the uniform scale factor for normalized FK solves.
fn normalization_scale(upper: Scalar) -> Scalar {
    if upper.is_finite() && upper > 0.0 {
        let scale = NORMALIZED_COORDINATE_UPPER_BOUND / upper;
        if scale.is_finite() && scale > 0.0 {
            return scale;
        }
    }

    1.0
}

/// Maps a normalized FK candidate back to the caller's coordinate frames.
fn denormalize_solution(mut solution: FkSolution, transform: NormalizationTransform) -> FkSolution {
    solution.po = transform.denormalize_object_position(solution.po);
    solution.vo = transform.denormalize_virtual_object(solution.vo);
    solution
}

/// Precomputed coordinates and pairwise constraint rows for one FK update.
#[derive(Debug, Clone)]
struct ProblemData {
    formation_x: Vec<Scalar>,
    formation_y: Vec<Scalar>,
    formation_norm_squared: Vec<Scalar>,
    sheet_x: Vec<Scalar>,
    sheet_y: Vec<Scalar>,
    sheet_norm_squared: Vec<Scalar>,
    constraint_rows: Vec<ConstraintRow>,
}

impl ProblemData {
    /// Builds cached coordinate arrays and all pairwise linear constraints.
    fn new(formation: &RobotFormation, sheet: &SheetShape) -> Self {
        let point_count = formation.len();
        let mut formation_x = Vec::with_capacity(point_count);
        let mut formation_y = Vec::with_capacity(point_count);
        let mut formation_norm_squared = Vec::with_capacity(point_count);
        let mut sheet_x = Vec::with_capacity(point_count);
        let mut sheet_y = Vec::with_capacity(point_count);
        let mut sheet_norm_squared = Vec::with_capacity(point_count);

        for (formation_point, sheet_point) in formation.points().iter().zip(sheet.vertices()) {
            formation_x.push(formation_point.x);
            formation_y.push(formation_point.y);
            formation_norm_squared.push(
                formation_point.x * formation_point.x + formation_point.y * formation_point.y,
            );
            sheet_x.push(sheet_point.x);
            sheet_y.push(sheet_point.y);
            sheet_norm_squared.push(sheet_point.x * sheet_point.x + sheet_point.y * sheet_point.y);
        }

        let mut constraint_rows = Vec::with_capacity(point_count * point_count);
        // The constraint row for (id1, id) is the linearized difference between
        // the two cable-length equations.
        for id1 in 0..point_count {
            for id in 0..point_count {
                constraint_rows.push(ConstraintRow {
                    a: [
                        formation_x[id1] - formation_x[id],
                        formation_y[id1] - formation_y[id],
                        sheet_x[id] - sheet_x[id1],
                        sheet_y[id] - sheet_y[id1],
                    ],
                    b: 0.5
                        * (formation_norm_squared[id1] - sheet_norm_squared[id1]
                            + sheet_norm_squared[id]
                            - formation_norm_squared[id]),
                });
            }
        }

        Self {
            formation_x,
            formation_y,
            formation_norm_squared,
            sheet_x,
            sheet_y,
            sheet_norm_squared,
            constraint_rows,
        }
    }

    /// Returns the number of robots/sheet vertices in the cached problem.
    fn len(&self) -> usize {
        self.formation_x.len()
    }

    /// Returns the precomputed constraint row comparing cable `id` with
    /// reference cable `id1`.
    fn constraint_row(&self, id1: usize, id: usize) -> ConstraintRow {
        self.constraint_rows[id1 * self.len() + id]
    }
}

/// Linear constraint coefficients for one pairwise cable comparison.
#[derive(Debug, Clone, Copy)]
struct ConstraintRow {
    a: [Scalar; 4],
    b: Scalar,
}

/// Checks the necessary pairwise-distance condition for sheet reachability.
fn formation_feasible(problem: &ProblemData) -> bool {
    for i in 0..problem.len() {
        for j in 0..problem.len() {
            let formation_distance = point_distance(
                problem.formation_x[i],
                problem.formation_y[i],
                problem.formation_x[j],
                problem.formation_y[j],
            );
            let sheet_distance = point_distance(
                problem.sheet_x[i],
                problem.sheet_y[i],
                problem.sheet_x[j],
                problem.sheet_y[j],
            );

            if formation_distance > sheet_distance {
                return false;
            }
        }
    }

    true
}

/// Computes the Euclidean distance between two XY coordinates.
fn point_distance(x1: Scalar, y1: Scalar, x2: Scalar, y2: Scalar) -> Scalar {
    let dx = x1 - x2;
    let dy = y1 - y2;
    (dx * dx + dy * dy).sqrt()
}

/// Copies one cached pairwise constraint into the dense solve buffers.
fn fill_constraint_row(
    problem: &ProblemData,
    id1: usize,
    id: usize,
    row: usize,
    a: &mut DMatrix<Scalar>,
    b: &mut DVector<Scalar>,
) {
    let constraint = problem.constraint_row(id1, id);
    a[(row, 0)] = constraint.a[0];
    a[(row, 1)] = constraint.a[1];
    a[(row, 2)] = constraint.a[2];
    a[(row, 3)] = constraint.a[3];
    b[row] = constraint.b;
}

/// Returns a new matrix with `column` appended as the final column.
fn append_column(matrix: &DMatrix<Scalar>, column: &DVector<Scalar>) -> DMatrix<Scalar> {
    let mut out = DMatrix::<Scalar>::zeros(matrix.nrows(), matrix.ncols() + 1);

    for row in 0..matrix.nrows() {
        for col in 0..matrix.ncols() {
            out[(row, col)] = matrix[(row, col)];
        }
        out[(row, matrix.ncols())] = column[row];
    }

    out
}

/// Computes numerical matrix rank using singular values and [`RANK_EPS`].
fn matrix_rank(matrix: &DMatrix<Scalar>) -> usize {
    if matrix.is_empty() {
        return 0;
    }

    matrix.clone().svd(false, false).rank(RANK_EPS)
}

/// Solves the KKT/Lagrange system for object XY and virtual-object XY.
fn solve_lagrange_system(
    c: [Scalar; 4],
    a11: &DMatrix<Scalar>,
    b11: &DVector<Scalar>,
) -> Option<DVector<Scalar>> {
    const Q_DIAGONAL: [Scalar; 4] = [2.0, 2.0, -2.0, -2.0];

    let lagrange_size = 8;
    let constraint_count = a11.nrows();
    let constraint_start = lagrange_size - constraint_count;
    let mut lagrange_matrix = DMatrix::<Scalar>::zeros(lagrange_size, lagrange_size);
    let mut rhs = DVector::<Scalar>::zeros(lagrange_size);

    for row in 0..4 {
        rhs[row] = -c[row];
        lagrange_matrix[(row, row)] = Q_DIAGONAL[row];
    }

    for constraint in 0..constraint_count {
        let multiplier_col = constraint_start + constraint;
        let constraint_row = constraint_start + constraint;
        rhs[constraint_row] = b11[constraint];

        for variable in 0..4 {
            lagrange_matrix[(variable, multiplier_col)] = a11[(constraint, variable)];
            lagrange_matrix[(constraint_row, variable)] = a11[(constraint, variable)];
        }
    }

    lagrange_matrix
        .clone()
        .lu()
        .solve(&rhs)
        .or_else(|| lagrange_matrix.svd(true, true).solve(&rhs, RANK_EPS).ok())
}

/// Expands the Lagrange multipliers so the reference taut cable has an explicit
/// coefficient.
fn expand_lambda(lambda_raw: &DVector<Scalar>) -> DVector<Scalar> {
    let mut lambda = DVector::<Scalar>::zeros(lambda_raw.len() + 1);
    lambda[0] = (2.0 - lambda_raw.sum()) / 2.0;

    for i in 0..lambda_raw.len() {
        lambda[i + 1] = lambda_raw[i] / 2.0;
    }

    lambda
}

/// Tests whether a point lies inside a polygon with the ray-casting rule.
fn in_polygon(point: Point2, polygon: &[Point2]) -> bool {
    let mut inside = false;
    let mut j = polygon.len() - 1;

    for i in 0..polygon.len() {
        let pi = polygon[i];
        let pj = polygon[j];

        if ((pi.y > point.y) != (pj.y > point.y))
            && (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)
        {
            inside = !inside;
        }

        j = i;
    }

    inside
}

/// Updates every candidate's stable flag in place.
fn mark_stable_solutions(candidates: &mut [CandidateSolution]) {
    for candidate in candidates {
        candidate.solution.stable = is_locally_minimal(candidate);
    }
}

/// Applies the local-minimality stability test for one candidate branch.
fn is_locally_minimal(candidate: &CandidateSolution) -> bool {
    if candidate
        .lambda
        .iter()
        .all(|lambda| *lambda >= -STABILITY_EPS)
    {
        return true;
    }

    // Degenerate taut sets need a basis transform (`omega`) to search for a
    // non-negative multiplier representation. Non-degenerate candidates already
    // returned above, so a missing transform means this candidate is unstable in
    // this solver.
    let Some(omega) = &candidate.omega else {
        return false;
    };

    let basis_size = candidate.independent_taut_count;
    if basis_size == 0 || basis_size > candidate.taut_count {
        return false;
    }

    let mut locally_minimal = false;
    enumerate_combinations(candidate.taut_count, basis_size, |columns| {
        if locally_minimal {
            return;
        }

        let mut basis = DMatrix::<Scalar>::zeros(basis_size, basis_size);
        for (basis_col, &omega_col) in columns.iter().enumerate() {
            for row in 0..basis_size {
                basis[(row, basis_col)] = omega[(row, omega_col)];
            }
        }

        if matrix_rank(&basis) < basis_size {
            return;
        }

        let Some(lambda) = solve_square_system(&basis, &candidate.lambda) else {
            return;
        };

        locally_minimal = lambda.iter().all(|value| *value >= -STABILITY_EPS);
    });

    locally_minimal
}

/// Solves a square linear system with LU first and SVD as a fallback.
fn solve_square_system(matrix: &DMatrix<Scalar>, rhs: &DVector<Scalar>) -> Option<DVector<Scalar>> {
    matrix
        .clone()
        .lu()
        .solve(rhs)
        .or_else(|| matrix.clone().svd(true, true).solve(rhs, RANK_EPS).ok())
}

/// Enumerates all `k`-element index combinations from `0..n`.
fn enumerate_combinations<F>(n: usize, k: usize, mut visit: F)
where
    F: FnMut(&[usize]),
{
    fn recurse<F>(n: usize, k: usize, start: usize, current: &mut Vec<usize>, visit: &mut F)
    where
        F: FnMut(&[usize]),
    {
        if current.len() == k {
            visit(current);
            return;
        }

        let remaining = k - current.len();
        for value in start..=(n - remaining) {
            current.push(value);
            recurse(n, k, value + 1, current, visit);
            current.pop();
        }
    }

    if k == 0 || k > n {
        return;
    }

    let mut current = Vec::with_capacity(k);
    recurse(n, k, 0, &mut current, &mut visit);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalization_moves_inputs_to_positive_target_box() {
        let formation = RobotFormation::new(vec![
            Point2::new(-20.0, 40.0),
            Point2::new(30.0, -10.0),
            Point2::new(80.0, 90.0),
        ])
        .unwrap();
        let sheet = SheetShape::new(vec![
            Point2::new(-300.0, -100.0),
            Point2::new(700.0, -50.0),
            Point2::new(100.0, 500.0),
        ])
        .unwrap();

        let normalized = NormalizedProblem::new(&formation, &sheet, 250.0);

        assert_eq!(
            normalized.transform.formation_origin,
            Point2::new(-20.0, -10.0)
        );
        assert_eq!(
            normalized.transform.sheet_origin,
            Point2::new(-300.0, -100.0)
        );
        assert_close(normalized.transform.scale, 1.0, 1.0e-6);
        assert_close(normalized.hold_height, 250.0, 1.0e-6);

        for point in normalized.formation.points() {
            assert!(point.x >= 0.0);
            assert!(point.y >= 0.0);
            assert!(point.x <= NORMALIZED_COORDINATE_UPPER_BOUND);
            assert!(point.y <= NORMALIZED_COORDINATE_UPPER_BOUND);
        }
        for point in normalized.sheet.vertices() {
            assert!(point.x >= 0.0);
            assert!(point.y >= 0.0);
            assert!(point.x <= NORMALIZED_COORDINATE_UPPER_BOUND);
            assert!(point.y <= NORMALIZED_COORDINATE_UPPER_BOUND);
        }
    }

    #[test]
    fn normalization_scales_small_inputs_to_target_upper_bound() {
        let formation = RobotFormation::new(vec![
            Point2::new(2.0, 1.0),
            Point2::new(4.0, 1.5),
            Point2::new(3.0, 2.0),
        ])
        .unwrap();
        let sheet = SheetShape::new(vec![
            Point2::new(-0.5, -0.25),
            Point2::new(1.5, -0.25),
            Point2::new(1.0, 0.75),
        ])
        .unwrap();

        let normalized = NormalizedProblem::new(&formation, &sheet, 2.5);

        assert_close(normalized.transform.scale, 500.0, 1.0e-3);
        assert_close(normalized.hold_height, 1250.0, 1.0e-3);
        assert_close(
            max_coordinate(normalized.formation.points()),
            1000.0,
            1.0e-3,
        );
        assert_close(max_coordinate(normalized.sheet.vertices()), 1000.0, 1.0e-3);
    }

    fn max_coordinate(points: &[Point2]) -> Scalar {
        points
            .iter()
            .fold(0.0, |max_value, point| max_value.max(point.x).max(point.y))
    }

    fn assert_close(actual: Scalar, expected: Scalar, tolerance: Scalar) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "actual {actual}, expected {expected}"
        );
    }
}
