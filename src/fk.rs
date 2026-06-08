use crate::error::VvcmError;
use crate::math;
use crate::types::{FkSolution, FkSolutions, Point2, Point3, RobotFormation, Scalar, SheetShape};
use nalgebra::{DMatrix, DVector};

const RANK_EPS: Scalar = 1.0e-4;
const STABILITY_EPS: Scalar = 1.0e-8;
const SLACK_EPS: Scalar = 1.0e-8;

#[derive(Debug, Clone)]
pub struct VvcmFk {
    robot_count: usize,
    hold_height: Scalar,
    sheet: SheetShape,
    formation: Option<RobotFormation>,
    solutions: FkSolutions,
}

impl VvcmFk {
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

        let _sheet_matrix = math::sheet_to_matrix(&sheet);

        Ok(Self {
            robot_count,
            hold_height,
            sheet,
            formation: None,
            solutions: FkSolutions::default(),
        })
    }

    pub fn update_stable_solutions(
        &mut self,
        formation: RobotFormation,
    ) -> Result<&FkSolutions, VvcmError> {
        self.validate_formation(&formation)?;

        self.solutions = FkSolutions::default();
        self.formation = Some(formation.clone());

        let mut candidates = self.find_candidate_solutions(&formation)?;
        mark_stable_solutions(&mut candidates);

        self.solutions = FkSolutions::new(
            candidates
                .into_iter()
                .map(|candidate| candidate.solution)
                .collect(),
        );

        if self.solutions.stable_count() == 0 {
            Err(VvcmError::NoStableSolution)
        } else {
            Ok(&self.solutions)
        }
    }

    pub fn robot_count(&self) -> usize {
        self.robot_count
    }

    pub fn hold_height(&self) -> Scalar {
        self.hold_height
    }

    pub fn sheet(&self) -> &SheetShape {
        &self.sheet
    }

    pub fn current_formation(&self) -> Option<&RobotFormation> {
        self.formation.as_ref()
    }

    pub fn solutions(&self) -> &FkSolutions {
        &self.solutions
    }

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

    fn find_candidate_solutions(
        &self,
        formation: &RobotFormation,
    ) -> Result<Vec<CandidateSolution>, VvcmError> {
        let formation_matrix = math::formation_to_matrix(formation);
        let sheet_matrix = math::sheet_to_matrix(&self.sheet);

        if !formation_feasible(&formation_matrix, &sheet_matrix) {
            return Err(VvcmError::InfeasibleFormation);
        }

        let mut candidates = Vec::new();
        let max_taut_count = self.robot_count.min(5);

        for taut_count in 3..=max_taut_count {
            enumerate_combinations(self.robot_count, taut_count, |taut_cables| {
                if let Some(candidate) =
                    self.solve_for_taut_set(&formation_matrix, &sheet_matrix, taut_cables)
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

    fn solve_for_taut_set(
        &self,
        formation: &DMatrix<Scalar>,
        sheet: &DMatrix<Scalar>,
        taut_cables: &[usize],
    ) -> Option<CandidateSolution> {
        let taut_count = taut_cables.len();
        let slack_cables = slack_cables(self.robot_count, taut_cables);
        let slack_count = slack_cables.len();
        let id1 = taut_cables[0];

        let mut id2_to_n = Vec::with_capacity(taut_count - 1 + slack_count);
        id2_to_n.extend_from_slice(&taut_cables[1..]);
        id2_to_n.extend_from_slice(&slack_cables);

        let row_count = id2_to_n.len();
        let mut a = DMatrix::<Scalar>::zeros(row_count, 4);
        let mut b = DVector::<Scalar>::zeros(row_count);

        let x1 = formation[(id1, 0)];
        let y1 = formation[(id1, 1)];
        let xv1 = sheet[(id1, 0)];
        let yv1 = sheet[(id1, 1)];

        for (row, &id) in id2_to_n.iter().enumerate() {
            let x = formation[(id, 0)];
            let y = formation[(id, 1)];
            let xv = sheet[(id, 0)];
            let yv = sheet[(id, 1)];

            a[(row, 0)] = x1 - x;
            a[(row, 1)] = y1 - y;
            a[(row, 2)] = xv - xv1;
            a[(row, 3)] = yv - yv1;
            b[row] = 0.5
                * (x1 * x1 + y1 * y1 - xv1 * xv1 - yv1 * yv1 + xv * xv + yv * yv - x * x - y * y);
        }

        let constraint_count = taut_count - 1;
        let a1 = a.rows(0, constraint_count).into_owned();
        let b1 = b.rows(0, constraint_count).into_owned();
        let a1_bar = append_column(&a1, &b1);

        if matrix_rank(&a1) != matrix_rank(&a1_bar) {
            return None;
        }

        let r = a1_bar.clone().qr().r();
        let c_matrix = diagonal_scaling_from_qr_r(&r, constraint_count);
        let independent_taut_count = constraint_count + 1;
        let omega = build_omega(&c_matrix, independent_taut_count, taut_count);

        let q = DMatrix::<Scalar>::from_row_slice(
            4,
            4,
            &[
                2.0, 0.0, 0.0, 0.0, //
                0.0, 2.0, 0.0, 0.0, //
                0.0, 0.0, -2.0, 0.0, //
                0.0, 0.0, 0.0, -2.0,
            ],
        );
        let c = DVector::<Scalar>::from_vec(vec![-2.0 * x1, -2.0 * y1, 2.0 * xv1, 2.0 * yv1]);
        let f0 = x1 * x1 + y1 * y1 - xv1 * xv1 - yv1 * yv1;

        let solution = solve_lagrange_system(&q, &c, &a1, &b1)?;
        let x_bar = solution.rows(0, 4).into_owned();
        let lambda_raw = solution
            .rows(solution.len() - constraint_count, constraint_count)
            .into_owned();
        let lambda = expand_lambda(&lambda_raw);

        let q_x = &q * &x_bar;
        let term1 = 0.5 * x_bar.dot(&q_x);
        let term2 = c.dot(&x_bar);
        let tmp = -(term1 + term2 + f0);

        if tmp < 0.0 {
            return None;
        }

        let x_o = x_bar[0];
        let y_o = x_bar[1];
        let x_vo = x_bar[2];
        let y_vo = x_bar[3];
        let z_o = self.hold_height - tmp.sqrt();

        let taut_polygon: Vec<Point2> = taut_cables
            .iter()
            .map(|&idx| Point2::new(formation[(idx, 0)], formation[(idx, 1)]))
            .collect();
        if !in_polygon(Point2::new(x_o, y_o), &taut_polygon) {
            return None;
        }

        if slack_count > 0 {
            let residual = b.rows(constraint_count, slack_count).into_owned()
                - a.rows(constraint_count, slack_count).into_owned() * &x_bar;
            if residual.min() <= SLACK_EPS {
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
            omega: (taut_count != independent_taut_count).then_some(omega),
        })
    }
}

#[derive(Debug, Clone)]
struct CandidateSolution {
    solution: FkSolution,
    taut_count: usize,
    independent_taut_count: usize,
    lambda: DVector<Scalar>,
    omega: Option<DMatrix<Scalar>>,
}

fn formation_feasible(formation: &DMatrix<Scalar>, sheet: &DMatrix<Scalar>) -> bool {
    for i in 0..formation.nrows() {
        for j in 0..formation.nrows() {
            let formation_distance = row_distance(formation, i, j);
            let sheet_distance = row_distance(sheet, i, j);

            if formation_distance > sheet_distance {
                return false;
            }
        }
    }

    true
}

fn row_distance(matrix: &DMatrix<Scalar>, i: usize, j: usize) -> Scalar {
    let dx = matrix[(i, 0)] - matrix[(j, 0)];
    let dy = matrix[(i, 1)] - matrix[(j, 1)];
    (dx * dx + dy * dy).sqrt()
}

fn slack_cables(robot_count: usize, taut_cables: &[usize]) -> Vec<usize> {
    (0..robot_count)
        .filter(|candidate| !taut_cables.contains(candidate))
        .collect()
}

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

fn matrix_rank(matrix: &DMatrix<Scalar>) -> usize {
    if matrix.is_empty() {
        return 0;
    }

    matrix.clone().svd(false, false).rank(RANK_EPS)
}

fn diagonal_scaling_from_qr_r(r: &DMatrix<Scalar>, size: usize) -> DMatrix<Scalar> {
    let mut c_matrix = DMatrix::<Scalar>::identity(size, size);

    for i in 0..size {
        let diagonal = r[(i, i)];
        if diagonal != 0.0 {
            c_matrix[(i, i)] /= diagonal;
        }
    }

    c_matrix
}

fn build_omega(
    c_matrix: &DMatrix<Scalar>,
    independent_taut_count: usize,
    taut_count: usize,
) -> DMatrix<Scalar> {
    let mut omega = DMatrix::<Scalar>::zeros(independent_taut_count, taut_count);
    omega[(0, 0)] = 1.0;

    for col in 1..taut_count {
        let c_col = col - 1;
        let mut col_sum = 0.0;

        for row in 0..(independent_taut_count - 1) {
            let value = c_matrix[(row, c_col)];
            col_sum += value;
            omega[(row + 1, col)] = value;
        }

        omega[(0, col)] = 1.0 - col_sum;
    }

    omega
}

fn solve_lagrange_system(
    q: &DMatrix<Scalar>,
    c: &DVector<Scalar>,
    a11: &DMatrix<Scalar>,
    b11: &DVector<Scalar>,
) -> Option<DVector<Scalar>> {
    let lagrange_size = 8;
    let constraint_count = a11.nrows();
    let constraint_start = lagrange_size - constraint_count;
    let mut lagrange_matrix = DMatrix::<Scalar>::zeros(lagrange_size, lagrange_size);
    let mut rhs = DVector::<Scalar>::zeros(lagrange_size);

    for row in 0..4 {
        rhs[row] = -c[row];

        for col in 0..4 {
            lagrange_matrix[(row, col)] = q[(row, col)];
        }
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

fn expand_lambda(lambda_raw: &DVector<Scalar>) -> DVector<Scalar> {
    let mut lambda = DVector::<Scalar>::zeros(lambda_raw.len() + 1);
    lambda[0] = (2.0 - lambda_raw.sum()) / 2.0;

    for i in 0..lambda_raw.len() {
        lambda[i + 1] = lambda_raw[i] / 2.0;
    }

    lambda
}

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

fn mark_stable_solutions(candidates: &mut [CandidateSolution]) {
    for candidate in candidates {
        candidate.solution.stable = is_locally_minimal(candidate);
    }
}

fn is_locally_minimal(candidate: &CandidateSolution) -> bool {
    if candidate
        .lambda
        .iter()
        .all(|lambda| *lambda >= -STABILITY_EPS)
    {
        return true;
    }

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

fn solve_square_system(matrix: &DMatrix<Scalar>, rhs: &DVector<Scalar>) -> Option<DVector<Scalar>> {
    matrix
        .clone()
        .lu()
        .solve(rhs)
        .or_else(|| matrix.clone().svd(true, true).solve(rhs, RANK_EPS).ok())
}

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
