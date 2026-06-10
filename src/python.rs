#![allow(missing_docs)]

//! Python bindings for the public VVCM Rust API.

use crate::{
    FkSolution, FkSolutions, Point2, Point3, RobotFormation, Scalar, SheetShape,
    VvcmError as CoreVvcmError, VvcmFk, VvcmManualSimulation, VvcmSimulation,
};
use pyo3::exceptions::{PyIndexError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyAny, PyModule};

pyo3::create_exception!(vvcm_rs, VvcmError, pyo3::exceptions::PyException);
pyo3::create_exception!(vvcm_rs, DimensionMismatchError, VvcmError);
pyo3::create_exception!(vvcm_rs, InfeasibleFormationError, VvcmError);
pyo3::create_exception!(vvcm_rs, NoSolutionError, VvcmError);
pyo3::create_exception!(vvcm_rs, NoStableSolutionError, VvcmError);

#[pyclass(name = "Point2", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
/// Two-dimensional point or vector in the XY plane.
struct PyPoint2 {
    inner: Point2,
}

#[pymethods]
impl PyPoint2 {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0))]
    /// Create a point from explicit XY coordinates.
    fn new(x: Scalar, y: Scalar) -> Self {
        Self::from_inner(Point2::new(x, y))
    }

    #[staticmethod]
    /// Return the origin `(0, 0)`.
    fn zero() -> Self {
        Self::from_inner(Point2::zero())
    }

    #[getter]
    /// X coordinate.
    fn x(&self) -> Scalar {
        self.inner.x
    }

    #[setter]
    fn set_x(&mut self, value: Scalar) {
        self.inner.x = value;
    }

    #[getter]
    /// Y coordinate.
    fn y(&self) -> Scalar {
        self.inner.y
    }

    #[setter]
    fn set_y(&mut self, value: Scalar) {
        self.inner.y = value;
    }

    /// Return a new point with both coordinates multiplied by `factor`.
    fn scaled_by(&self, factor: Scalar) -> Self {
        Self::from_inner(self.inner.scaled_by(factor))
    }

    /// Return a new point translated by the XY `offset`.
    fn translated_by(&self, offset: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self::from_inner(
            self.inner.translated_by(point2_from_py(offset)?),
        ))
    }

    /// Return a new point expressed relative to `origin`.
    fn relative_to(&self, origin: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self::from_inner(
            self.inner.relative_to(point2_from_py(origin)?),
        ))
    }

    /// Compute the Euclidean distance to another 2D point.
    fn distance_to(&self, other: &Bound<'_, PyAny>) -> PyResult<Scalar> {
        Ok(self.inner.distance_to(point2_from_py(other)?))
    }

    /// Return `(x, y)` as a plain Python tuple.
    fn as_tuple(&self) -> (Scalar, Scalar) {
        (self.inner.x, self.inner.y)
    }

    fn __repr__(&self) -> String {
        format!("Point2(x={}, y={})", self.inner.x, self.inner.y)
    }
}

impl PyPoint2 {
    fn from_inner(inner: Point2) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "Point3", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
/// Three-dimensional point or vector.
struct PyPoint3 {
    inner: Point3,
}

#[pymethods]
impl PyPoint3 {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0))]
    /// Create a point from explicit XYZ coordinates.
    fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self::from_inner(Point3::new(x, y, z))
    }

    #[staticmethod]
    /// Return the origin `(0, 0, 0)`.
    fn zero() -> Self {
        Self::from_inner(Point3::zero())
    }

    #[getter]
    /// X coordinate.
    fn x(&self) -> Scalar {
        self.inner.x
    }

    #[setter]
    fn set_x(&mut self, value: Scalar) {
        self.inner.x = value;
    }

    #[getter]
    /// Y coordinate.
    fn y(&self) -> Scalar {
        self.inner.y
    }

    #[setter]
    fn set_y(&mut self, value: Scalar) {
        self.inner.y = value;
    }

    #[getter]
    /// Z coordinate.
    fn z(&self) -> Scalar {
        self.inner.z
    }

    #[setter]
    fn set_z(&mut self, value: Scalar) {
        self.inner.z = value;
    }

    /// Return a new point translated in XY, leaving Z unchanged.
    fn translated_xy_by(&self, offset: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self::from_inner(
            self.inner.translated_xy_by(point2_from_py(offset)?),
        ))
    }

    /// Return a new point expressed relative to an XY origin, leaving Z unchanged.
    fn relative_xy_to(&self, origin: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self::from_inner(
            self.inner.relative_xy_to(point2_from_py(origin)?),
        ))
    }

    /// Compute the Euclidean distance to another 3D point.
    fn distance_to(&self, other: &Bound<'_, PyAny>) -> PyResult<Scalar> {
        Ok(self.inner.distance_to(point3_from_py(other)?))
    }

    /// Return `(x, y, z)` as a plain Python tuple.
    fn as_tuple(&self) -> (Scalar, Scalar, Scalar) {
        (self.inner.x, self.inner.y, self.inner.z)
    }

    fn __repr__(&self) -> String {
        format!(
            "Point3(x={}, y={}, z={})",
            self.inner.x, self.inner.y, self.inner.z
        )
    }
}

impl PyPoint3 {
    fn from_inner(inner: Point3) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "RobotFormation", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
/// Ordered XY positions for robots or robot velocities.
struct PyRobotFormation {
    inner: RobotFormation,
}

#[pymethods]
impl PyRobotFormation {
    #[new]
    /// Create a non-empty robot formation from `Point2` values or length-2 rows.
    fn new(points: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self::from_inner(
            RobotFormation::new(point2_sequence_from_py(points, "robot formation")?)
                .map_err(map_vvcm_error)?,
        ))
    }

    #[staticmethod]
    /// Create a zero-valued formation with one point per robot.
    fn zeros(robot_count: usize) -> PyResult<Self> {
        Ok(Self::from_inner(
            RobotFormation::zeros(robot_count).map_err(map_vvcm_error)?,
        ))
    }

    #[getter]
    /// Ordered robot points.
    fn points(&self) -> Vec<PyPoint2> {
        point2_vec_to_py(self.inner.points())
    }

    /// Return the number of points in the formation.
    fn len(&self) -> usize {
        self.inner.len()
    }

    /// Return `True` when the formation contains no points.
    fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Compute the arithmetic mean of all points.
    fn centroid(&self) -> PyPoint2 {
        PyPoint2::from_inner(self.inner.centroid())
    }

    /// Return a new formation translated by `offset`.
    fn translated_by(&self, offset: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self::from_inner(
            self.inner.translated_by(point2_from_py(offset)?),
        ))
    }

    /// Return a new formation expressed relative to `origin`.
    fn relative_to(&self, origin: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self::from_inner(
            self.inner.relative_to(point2_from_py(origin)?),
        ))
    }

    /// Return `True` when every point is exactly `(0, 0)`.
    fn all_zero(&self) -> bool {
        self.inner.all_zero()
    }

    /// Return the ordered points as plain `(x, y)` tuples.
    fn as_tuples(&self) -> Vec<(Scalar, Scalar)> {
        self.inner
            .points()
            .iter()
            .map(|point| (point.x, point.y))
            .collect()
    }

    fn __len__(&self) -> usize {
        self.inner.len()
    }

    fn __getitem__(&self, index: isize) -> PyResult<PyPoint2> {
        let index = normalize_index(index, self.inner.len())?;
        Ok(PyPoint2::from_inner(self.inner.points()[index]))
    }

    fn __repr__(&self) -> String {
        format!("RobotFormation(points={:?})", self.as_tuples())
    }
}

impl PyRobotFormation {
    fn from_inner(inner: RobotFormation) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "SheetShape", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
/// Ordered sheet attachment vertices in the sheet-local XY frame.
struct PySheetShape {
    inner: SheetShape,
}

#[pymethods]
impl PySheetShape {
    #[new]
    /// Create a sheet shape from at least three `Point2` values or length-2 rows.
    fn new(vertices: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self::from_inner(
            SheetShape::new(point2_sequence_from_py(vertices, "sheet shape")?)
                .map_err(map_vvcm_error)?,
        ))
    }

    #[getter]
    /// Ordered sheet vertices.
    fn vertices(&self) -> Vec<PyPoint2> {
        point2_vec_to_py(self.inner.vertices())
    }

    /// Return the number of sheet vertices.
    fn len(&self) -> usize {
        self.inner.len()
    }

    /// Return `True` when the sheet contains no vertices.
    fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Return the ordered vertices as plain `(x, y)` tuples.
    fn as_tuples(&self) -> Vec<(Scalar, Scalar)> {
        self.inner
            .vertices()
            .iter()
            .map(|point| (point.x, point.y))
            .collect()
    }

    fn __len__(&self) -> usize {
        self.inner.len()
    }

    fn __getitem__(&self, index: isize) -> PyResult<PyPoint2> {
        let index = normalize_index(index, self.inner.len())?;
        Ok(PyPoint2::from_inner(self.inner.vertices()[index]))
    }

    fn __repr__(&self) -> String {
        format!("SheetShape(vertices={:?})", self.as_tuples())
    }
}

impl PySheetShape {
    fn from_inner(inner: SheetShape) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "FkSolution", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
/// A single forward-kinematics candidate solution.
struct PyFkSolution {
    inner: FkSolution,
}

#[pymethods]
impl PyFkSolution {
    #[new]
    #[pyo3(signature = (stable=false, po=None, vo=None, taut_cables=None))]
    /// Create a forward-kinematics solution value.
    fn new(
        stable: bool,
        po: Option<&Bound<'_, PyAny>>,
        vo: Option<&Bound<'_, PyAny>>,
        taut_cables: Option<Vec<usize>>,
    ) -> PyResult<Self> {
        Ok(Self::from_inner(FkSolution::new(
            stable,
            optional_point3(po)?,
            optional_point2(vo)?,
            taut_cables.unwrap_or_default(),
        )))
    }

    #[getter]
    /// Whether the candidate is locally stable.
    fn stable(&self) -> bool {
        self.inner.stable
    }

    #[setter]
    fn set_stable(&mut self, value: bool) {
        self.inner.stable = value;
    }

    #[getter]
    /// Object position `Po` in the formation-local frame.
    fn po(&self) -> PyPoint3 {
        PyPoint3::from_inner(self.inner.po)
    }

    #[setter]
    fn set_po(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.po = point3_from_py(value)?;
        Ok(())
    }

    #[getter]
    /// Virtual object point `Vo` in the sheet-local XY frame.
    fn vo(&self) -> PyPoint2 {
        PyPoint2::from_inner(self.inner.vo)
    }

    #[setter]
    fn set_vo(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.vo = point2_from_py(value)?;
        Ok(())
    }

    #[getter]
    /// Indices of the taut virtual cables for this candidate.
    fn taut_cables(&self) -> Vec<usize> {
        self.inner.taut_cables.clone()
    }

    #[setter]
    fn set_taut_cables(&mut self, value: Vec<usize>) {
        self.inner.taut_cables = value;
    }

    fn __repr__(&self) -> String {
        format!(
            "FkSolution(stable={}, po={:?}, vo={:?}, taut_cables={:?})",
            self.inner.stable, self.inner.po, self.inner.vo, self.inner.taut_cables
        )
    }
}

impl PyFkSolution {
    fn from_inner(inner: FkSolution) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "FkSolutions", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
/// Collection of forward-kinematics candidate solutions.
struct PyFkSolutions {
    inner: FkSolutions,
}

#[pymethods]
impl PyFkSolutions {
    #[new]
    #[pyo3(signature = (solutions=None))]
    /// Create a solution collection from an ordered list of candidates.
    fn new(solutions: Option<Vec<PyRef<'_, PyFkSolution>>>) -> Self {
        let solutions = solutions
            .unwrap_or_default()
            .into_iter()
            .map(|solution| solution.inner.clone())
            .collect();
        Self::from_inner(FkSolutions::new(solutions))
    }

    #[getter]
    /// All candidate solutions found during the most recent FK update.
    fn solutions(&self) -> Vec<PyFkSolution> {
        self.inner
            .solutions
            .iter()
            .cloned()
            .map(PyFkSolution::from_inner)
            .collect()
    }

    /// Return `True` when no candidates are stored.
    fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Return locally stable candidate solutions.
    fn stable(&self) -> Vec<PyFkSolution> {
        self.inner
            .stable()
            .cloned()
            .map(PyFkSolution::from_inner)
            .collect()
    }

    /// Return the stable solution closest to `reference` as `(index, solution)`.
    fn closest_stable_to(
        &self,
        reference: &Bound<'_, PyAny>,
    ) -> PyResult<Option<(usize, PyFkSolution)>> {
        Ok(self
            .inner
            .closest_stable_to(point3_from_py(reference)?)
            .map(|(index, solution)| (index, PyFkSolution::from_inner(solution.clone()))))
    }

    /// Count locally stable candidate solutions.
    fn stable_count(&self) -> usize {
        self.inner.stable_count()
    }

    /// Count all candidate solutions, stable and unstable.
    fn all_count(&self) -> usize {
        self.inner.all_count()
    }

    fn __len__(&self) -> usize {
        self.inner.all_count()
    }

    fn __getitem__(&self, index: isize) -> PyResult<PyFkSolution> {
        let index = normalize_index(index, self.inner.solutions.len())?;
        Ok(PyFkSolution::from_inner(
            self.inner.solutions[index].clone(),
        ))
    }

    fn __repr__(&self) -> String {
        format!(
            "FkSolutions(all_count={}, stable_count={})",
            self.inner.all_count(),
            self.inner.stable_count()
        )
    }
}

impl PyFkSolutions {
    fn from_inner(inner: FkSolutions) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "VvcmFk", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
/// Stateful forward-kinematics engine for a fixed deformable sheet.
struct PyVvcmFk {
    inner: VvcmFk,
}

#[pymethods]
impl PyVvcmFk {
    #[new]
    /// Create a solver for `robot_count` robots holding `sheet` at `hold_height`.
    fn new(robot_count: usize, hold_height: Scalar, sheet: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self::from_inner(
            VvcmFk::new(robot_count, hold_height, sheet_from_py(sheet)?).map_err(map_vvcm_error)?,
        ))
    }

    /// Solve and store forward-kinematics branches for `formation`.
    fn update_stable_solutions(&mut self, formation: &Bound<'_, PyAny>) -> PyResult<PyFkSolutions> {
        let solutions = self
            .inner
            .update_stable_solutions(formation_from_py(formation)?)
            .map_err(map_vvcm_error)?
            .clone();
        Ok(PyFkSolutions::from_inner(solutions))
    }

    #[getter]
    /// Fixed number of robots solved by this engine.
    fn robot_count(&self) -> usize {
        self.inner.robot_count()
    }

    #[getter]
    /// Fixed robot holding height used to recover the object Z coordinate.
    fn hold_height(&self) -> Scalar {
        self.inner.hold_height()
    }

    #[getter]
    /// Fixed sheet geometry.
    fn sheet(&self) -> PySheetShape {
        PySheetShape::from_inner(self.inner.sheet().clone())
    }

    #[getter]
    /// Most recent formation passed to `update_stable_solutions`, if any.
    fn current_formation(&self) -> Option<PyRobotFormation> {
        self.inner
            .current_formation()
            .cloned()
            .map(PyRobotFormation::from_inner)
    }

    #[getter]
    /// Most recent solution cache.
    fn solutions(&self) -> PyFkSolutions {
        PyFkSolutions::from_inner(self.inner.solutions().clone())
    }

    fn __repr__(&self) -> String {
        format!(
            "VvcmFk(robot_count={}, hold_height={})",
            self.inner.robot_count(),
            self.inner.hold_height()
        )
    }
}

impl PyVvcmFk {
    fn from_inner(inner: VvcmFk) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "VvcmSimulation", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
/// Fixed-step robot-velocity simulation built on `VvcmFk`.
struct PyVvcmSimulation {
    inner: VvcmSimulation,
}

#[pymethods]
impl PyVvcmSimulation {
    #[new]
    #[pyo3(signature = (
        robot_count,
        hold_height,
        sheet,
        initial_formation,
        po_initial=None,
        dt=0.033333335
    ))]
    /// Create a simulation from an absolute initial formation and object position.
    fn new(
        robot_count: usize,
        hold_height: Scalar,
        sheet: &Bound<'_, PyAny>,
        initial_formation: &Bound<'_, PyAny>,
        po_initial: Option<&Bound<'_, PyAny>>,
        dt: Scalar,
    ) -> PyResult<Self> {
        Ok(Self::from_inner(
            VvcmSimulation::new(
                robot_count,
                hold_height,
                sheet_from_py(sheet)?,
                formation_from_py(initial_formation)?,
                optional_point3(po_initial)?,
                dt,
            )
            .map_err(map_vvcm_error)?,
        ))
    }

    /// Set one XY velocity vector per robot.
    fn set_velocity(&mut self, velocity: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner
            .set_velocity(formation_from_py(velocity)?)
            .map_err(map_vvcm_error)
    }

    /// Advance the simulation by one fixed time step.
    fn step(&mut self) -> PyResult<()> {
        self.inner.step().map_err(map_vvcm_error)
    }

    /// Return the current robot formation in absolute coordinates.
    fn absolute_formation(&self) -> PyRobotFormation {
        PyRobotFormation::from_inner(self.inner.absolute_formation())
    }

    /// Return the selected object position in absolute coordinates.
    fn absolute_object_position(&self) -> PyPoint3 {
        PyPoint3::from_inner(self.inner.absolute_object_position())
    }

    #[getter]
    /// Snapshot of the underlying FK engine and its latest solution cache.
    fn fk_engine(&self) -> PyVvcmFk {
        PyVvcmFk::from_inner(self.inner.fk_engine().clone())
    }

    #[getter]
    /// Local-frame origin in absolute coordinates.
    fn global_position(&self) -> PyPoint2 {
        PyPoint2::from_inner(self.inner.global_position())
    }

    #[getter]
    /// Current robot formation in the local frame.
    fn formation(&self) -> PyRobotFormation {
        PyRobotFormation::from_inner(self.inner.formation().clone())
    }

    #[getter]
    /// Selected object position in the local frame.
    fn object_position(&self) -> PyPoint3 {
        PyPoint3::from_inner(self.inner.object_position())
    }

    #[getter]
    /// Taut cable indices for the currently selected branch.
    fn taut_cables(&self) -> Vec<usize> {
        self.inner.taut_cables().to_vec()
    }

    #[getter]
    /// Index of the selected branch in the FK solution cache.
    fn solution_index(&self) -> Option<usize> {
        self.inner.solution_index()
    }

    #[getter]
    /// Fixed integration time step.
    fn dt(&self) -> Scalar {
        self.inner.dt()
    }

    #[getter]
    /// Current per-robot velocity formation.
    fn velocity(&self) -> PyRobotFormation {
        PyRobotFormation::from_inner(self.inner.velocity().clone())
    }

    fn __repr__(&self) -> String {
        format!(
            "VvcmSimulation(robot_count={}, dt={})",
            self.inner.fk_engine().robot_count(),
            self.inner.dt()
        )
    }
}

impl PyVvcmSimulation {
    fn from_inner(inner: VvcmSimulation) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "VvcmManualSimulation", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
/// Simulation helper for externally supplied robot formations.
struct PyVvcmManualSimulation {
    inner: VvcmManualSimulation,
}

#[pymethods]
impl PyVvcmManualSimulation {
    #[new]
    /// Create a manual simulation wrapper for a fixed sheet.
    fn new(robot_count: usize, hold_height: Scalar, sheet: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self::from_inner(
            VvcmManualSimulation::new(robot_count, hold_height, sheet_from_py(sheet)?)
                .map_err(map_vvcm_error)?,
        ))
    }

    #[pyo3(signature = (formation, po_initial=None))]
    /// Initialize with the first absolute formation and reference object position.
    fn init(
        &mut self,
        formation: &Bound<'_, PyAny>,
        po_initial: Option<&Bound<'_, PyAny>>,
    ) -> PyResult<PyPoint3> {
        let position = self
            .inner
            .init(formation_from_py(formation)?, optional_point3(po_initial)?)
            .map_err(map_vvcm_error)?;
        Ok(PyPoint3::from_inner(position))
    }

    /// Update from a new absolute formation and return the closest stable object position.
    fn get_new_stable_solution(&mut self, formation: &Bound<'_, PyAny>) -> PyResult<PyPoint3> {
        let position = self
            .inner
            .get_new_stable_solution(formation_from_py(formation)?)
            .map_err(map_vvcm_error)?;
        Ok(PyPoint3::from_inner(position))
    }

    #[getter]
    /// Snapshot of the underlying FK engine and its latest solution cache.
    fn fk_engine(&self) -> PyVvcmFk {
        PyVvcmFk::from_inner(self.inner.fk_engine().clone())
    }

    #[getter]
    /// Current local-frame origin in absolute coordinates.
    fn global_position(&self) -> PyPoint2 {
        PyPoint2::from_inner(self.inner.global_position())
    }

    #[getter]
    /// Current robot formation in the centroid-relative local frame, if initialized.
    fn formation(&self) -> Option<PyRobotFormation> {
        self.inner
            .formation()
            .cloned()
            .map(PyRobotFormation::from_inner)
    }

    #[getter]
    /// Selected object position in the local frame, if initialized.
    fn object_position(&self) -> Option<PyPoint3> {
        self.inner.object_position().map(PyPoint3::from_inner)
    }

    #[getter]
    /// Selected object position in absolute coordinates, if initialized.
    fn absolute_object_position(&self) -> Option<PyPoint3> {
        self.inner
            .absolute_object_position()
            .map(PyPoint3::from_inner)
    }

    #[getter]
    /// Taut cable indices for the currently selected branch.
    fn taut_cables(&self) -> Vec<usize> {
        self.inner.taut_cables().to_vec()
    }

    #[getter]
    /// Index of the selected branch in the FK solution cache.
    fn solution_index(&self) -> Option<usize> {
        self.inner.solution_index()
    }

    fn __repr__(&self) -> String {
        format!(
            "VvcmManualSimulation(robot_count={})",
            self.inner.fk_engine().robot_count()
        )
    }
}

impl PyVvcmManualSimulation {
    fn from_inner(inner: VvcmManualSimulation) -> Self {
        Self { inner }
    }
}

#[pymodule]
fn _vvcm_rs(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add("VvcmError", py.get_type::<VvcmError>())?;
    m.add(
        "DimensionMismatchError",
        py.get_type::<DimensionMismatchError>(),
    )?;
    m.add(
        "InfeasibleFormationError",
        py.get_type::<InfeasibleFormationError>(),
    )?;
    m.add("NoSolutionError", py.get_type::<NoSolutionError>())?;
    m.add(
        "NoStableSolutionError",
        py.get_type::<NoStableSolutionError>(),
    )?;
    m.add_class::<PyPoint2>()?;
    m.add_class::<PyPoint3>()?;
    m.add_class::<PyRobotFormation>()?;
    m.add_class::<PySheetShape>()?;
    m.add_class::<PyFkSolution>()?;
    m.add_class::<PyFkSolutions>()?;
    m.add_class::<PyVvcmFk>()?;
    m.add_class::<PyVvcmSimulation>()?;
    m.add_class::<PyVvcmManualSimulation>()?;
    Ok(())
}

fn map_vvcm_error(error: CoreVvcmError) -> PyErr {
    let message = error.to_string();
    match error {
        CoreVvcmError::DimensionMismatch { .. } => DimensionMismatchError::new_err(message),
        CoreVvcmError::InfeasibleFormation => InfeasibleFormationError::new_err(message),
        CoreVvcmError::NoSolution => NoSolutionError::new_err(message),
        CoreVvcmError::NoStableSolution => NoStableSolutionError::new_err(message),
    }
}

fn point2_from_py(value: &Bound<'_, PyAny>) -> PyResult<Point2> {
    if let Ok(point) = value.extract::<PyRef<'_, PyPoint2>>() {
        return Ok(point.inner);
    }

    let len = value.len().map_err(|_| {
        PyTypeError::new_err("expected Point2 or a sequence with exactly two numeric values")
    })?;
    if len != 2 {
        return Err(PyValueError::new_err(
            "expected Point2 or a sequence with exactly two numeric values",
        ));
    }

    Ok(Point2::new(
        value.get_item(0)?.extract::<Scalar>()?,
        value.get_item(1)?.extract::<Scalar>()?,
    ))
}

fn point3_from_py(value: &Bound<'_, PyAny>) -> PyResult<Point3> {
    if let Ok(point) = value.extract::<PyRef<'_, PyPoint3>>() {
        return Ok(point.inner);
    }

    let len = value.len().map_err(|_| {
        PyTypeError::new_err("expected Point3 or a sequence with exactly three numeric values")
    })?;
    if len != 3 {
        return Err(PyValueError::new_err(
            "expected Point3 or a sequence with exactly three numeric values",
        ));
    }

    Ok(Point3::new(
        value.get_item(0)?.extract::<Scalar>()?,
        value.get_item(1)?.extract::<Scalar>()?,
        value.get_item(2)?.extract::<Scalar>()?,
    ))
}

fn optional_point2(value: Option<&Bound<'_, PyAny>>) -> PyResult<Point2> {
    value
        .map(point2_from_py)
        .transpose()
        .map(|value| value.unwrap_or_else(Point2::zero))
}

fn optional_point3(value: Option<&Bound<'_, PyAny>>) -> PyResult<Point3> {
    value
        .map(point3_from_py)
        .transpose()
        .map(|value| value.unwrap_or_else(Point3::zero))
}

fn point2_sequence_from_py(value: &Bound<'_, PyAny>, context: &str) -> PyResult<Vec<Point2>> {
    let len = value
        .len()
        .map_err(|_| PyTypeError::new_err(format!("{context} must be a sequence of points")))?;
    let mut points = Vec::with_capacity(len);

    for index in 0..len {
        let item = value.get_item(index)?;
        let point = point2_from_py(&item).map_err(|_| {
            PyValueError::new_err(format!(
                "{context}[{index}] must be a Point2 or a sequence with exactly two numeric values"
            ))
        })?;
        points.push(point);
    }

    Ok(points)
}

fn formation_from_py(value: &Bound<'_, PyAny>) -> PyResult<RobotFormation> {
    if let Ok(formation) = value.extract::<PyRef<'_, PyRobotFormation>>() {
        return Ok(formation.inner.clone());
    }

    RobotFormation::new(point2_sequence_from_py(value, "robot formation")?).map_err(map_vvcm_error)
}

fn sheet_from_py(value: &Bound<'_, PyAny>) -> PyResult<SheetShape> {
    if let Ok(sheet) = value.extract::<PyRef<'_, PySheetShape>>() {
        return Ok(sheet.inner.clone());
    }

    SheetShape::new(point2_sequence_from_py(value, "sheet shape")?).map_err(map_vvcm_error)
}

fn point2_vec_to_py(points: &[Point2]) -> Vec<PyPoint2> {
    points.iter().copied().map(PyPoint2::from_inner).collect()
}

fn normalize_index(index: isize, len: usize) -> PyResult<usize> {
    let normalized = if index < 0 {
        len as isize + index
    } else {
        index
    };

    if normalized < 0 || normalized >= len as isize {
        Err(PyIndexError::new_err("index out of range"))
    } else {
        Ok(normalized as usize)
    }
}
