//! Python bindings for the VVCM solver.
//!
//! The Python API is NumPy-first: formation, sheet, and velocity inputs are
//! C-contiguous `float32` arrays with shape `(n, 2)`, and result sets expose
//! bulk NumPy arrays instead of per-point wrapper objects.

use crate::{
    FkSolution as CoreFkSolution, FkSolutions as CoreFkSolutions, Point2, Point3, Scalar, Vector2,
    VvcmError as CoreVvcmError, VvcmFk as CoreFk, VvcmManualSimulation as CoreManualSimulation,
    VvcmSimulation as CoreSimulation,
};
use numpy::{
    PyArray1, PyArray2, PyArrayMethods, PyReadonlyArray1, PyReadonlyArray2, PyUntypedArrayMethods,
};
use pyo3::exceptions::{PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyModule;

pyo3::create_exception!(vvcm_rs, VvcmError, pyo3::exceptions::PyException);
pyo3::create_exception!(vvcm_rs, DimensionMismatchError, VvcmError);
pyo3::create_exception!(vvcm_rs, InfeasibleFormationError, VvcmError);
pyo3::create_exception!(vvcm_rs, NoSolutionError, VvcmError);
pyo3::create_exception!(vvcm_rs, NoStableSolutionError, VvcmError);

#[pyclass(name = "FkSolution", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone, Default)]
struct PyFkSolution {
    stable: bool,
    po: Point3,
    vo: Point2,
    taut_cables: Vec<usize>,
    lambda_values: Vec<Scalar>,
}

#[pymethods]
impl PyFkSolution {
    /// Whether the candidate is locally stable.
    #[getter]
    fn stable(&self) -> bool {
        self.stable
    }

    /// Object position `Po` as a `float32` array with shape `(3,)`.
    #[getter]
    fn po<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<Scalar>> {
        point3_to_array(py, self.po)
    }

    /// Virtual object point `Vo` as a `float32` array with shape `(2,)`.
    #[getter]
    fn vo<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<Scalar>> {
        point2_to_array(py, self.vo)
    }

    /// Taut cable indices for this candidate.
    #[getter]
    fn taut_cables<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<usize>> {
        PyArray1::from_vec(py, self.taut_cables.clone())
    }

    /// Lagrange multiplier coefficients matching `taut_cables`.
    #[getter]
    fn lambda_values<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<Scalar>> {
        PyArray1::from_vec(py, self.lambda_values.clone())
    }

    fn __repr__(&self) -> String {
        format!(
            "FkSolution(stable={}, taut_cables={:?})",
            self.stable, self.taut_cables
        )
    }
}

impl PyFkSolution {
    fn from_inner(inner: &CoreFkSolution) -> Self {
        Self {
            stable: inner.stable,
            po: inner.po,
            vo: inner.vo,
            taut_cables: inner.taut_cables.clone(),
            lambda_values: inner.lambda_values.clone(),
        }
    }
}

#[pyclass(name = "FkSolutions", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone, Default)]
struct PyFkSolutions {
    solutions: Vec<PyFkSolution>,
}

#[pymethods]
impl PyFkSolutions {
    /// Create an empty solution collection.
    #[new]
    fn new() -> Self {
        Self::default()
    }

    /// Ordered candidate solutions from the most recent FK update.
    #[getter]
    fn solutions(&self) -> Vec<PyFkSolution> {
        self.solutions.clone()
    }

    /// Return locally stable candidate solutions.
    fn stable(&self) -> Vec<PyFkSolution> {
        self.solutions
            .iter()
            .filter(|solution| solution.stable)
            .cloned()
            .collect()
    }

    /// Return `True` when no candidates are stored.
    fn is_empty(&self) -> bool {
        self.solutions.is_empty()
    }

    /// Count locally stable candidate solutions.
    fn stable_count(&self) -> usize {
        self.solutions
            .iter()
            .filter(|solution| solution.stable)
            .count()
    }

    /// Count all candidate solutions, stable and unstable.
    fn all_count(&self) -> usize {
        self.solutions.len()
    }

    /// Return the index of the stable branch closest to `reference`, or `None`.
    fn closest_stable_to(
        &self,
        reference: PyReadonlyArray1<'_, Scalar>,
    ) -> PyResult<Option<usize>> {
        let reference = point3_from_array(reference, "reference")?;
        let mut best: Option<(usize, Scalar)> = None;

        for (index, solution) in self.solutions.iter().enumerate() {
            if !solution.stable {
                continue;
            }

            let distance = (solution.po - reference).norm();
            if best.is_none_or(|(_, best_distance)| distance < best_distance) {
                best = Some((index, distance));
            }
        }

        Ok(best.map(|(index, _)| index))
    }

    fn __len__(&self) -> usize {
        self.solutions.len()
    }

    fn __repr__(&self) -> String {
        format!(
            "FkSolutions(all_count={}, stable_count={})",
            self.all_count(),
            self.stable_count()
        )
    }
}

impl PyFkSolutions {
    fn from_inner(inner: &CoreFkSolutions) -> Self {
        Self {
            solutions: inner.iter().map(PyFkSolution::from_inner).collect(),
        }
    }
}

#[pyclass(name = "VvcmFk", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
struct PyVvcmFk {
    inner: CoreFk,
}

#[pymethods]
impl PyVvcmFk {
    /// Create a solver from a C-contiguous `float32` sheet array with shape `(n, 2)`.
    #[new]
    fn new(hold_height: Scalar, sheet: PyReadonlyArray2<'_, Scalar>) -> PyResult<Self> {
        let sheet = point2_vec_from_array(sheet, "sheet")?;
        Ok(Self {
            inner: CoreFk::new(hold_height, sheet).map_err(map_vvcm_error)?,
        })
    }

    /// Solve and store forward-kinematics branches for `formation`.
    fn update_stable_solutions(
        &mut self,
        formation: PyReadonlyArray2<'_, Scalar>,
    ) -> PyResult<PyFkSolutions> {
        let formation = point2_vec_from_array(formation, "formation")?;
        self.inner
            .update_stable_solutions(&formation)
            .map_err(map_vvcm_error)?;
        Ok(PyFkSolutions::from_inner(self.inner.solutions()))
    }

    /// Fixed number of robots solved by this engine.
    #[getter]
    fn robot_count(&self) -> usize {
        self.inner.robot_count()
    }

    /// Fixed robot holding height used to recover the object Z coordinate.
    #[getter]
    fn hold_height(&self) -> Scalar {
        self.inner.hold_height()
    }

    /// Fixed sheet geometry as a `float32` array with shape `(n, 2)`.
    #[getter]
    fn sheet<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<Scalar>>> {
        point2_slice_to_array(py, self.inner.sheet())
    }

    /// Most recent formation passed to `update_stable_solutions`, if any.
    #[getter]
    fn current_formation<'py>(
        &self,
        py: Python<'py>,
    ) -> PyResult<Option<Bound<'py, PyArray2<Scalar>>>> {
        self.inner
            .current_formation()
            .map(|formation| point2_slice_to_array(py, formation))
            .transpose()
    }

    /// Most recent solution cache.
    #[getter]
    fn solutions(&self) -> PyFkSolutions {
        PyFkSolutions::from_inner(self.inner.solutions())
    }
}

impl PyVvcmFk {
    fn from_inner(inner: CoreFk) -> Self {
        Self { inner }
    }
}

#[pyclass(name = "VvcmSimulation", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
struct PyVvcmSimulation {
    inner: CoreSimulation,
}

#[pymethods]
impl PyVvcmSimulation {
    /// Create a velocity-driven simulation from NumPy matrix inputs.
    #[new]
    #[pyo3(signature = (hold_height, sheet, initial_formation, po_initial=None, dt=0.033333335))]
    fn new(
        hold_height: Scalar,
        sheet: PyReadonlyArray2<'_, Scalar>,
        initial_formation: PyReadonlyArray2<'_, Scalar>,
        po_initial: Option<PyReadonlyArray1<'_, Scalar>>,
        dt: Scalar,
    ) -> PyResult<Self> {
        let sheet = point2_vec_from_array(sheet, "sheet")?;
        let initial_formation = point2_vec_from_array(initial_formation, "initial formation")?;
        let po_initial = optional_point3_from_array(po_initial, "po_initial")?;
        Ok(Self {
            inner: CoreSimulation::new(hold_height, sheet, &initial_formation, po_initial, dt)
                .map_err(map_vvcm_error)?,
        })
    }

    /// Set one XY velocity vector per robot from a `(n, 2)` `float32` array.
    fn set_velocity(&mut self, velocity: PyReadonlyArray2<'_, Scalar>) -> PyResult<()> {
        let velocity = vector2_vec_from_array(velocity, "velocity")?;
        self.inner.set_velocity(&velocity).map_err(map_vvcm_error)
    }

    /// Advance the simulation by one fixed time step.
    fn step(&mut self) -> PyResult<()> {
        self.inner.step().map_err(map_vvcm_error)
    }

    /// Current robot formation in absolute coordinates.
    fn absolute_formation<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<Scalar>>> {
        point2_slice_to_array(py, self.inner.absolute_formation())
    }

    /// Selected object position in absolute coordinates.
    fn absolute_object_position<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<Scalar>> {
        point3_to_array(py, self.inner.absolute_object_position())
    }

    /// Snapshot of the underlying FK engine and its latest solution cache.
    #[getter]
    fn fk_engine(&self) -> PyVvcmFk {
        PyVvcmFk::from_inner(self.inner.fk_engine().clone())
    }

    /// Local-frame origin in absolute coordinates.
    #[getter]
    fn global_position<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<Scalar>> {
        point2_to_array(py, self.inner.global_position())
    }

    /// Current robot formation in the local frame.
    #[getter]
    fn formation<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<Scalar>>> {
        point2_slice_to_array(py, self.inner.formation())
    }

    /// Selected object position in the local frame.
    #[getter]
    fn object_position<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<Scalar>> {
        point3_to_array(py, self.inner.object_position())
    }

    /// Taut cable indices for the currently selected branch.
    #[getter]
    fn taut_cables<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<usize>> {
        PyArray1::from_vec(py, self.inner.taut_cables().to_vec())
    }

    /// Index of the selected branch in the FK solution cache.
    #[getter]
    fn solution_index(&self) -> Option<usize> {
        self.inner.solution_index()
    }

    /// Fixed integration time step.
    #[getter]
    fn dt(&self) -> Scalar {
        self.inner.dt()
    }

    /// Current per-robot velocity array.
    #[getter]
    fn velocity<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<Scalar>>> {
        vector2_slice_to_array(py, self.inner.velocity())
    }

    /// Cached FK solutions from the underlying solver.
    fn solutions(&self) -> PyFkSolutions {
        PyFkSolutions::from_inner(self.inner.fk_engine().solutions())
    }
}

#[pyclass(name = "VvcmManualSimulation", module = "vvcm_rs", skip_from_py_object)]
#[derive(Debug, Clone)]
struct PyVvcmManualSimulation {
    inner: CoreManualSimulation,
}

#[pymethods]
impl PyVvcmManualSimulation {
    /// Create a manual simulation wrapper for a fixed sheet.
    #[new]
    fn new(hold_height: Scalar, sheet: PyReadonlyArray2<'_, Scalar>) -> PyResult<Self> {
        let sheet = point2_vec_from_array(sheet, "sheet")?;
        Ok(Self {
            inner: CoreManualSimulation::new(hold_height, sheet).map_err(map_vvcm_error)?,
        })
    }

    /// Initialize with the first absolute formation and reference object position.
    #[pyo3(signature = (formation, po_initial=None))]
    fn init<'py>(
        &mut self,
        formation: PyReadonlyArray2<'py, Scalar>,
        po_initial: Option<PyReadonlyArray1<'py, Scalar>>,
    ) -> PyResult<Bound<'py, PyArray1<Scalar>>> {
        let py = formation.py();
        let formation = point2_vec_from_array(formation, "formation")?;
        let po_initial = optional_point3_from_array(po_initial, "po_initial")?;
        let position = self
            .inner
            .init(&formation, po_initial)
            .map_err(map_vvcm_error)?;
        Ok(point3_to_array(py, position))
    }

    /// Update from a new formation and return the closest stable object position.
    fn get_new_stable_solution<'py>(
        &mut self,
        formation: PyReadonlyArray2<'py, Scalar>,
    ) -> PyResult<Bound<'py, PyArray1<Scalar>>> {
        let py = formation.py();
        let formation = point2_vec_from_array(formation, "formation")?;
        let position = self
            .inner
            .get_new_stable_solution(&formation)
            .map_err(map_vvcm_error)?;
        Ok(point3_to_array(py, position))
    }

    /// Snapshot of the underlying FK engine and its latest solution cache.
    #[getter]
    fn fk_engine(&self) -> PyVvcmFk {
        PyVvcmFk::from_inner(self.inner.fk_engine().clone())
    }

    /// Current local-frame origin in absolute coordinates.
    #[getter]
    fn global_position<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<Scalar>> {
        point2_to_array(py, self.inner.global_position())
    }

    /// Current robot formation in the centroid-relative local frame, if initialized.
    #[getter]
    fn formation<'py>(&self, py: Python<'py>) -> PyResult<Option<Bound<'py, PyArray2<Scalar>>>> {
        self.inner
            .formation()
            .map(|formation| point2_slice_to_array(py, formation))
            .transpose()
    }

    /// Selected object position in the local frame, if initialized.
    #[getter]
    fn object_position<'py>(&self, py: Python<'py>) -> Option<Bound<'py, PyArray1<Scalar>>> {
        self.inner
            .object_position()
            .map(|position| point3_to_array(py, position))
    }

    /// Selected object position in absolute coordinates, if initialized.
    #[getter]
    fn absolute_object_position<'py>(
        &self,
        py: Python<'py>,
    ) -> Option<Bound<'py, PyArray1<Scalar>>> {
        self.inner
            .absolute_object_position()
            .map(|position| point3_to_array(py, position))
    }

    /// Taut cable indices for the currently selected branch.
    #[getter]
    fn taut_cables<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<usize>> {
        PyArray1::from_vec(py, self.inner.taut_cables().to_vec())
    }

    /// Index of the selected branch in the FK solution cache.
    #[getter]
    fn solution_index(&self) -> Option<usize> {
        self.inner.solution_index()
    }

    /// Cached FK solutions from the underlying solver.
    fn solutions(&self) -> PyFkSolutions {
        PyFkSolutions::from_inner(self.inner.fk_engine().solutions())
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

    m.add_class::<PyFkSolutions>()?;
    m.add_class::<PyFkSolution>()?;
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

fn point2_vec_from_array(
    array: PyReadonlyArray2<'_, Scalar>,
    context: &'static str,
) -> PyResult<Vec<Point2>> {
    let shape = array.shape();
    if shape[1] != 2 {
        return Err(PyValueError::new_err(format!(
            "{context} must have shape (n, 2); got ({}, {})",
            shape[0], shape[1]
        )));
    }

    if !array.is_c_contiguous() {
        return Err(PyTypeError::new_err(format!(
            "{context} must be C-contiguous"
        )));
    }
    let values = array.as_slice().map_err(|_| {
        PyTypeError::new_err(format!(
            "{context} must be a C-contiguous, aligned float32 array"
        ))
    })?;
    Ok(values
        .chunks_exact(2)
        .map(|row| Point2::new(row[0], row[1]))
        .collect())
}

fn vector2_vec_from_array(
    array: PyReadonlyArray2<'_, Scalar>,
    context: &'static str,
) -> PyResult<Vec<Vector2>> {
    let shape = array.shape();
    if shape[1] != 2 {
        return Err(PyValueError::new_err(format!(
            "{context} must have shape (n, 2); got ({}, {})",
            shape[0], shape[1]
        )));
    }

    if !array.is_c_contiguous() {
        return Err(PyTypeError::new_err(format!(
            "{context} must be C-contiguous"
        )));
    }
    let values = array.as_slice().map_err(|_| {
        PyTypeError::new_err(format!(
            "{context} must be a C-contiguous, aligned float32 array"
        ))
    })?;
    Ok(values
        .chunks_exact(2)
        .map(|row| Vector2::new(row[0], row[1]))
        .collect())
}

fn point3_from_array(
    array: PyReadonlyArray1<'_, Scalar>,
    context: &'static str,
) -> PyResult<Point3> {
    if array.shape()[0] != 3 {
        return Err(PyValueError::new_err(format!(
            "{context} must have shape (3,); got ({},)",
            array.shape()[0]
        )));
    }

    let values = array.as_slice().map_err(|_| {
        PyTypeError::new_err(format!(
            "{context} must be a C-contiguous, aligned float32 array"
        ))
    })?;
    Ok(Point3::new(values[0], values[1], values[2]))
}

fn optional_point3_from_array(
    value: Option<PyReadonlyArray1<'_, Scalar>>,
    context: &'static str,
) -> PyResult<Point3> {
    value
        .map(|value| point3_from_array(value, context))
        .transpose()
        .map(|value| value.unwrap_or_else(|| Point3::new(0.0, 0.0, 0.0)))
}

fn point2_slice_to_array<'py>(
    py: Python<'py>,
    points: &[Point2],
) -> PyResult<Bound<'py, PyArray2<Scalar>>> {
    let mut values = Vec::with_capacity(points.len() * 2);
    for point in points {
        values.extend_from_slice(&[point.x, point.y]);
    }
    array2_from_vec(py, values, points.len(), 2)
}

fn vector2_slice_to_array<'py>(
    py: Python<'py>,
    vectors: &[Vector2],
) -> PyResult<Bound<'py, PyArray2<Scalar>>> {
    let mut values = Vec::with_capacity(vectors.len() * 2);
    for vector in vectors {
        values.extend_from_slice(&[vector.x, vector.y]);
    }
    array2_from_vec(py, values, vectors.len(), 2)
}

fn point2_to_array(py: Python<'_>, point: Point2) -> Bound<'_, PyArray1<Scalar>> {
    PyArray1::from_vec(py, vec![point.x, point.y])
}

fn point3_to_array(py: Python<'_>, point: Point3) -> Bound<'_, PyArray1<Scalar>> {
    PyArray1::from_vec(py, vec![point.x, point.y, point.z])
}

fn array2_from_vec<'py>(
    py: Python<'py>,
    values: Vec<Scalar>,
    rows: usize,
    columns: usize,
) -> PyResult<Bound<'py, PyArray2<Scalar>>> {
    PyArray1::from_vec(py, values).reshape([rows, columns])
}
