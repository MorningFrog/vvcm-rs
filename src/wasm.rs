//! WebAssembly bindings for browser and bundler consumers.
//!
//! The bindings expose row-major typed arrays instead of per-point JavaScript
//! objects. Formation, sheet, and velocity inputs are `Float32Array` values
//! laid out as `[x0, y0, x1, y1, ...]`.

use crate::{
    FkSolutions, Point2, Point3, Scalar, Vector2, VvcmError as CoreError, VvcmFk as CoreFk,
    VvcmManualSimulation as CoreManualSimulation, VvcmSimulation as CoreSimulation,
};
use js_sys::{Array, Float32Array, Object, Reflect, Uint32Array};
use wasm_bindgen::prelude::*;

/// Returns the package version compiled into this WASM module.
#[wasm_bindgen(js_name = version)]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

/// Stateful forward-kinematics solver for JavaScript and TypeScript callers.
#[wasm_bindgen]
pub struct VvcmFk {
    inner: CoreFk,
}

#[wasm_bindgen]
impl VvcmFk {
    /// Creates a new FK solver from a row-major `Float32Array` sheet.
    #[wasm_bindgen(constructor)]
    pub fn new(hold_height: Scalar, sheet: Float32Array) -> Result<VvcmFk, JsValue> {
        let sheet = points_from_float32_array(&sheet, "sheet")?;
        let inner = CoreFk::new(hold_height, sheet).map_err(core_error_to_js)?;
        Ok(Self { inner })
    }

    /// Solves the current robot formation and returns every candidate branch.
    #[wasm_bindgen(js_name = updateStableSolutions)]
    pub fn update_stable_solutions(&mut self, formation: Float32Array) -> Result<JsValue, JsValue> {
        let formation = points_from_float32_array(&formation, "formation")?;
        let solutions = self
            .inner
            .update_stable_solutions(&formation)
            .map_err(core_error_to_js)?;
        solutions_to_js(solutions)
    }

    /// Returns the cached solutions from the most recent solve.
    pub fn solutions(&self) -> Result<JsValue, JsValue> {
        solutions_to_js(self.inner.solutions())
    }

    /// Returns the fixed robot count.
    #[wasm_bindgen(js_name = robotCount)]
    pub fn robot_count(&self) -> usize {
        self.inner.robot_count()
    }

    /// Returns the fixed robot holding height.
    #[wasm_bindgen(js_name = holdHeight)]
    pub fn hold_height(&self) -> Scalar {
        self.inner.hold_height()
    }
}

/// Velocity-driven VVCM simulation for JavaScript and TypeScript callers.
#[wasm_bindgen]
pub struct VvcmSimulation {
    inner: CoreSimulation,
}

#[wasm_bindgen]
impl VvcmSimulation {
    /// Creates a simulation from typed-array inputs.
    #[wasm_bindgen(constructor)]
    pub fn new(
        hold_height: Scalar,
        sheet: Float32Array,
        initial_formation: Float32Array,
        po_initial: Float32Array,
        dt: Scalar,
    ) -> Result<VvcmSimulation, JsValue> {
        let sheet = points_from_float32_array(&sheet, "sheet")?;
        let initial_formation = points_from_float32_array(&initial_formation, "initialFormation")?;
        let po_initial = point3_from_float32_array(&po_initial, "poInitial")?;
        let inner = CoreSimulation::new(hold_height, sheet, &initial_formation, po_initial, dt)
            .map_err(core_error_to_js)?;
        Ok(Self { inner })
    }

    /// Sets one XY velocity vector per robot.
    #[wasm_bindgen(js_name = setVelocity)]
    pub fn set_velocity(&mut self, velocity: Float32Array) -> Result<(), JsValue> {
        let velocity = vectors_from_float32_array(&velocity, "velocity")?;
        self.inner.set_velocity(&velocity).map_err(core_error_to_js)
    }

    /// Advances the simulation by one fixed time step.
    pub fn step(&mut self) -> Result<(), JsValue> {
        self.inner.step().map_err(core_error_to_js)
    }

    /// Returns the current robot formation in absolute coordinates.
    #[wasm_bindgen(js_name = absoluteFormation)]
    pub fn absolute_formation(&self) -> Float32Array {
        point_slice_to_float32_array(self.inner.absolute_formation())
    }

    /// Returns the selected object position in absolute coordinates.
    #[wasm_bindgen(js_name = absoluteObjectPosition)]
    pub fn absolute_object_position(&self) -> Float32Array {
        point3_to_float32_array(self.inner.absolute_object_position())
    }

    /// Returns the local-frame origin in absolute coordinates.
    #[wasm_bindgen(js_name = globalPosition)]
    pub fn global_position(&self) -> Float32Array {
        point2_to_float32_array(self.inner.global_position())
    }

    /// Returns the robot formation in the simulation-local frame.
    pub fn formation(&self) -> Float32Array {
        point_slice_to_float32_array(self.inner.formation())
    }

    /// Returns the selected object position in the simulation-local frame.
    #[wasm_bindgen(js_name = objectPosition)]
    pub fn object_position(&self) -> Float32Array {
        point3_to_float32_array(self.inner.object_position())
    }

    /// Returns the taut cable indices for the selected branch.
    #[wasm_bindgen(js_name = tautCables)]
    pub fn taut_cables(&self) -> Uint32Array {
        usize_slice_to_uint32_array(self.inner.taut_cables())
    }

    /// Returns the selected solution index, or `null` when none is selected.
    #[wasm_bindgen(js_name = solutionIndex)]
    pub fn solution_index(&self) -> JsValue {
        optional_index_to_js(self.inner.solution_index())
    }

    /// Returns the fixed integration time step.
    pub fn dt(&self) -> Scalar {
        self.inner.dt()
    }

    /// Returns the current per-robot velocity vectors.
    pub fn velocity(&self) -> Float32Array {
        vector_slice_to_float32_array(self.inner.velocity())
    }

    /// Returns the cached FK solutions from the underlying solver.
    pub fn solutions(&self) -> Result<JsValue, JsValue> {
        solutions_to_js(self.inner.fk_engine().solutions())
    }
}

/// Manual-formation VVCM simulation for JavaScript and TypeScript callers.
#[wasm_bindgen]
pub struct VvcmManualSimulation {
    inner: CoreManualSimulation,
}

#[wasm_bindgen]
impl VvcmManualSimulation {
    /// Creates a manual simulation wrapper for a fixed sheet.
    #[wasm_bindgen(constructor)]
    pub fn new(hold_height: Scalar, sheet: Float32Array) -> Result<VvcmManualSimulation, JsValue> {
        let sheet = points_from_float32_array(&sheet, "sheet")?;
        let inner = CoreManualSimulation::new(hold_height, sheet).map_err(core_error_to_js)?;
        Ok(Self { inner })
    }

    /// Initializes the wrapper and returns the selected absolute object pose.
    pub fn init(
        &mut self,
        formation: Float32Array,
        po_initial: Float32Array,
    ) -> Result<Float32Array, JsValue> {
        let formation = points_from_float32_array(&formation, "formation")?;
        let po_initial = point3_from_float32_array(&po_initial, "poInitial")?;
        let point = self
            .inner
            .init(&formation, po_initial)
            .map_err(core_error_to_js)?;
        Ok(point3_to_float32_array(point))
    }

    /// Updates the wrapper from a new formation and returns the selected pose.
    #[wasm_bindgen(js_name = getNewStableSolution)]
    pub fn get_new_stable_solution(
        &mut self,
        formation: Float32Array,
    ) -> Result<Float32Array, JsValue> {
        let formation = points_from_float32_array(&formation, "formation")?;
        let point = self
            .inner
            .get_new_stable_solution(&formation)
            .map_err(core_error_to_js)?;
        Ok(point3_to_float32_array(point))
    }

    /// Returns the local-frame origin in absolute coordinates.
    #[wasm_bindgen(js_name = globalPosition)]
    pub fn global_position(&self) -> Float32Array {
        point2_to_float32_array(self.inner.global_position())
    }

    /// Returns whether the wrapper has been initialized with a formation.
    #[wasm_bindgen(js_name = hasFormation)]
    pub fn has_formation(&self) -> bool {
        self.inner.formation().is_some()
    }

    /// Returns the current formation in the centroid-relative local frame.
    pub fn formation(&self) -> JsValue {
        self.inner.formation().map_or(JsValue::NULL, |formation| {
            point_slice_to_float32_array(formation).into()
        })
    }

    /// Returns the selected object position in the local frame.
    #[wasm_bindgen(js_name = objectPosition)]
    pub fn object_position(&self) -> JsValue {
        optional_point3_to_js(self.inner.object_position())
    }

    /// Returns the selected object position in absolute coordinates.
    #[wasm_bindgen(js_name = absoluteObjectPosition)]
    pub fn absolute_object_position(&self) -> JsValue {
        optional_point3_to_js(self.inner.absolute_object_position())
    }

    /// Returns the taut cable indices for the selected branch.
    #[wasm_bindgen(js_name = tautCables)]
    pub fn taut_cables(&self) -> Uint32Array {
        usize_slice_to_uint32_array(self.inner.taut_cables())
    }

    /// Returns the selected solution index, or `null` when none is selected.
    #[wasm_bindgen(js_name = solutionIndex)]
    pub fn solution_index(&self) -> JsValue {
        optional_index_to_js(self.inner.solution_index())
    }

    /// Returns the cached FK solutions from the underlying solver.
    pub fn solutions(&self) -> Result<JsValue, JsValue> {
        solutions_to_js(self.inner.fk_engine().solutions())
    }
}

fn points_from_float32_array(
    array: &Float32Array,
    context: &'static str,
) -> Result<Vec<Point2>, JsValue> {
    let values = array.to_vec();
    if values.len() % 2 != 0 {
        return Err(invalid_argument_error(format!(
            "{context} length must be divisible by 2"
        )));
    }

    Ok(values
        .chunks_exact(2)
        .map(|row| Point2::new(row[0], row[1]))
        .collect())
}

fn vectors_from_float32_array(
    array: &Float32Array,
    context: &'static str,
) -> Result<Vec<Vector2>, JsValue> {
    let values = array.to_vec();
    if values.len() % 2 != 0 {
        return Err(invalid_argument_error(format!(
            "{context} length must be divisible by 2"
        )));
    }

    Ok(values
        .chunks_exact(2)
        .map(|row| Vector2::new(row[0], row[1]))
        .collect())
}

fn point3_from_float32_array(
    array: &Float32Array,
    context: &'static str,
) -> Result<Point3, JsValue> {
    let values = array.to_vec();
    if values.len() != 3 {
        return Err(invalid_argument_error(format!(
            "{context} length must be exactly 3"
        )));
    }

    Ok(Point3::new(values[0], values[1], values[2]))
}

fn solutions_to_js(solutions: &FkSolutions) -> Result<JsValue, JsValue> {
    let solution_values = Array::new();
    for solution in solutions.iter() {
        let item = Object::new();
        set_property(&item, "stable", JsValue::from_bool(solution.stable))?;
        set_property(&item, "po", point3_to_object(solution.po).into())?;
        set_property(&item, "vo", point2_to_object(solution.vo).into())?;
        set_property(
            &item,
            "tautCables",
            usize_slice_to_array(&solution.taut_cables).into(),
        )?;
        set_property(
            &item,
            "lambdaValues",
            scalar_slice_to_array(&solution.lambda_values).into(),
        )?;
        solution_values.push(&item);
    }

    let object = Object::new();
    set_property(&object, "solutions", solution_values.into())?;
    set_property(
        &object,
        "allCount",
        JsValue::from_f64(solutions.all_count() as f64),
    )?;
    set_property(
        &object,
        "stableCount",
        JsValue::from_f64(solutions.stable_count() as f64),
    )?;

    Ok(object.into())
}

fn point_slice_to_float32_array(points: &[Point2]) -> Float32Array {
    let mut values = Vec::with_capacity(points.len() * 2);
    for point in points {
        values.extend_from_slice(&[point.x, point.y]);
    }
    Float32Array::from(values.as_slice())
}

fn vector_slice_to_float32_array(vectors: &[Vector2]) -> Float32Array {
    let mut values = Vec::with_capacity(vectors.len() * 2);
    for vector in vectors {
        values.extend_from_slice(&[vector.x, vector.y]);
    }
    Float32Array::from(values.as_slice())
}

fn point2_to_float32_array(point: Point2) -> Float32Array {
    Float32Array::from([point.x, point.y].as_slice())
}

fn point3_to_float32_array(point: Point3) -> Float32Array {
    Float32Array::from([point.x, point.y, point.z].as_slice())
}

fn point2_to_object(point: Point2) -> Object {
    let object = Object::new();
    let _ = set_property(&object, "x", JsValue::from_f64(point.x as f64));
    let _ = set_property(&object, "y", JsValue::from_f64(point.y as f64));
    object
}

fn point3_to_object(point: Point3) -> Object {
    let object = Object::new();
    let _ = set_property(&object, "x", JsValue::from_f64(point.x as f64));
    let _ = set_property(&object, "y", JsValue::from_f64(point.y as f64));
    let _ = set_property(&object, "z", JsValue::from_f64(point.z as f64));
    object
}

fn usize_slice_to_array(values: &[usize]) -> Array {
    let array = Array::new();
    for value in values {
        array.push(&JsValue::from_f64(*value as f64));
    }
    array
}

fn scalar_slice_to_array(values: &[Scalar]) -> Array {
    let array = Array::new();
    for value in values {
        array.push(&JsValue::from_f64(*value as f64));
    }
    array
}

fn usize_slice_to_uint32_array(values: &[usize]) -> Uint32Array {
    usize_vec_to_uint32_array(values)
}

fn usize_vec_to_uint32_array(values: &[usize]) -> Uint32Array {
    let values = values.iter().map(|value| *value as u32).collect::<Vec<_>>();
    Uint32Array::from(values.as_slice())
}

fn optional_index_to_js(value: Option<usize>) -> JsValue {
    value.map_or(JsValue::NULL, |index| JsValue::from_f64(index as f64))
}

fn optional_point3_to_js(value: Option<Point3>) -> JsValue {
    value.map_or(JsValue::NULL, |point| point3_to_float32_array(point).into())
}

fn core_error_to_js(error: CoreError) -> JsValue {
    match error {
        CoreError::DimensionMismatch {
            context,
            expected,
            actual,
        } => {
            let value = vvcm_error("DIMENSION_MISMATCH", error.to_string());
            let _ = set_property(&value, "context", JsValue::from_str(context));
            let _ = set_property(&value, "expected", JsValue::from_f64(expected as f64));
            let _ = set_property(&value, "actual", JsValue::from_f64(actual as f64));
            value
        }
        CoreError::InfeasibleFormation => vvcm_error("INFEASIBLE_FORMATION", error.to_string()),
        CoreError::NoSolution => vvcm_error("NO_SOLUTION", error.to_string()),
        CoreError::NoStableSolution => vvcm_error("NO_STABLE_SOLUTION", error.to_string()),
    }
}

fn invalid_argument_error(message: impl Into<String>) -> JsValue {
    vvcm_error("INVALID_ARGUMENT", message.into())
}

fn vvcm_error(code: &'static str, message: String) -> JsValue {
    let error = js_sys::Error::new(&message);
    let value: JsValue = error.into();
    let _ = set_property(&value, "name", JsValue::from_str("VvcmError"));
    let _ = set_property(&value, "code", JsValue::from_str(code));
    value
}

fn set_property(target: &JsValue, key: &str, value: JsValue) -> Result<(), JsValue> {
    Reflect::set(target, &JsValue::from_str(key), &value).map(|_| ())
}
