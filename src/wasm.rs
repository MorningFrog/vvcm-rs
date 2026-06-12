//! WebAssembly bindings for browser and bundler consumers.
//!
//! The bindings keep the core solver types private and expose plain JavaScript
//! arrays and objects through `wasm-bindgen`.

use crate::{
    FkSolution, FkSolutions, Point2, Point3, RobotFormation, Scalar, SheetShape,
    VvcmError as CoreError, VvcmFk as CoreFk, VvcmManualSimulation as CoreManualSimulation,
    VvcmSimulation as CoreSimulation,
};
use js_sys::Reflect;
use serde::{Deserialize, Serialize};
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
    /// Creates a new FK solver.
    #[wasm_bindgen(constructor)]
    pub fn new(robot_count: usize, hold_height: Scalar, sheet: JsValue) -> Result<VvcmFk, JsValue> {
        let sheet = sheet_from_js(sheet)?;
        let inner = CoreFk::new(robot_count, hold_height, sheet).map_err(core_error_to_js)?;
        Ok(Self { inner })
    }

    /// Solves the current robot formation and returns every candidate branch.
    #[wasm_bindgen(js_name = updateStableSolutions)]
    pub fn update_stable_solutions(&mut self, formation: JsValue) -> Result<JsValue, JsValue> {
        let formation = formation_from_js(formation)?;
        let solutions = self
            .inner
            .update_stable_solutions(formation)
            .map_err(core_error_to_js)?;
        to_js_value(&solutions_output(solutions))
    }

    /// Returns the cached solutions from the most recent solve.
    pub fn solutions(&self) -> Result<JsValue, JsValue> {
        to_js_value(&solutions_output(self.inner.solutions()))
    }

    /// Returns only stable cached solutions from the most recent solve.
    #[wasm_bindgen(js_name = stableSolutions)]
    pub fn stable_solutions(&self) -> Result<JsValue, JsValue> {
        to_js_value(&stable_solutions_output(self.inner.solutions()))
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
    /// Creates a simulation from an initial formation and object pose.
    #[wasm_bindgen(constructor)]
    pub fn new(
        robot_count: usize,
        hold_height: Scalar,
        sheet: JsValue,
        initial_formation: JsValue,
        po_initial: JsValue,
        dt: Scalar,
    ) -> Result<VvcmSimulation, JsValue> {
        let sheet = sheet_from_js(sheet)?;
        let initial_formation = formation_from_js(initial_formation)?;
        let po_initial = point3_from_js(po_initial)?;
        let inner = CoreSimulation::new(
            robot_count,
            hold_height,
            sheet,
            initial_formation,
            po_initial,
            dt,
        )
        .map_err(core_error_to_js)?;
        Ok(Self { inner })
    }

    /// Sets one XY velocity vector per robot.
    #[wasm_bindgen(js_name = setVelocity)]
    pub fn set_velocity(&mut self, velocity: JsValue) -> Result<(), JsValue> {
        let velocity = formation_from_js(velocity)?;
        self.inner.set_velocity(velocity).map_err(core_error_to_js)
    }

    /// Advances the simulation by one fixed time step.
    pub fn step(&mut self) -> Result<(), JsValue> {
        self.inner.step().map_err(core_error_to_js)
    }

    /// Returns the current robot formation in absolute coordinates.
    #[wasm_bindgen(js_name = absoluteFormation)]
    pub fn absolute_formation(&self) -> Result<JsValue, JsValue> {
        let formation = self.inner.absolute_formation();
        to_js_value(&formation_output(&formation))
    }

    /// Returns the selected object position in absolute coordinates.
    #[wasm_bindgen(js_name = absoluteObjectPosition)]
    pub fn absolute_object_position(&self) -> Result<JsValue, JsValue> {
        to_js_value(&Point3Output::from(self.inner.absolute_object_position()))
    }

    /// Returns the local-frame origin in absolute coordinates.
    #[wasm_bindgen(js_name = globalPosition)]
    pub fn global_position(&self) -> Result<JsValue, JsValue> {
        to_js_value(&Point2Output::from(self.inner.global_position()))
    }

    /// Returns the robot formation in the simulation-local frame.
    pub fn formation(&self) -> Result<JsValue, JsValue> {
        to_js_value(&formation_output(self.inner.formation()))
    }

    /// Returns the selected object position in the simulation-local frame.
    #[wasm_bindgen(js_name = objectPosition)]
    pub fn object_position(&self) -> Result<JsValue, JsValue> {
        to_js_value(&Point3Output::from(self.inner.object_position()))
    }

    /// Returns the taut cable indices for the selected branch.
    #[wasm_bindgen(js_name = tautCables)]
    pub fn taut_cables(&self) -> Result<JsValue, JsValue> {
        to_js_value(&self.inner.taut_cables().to_vec())
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
    pub fn velocity(&self) -> Result<JsValue, JsValue> {
        to_js_value(&formation_output(self.inner.velocity()))
    }

    /// Returns the cached FK solutions from the underlying solver.
    pub fn solutions(&self) -> Result<JsValue, JsValue> {
        to_js_value(&solutions_output(self.inner.fk_engine().solutions()))
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
    pub fn new(
        robot_count: usize,
        hold_height: Scalar,
        sheet: JsValue,
    ) -> Result<VvcmManualSimulation, JsValue> {
        let sheet = sheet_from_js(sheet)?;
        let inner =
            CoreManualSimulation::new(robot_count, hold_height, sheet).map_err(core_error_to_js)?;
        Ok(Self { inner })
    }

    /// Initializes the wrapper and returns the selected absolute object pose.
    pub fn init(&mut self, formation: JsValue, po_initial: JsValue) -> Result<JsValue, JsValue> {
        let formation = formation_from_js(formation)?;
        let po_initial = point3_from_js(po_initial)?;
        let point = self
            .inner
            .init(formation, po_initial)
            .map_err(core_error_to_js)?;
        to_js_value(&Point3Output::from(point))
    }

    /// Updates the wrapper from a new formation and returns the selected pose.
    #[wasm_bindgen(js_name = getNewStableSolution)]
    pub fn get_new_stable_solution(&mut self, formation: JsValue) -> Result<JsValue, JsValue> {
        let formation = formation_from_js(formation)?;
        let point = self
            .inner
            .get_new_stable_solution(formation)
            .map_err(core_error_to_js)?;
        to_js_value(&Point3Output::from(point))
    }

    /// Returns the local-frame origin in absolute coordinates.
    #[wasm_bindgen(js_name = globalPosition)]
    pub fn global_position(&self) -> Result<JsValue, JsValue> {
        to_js_value(&Point2Output::from(self.inner.global_position()))
    }

    /// Returns whether the wrapper has been initialized with a formation.
    #[wasm_bindgen(js_name = hasFormation)]
    pub fn has_formation(&self) -> bool {
        self.inner.formation().is_some()
    }

    /// Returns the current formation in the centroid-relative local frame.
    pub fn formation(&self) -> Result<JsValue, JsValue> {
        match self.inner.formation() {
            Some(formation) => to_js_value(&formation_output(formation)),
            None => Ok(JsValue::NULL),
        }
    }

    /// Returns the selected object position in the local frame.
    #[wasm_bindgen(js_name = objectPosition)]
    pub fn object_position(&self) -> Result<JsValue, JsValue> {
        optional_point3_to_js(self.inner.object_position())
    }

    /// Returns the selected object position in absolute coordinates.
    #[wasm_bindgen(js_name = absoluteObjectPosition)]
    pub fn absolute_object_position(&self) -> Result<JsValue, JsValue> {
        optional_point3_to_js(self.inner.absolute_object_position())
    }

    /// Returns the taut cable indices for the selected branch.
    #[wasm_bindgen(js_name = tautCables)]
    pub fn taut_cables(&self) -> Result<JsValue, JsValue> {
        to_js_value(&self.inner.taut_cables().to_vec())
    }

    /// Returns the selected solution index, or `null` when none is selected.
    #[wasm_bindgen(js_name = solutionIndex)]
    pub fn solution_index(&self) -> JsValue {
        optional_index_to_js(self.inner.solution_index())
    }

    /// Returns the cached FK solutions from the underlying solver.
    pub fn solutions(&self) -> Result<JsValue, JsValue> {
        to_js_value(&solutions_output(self.inner.fk_engine().solutions()))
    }
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(untagged)]
enum Point2Input {
    Tuple([Scalar; 2]),
    Object { x: Scalar, y: Scalar },
}

impl From<Point2Input> for Point2 {
    fn from(value: Point2Input) -> Self {
        match value {
            Point2Input::Tuple([x, y]) | Point2Input::Object { x, y } => Point2::new(x, y),
        }
    }
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(untagged)]
enum Point3Input {
    Tuple([Scalar; 3]),
    Object { x: Scalar, y: Scalar, z: Scalar },
}

impl From<Point3Input> for Point3 {
    fn from(value: Point3Input) -> Self {
        match value {
            Point3Input::Tuple([x, y, z]) | Point3Input::Object { x, y, z } => Point3::new(x, y, z),
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
struct Point2Output {
    x: Scalar,
    y: Scalar,
}

impl From<Point2> for Point2Output {
    fn from(value: Point2) -> Self {
        Self {
            x: value.x,
            y: value.y,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
struct Point3Output {
    x: Scalar,
    y: Scalar,
    z: Scalar,
}

impl From<Point3> for Point3Output {
    fn from(value: Point3) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
struct FkSolutionOutput {
    stable: bool,
    po: Point3Output,
    vo: Point2Output,
    taut_cables: Vec<usize>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
struct FkSolutionsOutput {
    solutions: Vec<FkSolutionOutput>,
    all_count: usize,
    stable_count: usize,
}

fn point2_list_from_js(value: JsValue, context: &'static str) -> Result<Vec<Point2>, JsValue> {
    let inputs: Vec<Point2Input> = serde_wasm_bindgen::from_value(value).map_err(|error| {
        invalid_argument_error(format!(
            "invalid {context}: expected an array of [x, y] tuples or {{ x, y }} objects ({error})"
        ))
    })?;

    Ok(inputs.into_iter().map(Point2::from).collect())
}

fn point3_from_js(value: JsValue) -> Result<Point3, JsValue> {
    let input: Point3Input = serde_wasm_bindgen::from_value(value).map_err(|error| {
        invalid_argument_error(format!(
            "invalid point3: expected [x, y, z] or {{ x, y, z }} ({error})"
        ))
    })?;

    Ok(Point3::from(input))
}

fn formation_from_js(value: JsValue) -> Result<RobotFormation, JsValue> {
    RobotFormation::new(point2_list_from_js(value, "robot formation")?).map_err(core_error_to_js)
}

fn sheet_from_js(value: JsValue) -> Result<SheetShape, JsValue> {
    SheetShape::new(point2_list_from_js(value, "sheet shape")?).map_err(core_error_to_js)
}

fn formation_output(formation: &RobotFormation) -> Vec<Point2Output> {
    formation
        .points()
        .iter()
        .copied()
        .map(Point2Output::from)
        .collect()
}

fn solution_output(solution: &FkSolution) -> FkSolutionOutput {
    FkSolutionOutput {
        stable: solution.stable,
        po: Point3Output::from(solution.po),
        vo: Point2Output::from(solution.vo),
        taut_cables: solution.taut_cables.clone(),
    }
}

fn solutions_output(solutions: &FkSolutions) -> FkSolutionsOutput {
    FkSolutionsOutput {
        solutions: solutions.iter().map(solution_output).collect(),
        all_count: solutions.all_count(),
        stable_count: solutions.stable_count(),
    }
}

fn stable_solutions_output(solutions: &FkSolutions) -> Vec<FkSolutionOutput> {
    solutions.stable().map(solution_output).collect()
}

fn optional_index_to_js(value: Option<usize>) -> JsValue {
    value.map_or(JsValue::NULL, |index| JsValue::from_f64(index as f64))
}

fn optional_point3_to_js(value: Option<Point3>) -> Result<JsValue, JsValue> {
    match value {
        Some(point) => to_js_value(&Point3Output::from(point)),
        None => Ok(JsValue::NULL),
    }
}

fn to_js_value(value: &impl Serialize) -> Result<JsValue, JsValue> {
    serde_wasm_bindgen::to_value(value).map_err(|error| {
        invalid_argument_error(format!("failed to serialize VVCM WASM output: {error}"))
    })
}

fn core_error_to_js(error: CoreError) -> JsValue {
    match error {
        CoreError::DimensionMismatch {
            context,
            expected,
            actual,
        } => {
            let value = vvcm_error("DIMENSION_MISMATCH", error.to_string());
            set_property(&value, "context", JsValue::from_str(context));
            set_property(&value, "expected", JsValue::from_f64(expected as f64));
            set_property(&value, "actual", JsValue::from_f64(actual as f64));
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
    set_property(&value, "name", JsValue::from_str("VvcmError"));
    set_property(&value, "code", JsValue::from_str(code));
    value
}

fn set_property(target: &JsValue, key: &str, value: JsValue) {
    let _ = Reflect::set(target, &JsValue::from_str(key), &value);
}
