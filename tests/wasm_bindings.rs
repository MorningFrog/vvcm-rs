#![cfg(all(target_arch = "wasm32", feature = "wasm"))]

use serde::{Deserialize, Serialize};
use vvcm_rs::wasm::{VvcmFk, VvcmManualSimulation, VvcmSimulation};
use wasm_bindgen::JsValue;
use wasm_bindgen_test::*;

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct FkSolutionsOutput {
    solutions: Vec<FkSolutionOutput>,
    all_count: usize,
    stable_count: usize,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct FkSolutionOutput {
    stable: bool,
    po: Point3Output,
    vo: Point2Output,
    taut_cables: Vec<usize>,
}

#[derive(Debug, Deserialize)]
struct Point2Output {
    x: f32,
    y: f32,
}

#[derive(Debug, Deserialize)]
struct Point3Output {
    x: f32,
    y: f32,
    z: f32,
}

#[wasm_bindgen_test]
fn fk_readme_sample_matches_expected_solutions() {
    let mut fk = VvcmFk::new(4, 1000.0, js(&readme_sheet())).unwrap();

    let output: FkSolutionsOutput =
        from_js(fk.update_stable_solutions(js(&readme_formation())).unwrap());

    assert_eq!(output.all_count, 3);
    assert_eq!(output.stable_count, 2);

    let stable: Vec<_> = output
        .solutions
        .iter()
        .filter(|solution| solution.stable)
        .collect();

    assert_point3_close(
        &stable[0].po,
        &Point3Output {
            x: 568.8123,
            y: 324.72644,
            z: 336.73608,
        },
        0.05,
    );
    assert_point2_close(
        &stable[0].vo,
        &Point2Output {
            x: 238.6181,
            y: 125.02439,
        },
        0.05,
    );
    assert_eq!(stable[0].taut_cables, vec![0, 1, 2]);

    let stable_value: Vec<FkSolutionOutput> = from_js(fk.stable_solutions().unwrap());
    assert_eq!(stable_value.len(), 2);
}

#[wasm_bindgen_test]
fn fk_errors_have_stable_codes() {
    let error = match VvcmFk::new(4, 1000.0, js(&[[0.0_f32, 0.0]])) {
        Ok(_) => panic!("expected dimension mismatch for one-vertex sheet"),
        Err(error) => error,
    };
    assert_eq!(error_code(&error), "DIMENSION_MISMATCH");

    let mut fk = VvcmFk::new(
        4,
        10.0,
        js(&[[0.0_f32, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]),
    )
    .unwrap();

    let error = fk
        .update_stable_solutions(js(&[[0.0_f32, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]))
        .unwrap_err();
    assert_eq!(error_code(&error), "INFEASIBLE_FORMATION");
}

#[wasm_bindgen_test]
fn manual_simulation_returns_expected_branch() {
    let mut simulation = VvcmManualSimulation::new(6, 823.0, js(&six_robot_sheet())).unwrap();

    let po: Point3Output = from_js(
        simulation
            .init(js(&six_robot_formation()), js(&[0.0_f32, 0.0, 0.0]))
            .unwrap(),
    );

    assert_point3_close(
        &po,
        &Point3Output {
            x: 110.255,
            y: 244.585,
            z: 301.218,
        },
        0.2,
    );
    assert!(simulation.has_formation());
}

#[wasm_bindgen_test]
fn velocity_simulation_initializes_and_steps_consistently() {
    let mut simulation = VvcmSimulation::new(
        6,
        823.0,
        js(&six_robot_sheet()),
        js(&six_robot_formation()),
        js(&[0.0_f32, 0.0, 0.0]),
        1.0 / 30.0,
    )
    .unwrap();

    let object_position: Point3Output = from_js(simulation.object_position().unwrap());
    assert_point3_close(
        &object_position,
        &Point3Output {
            x: 137.674,
            y: 420.879,
            z: 301.218,
        },
        0.2,
    );

    simulation
        .set_velocity(js(&[
            [5.0_f32, 5.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
        ]))
        .unwrap();
    simulation.step().unwrap();

    let global_position: Point2Output = from_js(simulation.global_position().unwrap());
    assert_point2_close(
        &global_position,
        &Point2Output {
            x: -27.252517,
            y: -176.12718,
        },
        0.01,
    );
}

fn readme_formation() -> [[f32; 2]; 4] {
    [
        [213.7, 122.7],
        [804.6, 37.2],
        [904.0, 550.0],
        [439.3, 715.9],
    ]
}

fn readme_sheet() -> [[f32; 2]; 4] {
    [
        [-316.1, -421.9],
        [803.4, -384.1],
        [746.1, 712.8],
        [-367.3, 664.2],
    ]
}

fn six_robot_formation() -> [[f32; 2]; 6] {
    [
        [-27.419184, -176.293854],
        [398.141083, -35.190411],
        [517.018127, 338.271301],
        [285.155762, 609.95575],
        [-175.608231, 569.463562],
        [-301.437988, 194.695297],
    ]
}

fn six_robot_sheet() -> [[f32; 2]; 6] {
    [
        [-131.665741, -376.508026],
        [480.675873, -388.066681],
        [877.700256, 217.088806],
        [562.778748, 826.754089],
        [-107.442101, 918.166626],
        [-453.516937, 284.887146],
    ]
}

fn js(value: &impl Serialize) -> JsValue {
    serde_wasm_bindgen::to_value(value).unwrap()
}

fn from_js<T: for<'de> Deserialize<'de>>(value: JsValue) -> T {
    serde_wasm_bindgen::from_value(value).unwrap()
}

fn error_code(error: &JsValue) -> String {
    js_sys::Reflect::get(error, &JsValue::from_str("code"))
        .unwrap()
        .as_string()
        .unwrap()
}

fn assert_point2_close(actual: &Point2Output, expected: &Point2Output, tolerance: f32) {
    assert!(
        (actual.x - expected.x).abs() <= tolerance,
        "x differs: actual {}, expected {}",
        actual.x,
        expected.x
    );
    assert!(
        (actual.y - expected.y).abs() <= tolerance,
        "y differs: actual {}, expected {}",
        actual.y,
        expected.y
    );
}

fn assert_point3_close(actual: &Point3Output, expected: &Point3Output, tolerance: f32) {
    assert_point2_close(
        &Point2Output {
            x: actual.x,
            y: actual.y,
        },
        &Point2Output {
            x: expected.x,
            y: expected.y,
        },
        tolerance,
    );
    assert!(
        (actual.z - expected.z).abs() <= tolerance,
        "z differs: actual {}, expected {}",
        actual.z,
        expected.z
    );
}
