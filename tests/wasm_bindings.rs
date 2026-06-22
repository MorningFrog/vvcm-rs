#![cfg(all(target_arch = "wasm32", feature = "wasm"))]
#![allow(clippy::excessive_precision)]

use js_sys::{Array, Float32Array, Reflect};
use vvcm_rs::wasm::{VvcmFk, VvcmManualSimulation, VvcmSimulation};
use wasm_bindgen::JsCast;
use wasm_bindgen::JsValue;
use wasm_bindgen_test::*;

#[wasm_bindgen_test]
fn fk_readme_sample_matches_expected_solutions() {
    let mut fk = VvcmFk::new(1000.0, f32_array(&readme_sheet())).unwrap();

    let output = fk
        .update_stable_solutions(f32_array(&readme_formation()))
        .unwrap();

    assert_eq!(number_property(&output, "allCount"), 3);
    assert_eq!(number_property(&output, "stableCount"), 2);

    let solutions = array_property(&output, "solutions");
    assert_eq!(solutions.length(), 3);

    let first = solutions.get(0);
    let second = solutions.get(1);
    let third = solutions.get(2);
    assert!(bool_property(&first, "stable"));
    assert!(bool_property(&second, "stable"));
    assert!(!bool_property(&third, "stable"));

    assert_point3_close(
        &point3_property(&first, "po"),
        &[568.8123, 324.72644, 336.73608],
        0.05,
    );
    assert_point2_close(&point2_property(&first, "vo"), &[238.6181, 125.02439], 0.05);

    let taut_cables = number_array_property(&first, "tautCables");
    let lambda_values = number_array_property(&first, "lambdaValues");
    assert_eq!(taut_cables, vec![0.0, 1.0, 2.0]);
    assert_eq!(lambda_values.len(), taut_cables.len());
    assert!(
        lambda_values
            .iter()
            .all(|value| value.is_finite() && *value >= -1.0e-4)
    );
}

#[wasm_bindgen_test]
fn fk_errors_have_stable_codes() {
    let error = match VvcmFk::new(1000.0, f32_array(&[0.0_f32, 0.0])) {
        Ok(_) => panic!("expected dimension mismatch for one-vertex sheet"),
        Err(error) => error,
    };
    assert_eq!(error_code(&error), "DIMENSION_MISMATCH");

    let mut fk = VvcmFk::new(
        10.0,
        f32_array(&[0.0_f32, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0]),
    )
    .unwrap();

    let error = fk
        .update_stable_solutions(f32_array(&[0.0_f32, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0]))
        .unwrap_err();
    assert_eq!(error_code(&error), "INFEASIBLE_FORMATION");
}

#[wasm_bindgen_test]
fn manual_simulation_returns_expected_branch() {
    let mut simulation = VvcmManualSimulation::new(823.0, f32_array(&six_robot_sheet())).unwrap();

    let po = simulation
        .init(
            f32_array(&six_robot_formation()),
            f32_array(&[0.0_f32, 0.0, 0.0]),
        )
        .unwrap()
        .to_vec();

    assert_point3_close(&po, &[110.255, 244.585, 301.218], 0.2);
    assert!(simulation.has_formation());
}

#[wasm_bindgen_test]
fn velocity_simulation_initializes_and_steps_consistently() {
    let mut simulation = VvcmSimulation::new(
        823.0,
        f32_array(&six_robot_sheet()),
        f32_array(&six_robot_formation()),
        f32_array(&[0.0_f32, 0.0, 0.0]),
        1.0 / 30.0,
    )
    .unwrap();

    let object_position = simulation.object_position().to_vec();
    assert_point3_close(&object_position, &[137.674, 420.879, 301.218], 0.2);

    simulation
        .set_velocity(f32_array(&[
            5.0_f32, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]))
        .unwrap();
    simulation.step().unwrap();

    let global_position = simulation.global_position().to_vec();
    assert_point2_close(&global_position, &[-27.252517, -176.12718], 0.01);
}

fn readme_formation() -> [f32; 8] {
    [213.7, 122.7, 804.6, 37.2, 904.0, 550.0, 439.3, 715.9]
}

fn readme_sheet() -> [f32; 8] {
    [-316.1, -421.9, 803.4, -384.1, 746.1, 712.8, -367.3, 664.2]
}

fn six_robot_formation() -> [f32; 12] {
    [
        -27.419184,
        -176.293854,
        398.141083,
        -35.190411,
        517.018127,
        338.271301,
        285.155762,
        609.95575,
        -175.608231,
        569.463562,
        -301.437988,
        194.695297,
    ]
}

fn six_robot_sheet() -> [f32; 12] {
    [
        -131.665741,
        -376.508026,
        480.675873,
        -388.066681,
        877.700256,
        217.088806,
        562.778748,
        826.754089,
        -107.442101,
        918.166626,
        -453.516937,
        284.887146,
    ]
}

fn f32_array(values: &[f32]) -> Float32Array {
    Float32Array::from(values)
}

fn array_property(value: &JsValue, name: &str) -> Array {
    Reflect::get(value, &JsValue::from_str(name))
        .unwrap()
        .unchecked_into()
}

fn bool_property(value: &JsValue, name: &str) -> bool {
    Reflect::get(value, &JsValue::from_str(name))
        .unwrap()
        .as_bool()
        .unwrap()
}

fn point2_property(value: &JsValue, name: &str) -> [f32; 2] {
    let point = Reflect::get(value, &JsValue::from_str(name)).unwrap();
    [
        numeric_property(&point, "x") as f32,
        numeric_property(&point, "y") as f32,
    ]
}

fn point3_property(value: &JsValue, name: &str) -> [f32; 3] {
    let point = Reflect::get(value, &JsValue::from_str(name)).unwrap();
    [
        numeric_property(&point, "x") as f32,
        numeric_property(&point, "y") as f32,
        numeric_property(&point, "z") as f32,
    ]
}

fn number_array_property(value: &JsValue, name: &str) -> Vec<f64> {
    let array = array_property(value, name);
    (0..array.length())
        .map(|index| array.get(index).as_f64().unwrap())
        .collect()
}

fn number_property(value: &JsValue, name: &str) -> u32 {
    numeric_property(value, name) as u32
}

fn numeric_property(value: &JsValue, name: &str) -> f64 {
    Reflect::get(value, &JsValue::from_str(name))
        .unwrap()
        .as_f64()
        .unwrap()
}

fn error_code(error: &JsValue) -> String {
    Reflect::get(error, &JsValue::from_str("code"))
        .unwrap()
        .as_string()
        .unwrap()
}

fn assert_point2_close(actual: &[f32], expected: &[f32], tolerance: f32) {
    assert!(
        (actual[0] - expected[0]).abs() <= tolerance,
        "x differs: actual {}, expected {}",
        actual[0],
        expected[0]
    );
    assert!(
        (actual[1] - expected[1]).abs() <= tolerance,
        "y differs: actual {}, expected {}",
        actual[1],
        expected[1]
    );
}

fn assert_point3_close(actual: &[f32], expected: &[f32], tolerance: f32) {
    assert_point2_close(actual, expected, tolerance);
    assert!(
        (actual[2] - expected[2]).abs() <= tolerance,
        "z differs: actual {}, expected {}",
        actual[2],
        expected[2]
    );
}
