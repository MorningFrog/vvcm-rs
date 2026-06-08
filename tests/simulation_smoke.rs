#![allow(clippy::excessive_precision)]

use vvcm_rs::{
    Point2, Point3, RobotFormation, Scalar, SheetShape, VvcmManualSimulation, VvcmSimulation,
};

#[test]
fn manual_simulation_returns_expected_branch() {
    let mut simulation = VvcmManualSimulation::new(6, 823.0, six_robot_sheet()).unwrap();

    let po = simulation
        .init(six_robot_formation(), Point3::zero())
        .unwrap();

    assert_point3_close(po, Point3::new(110.255, 244.585, 301.218), 0.2);
    assert_point3_close(
        simulation.absolute_object_position().unwrap(),
        Point3::new(110.255, 244.585, 301.218),
        0.2,
    );
    assert!(simulation.solution_index().is_some());
    assert!(!simulation.taut_cables().is_empty());

    let po = simulation
        .get_new_stable_solution(six_robot_formation())
        .unwrap();

    assert_point3_close(po, Point3::new(110.255, 244.585, 301.218), 0.2);
}

#[test]
fn velocity_simulation_initializes_and_steps_consistently() {
    let mut simulation = VvcmSimulation::new(
        6,
        823.0,
        six_robot_sheet(),
        six_robot_formation(),
        Point3::zero(),
        1.0 / 30.0,
    )
    .unwrap();

    assert_point2_close(
        simulation.global_position(),
        Point2::new(-27.419184, -176.293854),
        0.001,
    );
    assert_point2_close(simulation.formation().points()[0], Point2::zero(), 0.001);
    assert_point3_close(
        simulation.object_position(),
        Point3::new(137.674, 420.879, 301.218),
        0.2,
    );
    assert_point3_close(
        simulation.absolute_object_position(),
        Point3::new(110.255, 244.585, 301.218),
        0.2,
    );
    assert!(simulation.solution_index().is_some());

    let before_zero_step = simulation.object_position();
    simulation.step().unwrap();
    assert_eq!(simulation.object_position(), before_zero_step);

    simulation
        .set_velocity(
            RobotFormation::new(vec![
                Point2::new(5.0, 5.0),
                Point2::zero(),
                Point2::zero(),
                Point2::zero(),
                Point2::zero(),
                Point2::zero(),
            ])
            .unwrap(),
        )
        .unwrap();
    simulation.step().unwrap();

    assert_point2_close(
        simulation.global_position(),
        Point2::new(-27.252517, -176.12718),
        0.01,
    );
    assert_point2_close(simulation.formation().points()[0], Point2::zero(), 0.001);
    assert_point2_close(
        simulation.formation().points()[1],
        Point2::new(425.394, 140.937),
        0.02,
    );
    assert_point3_close(
        simulation.object_position(),
        Point3::new(137.54, 420.572, 301.209),
        0.25,
    );
}

fn six_robot_formation() -> RobotFormation {
    RobotFormation::new(vec![
        Point2::new(-27.419184, -176.293854),
        Point2::new(398.141083, -35.190411),
        Point2::new(517.018127, 338.271301),
        Point2::new(285.155762, 609.95575),
        Point2::new(-175.608231, 569.463562),
        Point2::new(-301.437988, 194.695297),
    ])
    .unwrap()
}

fn six_robot_sheet() -> SheetShape {
    SheetShape::new(vec![
        Point2::new(-131.665741, -376.508026),
        Point2::new(480.675873, -388.066681),
        Point2::new(877.700256, 217.088806),
        Point2::new(562.778748, 826.754089),
        Point2::new(-107.442101, 918.166626),
        Point2::new(-453.516937, 284.887146),
    ])
    .unwrap()
}

fn assert_point2_close(actual: Point2, expected: Point2, tolerance: Scalar) {
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

fn assert_point3_close(actual: Point3, expected: Point3, tolerance: Scalar) {
    assert_point2_close(
        Point2::new(actual.x, actual.y),
        Point2::new(expected.x, expected.y),
        tolerance,
    );
    assert!(
        (actual.z - expected.z).abs() <= tolerance,
        "z differs: actual {}, expected {}",
        actual.z,
        expected.z
    );
}
