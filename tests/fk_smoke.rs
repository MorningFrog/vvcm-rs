use vvcm_rs::{
    FkSolution, FkSolutions, Point2, Point3, RobotFormation, Scalar, SheetShape, VvcmError, VvcmFk,
};

#[test]
fn readme_sample_matches_expected_solutions() {
    // Reuse the exact README sample so the docs and regression test stay aligned.
    let formation = readme_formation();
    let sheet = readme_sheet();
    // Build the solver with four robots and a 1000 mm hold height.
    let mut fk = VvcmFk::new(4, 1000.0, sheet).unwrap();

    assert_eq!(fk.robot_count(), 4);
    assert_eq!(fk.hold_height(), 1000.0);

    // One solve should populate both the complete and stable solution sets.
    let solutions = fk.update_stable_solutions(formation).unwrap();

    assert_eq!(solutions.all_count(), 3);
    assert_eq!(solutions.stable_count(), 2);

    // Check the stable branches in the order returned by the solver.
    let stable: Vec<_> = solutions.stable().collect();
    assert_point3_close(
        stable[0].po,
        Point3::new(568.8123, 324.72644, 336.73608),
        0.05,
    );
    assert_point2_close(stable[0].vo, Point2::new(238.6181, 125.02439), 0.05);
    assert_eq!(stable[0].taut_cables, vec![0, 1, 2]);

    assert_point3_close(
        stable[1].po,
        Point3::new(557.9307, 341.23087, 337.2464),
        0.05,
    );
    assert_point2_close(stable[1].vo, Point2::new(208.79898, 152.53357), 0.05);
    assert_eq!(stable[1].taut_cables, vec![0, 2, 3]);

    // The full set should still contain at least one unstable branch.
    assert!(solutions.iter().any(|solution| !solution.stable));
}

#[test]
fn fk_solutions_track_stability_per_solution() {
    // FkSolutions should count stable branches independently from all branches.
    let solutions = FkSolutions::new(vec![
        FkSolution::new(
            false,
            Point3::new(1.0, 2.0, 3.0),
            Point2::new(4.0, 5.0),
            vec![0, 1, 2],
        ),
        FkSolution::new(
            true,
            Point3::new(6.0, 7.0, 8.0),
            Point2::new(9.0, 10.0),
            vec![1, 2, 3],
        ),
    ]);

    assert_eq!(solutions.all_count(), 2);
    assert_eq!(solutions.stable_count(), 1);
    assert_eq!(
        solutions.stable().next().unwrap().po,
        Point3::new(6.0, 7.0, 8.0)
    );
}

#[test]
#[allow(clippy::excessive_precision)]
fn six_robot_local_sample_matches_expected_solution() {
    // Start from robot endpoints in the world-frame XY plane, then shift them to a local origin.
    let absolute_formation = RobotFormation::new(vec![
        Point2::new(-27.419184, -176.293854),
        Point2::new(398.141083, -35.190411),
        Point2::new(517.018127, 338.271301),
        Point2::new(285.155762, 609.95575),
        Point2::new(-175.608231, 569.463562),
        Point2::new(-301.437988, 194.695297),
    ])
    .unwrap();
    let origin = absolute_formation.points()[0];
    let local_formation = absolute_formation.relative_to(origin);
    // Use the matching six-robot sheet fixture in the sheet-local XY frame.
    let sheet = SheetShape::new(vec![
        Point2::new(-131.665741, -376.508026),
        Point2::new(480.675873, -388.066681),
        Point2::new(877.700256, 217.088806),
        Point2::new(562.778748, 826.754089),
        Point2::new(-107.442101, 918.166626),
        Point2::new(-453.516937, 284.887146),
    ])
    .unwrap();
    // Solve the local-frame formation and keep the stable branch nearest the reference pose.
    let mut fk = VvcmFk::new(6, 823.0, sheet).unwrap();

    let solutions = fk.update_stable_solutions(local_formation).unwrap();
    let expected = Point3::new(137.674, 420.879, 301.218);
    let closest = solutions
        .stable()
        .min_by(|left, right| {
            left.po
                .distance_to(expected)
                .total_cmp(&right.po.distance_to(expected))
        })
        .unwrap();

    assert_point3_close(closest.po, expected, 0.15);
}

#[test]
fn infeasible_formation_is_reported() {
    // A stretched formation outside the sheet should be rejected as infeasible.
    let sheet = SheetShape::new(vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(0.0, 1.0),
    ])
    .unwrap();
    let formation = RobotFormation::new(vec![
        Point2::new(0.0, 0.0),
        Point2::new(2.0, 0.0),
        Point2::new(2.0, 2.0),
        Point2::new(0.0, 2.0),
    ])
    .unwrap();
    let mut fk = VvcmFk::new(4, 10.0, sheet).unwrap();

    let error = fk.update_stable_solutions(formation).unwrap_err();

    assert_eq!(error, VvcmError::InfeasibleFormation);
    assert!(fk.solutions().is_empty());
}

#[test]
fn formation_dimension_mismatch_is_reported() {
    // The robot count should be validated before the solver attempts any geometry work.
    let mut fk = VvcmFk::new(4, 1000.0, readme_sheet()).unwrap();
    let formation = RobotFormation::new(vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(0.0, 1.0),
    ])
    .unwrap();

    let error = fk.update_stable_solutions(formation).unwrap_err();

    assert_eq!(
        error,
        VvcmError::DimensionMismatch {
            context: "robot formation point count",
            expected: 4,
            actual: 3,
        }
    );
}

fn readme_formation() -> RobotFormation {
    // Keep this fixture identical to the README usage snippet; the robot endpoints live on the world-frame XY plane.
    RobotFormation::new(vec![
        Point2::new(213.7, 122.7),
        Point2::new(804.6, 37.2),
        Point2::new(904.0, 550.0),
        Point2::new(439.3, 715.9),
    ])
    .unwrap()
}

fn readme_sheet() -> SheetShape {
    // Keep this fixture identical to the README usage snippet; the sheet vertices live in the sheet-local XY frame.
    SheetShape::new(vec![
        Point2::new(-316.1, -421.9),
        Point2::new(803.4, -384.1),
        Point2::new(746.1, 712.8),
        Point2::new(-367.3, 664.2),
    ])
    .unwrap()
}

fn assert_point2_close(actual: Point2, expected: Point2, tolerance: Scalar) {
    // Compare floating-point coordinates with a tolerance instead of exact equality.
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
    // Reuse the 2D check for x/y, then compare z separately for clearer failures.
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
