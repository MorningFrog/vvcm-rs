use vvcm_rs::{Point2, RobotFormation, SheetShape, VvcmError, VvcmFk};

#[test]
fn fk_scaffold_can_be_constructed() {
    let sheet = SheetShape::new(vec![
        Point2::new(-316.1, -421.9),
        Point2::new(803.4, -384.1),
        Point2::new(746.1, 712.8),
        Point2::new(-367.3, 664.2),
    ])
    .unwrap();

    let formation = RobotFormation::new(vec![
        Point2::new(213.7, 122.7),
        Point2::new(804.6, 37.2),
        Point2::new(904.0, 550.0),
        Point2::new(439.3, 715.9),
    ])
    .unwrap();

    let mut fk = VvcmFk::new(4, 1000.0, sheet).unwrap();

    let error = fk.update_stable_solutions(formation).unwrap_err();

    assert_eq!(error, VvcmError::NotImplemented);
    assert_eq!(fk.robot_count(), 4);
    assert_eq!(fk.hold_height(), 1000.0);
    assert_eq!(fk.solutions().stable_count(), 0);
    assert_eq!(fk.solutions().all_count(), 0);
}
