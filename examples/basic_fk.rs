use vvcm_rs::{Point2, RobotFormation, SheetShape, VvcmError, VvcmFk};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let formation = RobotFormation::new(vec![
        Point2::new(213.7, 122.7),
        Point2::new(804.6, 37.2),
        Point2::new(904.0, 550.0),
        Point2::new(439.3, 715.9),
    ])?;

    let sheet = SheetShape::new(vec![
        Point2::new(-316.1, -421.9),
        Point2::new(803.4, -384.1),
        Point2::new(746.1, 712.8),
        Point2::new(-367.3, 664.2),
    ])?;

    let mut fk = VvcmFk::new(4, 1000.0, sheet)?;

    match fk.update_stable_solutions(formation) {
        Ok(solutions) => {
            println!("stable solutions: {}", solutions.stable_count());
        }
        Err(VvcmError::NotImplemented) => {
            println!("VVCM FK core is not implemented yet.");
        }
        Err(error) => return Err(error.into()),
    }

    Ok(())
}
