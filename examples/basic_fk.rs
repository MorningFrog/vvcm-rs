use vvcm_rs::{Point2, RobotFormation, SheetShape, VvcmFk};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Robot endpoints on the world-frame XY plane, using the millimeter scale of the sample data.
    let formation = RobotFormation::new(vec![
        Point2::new(213.7, 122.7),
        Point2::new(804.6, 37.2),
        Point2::new(904.0, 550.0),
        Point2::new(439.3, 715.9),
    ])?;

    // Sheet vertices in the sheet-local XY frame, using the same millimeter scale.
    let sheet = SheetShape::new(vec![
        Point2::new(-316.1, -421.9),
        Point2::new(803.4, -384.1),
        Point2::new(746.1, 712.8),
        Point2::new(-367.3, 664.2),
    ])?;

    // Create the solver for four robots and a 1000 mm hold height.
    let mut fk = VvcmFk::new(4, 1000.0, sheet)?;

    // Enumerate all candidate equilibria for this robot formation.
    let solutions = fk.update_stable_solutions(formation)?;

    // Report the total branch count and the subset that is stable.
    println!("all solutions: {}", solutions.all_count());
    println!("stable solutions: {}", solutions.stable_count());

    // Print each stable branch with object pose, planar velocity, and taut cables.
    for (index, solution) in solutions.stable().enumerate() {
        println!(
            "#{index}: Po=({:.3}, {:.3}, {:.3}), Vo=({:.3}, {:.3}), taut={:?}",
            solution.po.x,
            solution.po.y,
            solution.po.z,
            solution.vo.x,
            solution.vo.y,
            solution.taut_cables,
        );
    }

    Ok(())
}
