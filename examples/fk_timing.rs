#![allow(clippy::excessive_precision)]

use std::time::Instant;

use vvcm_rs::{Point2, RobotFormation, SheetShape, VvcmFk};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Robot formation: these 20 Point2 values are robot node positions on the world-coordinate XY plane, in millimeters.
    let formation = RobotFormation::new(vec![
        Point2::new(576.276881720430, 162.627551020408),
        Point2::new(636.088709677419, 181.760204081633),
        Point2::new(677.083333333333, 225.127551020408),
        Point2::new(715.389784946237, 288.903061224490),
        Point2::new(744.287634408602, 361.607142857143),
        Point2::new(765.120967741936, 447.066326530613),
        Point2::new(767.137096774194, 544.005102040817),
        Point2::new(748.991935483871, 646.045918367347),
        Point2::new(710.013440860215, 709.821428571429),
        Point2::new(658.938172043011, 745.535714285715),
        Point2::new(604.502688172043, 765.943877551021),
        Point2::new(546.034946236559, 767.219387755102),
        Point2::new(495.631720430108, 726.403061224490),
        Point2::new(461.357526881720, 640.943877551021),
        Point2::new(459.341397849462, 549.107142857143),
        Point2::new(465.389784946237, 461.096938775510),
        Point2::new(481.518817204301, 366.709183673470),
        Point2::new(503.024193548387, 296.556122448980),
        Point2::new(520.497311827957, 235.331632653062),
        Point2::new(542.674731182796, 188.137755102041),
    ])?;

    // Unfolded sheet: each Point2 is a vertex in the sheet's local coordinate frame, in millimeters.
    let sheet = SheetShape::new(vec![
        Point2::new(512.432795698925, 55.4846938775513),
        Point2::new(621.975806451613, 59.3112244897961),
        Point2::new(725.470430107527, 86.0969387755105),
        Point2::new(814.852150537634, 153.698979591837),
        Point2::new(878.024193548387, 250.637755102041),
        Point2::new(923.723118279570, 392.219387755102),
        Point2::new(933.803763440860, 532.525510204082),
        Point2::new(908.266129032258, 669.005102040817),
        Point2::new(853.158602150538, 796.556122448980),
        Point2::new(769.153225806452, 887.117346938776),
        Point2::new(667.674731182796, 948.341836734694),
        Point2::new(558.131720430108, 964.923469387755),
        Point2::new(445.228494623656, 943.239795918368),
        Point2::new(353.830645161290, 883.290816326531),
        Point2::new(273.185483870968, 781.250000000000),
        Point2::new(203.293010752688, 619.260204081633),
        Point2::new(185.819892473118, 464.923469387755),
        Point2::new(206.653225806452, 325.892857142857),
        Point2::new(269.153225806452, 197.066326530613),
        Point2::new(384.072580645161, 106.505102040817),
    ])?;

    println!("----------------------");

    // Build the solver and time a single stable-solution update.
    let mut fk = VvcmFk::new(20, 1000.0, sheet)?;
    let start = Instant::now();
    let solutions = fk.update_stable_solutions(formation)?;
    let elapsed = start.elapsed();

    // Print the solve time and the number of stable branches found.
    println!("{:.6}s", elapsed.as_secs_f64());
    println!("M: {}", solutions.stable_count());

    // Dump the stable object positions first.
    println!("Po:");
    for solution in solutions.stable() {
        println!(
            "{:.3} {:.3} {:.3}",
            solution.po.x, solution.po.y, solution.po.z
        );
    }

    // Then dump the matching planar velocities.
    println!("Vo:");
    for solution in solutions.stable() {
        println!("{:.3} {:.3}", solution.vo.x, solution.vo.y);
    }

    println!("----------------------");

    Ok(())
}
