//! Velocity-driven simulation wrapper around the VVCM FK solver.
//!
//! [`VvcmSimulation`] integrates robot XY velocities over a fixed time step and
//! asks [`VvcmFk`] for the closest stable branch after each step. Internally it
//! stores robot and object positions in a local frame whose origin is the first
//! robot's initial position; convenience accessors return absolute coordinates
//! when needed.

use crate::error::VvcmError;
use crate::fk::VvcmFk;
use crate::types::{Point2, Point3, RobotFormation, Scalar, SheetShape};

/// Fixed-step robot-velocity simulation built on [`VvcmFk`].
///
/// The simulation keeps the closest stable FK branch to the previously selected
/// object position, which provides continuity when multiple stable branches are
/// available.
#[derive(Debug, Clone)]
pub struct VvcmSimulation {
    fk_engine: VvcmFk,
    global_position: Point2,
    formation: RobotFormation,
    object_position: Point3,
    taut_cables: Vec<usize>,
    solution_index: Option<usize>,
    dt: Scalar,
    velocity: RobotFormation,
}

impl VvcmSimulation {
    /// Creates a simulation from an absolute initial formation and object
    /// position.
    ///
    /// The first robot becomes the local-frame origin. `po_initial` is used
    /// only as a reference for selecting the initial stable branch; the stored
    /// object position is replaced by the closest stable FK result.
    ///
    /// # Errors
    ///
    /// Returns any construction or solving error reported by [`VvcmFk`], or a
    /// [`VvcmError::DimensionMismatch`] if `initial_formation` does not contain
    /// `robot_count` points.
    pub fn new(
        robot_count: usize,
        hold_height: Scalar,
        sheet: SheetShape,
        initial_formation: RobotFormation,
        po_initial: Point3,
        dt: Scalar,
    ) -> Result<Self, VvcmError> {
        let mut fk_engine = VvcmFk::new(robot_count, hold_height, sheet)?;
        fk_engine.validate_formation(&initial_formation)?;

        let global_position = initial_formation.points()[0];
        let formation = initial_formation.relative_to(global_position);
        let reference = po_initial.relative_xy_to(global_position);
        let velocity = RobotFormation::zeros(robot_count)?;
        let (solution_index, object_position, taut_cables) =
            solve_closest_stable(&mut fk_engine, formation.clone(), reference)?;

        Ok(Self {
            fk_engine,
            global_position,
            formation,
            object_position,
            taut_cables,
            solution_index: Some(solution_index),
            dt,
            velocity,
        })
    }

    /// Sets one XY velocity vector per robot.
    ///
    /// Velocities use the same point ordering as the formation and are
    /// integrated by [`VvcmSimulation::step`] using the fixed `dt` value.
    ///
    /// # Errors
    ///
    /// Returns [`VvcmError::DimensionMismatch`] if the velocity formation size
    /// does not match the simulation robot count.
    pub fn set_velocity(&mut self, velocity: RobotFormation) -> Result<(), VvcmError> {
        self.fk_engine.validate_formation(&velocity)?;
        self.velocity = velocity;
        Ok(())
    }

    /// Advances the simulation by one fixed time step.
    ///
    /// When all velocities are exactly zero, this method is a no-op and does
    /// not rerun the FK solver. Otherwise it updates the local formation,
    /// selects the closest stable branch to the previous object position, and
    /// refreshes the stored taut cable set.
    ///
    /// # Errors
    ///
    /// Returns any solving error reported by [`VvcmFk`] for the updated local
    /// formation.
    pub fn step(&mut self) -> Result<(), VvcmError> {
        if self.velocity.all_zero() {
            return Ok(());
        }

        // The first robot's motion defines global translation. The local
        // formation is updated by each robot's own displacement and then
        // re-centered by this global displacement, keeping robot 0 at the local
        // origin.
        let delta_global = self.velocity.points()[0].scaled_by(self.dt);
        self.global_position = self.global_position.translated_by(delta_global);

        let points = self
            .formation
            .points()
            .iter()
            .zip(self.velocity.points())
            .map(|(point, velocity)| {
                point
                    .translated_by(velocity.scaled_by(self.dt))
                    .relative_to(delta_global)
            })
            .collect();
        self.formation = RobotFormation::new(points)?;

        let (solution_index, object_position, taut_cables) = solve_closest_stable(
            &mut self.fk_engine,
            self.formation.clone(),
            self.object_position,
        )?;
        self.solution_index = Some(solution_index);
        self.object_position = object_position;
        self.taut_cables = taut_cables;

        Ok(())
    }

    /// Returns the current robot formation in absolute coordinates.
    pub fn absolute_formation(&self) -> RobotFormation {
        self.formation.translated_by(self.global_position)
    }

    /// Returns the selected object position in absolute coordinates.
    pub fn absolute_object_position(&self) -> Point3 {
        self.object_position.translated_xy_by(self.global_position)
    }

    /// Borrows the underlying FK engine and its latest solution cache.
    pub fn fk_engine(&self) -> &VvcmFk {
        &self.fk_engine
    }

    /// Returns the local-frame origin in absolute coordinates.
    pub fn global_position(&self) -> Point2 {
        self.global_position
    }

    /// Borrows the current robot formation in the local frame.
    pub fn formation(&self) -> &RobotFormation {
        &self.formation
    }

    /// Returns the selected object position in the local frame.
    pub fn object_position(&self) -> Point3 {
        self.object_position
    }

    /// Borrows the taut cable indices for the currently selected branch.
    pub fn taut_cables(&self) -> &[usize] {
        &self.taut_cables
    }

    /// Returns the index of the selected branch in the FK solution cache.
    pub fn solution_index(&self) -> Option<usize> {
        self.solution_index
    }

    /// Returns the fixed integration time step.
    pub fn dt(&self) -> Scalar {
        self.dt
    }

    /// Borrows the current per-robot velocity formation.
    pub fn velocity(&self) -> &RobotFormation {
        &self.velocity
    }
}

/// Runs FK for `formation` and picks the stable branch closest to `reference`.
fn solve_closest_stable(
    fk_engine: &mut VvcmFk,
    formation: RobotFormation,
    reference: Point3,
) -> Result<(usize, Point3, Vec<usize>), VvcmError> {
    let solutions = fk_engine.update_stable_solutions(formation)?;
    let (solution_index, solution) = solutions
        .closest_stable_to(reference)
        .ok_or(VvcmError::NoStableSolution)?;

    Ok((solution_index, solution.po, solution.taut_cables.clone()))
}
