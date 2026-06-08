//! Manual-formation simulation wrapper around the VVCM FK solver.
//!
//! [`VvcmManualSimulation`] is useful when another system supplies complete
//! robot formations directly instead of velocities. Each supplied formation is
//! converted to a centroid-relative local frame before the closest stable FK
//! branch is selected.

use crate::error::VvcmError;
use crate::fk::VvcmFk;
use crate::types::{Point2, Point3, RobotFormation, Scalar, SheetShape};

/// Simulation helper for externally supplied robot formations.
///
/// Unlike [`crate::VvcmSimulation`], this wrapper does not integrate
/// velocities. Call [`VvcmManualSimulation::init`] once with an initial
/// formation, then call [`VvcmManualSimulation::get_new_stable_solution`] for
/// each new absolute formation.
#[derive(Debug, Clone)]
pub struct VvcmManualSimulation {
    fk_engine: VvcmFk,
    global_position: Point2,
    formation: Option<RobotFormation>,
    object_position: Option<Point3>,
    taut_cables: Vec<usize>,
    solution_index: Option<usize>,
}

impl VvcmManualSimulation {
    /// Creates a manual simulation wrapper for a fixed sheet.
    ///
    /// # Errors
    ///
    /// Returns any construction error reported by [`VvcmFk::new`].
    pub fn new(
        robot_count: usize,
        hold_height: Scalar,
        sheet: SheetShape,
    ) -> Result<Self, VvcmError> {
        Ok(Self {
            fk_engine: VvcmFk::new(robot_count, hold_height, sheet)?,
            global_position: Point2::zero(),
            formation: None,
            object_position: None,
            taut_cables: Vec::new(),
            solution_index: None,
        })
    }

    /// Initializes the wrapper with the first absolute formation and reference
    /// object position.
    ///
    /// The formation centroid becomes the local-frame origin. `po_initial` is
    /// used as the branch-selection reference, and the returned value is the
    /// selected stable object position in absolute coordinates.
    ///
    /// # Errors
    ///
    /// Returns a dimension mismatch for an incorrectly sized formation, or any
    /// solving error reported by [`VvcmFk`].
    pub fn init(
        &mut self,
        formation: RobotFormation,
        po_initial: Point3,
    ) -> Result<Point3, VvcmError> {
        self.fk_engine.validate_formation(&formation)?;

        self.global_position = formation.centroid();
        let local_formation = formation.relative_to(self.global_position);
        self.formation = Some(local_formation.clone());

        let reference = po_initial.relative_xy_to(self.global_position);
        self.update_from_fk(local_formation, reference)
    }

    /// Updates the wrapper from a new absolute formation and returns the
    /// closest stable object position in absolute coordinates.
    ///
    /// If [`VvcmManualSimulation::init`] has already selected a branch, that
    /// previous local object position is used as the reference. Otherwise the
    /// local origin is used as the reference.
    ///
    /// # Errors
    ///
    /// Returns a dimension mismatch for an incorrectly sized formation, or any
    /// solving error reported by [`VvcmFk`].
    pub fn get_new_stable_solution(
        &mut self,
        formation: RobotFormation,
    ) -> Result<Point3, VvcmError> {
        self.fk_engine.validate_formation(&formation)?;

        self.global_position = formation.centroid();
        let local_formation = formation.relative_to(self.global_position);
        self.formation = Some(local_formation.clone());

        let reference = self.object_position.unwrap_or_else(Point3::zero);
        self.update_from_fk(local_formation, reference)
    }

    /// Borrows the underlying FK engine and its latest solution cache.
    pub fn fk_engine(&self) -> &VvcmFk {
        &self.fk_engine
    }

    /// Returns the current local-frame origin in absolute coordinates.
    pub fn global_position(&self) -> Point2 {
        self.global_position
    }

    /// Borrows the current robot formation in the centroid-relative local
    /// frame, if initialized.
    pub fn formation(&self) -> Option<&RobotFormation> {
        self.formation.as_ref()
    }

    /// Returns the selected object position in the local frame, if initialized.
    pub fn object_position(&self) -> Option<Point3> {
        self.object_position
    }

    /// Returns the selected object position in absolute coordinates, if
    /// initialized.
    pub fn absolute_object_position(&self) -> Option<Point3> {
        self.object_position
            .map(|position| position.translated_xy_by(self.global_position))
    }

    /// Borrows the taut cable indices for the currently selected branch.
    pub fn taut_cables(&self) -> &[usize] {
        &self.taut_cables
    }

    /// Returns the index of the selected branch in the FK solution cache.
    pub fn solution_index(&self) -> Option<usize> {
        self.solution_index
    }

    /// Runs FK in the local frame, stores the selected branch, and returns its
    /// absolute object position.
    fn update_from_fk(
        &mut self,
        local_formation: RobotFormation,
        reference: Point3,
    ) -> Result<Point3, VvcmError> {
        let solutions = self.fk_engine.update_stable_solutions(local_formation)?;
        let (solution_index, solution) = solutions
            .closest_stable_to(reference)
            .ok_or(VvcmError::NoStableSolution)?;

        self.solution_index = Some(solution_index);
        self.object_position = Some(solution.po);
        self.taut_cables = solution.taut_cables.clone();

        Ok(solution.po.translated_xy_by(self.global_position))
    }
}
