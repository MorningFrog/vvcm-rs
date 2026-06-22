//! Manual-formation simulation wrapper around the VVCM FK solver.
//!
//! [`VvcmManualSimulation`] is useful when another system supplies complete
//! robot formations directly instead of velocities. Each supplied formation is
//! converted to a centroid-relative local frame before the closest stable FK
//! branch is selected.

use crate::error::VvcmError;
use crate::fk::VvcmFk;
use crate::types::{
    Point2, Point3, Scalar, point2_zero, point3_zero, relative_xy_to, translated_xy_by,
};

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
    formation: Option<Vec<Point2>>,
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
    pub fn new(hold_height: Scalar, sheet: Vec<Point2>) -> Result<Self, VvcmError> {
        Ok(Self {
            fk_engine: VvcmFk::new(hold_height, sheet)?,
            global_position: point2_zero(),
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
    pub fn init(&mut self, formation: &[Point2], po_initial: Point3) -> Result<Point3, VvcmError> {
        self.fk_engine.validate_formation(formation)?;

        self.global_position = centroid(formation);
        let local_formation = relative_points(formation, self.global_position);
        self.formation = Some(local_formation.clone());

        let reference = relative_xy_to(po_initial, self.global_position);
        self.update_from_fk(&local_formation, reference)
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
    pub fn get_new_stable_solution(&mut self, formation: &[Point2]) -> Result<Point3, VvcmError> {
        self.fk_engine.validate_formation(formation)?;

        self.global_position = centroid(formation);
        let local_formation = relative_points(formation, self.global_position);
        self.formation = Some(local_formation.clone());

        let reference = self.object_position.unwrap_or_else(point3_zero);
        self.update_from_fk(&local_formation, reference)
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
    pub fn formation(&self) -> Option<&[Point2]> {
        self.formation.as_deref()
    }

    /// Returns the selected object position in the local frame, if initialized.
    pub fn object_position(&self) -> Option<Point3> {
        self.object_position
    }

    /// Returns the selected object position in absolute coordinates, if
    /// initialized.
    pub fn absolute_object_position(&self) -> Option<Point3> {
        self.object_position
            .map(|position| translated_xy_by(position, self.global_position))
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
        local_formation: &[Point2],
        reference: Point3,
    ) -> Result<Point3, VvcmError> {
        let solutions = self.fk_engine.update_stable_solutions(local_formation)?;
        let (solution_index, solution) = solutions
            .closest_stable_to(reference)
            .ok_or(VvcmError::NoStableSolution)?;

        self.solution_index = Some(solution_index);
        self.object_position = Some(solution.po);
        self.taut_cables = solution.taut_cables.clone();

        Ok(translated_xy_by(solution.po, self.global_position))
    }
}

fn centroid(points: &[Point2]) -> Point2 {
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    for point in points {
        sum_x += point.x;
        sum_y += point.y;
    }

    Point2::new(
        sum_x / points.len() as Scalar,
        sum_y / points.len() as Scalar,
    )
}

fn relative_points(points: &[Point2], origin: Point2) -> Vec<Point2> {
    points
        .iter()
        .map(|point| Point2::new(point.x - origin.x, point.y - origin.y))
        .collect()
}
