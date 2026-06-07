use crate::error::VvcmError;
use crate::fk::VvcmFk;
use crate::types::{FkSolution, Point2, Point3, RobotFormation, Scalar, SheetShape};

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

    pub fn fk_engine(&self) -> &VvcmFk {
        &self.fk_engine
    }

    pub fn global_position(&self) -> Point2 {
        self.global_position
    }

    pub fn formation(&self) -> Option<&RobotFormation> {
        self.formation.as_ref()
    }

    pub fn object_position(&self) -> Option<Point3> {
        self.object_position
    }

    pub fn taut_cables(&self) -> &[usize] {
        &self.taut_cables
    }

    pub fn solution_index(&self) -> Option<usize> {
        self.solution_index
    }

    fn update_from_fk(
        &mut self,
        local_formation: RobotFormation,
        reference: Point3,
    ) -> Result<Point3, VvcmError> {
        let solutions = self.fk_engine.update_stable_solutions(local_formation)?;
        let (solution_index, solution) =
            closest_solution(&solutions.stable, reference).ok_or(VvcmError::NoStableSolution)?;

        self.solution_index = Some(solution_index);
        self.object_position = Some(solution.po);
        self.taut_cables = solution.taut_cables.clone();

        Ok(solution.po.translated_xy_by(self.global_position))
    }
}

fn closest_solution(solutions: &[FkSolution], reference: Point3) -> Option<(usize, &FkSolution)> {
    solutions
        .iter()
        .enumerate()
        .min_by(|(_, left), (_, right)| {
            left.po
                .distance_to(reference)
                .total_cmp(&right.po.distance_to(reference))
        })
}
