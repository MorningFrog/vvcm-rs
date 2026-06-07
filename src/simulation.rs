use crate::error::VvcmError;
use crate::fk::VvcmFk;
use crate::types::{Point2, Point3, RobotFormation, Scalar, SheetShape};

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
    pub fn new(
        robot_count: usize,
        hold_height: Scalar,
        sheet: SheetShape,
        initial_formation: RobotFormation,
        po_initial: Point3,
        dt: Scalar,
    ) -> Result<Self, VvcmError> {
        let fk_engine = VvcmFk::new(robot_count, hold_height, sheet)?;
        fk_engine.validate_formation(&initial_formation)?;

        let global_position = initial_formation.points()[0];
        let formation = initial_formation.relative_to(global_position);
        let object_position = po_initial.relative_xy_to(global_position);
        let velocity = RobotFormation::zeros(robot_count)?;

        Ok(Self {
            fk_engine,
            global_position,
            formation,
            object_position,
            taut_cables: Vec::new(),
            solution_index: None,
            dt,
            velocity,
        })
    }

    pub fn set_velocity(&mut self, velocity: RobotFormation) -> Result<(), VvcmError> {
        self.fk_engine.validate_formation(&velocity)?;
        self.velocity = velocity;
        Ok(())
    }

    pub fn step(&mut self) -> Result<(), VvcmError> {
        if self.velocity.all_zero() {
            return Ok(());
        }

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

        self.fk_engine
            .update_stable_solutions(self.formation.clone())?;

        Err(VvcmError::NotImplemented)
    }

    pub fn absolute_formation(&self) -> RobotFormation {
        self.formation.translated_by(self.global_position)
    }

    pub fn fk_engine(&self) -> &VvcmFk {
        &self.fk_engine
    }

    pub fn global_position(&self) -> Point2 {
        self.global_position
    }

    pub fn formation(&self) -> &RobotFormation {
        &self.formation
    }

    pub fn object_position(&self) -> Point3 {
        self.object_position
    }

    pub fn taut_cables(&self) -> &[usize] {
        &self.taut_cables
    }

    pub fn solution_index(&self) -> Option<usize> {
        self.solution_index
    }

    pub fn dt(&self) -> Scalar {
        self.dt
    }

    pub fn velocity(&self) -> &RobotFormation {
        &self.velocity
    }
}
