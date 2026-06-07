use crate::error::VvcmError;
use crate::math;
use crate::types::{FkSolutions, RobotFormation, Scalar, SheetShape};

#[derive(Debug, Clone)]
pub struct VvcmFk {
    robot_count: usize,
    hold_height: Scalar,
    sheet: SheetShape,
    formation: Option<RobotFormation>,
    solutions: FkSolutions,
}

impl VvcmFk {
    pub fn new(
        robot_count: usize,
        hold_height: Scalar,
        sheet: SheetShape,
    ) -> Result<Self, VvcmError> {
        if robot_count < 3 {
            return Err(VvcmError::DimensionMismatch {
                context: "robot count",
                expected: 3,
                actual: robot_count,
            });
        }

        if sheet.len() != robot_count {
            return Err(VvcmError::DimensionMismatch {
                context: "sheet vertex count",
                expected: robot_count,
                actual: sheet.len(),
            });
        }

        let _sheet_matrix = math::sheet_to_matrix(&sheet);

        Ok(Self {
            robot_count,
            hold_height,
            sheet,
            formation: None,
            solutions: FkSolutions::default(),
        })
    }

    pub fn update_stable_solutions(
        &mut self,
        formation: RobotFormation,
    ) -> Result<&FkSolutions, VvcmError> {
        self.validate_formation(&formation)?;

        let _formation_matrix = math::formation_to_matrix(&formation);
        let _sheet_matrix = math::sheet_to_matrix(&self.sheet);

        self.formation = Some(formation);
        self.solutions = FkSolutions::default();

        Err(VvcmError::NotImplemented)
    }

    pub fn robot_count(&self) -> usize {
        self.robot_count
    }

    pub fn hold_height(&self) -> Scalar {
        self.hold_height
    }

    pub fn sheet(&self) -> &SheetShape {
        &self.sheet
    }

    pub fn current_formation(&self) -> Option<&RobotFormation> {
        self.formation.as_ref()
    }

    pub fn solutions(&self) -> &FkSolutions {
        &self.solutions
    }

    pub(crate) fn validate_formation(&self, formation: &RobotFormation) -> Result<(), VvcmError> {
        if formation.len() != self.robot_count {
            return Err(VvcmError::DimensionMismatch {
                context: "robot formation point count",
                expected: self.robot_count,
                actual: formation.len(),
            });
        }

        Ok(())
    }
}
