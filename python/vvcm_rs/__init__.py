"""Python bindings for the vvcm-rs forward-kinematics library."""

from ._vvcm_rs import (
    __version__,
    FkSolution,
    FkSolutions,
    Point2,
    Point3,
    RobotFormation,
    SheetShape,
    VvcmError,
    VvcmFk,
    VvcmManualSimulation,
    VvcmSimulation,
)

VVCM_FK = VvcmFk
VVCM_Simulation = VvcmSimulation
VVCM_ManualSimulation = VvcmManualSimulation

__all__ = [
    "__version__",
    "Point2",
    "Point3",
    "RobotFormation",
    "SheetShape",
    "FkSolution",
    "FkSolutions",
    "VvcmError",
    "VvcmFk",
    "VvcmSimulation",
    "VvcmManualSimulation",
    "VVCM_FK",
    "VVCM_Simulation",
    "VVCM_ManualSimulation",
]
