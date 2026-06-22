"""NumPy-first Python bindings for the vvcm-rs forward-kinematics library."""

from ._vvcm_rs import (
    __version__,
    DimensionMismatchError,
    FkSolution,
    FkSolutions,
    InfeasibleFormationError,
    NoSolutionError,
    NoStableSolutionError,
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
    "FkSolution",
    "FkSolutions",
    "VvcmError",
    "DimensionMismatchError",
    "InfeasibleFormationError",
    "NoSolutionError",
    "NoStableSolutionError",
    "VvcmFk",
    "VvcmSimulation",
    "VvcmManualSimulation",
    "VVCM_FK",
    "VVCM_Simulation",
    "VVCM_ManualSimulation",
]
