from typing import Optional

import numpy as np
from numpy.typing import NDArray

__version__: str

FloatArray = NDArray[np.float32]
IndexArray = NDArray[np.uintp]


class VvcmError(Exception):
    """Exception raised when a VVCM operation cannot produce a valid result."""


class DimensionMismatchError(VvcmError):
    """Exception raised when an input collection has the wrong number of elements."""


class InfeasibleFormationError(VvcmError):
    """Exception raised when the robot formation cannot be realized by the sheet geometry."""


class NoSolutionError(VvcmError):
    """Exception raised when no candidate forward-kinematics solution can be constructed."""


class NoStableSolutionError(VvcmError):
    """Exception raised when candidate solutions exist but none pass the stability test."""


class FkSolution:
    """A single forward-kinematics candidate solution."""

    @property
    def stable(self) -> bool: ...

    @property
    def po(self) -> FloatArray:
        """Object position with shape `(3,)`."""

    @property
    def vo(self) -> FloatArray:
        """Virtual object point with shape `(2,)`."""

    @property
    def taut_cables(self) -> IndexArray:
        """Taut cable indices for this candidate."""

    @property
    def lambda_values(self) -> FloatArray:
        """Taut-only lambda values matching `taut_cables`."""


class FkSolutions:
    """Forward-kinematics candidate solutions."""

    def __init__(self) -> None: ...

    @property
    def solutions(self) -> list[FkSolution]:
        """Ordered candidate solutions."""

    def stable(self) -> list[FkSolution]: ...
    def is_empty(self) -> bool: ...
    def stable_count(self) -> int: ...
    def all_count(self) -> int: ...
    def closest_stable_to(self, reference: FloatArray) -> Optional[int]: ...
    def __len__(self) -> int: ...


class VvcmFk:
    """Stateful forward-kinematics engine for a fixed deformable sheet."""

    def __init__(self, hold_height: float, sheet: FloatArray) -> None:
        """Create a solver from a C-contiguous `float32` sheet array with shape `(n, 2)`."""

    def update_stable_solutions(self, formation: FloatArray) -> FkSolutions:
        """Solve and store forward-kinematics branches for a `(n, 2)` formation array."""

    @property
    def robot_count(self) -> int: ...

    @property
    def hold_height(self) -> float: ...

    @property
    def sheet(self) -> FloatArray: ...

    @property
    def current_formation(self) -> Optional[FloatArray]: ...

    @property
    def solutions(self) -> FkSolutions: ...


class VvcmSimulation:
    """Fixed-step robot-velocity simulation built on `VvcmFk`."""

    def __init__(
        self,
        hold_height: float,
        sheet: FloatArray,
        initial_formation: FloatArray,
        po_initial: Optional[FloatArray] = None,
        dt: float = 0.033333335,
    ) -> None:
        """Create a simulation from C-contiguous `float32` arrays."""

    def set_velocity(self, velocity: FloatArray) -> None:
        """Set one XY velocity vector per robot from a `(n, 2)` array."""

    def step(self) -> None: ...
    def absolute_formation(self) -> FloatArray: ...
    def absolute_object_position(self) -> FloatArray: ...

    @property
    def fk_engine(self) -> VvcmFk: ...

    @property
    def global_position(self) -> FloatArray: ...

    @property
    def formation(self) -> FloatArray: ...

    @property
    def object_position(self) -> FloatArray: ...

    @property
    def taut_cables(self) -> IndexArray: ...

    @property
    def solution_index(self) -> Optional[int]: ...

    @property
    def dt(self) -> float: ...

    @property
    def velocity(self) -> FloatArray: ...

    def solutions(self) -> FkSolutions: ...


class VvcmManualSimulation:
    """Simulation helper for externally supplied robot formations."""

    def __init__(self, hold_height: float, sheet: FloatArray) -> None: ...
    def init(
        self, formation: FloatArray, po_initial: Optional[FloatArray] = None
    ) -> FloatArray: ...
    def get_new_stable_solution(self, formation: FloatArray) -> FloatArray: ...

    @property
    def fk_engine(self) -> VvcmFk: ...

    @property
    def global_position(self) -> FloatArray: ...

    @property
    def formation(self) -> Optional[FloatArray]: ...

    @property
    def object_position(self) -> Optional[FloatArray]: ...

    @property
    def absolute_object_position(self) -> Optional[FloatArray]: ...

    @property
    def taut_cables(self) -> IndexArray: ...

    @property
    def solution_index(self) -> Optional[int]: ...

    def solutions(self) -> FkSolutions: ...


VVCM_FK = VvcmFk
VVCM_Simulation = VvcmSimulation
VVCM_ManualSimulation = VvcmManualSimulation
