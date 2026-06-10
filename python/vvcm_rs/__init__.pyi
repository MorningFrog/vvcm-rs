from typing import Optional, Sequence, Tuple, Union

__version__: str

Point2Input = Union["Point2", Sequence[float]]
Point3Input = Union["Point3", Sequence[float]]
Point2Rows = Sequence[Point2Input]
FormationInput = Union["RobotFormation", Point2Rows]
SheetInput = Union["SheetShape", Point2Rows]
PointTuples2 = Sequence[Tuple[float, float]]
PointTuples3 = Sequence[Tuple[float, float, float]]


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


class Point2:
    """Two-dimensional point or vector in the XY plane."""

    def __init__(self, x: float = 0.0, y: float = 0.0) -> None:
        """Create a point from explicit XY coordinates."""

    @staticmethod
    def zero() -> "Point2":
        """Return the origin `(0, 0)`."""

    @property
    def x(self) -> float:
        """X coordinate."""

    @x.setter
    def x(self, value: float) -> None: ...

    @property
    def y(self) -> float:
        """Y coordinate."""

    @y.setter
    def y(self, value: float) -> None: ...

    def scaled_by(self, factor: float) -> "Point2":
        """Return a new point with both coordinates multiplied by `factor`."""

    def translated_by(self, offset: Point2Input) -> "Point2":
        """Return a new point translated by the XY `offset`."""

    def relative_to(self, origin: Point2Input) -> "Point2":
        """Return a new point expressed relative to `origin`."""

    def distance_to(self, other: Point2Input) -> float:
        """Compute the Euclidean distance to another 2D point."""

    def as_tuple(self) -> Tuple[float, float]:
        """Return `(x, y)` as a plain Python tuple."""


class Point3:
    """Three-dimensional point or vector."""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """Create a point from explicit XYZ coordinates."""

    @staticmethod
    def zero() -> "Point3":
        """Return the origin `(0, 0, 0)`."""

    @property
    def x(self) -> float:
        """X coordinate."""

    @x.setter
    def x(self, value: float) -> None: ...

    @property
    def y(self) -> float:
        """Y coordinate."""

    @y.setter
    def y(self, value: float) -> None: ...

    @property
    def z(self) -> float:
        """Z coordinate."""

    @z.setter
    def z(self, value: float) -> None: ...

    def translated_xy_by(self, offset: Point2Input) -> "Point3":
        """Return a new point translated in XY, leaving Z unchanged."""

    def relative_xy_to(self, origin: Point2Input) -> "Point3":
        """Return a new point expressed relative to an XY origin, leaving Z unchanged."""

    def distance_to(self, other: Point3Input) -> float:
        """Compute the Euclidean distance to another 3D point."""

    def as_tuple(self) -> Tuple[float, float, float]:
        """Return `(x, y, z)` as a plain Python tuple."""


class RobotFormation:
    """
    Ordered XY positions for robots or robot velocities.

    The constructor accepts `Point2` values, length-2 rows, or any sequence-like
    two-column data object such as a NumPy `N x 2` array.
    """

    def __init__(self, points: Point2Rows) -> None:
        """Create a non-empty robot formation."""

    @staticmethod
    def zeros(robot_count: int) -> "RobotFormation":
        """Create a zero-valued formation with one point per robot."""

    @property
    def points(self) -> Sequence[Point2]:
        """Ordered robot points."""

    def len(self) -> int:
        """Return the number of points in the formation."""

    def is_empty(self) -> bool:
        """Return `True` when the formation contains no points."""

    def centroid(self) -> Point2:
        """Compute the arithmetic mean of all points."""

    def translated_by(self, offset: Point2Input) -> "RobotFormation":
        """Return a new formation translated by `offset`."""

    def relative_to(self, origin: Point2Input) -> "RobotFormation":
        """Return a new formation expressed relative to `origin`."""

    def all_zero(self) -> bool:
        """Return `True` when every point is exactly `(0, 0)`."""

    def as_tuples(self) -> PointTuples2:
        """Return the ordered points as plain `(x, y)` tuples."""

    def __len__(self) -> int: ...
    def __getitem__(self, index: int) -> Point2: ...


class SheetShape:
    """
    Ordered sheet attachment vertices in the sheet-local XY frame.

    The constructor accepts `Point2` values, length-2 rows, or any sequence-like
    two-column data object such as a NumPy `N x 2` array.
    """

    def __init__(self, vertices: Point2Rows) -> None:
        """Create a sheet shape with at least three vertices."""

    @property
    def vertices(self) -> Sequence[Point2]:
        """Ordered sheet vertices."""

    def len(self) -> int:
        """Return the number of sheet vertices."""

    def is_empty(self) -> bool:
        """Return `True` when the sheet contains no vertices."""

    def as_tuples(self) -> PointTuples2:
        """Return the ordered vertices as plain `(x, y)` tuples."""

    def __len__(self) -> int: ...
    def __getitem__(self, index: int) -> Point2: ...


class FkSolution:
    """A single forward-kinematics candidate solution."""

    def __init__(
        self,
        stable: bool = False,
        po: Optional[Point3Input] = None,
        vo: Optional[Point2Input] = None,
        taut_cables: Optional[Sequence[int]] = None,
    ) -> None:
        """Create a forward-kinematics solution value."""

    @property
    def stable(self) -> bool:
        """Whether the candidate is locally stable."""

    @stable.setter
    def stable(self, value: bool) -> None: ...

    @property
    def po(self) -> Point3:
        """Object position `Po` in the formation-local frame."""

    @po.setter
    def po(self, value: Point3Input) -> None: ...

    @property
    def vo(self) -> Point2:
        """Virtual object point `Vo` in the sheet-local XY frame."""

    @vo.setter
    def vo(self, value: Point2Input) -> None: ...

    @property
    def taut_cables(self) -> Sequence[int]:
        """Indices of the taut virtual cables for this candidate."""

    @taut_cables.setter
    def taut_cables(self, value: Sequence[int]) -> None: ...


class FkSolutions:
    """Collection of forward-kinematics candidate solutions."""

    def __init__(self, solutions: Optional[Sequence[FkSolution]] = None) -> None:
        """Create a solution collection from an ordered list of candidates."""

    @property
    def solutions(self) -> Sequence[FkSolution]:
        """All candidate solutions found during the most recent FK update."""

    def is_empty(self) -> bool:
        """Return `True` when no candidates are stored."""

    def stable(self) -> Sequence[FkSolution]:
        """Return locally stable candidate solutions."""

    def closest_stable_to(
        self, reference: Point3Input
    ) -> Optional[Tuple[int, FkSolution]]:
        """Return the stable solution closest to `reference` as `(index, solution)`."""

    def stable_count(self) -> int:
        """Count locally stable candidate solutions."""

    def all_count(self) -> int:
        """Count all candidate solutions, stable and unstable."""

    def __len__(self) -> int: ...
    def __getitem__(self, index: int) -> FkSolution: ...


class VvcmFk:
    """Stateful forward-kinematics engine for a fixed deformable sheet."""

    def __init__(self, robot_count: int, hold_height: float, sheet: SheetInput) -> None:
        """Create a solver for `robot_count` robots holding `sheet` at `hold_height`."""

    def update_stable_solutions(self, formation: FormationInput) -> FkSolutions:
        """Solve and store forward-kinematics branches for `formation`."""

    @property
    def robot_count(self) -> int:
        """Fixed number of robots solved by this engine."""

    @property
    def hold_height(self) -> float:
        """Fixed robot holding height used to recover the object Z coordinate."""

    @property
    def sheet(self) -> SheetShape:
        """Fixed sheet geometry."""

    @property
    def current_formation(self) -> Optional[RobotFormation]:
        """Most recent formation passed to `update_stable_solutions`, if any."""

    @property
    def solutions(self) -> FkSolutions:
        """Most recent solution cache."""


class VvcmSimulation:
    """Fixed-step robot-velocity simulation built on `VvcmFk`."""

    def __init__(
        self,
        robot_count: int,
        hold_height: float,
        sheet: SheetInput,
        initial_formation: FormationInput,
        po_initial: Optional[Point3Input] = None,
        dt: float = 0.033333335,
    ) -> None:
        """Create a simulation from an absolute initial formation and object position."""

    def set_velocity(self, velocity: FormationInput) -> None:
        """Set one XY velocity vector per robot."""

    def step(self) -> None:
        """Advance the simulation by one fixed time step."""

    def absolute_formation(self) -> RobotFormation:
        """Return the current robot formation in absolute coordinates."""

    def absolute_object_position(self) -> Point3:
        """Return the selected object position in absolute coordinates."""

    @property
    def fk_engine(self) -> VvcmFk:
        """Snapshot of the underlying FK engine and its latest solution cache."""

    @property
    def global_position(self) -> Point2:
        """Local-frame origin in absolute coordinates."""

    @property
    def formation(self) -> RobotFormation:
        """Current robot formation in the local frame."""

    @property
    def object_position(self) -> Point3:
        """Selected object position in the local frame."""

    @property
    def taut_cables(self) -> Sequence[int]:
        """Taut cable indices for the currently selected branch."""

    @property
    def solution_index(self) -> Optional[int]:
        """Index of the selected branch in the FK solution cache."""

    @property
    def dt(self) -> float:
        """Fixed integration time step."""

    @property
    def velocity(self) -> RobotFormation:
        """Current per-robot velocity formation."""


class VvcmManualSimulation:
    """Simulation helper for externally supplied robot formations."""

    def __init__(self, robot_count: int, hold_height: float, sheet: SheetInput) -> None:
        """Create a manual simulation wrapper for a fixed sheet."""

    def init(
        self, formation: FormationInput, po_initial: Optional[Point3Input] = None
    ) -> Point3:
        """Initialize with the first absolute formation and reference object position."""

    def get_new_stable_solution(self, formation: FormationInput) -> Point3:
        """Update from a new absolute formation and return the closest stable object position."""

    @property
    def fk_engine(self) -> VvcmFk:
        """Snapshot of the underlying FK engine and its latest solution cache."""

    @property
    def global_position(self) -> Point2:
        """Current local-frame origin in absolute coordinates."""

    @property
    def formation(self) -> Optional[RobotFormation]:
        """Current robot formation in the centroid-relative local frame, if initialized."""

    @property
    def object_position(self) -> Optional[Point3]:
        """Selected object position in the local frame, if initialized."""

    @property
    def absolute_object_position(self) -> Optional[Point3]:
        """Selected object position in absolute coordinates, if initialized."""

    @property
    def taut_cables(self) -> Sequence[int]:
        """Taut cable indices for the currently selected branch."""

    @property
    def solution_index(self) -> Optional[int]:
        """Index of the selected branch in the FK solution cache."""


VVCM_FK = VvcmFk
VVCM_Simulation = VvcmSimulation
VVCM_ManualSimulation = VvcmManualSimulation
