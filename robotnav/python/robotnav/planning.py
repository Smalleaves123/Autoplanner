"""High-level Python wrappers for the C++ path-planning backend."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence


class BackendUnavailableError(ImportError):
    """Raised when the compiled RobotNav Python backend is not available."""


def _backend() -> Any:
    try:
        import autoplanner
    except ImportError as error:  # pragma: no cover - depends on installation
        raise BackendUnavailableError(
            "The compiled autoplanner backend is unavailable. Build the "
            "Python bindings or install the robotnav wheel first."
        ) from error
    return autoplanner


def _point(value: Sequence[int] | Any, backend: Any) -> Any:
    if isinstance(value, backend.Point2i):
        return value
    if len(value) != 2:
        raise ValueError("points must contain exactly two coordinates")
    return backend.Point2i(int(value[0]), int(value[1]))


@dataclass
class PlannerConfig:
    """Common planner configuration exposed by the Python facade.

    Unused fields are ignored by planners that do not need them.  Values are
    copied into a native ``autoplanner.PlannerOptions`` object at call time.
    """

    planner: str = "astar"
    allow_diagonal: bool = True
    robot_radius: float = 0.0
    heuristic_weight: float = 1.0
    weighted_astar_weight: float = 1.5
    obstacle_weight: float = 2.0
    turning_weight: float = 0.5
    step_size: float = 2.0
    max_iterations: int = 5000
    goal_sample_rate: float = 0.1
    goal_tolerance: float = 2.0
    rewire_radius: float = 5.0
    turning_radius: float = 5.0
    angle_bins: int = 72
    allow_reverse: bool = True
    reverse_penalty: float = 1.2
    collision_check_resolution: float = 0.25

    def __post_init__(self) -> None:
        if not self.planner or not isinstance(self.planner, str):
            raise ValueError("planner must be a non-empty string")
        if self.robot_radius < 0.0:
            raise ValueError("robot_radius must be non-negative")
        if self.max_iterations <= 0:
            raise ValueError("max_iterations must be positive")
        if self.angle_bins <= 0:
            raise ValueError("angle_bins must be positive")

    def to_native(self, backend: Any | None = None) -> Any:
        backend = backend or _backend()
        options = backend.PlannerOptions()
        for name in (
            "allow_diagonal", "robot_radius", "heuristic_weight",
            "weighted_astar_weight", "obstacle_weight", "turning_weight",
            "step_size", "max_iterations", "goal_sample_rate",
            "goal_tolerance", "rewire_radius", "turning_radius",
            "angle_bins", "allow_reverse", "reverse_penalty",
            "collision_check_resolution",
        ):
            if hasattr(options, name):
                setattr(options, name, getattr(self, name))
        return options


@dataclass(frozen=True)
class PlanResult:
    """Stable, Python-native view of a C++ planner result."""

    success: bool
    status_code: str
    planner_name: str
    path: tuple[tuple[float, float], ...]
    path_length: float
    planning_time_ms: float
    expanded_nodes: int
    iterations: int
    collision_free: bool
    turning_count: int
    total_turning: float
    average_curvature: float
    smoothness_score: float
    minimum_obstacle_distance: float
    message: str
    native: Any = None

    @classmethod
    def from_native(cls, result: Any) -> "PlanResult":
        return cls(
            success=bool(result.success),
            status_code=str(result.status_code),
            planner_name=str(result.planner_name),
            path=tuple((float(point.x), float(point.y)) for point in result.path),
            path_length=float(result.path_length),
            planning_time_ms=float(result.planning_time_ms),
            expanded_nodes=int(result.expanded_nodes),
            iterations=int(result.iterations),
            collision_free=bool(result.collision_free),
            turning_count=int(result.turning_count),
            total_turning=float(result.total_turning),
            average_curvature=float(result.average_curvature),
            smoothness_score=float(result.smoothness_score),
            minimum_obstacle_distance=float(result.minimum_obstacle_distance),
            message=str(result.message),
            native=result,
        )


def _load_map(value: str | Path | Any, backend: Any) -> Any:
    if isinstance(value, backend.GridMap):
        return value
    path = Path(value).expanduser()
    if not path.exists():
        raise FileNotFoundError(path)
    grid = backend.GridMap()
    if not grid.load_from_txt(str(path)):
        raise ValueError(f"failed to load occupancy map: {path}")
    return grid


def plan(
    map: str | Path | Any,
    start: Sequence[int] | Any,
    goal: Sequence[int] | Any,
    config: PlannerConfig | None = None,
) -> PlanResult:
    """Plan a path from ``start`` to ``goal``.

    ``map`` may be a map filename or a native ``autoplanner.GridMap``.
    Coordinates are integer grid cells.  Planning runs in C++ with the
    Python GIL released.
    """

    backend = _backend()
    config = config or PlannerConfig()
    if not isinstance(config, PlannerConfig):
        raise TypeError("config must be a PlannerConfig")
    native_map = _load_map(map, backend)
    native_start = _point(start, backend)
    native_goal = _point(goal, backend)
    native_result = backend.plan(
        config.planner,
        native_map,
        native_start,
        native_goal,
        config.to_native(backend),
    )
    return PlanResult.from_native(native_result)


class Planner:
    """Reusable planner facade for applications with repeated requests."""

    def __init__(self, config: PlannerConfig | None = None) -> None:
        self.config = config or PlannerConfig()
        if not isinstance(self.config, PlannerConfig):
            raise TypeError("config must be a PlannerConfig")

    def plan(
        self,
        map: str | Path | Any,
        start: Sequence[int] | Any,
        goal: Sequence[int] | Any,
    ) -> PlanResult:
        return plan(map, start, goal, self.config)


def available_planners() -> tuple[str, ...]:
    """Return planner names accepted by the current C++ factory."""

    # Keep this list explicit and stable for UI/autocomplete callers.  The
    # native factory remains authoritative and reports unknown names.
    return (
        "astar", "weighted_astar", "improved_astar", "dijkstra", "jps",
        "dstar_lite", "rrt", "rrt_star", "informed_rrt_star", "bi_rrt",
        "hybrid_astar",
    )
