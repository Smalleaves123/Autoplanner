"""Python facade for the C++ dynamic navigation pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

from .planning import BackendUnavailableError


def _backend() -> Any:
    try:
        from . import _robotnav
    except ImportError as error:  # pragma: no cover - installation dependent
        raise BackendUnavailableError(
            "The compiled dynamic RobotNav backend is unavailable. Build "
            "with BUILD_PYTHON_BINDINGS=ON first."
        ) from error
    return _robotnav


@dataclass
class MovingObstacle:
    """Time-indexed moving obstacle used by the dynamic planner."""

    start_frame: int
    end_frame: int
    x: int
    y: int
    dx: int = 0
    dy: int = 0
    radius: float = 0.0
    uncertainty_growth: float = 0.0
    acceleration_x: float = 0.0
    acceleration_y: float = 0.0
    covariance_xx: float = 0.0
    covariance_xy: float = 0.0
    covariance_yy: float = 0.0
    covariance_growth_xx: float = 0.0
    covariance_growth_xy: float = 0.0
    covariance_growth_yy: float = 0.0
    confidence_scale: float = 2.0

    def __post_init__(self) -> None:
        if self.start_frame < 0 or self.end_frame < self.start_frame:
            raise ValueError("moving obstacle frame range is invalid")
        if self.radius < 0.0 or self.uncertainty_growth < 0.0:
            raise ValueError("obstacle safety values must be non-negative")
        if self.confidence_scale < 0.0:
            raise ValueError("confidence_scale must be non-negative")

    def to_native(self, backend: Any | None = None) -> Any:
        backend = backend or _backend()
        obstacle = backend.MovingObstacle()
        for name in (
            "start_frame", "end_frame", "x", "y", "dx", "dy", "radius",
            "uncertainty_growth", "acceleration_x", "acceleration_y",
            "covariance_xx", "covariance_xy", "covariance_yy",
            "covariance_growth_xx", "covariance_growth_xy",
            "covariance_growth_yy", "confidence_scale",
        ):
            setattr(obstacle, name, getattr(self, name))
        return obstacle


@dataclass
class ObstacleUpdate:
    frame: int
    x: int
    y: int
    occupied: bool = True

    def to_native(self, backend: Any | None = None) -> Any:
        backend = backend or _backend()
        update = backend.ObstacleUpdate()
        update.frame = self.frame
        update.x = self.x
        update.y = self.y
        update.occupied = self.occupied
        return update


@dataclass
class DynamicConfig:
    planner: str = "astar"
    controller: str = "stanley"
    local_planner: str = "none"
    frames: int = 5
    steps_per_frame: int = 40
    auto_insert_obstacles: bool = True
    prediction_risk_weight: float = 0.0
    prediction_risk_clearance: float = 0.0
    moving_obstacles: tuple[MovingObstacle, ...] = field(default_factory=tuple)
    obstacle_updates: tuple[ObstacleUpdate, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if self.frames <= 0 or self.steps_per_frame <= 0:
            raise ValueError("frames and steps_per_frame must be positive")
        if self.prediction_risk_weight < 0.0 or self.prediction_risk_clearance < 0.0:
            raise ValueError("prediction risk values must be non-negative")


@dataclass(frozen=True)
class DynamicResult:
    success: bool
    status_code: str
    message: str
    path: tuple[tuple[float, float], ...]
    metrics: dict[str, Any]
    trace: tuple[dict[str, Any], ...]

    @classmethod
    def from_native(cls, result: dict[str, Any]) -> "DynamicResult":
        return cls(
            success=bool(result["success"]),
            status_code=str(result["status_code"]),
            message=str(result["message"]),
            path=tuple((float(point[0]), float(point[1]))
                       for point in result["path"]),
            metrics=dict(result["metrics"]),
            trace=tuple(dict(sample) for sample in result["trace"]),
        )


def run_dynamic(
    map: str | Path,
    start: Sequence[int],
    goal: Sequence[int],
    config: DynamicConfig | None = None,
) -> DynamicResult:
    """Run dynamic navigation directly through the compiled C++ backend."""

    if len(start) != 2 or len(goal) != 2:
        raise ValueError("start and goal must contain exactly two coordinates")
    path = Path(map).expanduser()
    if not path.exists():
        raise FileNotFoundError(path)
    config = config or DynamicConfig()
    if not isinstance(config, DynamicConfig):
        raise TypeError("config must be a DynamicConfig")
    backend = _backend()
    native_obstacles = [obstacle.to_native(backend)
                        for obstacle in config.moving_obstacles]
    native_updates = [update.to_native(backend)
                      for update in config.obstacle_updates]
    result = backend.run_dynamic(
        str(path), int(start[0]), int(start[1]), int(goal[0]), int(goal[1]),
        config.planner, config.controller, config.local_planner,
        config.frames, config.steps_per_frame, config.auto_insert_obstacles,
        config.prediction_risk_weight, config.prediction_risk_clearance,
        native_obstacles, native_updates,
    )
    return DynamicResult.from_native(result)
