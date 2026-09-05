"""Python facade for the C++ dynamic navigation pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

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
    max_replanning_retries: int = 1
    replanning_cooldown_frames: int = 0
    recovery_stop_steps: int = 10
    moving_obstacles: tuple[MovingObstacle, ...] = field(default_factory=tuple)
    obstacle_updates: tuple[ObstacleUpdate, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if self.frames <= 0 or self.steps_per_frame <= 0:
            raise ValueError("frames and steps_per_frame must be positive")
        if self.prediction_risk_weight < 0.0 or self.prediction_risk_clearance < 0.0:
            raise ValueError("prediction risk values must be non-negative")
        if self.max_replanning_retries < 0 or self.replanning_cooldown_frames < 0:
            raise ValueError("replanning retry and cooldown values must be non-negative")
        if self.recovery_stop_steps <= 0:
            raise ValueError("recovery_stop_steps must be positive")


@dataclass(frozen=True)
class DynamicResult:
    success: bool
    status_code: str
    message: str
    path: tuple[tuple[float, float], ...]
    metrics: dict[str, Any]
    trace: tuple[dict[str, Any], ...]
    state_transitions: tuple[dict[str, Any], ...] = field(default_factory=tuple)

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
            state_transitions=tuple(
                dict(transition)
                for transition in result.get("state_transitions", ())),
        )

    @classmethod
    def from_dict(cls, result: Mapping[str, Any]) -> "DynamicResult":
        """Restore a result previously written by :meth:`save_json`."""

        return cls(
            success=bool(result["success"]),
            status_code=str(result["status_code"]),
            message=str(result["message"]),
            path=tuple((float(point[0]), float(point[1]))
                       for point in result.get("path", ())),
            metrics=dict(result.get("metrics", {})),
            trace=tuple(dict(sample) for sample in result.get("trace", ())),
            state_transitions=tuple(
                dict(transition)
                for transition in result.get("state_transitions", ())),
        )

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible, stable representation of the result."""

        return {
            "schema_version": 1,
            "success": self.success,
            "status_code": self.status_code,
            "message": self.message,
            "path": [list(point) for point in self.path],
            "metrics": self.metrics,
            "trace": list(self.trace),
            "state_transitions": list(self.state_transitions),
        }

    def save_json(self, path: str | Path) -> Path:
        """Write the result as an indented UTF-8 JSON replay artifact."""

        output = Path(path).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as stream:
            json.dump(self.to_dict(), stream, indent=2, sort_keys=True)
            stream.write("\n")
        return output

    @classmethod
    def load_json(cls, path: str | Path) -> "DynamicResult":
        """Load a result replay artifact from disk."""

        source = Path(path).expanduser()
        with source.open(encoding="utf-8") as stream:
            data = json.load(stream)
        if data.get("schema_version") != 1:
            raise ValueError("unsupported dynamic result schema version")
        return cls.from_dict(data)


@dataclass(frozen=True)
class DynamicScenario:
    """Self-contained dynamic navigation input for repeatable replays."""

    map: str | Path
    start: tuple[int, int]
    goal: tuple[int, int]
    config: DynamicConfig = field(default_factory=DynamicConfig)

    def __post_init__(self) -> None:
        if len(self.start) != 2 or len(self.goal) != 2:
            raise ValueError("start and goal must contain exactly two coordinates")
        if not isinstance(self.config, DynamicConfig):
            raise TypeError("config must be a DynamicConfig")

    def to_dict(self) -> dict[str, Any]:
        """Return a portable JSON representation of this scenario."""

        config = self.config
        return {
            "schema_version": 1,
            "map": str(Path(self.map).expanduser()),
            "start": list(self.start),
            "goal": list(self.goal),
            "config": {
                "planner": config.planner,
                "controller": config.controller,
                "local_planner": config.local_planner,
                "frames": config.frames,
                "steps_per_frame": config.steps_per_frame,
                "auto_insert_obstacles": config.auto_insert_obstacles,
                "prediction_risk_weight": config.prediction_risk_weight,
                "prediction_risk_clearance": config.prediction_risk_clearance,
                "max_replanning_retries": config.max_replanning_retries,
                "replanning_cooldown_frames": config.replanning_cooldown_frames,
                "recovery_stop_steps": config.recovery_stop_steps,
                "moving_obstacles": [obstacle.__dict__ for obstacle in config.moving_obstacles],
                "obstacle_updates": [update.__dict__ for update in config.obstacle_updates],
            },
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "DynamicScenario":
        """Restore a scenario written by :meth:`save_json`."""

        if data.get("schema_version") != 1:
            raise ValueError("unsupported dynamic scenario schema version")
        raw = data["config"]
        obstacles = tuple(MovingObstacle(**item)
                          for item in raw.get("moving_obstacles", ()))
        updates = tuple(ObstacleUpdate(**item)
                        for item in raw.get("obstacle_updates", ()))
        config = DynamicConfig(
            planner=raw.get("planner", "astar"),
            controller=raw.get("controller", "stanley"),
            local_planner=raw.get("local_planner", "none"),
            frames=raw.get("frames", 5),
            steps_per_frame=raw.get("steps_per_frame", 40),
            auto_insert_obstacles=raw.get("auto_insert_obstacles", True),
            prediction_risk_weight=raw.get("prediction_risk_weight", 0.0),
            prediction_risk_clearance=raw.get("prediction_risk_clearance", 0.0),
            max_replanning_retries=raw.get("max_replanning_retries", 1),
            replanning_cooldown_frames=raw.get("replanning_cooldown_frames", 0),
            recovery_stop_steps=raw.get("recovery_stop_steps", 10),
            moving_obstacles=obstacles,
            obstacle_updates=updates,
        )
        return cls(
            map=data["map"],
            start=tuple(data["start"]),
            goal=tuple(data["goal"]),
            config=config,
        )

    def save_json(self, path: str | Path) -> Path:
        """Write this scenario as an indented UTF-8 JSON file."""

        output = Path(path).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as stream:
            json.dump(self.to_dict(), stream, indent=2, sort_keys=True)
            stream.write("\n")
        return output

    @classmethod
    def load_json(cls, path: str | Path) -> "DynamicScenario":
        source = Path(path).expanduser()
        with source.open(encoding="utf-8") as stream:
            return cls.from_dict(json.load(stream))

    def run(self) -> DynamicResult:
        return run_dynamic(self.map, self.start, self.goal, self.config)


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
        config.max_replanning_retries, config.replanning_cooldown_frames,
        config.recovery_stop_steps,
    )
    return DynamicResult.from_native(result)


def run_dynamic_scenario(scenario: DynamicScenario) -> DynamicResult:
    """Run a validated, self-contained dynamic navigation scenario."""

    if not isinstance(scenario, DynamicScenario):
        raise TypeError("scenario must be a DynamicScenario")
    return scenario.run()
