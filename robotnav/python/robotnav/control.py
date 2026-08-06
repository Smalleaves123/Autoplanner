"""Python facade for trajectory generation, control, and local simulation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Sequence

from .planning import BackendUnavailableError, PlanResult


def _backend() -> Any:
    try:
        import autompc
    except ImportError as error:  # pragma: no cover - installation dependent
        raise BackendUnavailableError(
            "The compiled autompc backend is unavailable. Build the Python "
            "bindings or install the robotnav wheel first."
        ) from error
    return autompc


@dataclass
class TrajectoryConfig:
    """Options for converting a polyline into a controller-ready trajectory."""

    sample_spacing: float = 0.5
    target_velocity: float = 1.0
    max_velocity: float = 2.0
    max_acceleration: float = 1.5
    max_deceleration: float = 2.0
    max_lateral_acceleration: float = 1.5
    allow_reverse: bool = False
    max_reverse_velocity: float = 1.0
    max_curvature: float = 0.0

    def __post_init__(self) -> None:
        if self.sample_spacing <= 0.0:
            raise ValueError("sample_spacing must be positive")
        if self.max_velocity < 0.0 or self.max_reverse_velocity < 0.0:
            raise ValueError("velocity limits must be non-negative")
        if self.max_acceleration < 0.0 or self.max_deceleration < 0.0:
            raise ValueError("acceleration limits must be non-negative")
        if self.max_curvature < 0.0:
            raise ValueError("max_curvature must be non-negative")

    def to_native(self, backend: Any | None = None) -> Any:
        backend = backend or _backend()
        options = backend.TrajectoryOptions()
        for name in (
            "sample_spacing", "target_velocity", "max_velocity",
            "max_acceleration", "max_deceleration",
            "max_lateral_acceleration", "allow_reverse",
            "max_reverse_velocity", "max_curvature",
        ):
            if hasattr(options, name):
                setattr(options, name, getattr(self, name))
        return options


@dataclass(frozen=True)
class TrajectoryPoint:
    x: float
    y: float
    theta: float
    velocity: float
    curvature: float
    acceleration: float


@dataclass(frozen=True)
class TrajectoryResult:
    """Python-native trajectory with access to the underlying C++ points."""

    points: tuple[TrajectoryPoint, ...]
    native: Any

    @property
    def length(self) -> float:
        return sum(
            math.hypot(point.x1 - point.x0, point.y1 - point.y0)
            for point in (
                _Segment(self.points[index - 1], self.points[index])
                for index in range(1, len(self.points))
            )
        )

    @property
    def empty(self) -> bool:
        return not self.points


@dataclass(frozen=True)
class _Segment:
    first: TrajectoryPoint
    second: TrajectoryPoint

    @property
    def x0(self) -> float:
        return self.first.x

    @property
    def y0(self) -> float:
        return self.first.y

    @property
    def x1(self) -> float:
        return self.second.x

    @property
    def y1(self) -> float:
        return self.second.y


def _trajectory_result(native_trajectory: Any) -> TrajectoryResult:
    points = tuple(
        TrajectoryPoint(
            float(point.x), float(point.y), float(point.theta), float(point.v),
            float(point.curvature), float(point.acceleration),
        )
        for point in native_trajectory
    )
    return TrajectoryResult(points=points, native=native_trajectory)


def _waypoints(path: Iterable[Sequence[float] | Any], backend: Any) -> list[Any]:
    result = []
    for point in path:
        if hasattr(point, "x") and hasattr(point, "y"):
            result.append(backend.Waypoint2d(float(point.x), float(point.y)))
        else:
            if len(point) != 2:
                raise ValueError("path points must contain exactly two coordinates")
            result.append(backend.Waypoint2d(float(point[0]), float(point[1])))
    if len(result) < 2:
        raise ValueError("at least two path points are required")
    return result


def generate_trajectory(
    path: Iterable[Sequence[float] | Any] | PlanResult,
    config: TrajectoryConfig | None = None,
    motion_directions: Sequence[int] | None = None,
) -> TrajectoryResult:
    """Generate a C++ trajectory from a path or ``PlanResult``."""

    backend = _backend()
    config = config or TrajectoryConfig()
    if not isinstance(config, TrajectoryConfig):
        raise TypeError("config must be a TrajectoryConfig")
    points = path.path if isinstance(path, PlanResult) else path
    native_waypoints = _waypoints(points, backend)
    native_options = config.to_native(backend)
    if motion_directions is None:
        native_trajectory = backend.generate_trajectory(
            native_waypoints, native_options)
    else:
        directions = [int(direction) for direction in motion_directions]
        if len(directions) != len(native_waypoints):
            raise ValueError("motion_directions must match path length")
        if not hasattr(backend, "generate_trajectory_with_directions"):
            raise BackendUnavailableError(
                "the installed autompc binding does not support motion directions"
            )
        native_trajectory = backend.generate_trajectory_with_directions(
            native_waypoints, directions, native_options)
    return _trajectory_result(native_trajectory)


@dataclass
class ControllerConfig:
    """Common controller selection and execution parameters."""

    controller: str = "stanley"
    target_velocity: float = 1.0
    dt: float = 0.05
    wheelbase: float = 1.0
    lookahead: float = 2.0
    stanley_gain: float = 0.5
    mpc_horizon: int = 15
    max_velocity: float = 2.0
    max_steering: float = 0.7
    max_acceleration: float = 1.5
    max_deceleration: float = 2.0
    max_steering_rate: float = 1.5

    def __post_init__(self) -> None:
        if self.controller not in {"pid", "pure_pursuit", "stanley", "mpc"}:
            raise ValueError(f"unknown controller: {self.controller}")
        if self.dt <= 0.0 or self.wheelbase <= 0.0:
            raise ValueError("dt and wheelbase must be positive")
        if self.mpc_horizon <= 0:
            raise ValueError("mpc_horizon must be positive")


@dataclass(frozen=True)
class RobotState:
    x: float
    y: float
    theta: float = 0.0
    velocity: float = 0.0

    def to_native(self, backend: Any | None = None) -> Any:
        backend = backend or _backend()
        return backend.State(self.x, self.y, self.theta, self.velocity)

    @classmethod
    def from_native(cls, state: Any) -> "RobotState":
        return cls(float(state.x), float(state.y), float(state.theta),
                   float(state.v))


@dataclass(frozen=True)
class ControlCommand:
    velocity: float
    steering: float


class Controller:
    """Stateful controller facade with one common ``compute`` method."""

    def __init__(self, config: ControllerConfig | None = None) -> None:
        self.config = config or ControllerConfig()
        if not isinstance(self.config, ControllerConfig):
            raise TypeError("config must be a ControllerConfig")
        backend = _backend()
        if self.config.controller == "pid":
            self._native = backend.PIDController()
        elif self.config.controller == "pure_pursuit":
            self._native = backend.PurePursuitController(
                self.config.lookahead, self.config.wheelbase)
        elif self.config.controller == "stanley":
            self._native = backend.StanleyController(
                self.config.stanley_gain, self.config.wheelbase)
        else:
            if not hasattr(backend, "MPCController"):
                raise BackendUnavailableError(
                    "MPCController is unavailable; rebuild with Eigen3"
                )
            self._native = backend.MPCController(
                self.config.mpc_horizon, self.config.dt,
                self.config.wheelbase, self.config.max_velocity,
                self.config.max_steering, self.config.max_acceleration,
                self.config.max_deceleration, self.config.max_steering_rate)

    def reset(self) -> None:
        if hasattr(self._native, "reset"):
            self._native.reset()

    @staticmethod
    def _nearest(native_state: Any, native_trajectory: Any) -> Any:
        return min(
            native_trajectory,
            key=lambda point: (float(point.x) - native_state.x) ** 2 +
            (float(point.y) - native_state.y) ** 2,
        )

    def compute(
        self,
        state: RobotState,
        trajectory: TrajectoryResult,
        target_velocity: float | None = None,
    ) -> ControlCommand:
        if trajectory.empty:
            raise ValueError("trajectory must not be empty")
        backend = _backend()
        native_state = state.to_native(backend)
        native_trajectory = trajectory.native
        velocity = (self.config.target_velocity if target_velocity is None
                    else float(target_velocity))
        if self.config.controller == "pure_pursuit":
            command = self._native.compute(
                native_state, native_trajectory, velocity)
        elif self.config.controller == "mpc":
            command = self._native.compute(
                native_state, native_trajectory, velocity)
        else:
            reference = self._nearest(native_state, native_trajectory)
            if self.config.controller == "pid":
                command = self._native.compute(
                    native_state, reference, self.config.dt)
            else:
                command = self._native.compute(
                    native_state, reference, velocity)
        return ControlCommand(float(command.velocity), float(command.steering))


@dataclass
class SimulationConfig:
    dt: float = 0.05
    wheelbase: float = 1.0
    max_velocity: float = 2.0
    max_acceleration: float = 1.5
    max_deceleration: float = 2.0
    max_steering: float = 0.7
    max_steering_rate: float = 1.5
    allow_reverse: bool = False
    max_reverse_velocity: float = 1.0

    def to_native(self, backend: Any | None = None) -> Any:
        backend = backend or _backend()
        options = backend.SimulationOptions()
        for name in (
            "dt", "wheelbase", "max_velocity", "max_acceleration",
            "max_deceleration", "max_steering", "max_steering_rate",
            "allow_reverse", "max_reverse_velocity",
        ):
            setattr(options, name, getattr(self, name))
        return options


@dataclass(frozen=True)
class TrackingMetrics:
    max_cross_track: float
    mean_cross_track: float
    max_heading_error: float
    mean_heading_error: float


@dataclass(frozen=True)
class SimulationResult:
    states: tuple[RobotState, ...]
    controls: tuple[ControlCommand, ...]
    metrics: TrackingMetrics


def simulate(
    initial: RobotState,
    trajectory: TrajectoryResult,
    controller: Controller,
    config: SimulationConfig | None = None,
    max_time: float = 30.0,
) -> SimulationResult:
    """Run a controller against the C++ kinematic bicycle simulator."""

    if max_time <= 0.0:
        raise ValueError("max_time must be positive")
    if not isinstance(controller, Controller):
        raise TypeError("controller must be a Controller")
    if trajectory.empty:
        raise ValueError("trajectory must not be empty")
    config = config or SimulationConfig()
    backend = _backend()
    native_simulator = backend.KinematicBicycleSimulator(
        initial.to_native(backend), config.to_native(backend))
    states: list[RobotState] = []
    controls: list[ControlCommand] = []
    elapsed = 0.0
    while elapsed < max_time:
        state = RobotState.from_native(native_simulator.state)
        command = controller.compute(state, trajectory)
        native_command = backend.Control(command.velocity, command.steering)
        next_native_state = native_simulator.step(native_command)
        states.append(RobotState.from_native(next_native_state))
        controls.append(command)
        elapsed += config.dt
        if math.hypot(
            states[-1].x - trajectory.points[-1].x,
            states[-1].y - trajectory.points[-1].y,
        ) <= 0.75 and elapsed > config.dt:
            break

    native_states = [state.to_native(backend) for state in states]
    errors = backend.compute_errors(native_states, trajectory.native)
    metrics = TrackingMetrics(
        float(errors.max_cross_track), float(errors.mean_cross_track),
        float(errors.max_heading_err), float(errors.mean_heading_err),
    )
    return SimulationResult(tuple(states), tuple(controls), metrics)
