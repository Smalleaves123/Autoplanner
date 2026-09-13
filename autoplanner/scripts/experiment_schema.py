#!/usr/bin/env python3
"""Versioned scenario and result artifacts for RobotNav experiments.

The schema deliberately contains no simulator imports.  It can therefore be
used to compare, archive, and replay kinematic and optional physics runs on a
machine that does not have MuJoCo or PyBullet installed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = 1
SCENARIO_TYPE = "robotnav.experiment.scenario"
RUN_TYPE = "robotnav.experiment.run"
MANIFEST_TYPE = "robotnav.experiment.manifest"

_BACKEND_MODELS = {
    "kinematic": {"constrained_bicycle"},
    "mujoco": {"planar"},
    "pybullet": {"planar", "racecar"},
}
_RUN_STATUSES = {"goal_reached", "step_limit", "safe_stop", "failed"}


def _finite(name: str, value: Any, *, positive: bool = False,
            non_negative: bool = False) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    if positive and result <= 0.0:
        raise ValueError(f"{name} must be positive")
    if non_negative and result < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return result


def _positive_int(name: str, value: Any, *, allow_zero: bool = False) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be an integer") from error
    if result != value or result < (0 if allow_zero else 1):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be a {qualifier} integer")
    return result


def _point(name: str, value: Sequence[Any]) -> tuple[float, float]:
    try:
        valid_length = not isinstance(value, (str, bytes)) and len(value) == 2
    except TypeError:
        valid_length = False
    if not valid_length:
        raise ValueError(f"{name} must contain exactly two coordinates")
    return (_finite(f"{name}[0]", value[0]),
            _finite(f"{name}[1]", value[1]))


def _mapping(name: str, value: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    result = dict(value)
    try:
        json.dumps(result, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain JSON-compatible values") from error
    return result


def _require_version(data: Mapping[str, Any], artifact_type: str) -> None:
    if data.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            f"unsupported experiment schema version: "
            f"{data.get('schema_version')!r}")
    if data.get("artifact_type") != artifact_type:
        raise ValueError(f"expected artifact_type {artifact_type!r}")


def save_json(path: str | Path, data: Mapping[str, Any]) -> Path:
    """Write a schema object as deterministic, standards-compliant JSON."""

    output = Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return output


@dataclass(frozen=True)
class BackendSpec:
    """Execution backend and its concrete dynamics model."""

    name: str
    model: str

    def __post_init__(self) -> None:
        if self.name not in _BACKEND_MODELS:
            raise ValueError(f"unsupported execution backend: {self.name!r}")
        if self.model not in _BACKEND_MODELS[self.name]:
            raise ValueError(
                f"unsupported {self.name} model: {self.model!r}")

    def to_dict(self) -> dict[str, str]:
        return {"name": self.name, "model": self.model}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "BackendSpec":
        return cls(name=str(data["name"]), model=str(data["model"]))


@dataclass(frozen=True)
class SimulationSpec:
    """Time step and actuator limits shared by every execution backend."""

    dt: float = 0.05
    wheelbase: float = 1.0
    max_velocity: float = 2.0
    max_acceleration: float = 1.5
    max_deceleration: float = 2.0
    max_steering: float = 0.7
    max_steering_rate: float = 1.5

    def __post_init__(self) -> None:
        object.__setattr__(self, "dt", _finite("dt", self.dt, positive=True))
        object.__setattr__(self, "wheelbase", _finite(
            "wheelbase", self.wheelbase, positive=True))
        for name in (
                "max_velocity", "max_acceleration", "max_deceleration",
                "max_steering", "max_steering_rate"):
            object.__setattr__(self, name, _finite(
                name, getattr(self, name), non_negative=True))

    def to_dict(self) -> dict[str, float]:
        return {
            "dt": self.dt,
            "wheelbase": self.wheelbase,
            "max_velocity": self.max_velocity,
            "max_acceleration": self.max_acceleration,
            "max_deceleration": self.max_deceleration,
            "max_steering": self.max_steering,
            "max_steering_rate": self.max_steering_rate,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "SimulationSpec":
        return cls(**{name: data[name] for name in (
            "dt", "wheelbase", "max_velocity", "max_acceleration",
            "max_deceleration", "max_steering", "max_steering_rate")})


@dataclass(frozen=True)
class ScenarioSpec:
    """A backend-specific execution of a backend-independent route."""

    scenario_id: str
    backend: BackendSpec
    simulation: SimulationSpec
    map_path: str
    path: str
    start: tuple[float, float]
    goal: tuple[float, float]
    planner: str
    controller: str
    local_planner: str = "none"
    max_steps: int = 1
    goal_tolerance: float = 0.75
    initial_offset: float = 0.0
    seed: int = 0
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.scenario_id.strip():
            raise ValueError("scenario_id must not be empty")
        if not isinstance(self.backend, BackendSpec):
            raise TypeError("backend must be a BackendSpec")
        if not isinstance(self.simulation, SimulationSpec):
            raise TypeError("simulation must be a SimulationSpec")
        for name in ("planner", "controller", "local_planner"):
            if not str(getattr(self, name)).strip():
                raise ValueError(f"{name} must not be empty")
        object.__setattr__(self, "start", _point("start", self.start))
        object.__setattr__(self, "goal", _point("goal", self.goal))
        object.__setattr__(self, "max_steps", _positive_int(
            "max_steps", self.max_steps))
        object.__setattr__(self, "goal_tolerance", _finite(
            "goal_tolerance", self.goal_tolerance, non_negative=True))
        object.__setattr__(self, "initial_offset", _finite(
            "initial_offset", self.initial_offset))
        object.__setattr__(self, "seed", _positive_int(
            "seed", self.seed, allow_zero=True))
        object.__setattr__(self, "metadata", _mapping(
            "metadata", self.metadata))

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "artifact_type": SCENARIO_TYPE,
            "scenario_id": self.scenario_id,
            "backend": self.backend.to_dict(),
            "simulation": self.simulation.to_dict(),
            "route": {
                "map": self.map_path,
                "path": self.path,
                "start": list(self.start),
                "goal": list(self.goal),
            },
            "components": {
                "planner": self.planner,
                "controller": self.controller,
                "local_planner": self.local_planner,
            },
            "run": {
                "max_steps": self.max_steps,
                "goal_tolerance": self.goal_tolerance,
                "initial_offset": self.initial_offset,
                "seed": self.seed,
            },
            "metadata": self.metadata,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ScenarioSpec":
        _require_version(data, SCENARIO_TYPE)
        route = data["route"]
        components = data["components"]
        run = data["run"]
        return cls(
            scenario_id=str(data["scenario_id"]),
            backend=BackendSpec.from_dict(data["backend"]),
            simulation=SimulationSpec.from_dict(data["simulation"]),
            map_path=str(route["map"]), path=str(route["path"]),
            start=tuple(route["start"]), goal=tuple(route["goal"]),
            planner=str(components["planner"]),
            controller=str(components["controller"]),
            local_planner=str(components.get("local_planner", "none")),
            max_steps=run["max_steps"],
            goal_tolerance=run["goal_tolerance"],
            initial_offset=run.get("initial_offset", 0.0),
            seed=run.get("seed", 0),
            metadata=dict(data.get("metadata", {})),
        )

    def save_json(self, path: str | Path) -> Path:
        return save_json(path, self.to_dict())

    @classmethod
    def load_json(cls, path: str | Path) -> "ScenarioSpec":
        with Path(path).expanduser().open(encoding="utf-8") as stream:
            return cls.from_dict(json.load(stream))


@dataclass(frozen=True)
class RunMetrics:
    """Backend-neutral metrics emitted for every attempted execution."""

    status: str
    run_success: bool
    goal_reached: bool
    steps: int
    goal_time_s: float | None
    goal_distance: float
    actual_path_length: float
    collision_steps: int = 0
    safe_stop: bool = False
    safe_stop_steps: int = 0
    max_cross_track: float = 0.0
    mean_cross_track: float = 0.0
    max_heading_error: float = 0.0
    mean_heading_error: float = 0.0
    control_effort: float = 0.0
    compute_latency_ms: dict[str, float | None] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.status not in _RUN_STATUSES:
            raise ValueError(f"unsupported run status: {self.status!r}")
        object.__setattr__(self, "steps", _positive_int(
            "steps", self.steps, allow_zero=True))
        object.__setattr__(self, "collision_steps", _positive_int(
            "collision_steps", self.collision_steps, allow_zero=True))
        object.__setattr__(self, "safe_stop_steps", _positive_int(
            "safe_stop_steps", self.safe_stop_steps, allow_zero=True))
        if self.goal_reached and self.goal_time_s is None:
            raise ValueError("goal_time_s is required when goal_reached is true")
        if self.goal_time_s is not None:
            object.__setattr__(self, "goal_time_s", _finite(
                "goal_time_s", self.goal_time_s, non_negative=True))
        for name in (
                "goal_distance", "actual_path_length", "max_cross_track",
                "mean_cross_track", "max_heading_error",
                "mean_heading_error", "control_effort"):
            object.__setattr__(self, name, _finite(
                name, getattr(self, name), non_negative=True))
        latency = dict(self.compute_latency_ms)
        for name, value in latency.items():
            if value is not None:
                latency[name] = _finite(
                    f"compute_latency_ms.{name}", value, non_negative=True)
        object.__setattr__(self, "compute_latency_ms", latency)

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "run_success": self.run_success,
            "goal_reached": self.goal_reached,
            "steps": self.steps,
            "goal_time_s": self.goal_time_s,
            "goal_distance": self.goal_distance,
            "actual_path_length": self.actual_path_length,
            "collision_steps": self.collision_steps,
            "safe_stop": self.safe_stop,
            "safe_stop_steps": self.safe_stop_steps,
            "max_cross_track": self.max_cross_track,
            "mean_cross_track": self.mean_cross_track,
            "max_heading_error": self.max_heading_error,
            "mean_heading_error": self.mean_heading_error,
            "control_effort": self.control_effort,
            "compute_latency_ms": self.compute_latency_ms,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "RunMetrics":
        return cls(**{name: data[name] for name in (
            "status", "run_success", "goal_reached", "steps", "goal_time_s",
            "goal_distance", "actual_path_length", "collision_steps",
            "safe_stop", "safe_stop_steps", "max_cross_track",
            "mean_cross_track", "max_heading_error", "mean_heading_error",
            "control_effort", "compute_latency_ms")})


@dataclass(frozen=True)
class RunArtifact:
    """One complete run description, metrics, and external trace reference."""

    scenario: ScenarioSpec
    metrics: RunMetrics
    trace_csv: str
    planner_metrics: dict[str, Any] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.scenario, ScenarioSpec):
            raise TypeError("scenario must be a ScenarioSpec")
        if not isinstance(self.metrics, RunMetrics):
            raise TypeError("metrics must be RunMetrics")
        object.__setattr__(self, "planner_metrics", _mapping(
            "planner_metrics", self.planner_metrics))
        object.__setattr__(self, "metadata", _mapping(
            "metadata", self.metadata))

    def to_dict(self) -> dict[str, Any]:
        data = {
            "schema_version": SCHEMA_VERSION,
            "artifact_type": RUN_TYPE,
            "scenario": self.scenario.to_dict(),
            "metrics": self.metrics.to_dict(),
            "artifacts": {"trace_csv": self.trace_csv},
            "planner_metrics": self.planner_metrics,
            "metadata": self.metadata,
        }
        # Keep the pre-v1 flat summary fields during the versioned migration.
        # Readers should prefer scenario/metrics/artifacts; these aliases can
        # be removed only in a future schema version.
        legacy_model = self.scenario.backend.model
        if self.scenario.backend.name == "mujoco":
            legacy_model = "planar_mujoco"
        data.update({
            "backend": self.scenario.backend.name,
            "physics_model": legacy_model,
            "controller": self.scenario.controller,
            "wheelbase": self.scenario.simulation.wheelbase,
            "path": self.scenario.path,
            "planner": self.planner_metrics,
            "steps": self.metrics.steps,
            "goal_reached": self.metrics.goal_reached,
            "goal_distance": self.metrics.goal_distance,
            "actual_path_length": self.metrics.actual_path_length,
            "max_cross_track": self.metrics.max_cross_track,
            "mean_cross_track": self.metrics.mean_cross_track,
            "max_heading_error": self.metrics.max_heading_error,
            "mean_heading_error": self.metrics.mean_heading_error,
            "collision_steps": self.metrics.collision_steps,
            "run_success": self.metrics.run_success,
            "csv": self.trace_csv,
        })
        return data

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "RunArtifact":
        _require_version(data, RUN_TYPE)
        return cls(
            scenario=ScenarioSpec.from_dict(data["scenario"]),
            metrics=RunMetrics.from_dict(data["metrics"]),
            trace_csv=str(data.get("artifacts", {}).get("trace_csv", "")),
            planner_metrics=dict(data.get("planner_metrics", {})),
            metadata=dict(data.get("metadata", {})),
        )

    def save_json(self, path: str | Path) -> Path:
        return save_json(path, self.to_dict())

    @classmethod
    def load_json(cls, path: str | Path) -> "RunArtifact":
        with Path(path).expanduser().open(encoding="utf-8") as stream:
            return cls.from_dict(json.load(stream))


@dataclass(frozen=True)
class ExperimentManifest:
    """Index of comparable runs of the same logical scenario."""

    scenario_id: str
    runs: tuple[RunArtifact, ...]

    def __post_init__(self) -> None:
        if not self.scenario_id.strip():
            raise ValueError("scenario_id must not be empty")
        for run in self.runs:
            if run.scenario.scenario_id != self.scenario_id:
                raise ValueError("manifest runs must share scenario_id")

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "artifact_type": MANIFEST_TYPE,
            "scenario_id": self.scenario_id,
            "runs": [run.to_dict() for run in self.runs],
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ExperimentManifest":
        _require_version(data, MANIFEST_TYPE)
        return cls(
            scenario_id=str(data["scenario_id"]),
            runs=tuple(RunArtifact.from_dict(run) for run in data["runs"]),
        )

    def save_json(self, path: str | Path) -> Path:
        return save_json(path, self.to_dict())

    @classmethod
    def load_json(cls, path: str | Path) -> "ExperimentManifest":
        with Path(path).expanduser().open(encoding="utf-8") as stream:
            return cls.from_dict(json.load(stream))


def artifact_from_legacy_summary(
        summary: Mapping[str, Any], scenario_id: str = "legacy") -> RunArtifact:
    """Convert the pre-v1 physics tracking summary into a v1 run artifact."""

    backend_name = str(summary["backend"])
    legacy_model = str(summary.get("physics_model", ""))
    model = {
        ("kinematic", ""): "constrained_bicycle",
        ("mujoco", "planar_mujoco"): "planar",
    }.get((backend_name, legacy_model), legacy_model)
    if backend_name == "kinematic" and model != "constrained_bicycle":
        model = "constrained_bicycle"
    steps = _positive_int("steps", summary.get("steps", 0), allow_zero=True)
    dt = _finite("dt", summary.get("dt", 0.05), positive=True)
    goal_reached = bool(summary.get("goal_reached", False))
    scenario = ScenarioSpec(
        scenario_id=scenario_id,
        backend=BackendSpec(backend_name, model),
        simulation=SimulationSpec(
            dt=dt, wheelbase=summary.get("wheelbase", 1.0)),
        map_path=str(summary.get("map", "")),
        path=str(summary.get("path", "")),
        start=(0.0, 0.0), goal=(0.0, 0.0),
        planner=str(summary.get("planner_name", "legacy")),
        controller=str(summary.get("controller", "legacy")),
        max_steps=max(steps, 1),
        metadata={"converted_from": "legacy_physics_summary"},
    )
    metrics = RunMetrics(
        status="goal_reached" if goal_reached else "step_limit",
        run_success=bool(summary.get("run_success", False)),
        goal_reached=goal_reached,
        steps=steps,
        goal_time_s=steps * dt if goal_reached else None,
        goal_distance=summary.get("goal_distance", 0.0),
        actual_path_length=summary.get("actual_path_length", 0.0),
        collision_steps=summary.get("collision_steps", 0),
        max_cross_track=summary.get("max_cross_track", 0.0),
        mean_cross_track=summary.get("mean_cross_track", 0.0),
        max_heading_error=summary.get("max_heading_error", 0.0),
        mean_heading_error=summary.get("mean_heading_error", 0.0),
    )
    planner_metrics = summary.get("planner", {})
    if not isinstance(planner_metrics, Mapping):
        planner_metrics = {"legacy_value": planner_metrics}
    return RunArtifact(
        scenario=scenario, metrics=metrics,
        trace_csv=str(summary.get("csv", "")),
        planner_metrics=dict(planner_metrics),
    )
