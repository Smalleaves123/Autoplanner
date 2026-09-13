#!/usr/bin/env python3
"""Deterministic robustness cases for RobotNav validation runs."""

from __future__ import annotations

from dataclasses import dataclass, field
from collections import deque
import json
import math
from pathlib import Path
import random
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = 1
SUITE_TYPE = "robotnav.validation.suite"
_CASE_KINDS = {"tracking", "dynamic_agents"}


def _finite_non_negative(name: str, value: Any) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be finite and non-negative") from error
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return result


def _optional_positive(name: str, value: Any | None) -> float | None:
    if value is None:
        return None
    result = _finite_non_negative(name, value)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive when provided")
    return result


def _non_negative_int(name: str, value: Any) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a non-negative integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a non-negative integer") from error
    if result != value or result < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return result


def _integer(name: str, value: Any) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be an integer") from error
    if result != value:
        raise ValueError(f"{name} must be an integer")
    return result


@dataclass(frozen=True)
class PerturbationSpec:
    """Observation and command perturbations applied around any simulator."""

    position_noise_std: float = 0.0
    heading_noise_std: float = 0.0
    velocity_noise_std: float = 0.0
    observation_latency_steps: int = 0
    command_velocity_limit: float | None = None
    command_steering_limit: float | None = None

    def __post_init__(self) -> None:
        for name in (
                "position_noise_std", "heading_noise_std",
                "velocity_noise_std"):
            object.__setattr__(self, name, _finite_non_negative(
                name, getattr(self, name)))
        object.__setattr__(self, "observation_latency_steps",
                           _non_negative_int(
                               "observation_latency_steps",
                               self.observation_latency_steps))
        for name in ("command_velocity_limit", "command_steering_limit"):
            object.__setattr__(self, name, _optional_positive(
                name, getattr(self, name)))

    @property
    def active(self) -> bool:
        return any((
            self.position_noise_std > 0.0,
            self.heading_noise_std > 0.0,
            self.velocity_noise_std > 0.0,
            self.observation_latency_steps > 0,
            self.command_velocity_limit is not None,
            self.command_steering_limit is not None,
        ))

    def to_dict(self) -> dict[str, Any]:
        return {
            "position_noise_std": self.position_noise_std,
            "heading_noise_std": self.heading_noise_std,
            "velocity_noise_std": self.velocity_noise_std,
            "observation_latency_steps": self.observation_latency_steps,
            "command_velocity_limit": self.command_velocity_limit,
            "command_steering_limit": self.command_steering_limit,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "PerturbationSpec":
        return cls(**dict(data))

    def cli_args(self) -> list[str]:
        args = [
            "--position-noise-std", str(self.position_noise_std),
            "--heading-noise-std", str(self.heading_noise_std),
            "--velocity-noise-std", str(self.velocity_noise_std),
            "--observation-latency-steps",
            str(self.observation_latency_steps),
        ]
        if self.command_velocity_limit is not None:
            args += ["--command-velocity-limit",
                     str(self.command_velocity_limit)]
        if self.command_steering_limit is not None:
            args += ["--command-steering-limit",
                     str(self.command_steering_limit)]
        return args


class PerturbedSimulator:
    """Apply deterministic sensor noise, delay, and command saturation."""

    def __init__(self, simulator: Any, spec: PerturbationSpec, seed: int):
        self.simulator = simulator
        self.spec = spec
        self.seed = _non_negative_int("seed", seed)
        self.random = random.Random(self.seed)
        self.history: deque[dict[str, float]] = deque(
            maxlen=spec.observation_latency_steps + 1)
        self.last_truth: dict[str, float] = {}
        self.last_applied_velocity = 0.0
        self.last_applied_steering = 0.0

    def reset(self, x: float = 0.0, y: float = 0.0,
              theta: float = 0.0, velocity: float = 0.0) -> None:
        self.simulator.reset(x, y, theta, velocity)
        self.random.seed(self.seed)
        self.history.clear()
        truth = self._truth()
        self.last_truth = truth
        for _ in range(self.spec.observation_latency_steps + 1):
            self.history.append(dict(truth))
        self.last_applied_velocity = velocity
        self.last_applied_steering = 0.0

    def _truth(self) -> dict[str, float]:
        return {name: float(value) for name, value in
                self.simulator.observe().items()}

    def _noisy(self, state: Mapping[str, float]) -> dict[str, float]:
        result = dict(state)
        result["x"] += self.random.gauss(0.0, self.spec.position_noise_std)
        result["y"] += self.random.gauss(0.0, self.spec.position_noise_std)
        result["theta"] += self.random.gauss(0.0, self.spec.heading_noise_std)
        result["v"] += self.random.gauss(0.0, self.spec.velocity_noise_std)
        return result

    def observe(self) -> dict[str, float]:
        if not self.history:
            self.history.append(self._truth())
        return self._noisy(self.history[0])

    def step(self, velocity: float, steering: float) -> dict[str, float]:
        velocity_limit = self.spec.command_velocity_limit
        steering_limit = self.spec.command_steering_limit
        self.last_applied_velocity = max(
            -velocity_limit if velocity_limit is not None else -math.inf,
            min(velocity_limit if velocity_limit is not None else math.inf,
                float(velocity)))
        self.last_applied_steering = max(
            -steering_limit if steering_limit is not None else -math.inf,
            min(steering_limit if steering_limit is not None else math.inf,
                float(steering)))
        self.simulator.step(
            self.last_applied_velocity, self.last_applied_steering)
        self.last_truth = self._truth()
        self.history.append(self.last_truth)
        return self.observe()

    def close(self) -> None:
        close = getattr(self.simulator, "close", None)
        if close is not None:
            close()


@dataclass(frozen=True)
class DynamicAgentSpec:
    """Integer-grid constant-velocity agent accepted by the dynamic CLI."""

    start_frame: int
    end_frame: int
    x: int
    y: int
    dx: int
    dy: int

    def __post_init__(self) -> None:
        for name in ("start_frame", "end_frame"):
            object.__setattr__(self, name, _non_negative_int(
                name, getattr(self, name)))
        if self.end_frame < self.start_frame:
            raise ValueError("end_frame must not precede start_frame")
        for name in ("x", "y", "dx", "dy"):
            object.__setattr__(self, name, _integer(name, getattr(self, name)))
        if self.dx == 0 and self.dy == 0:
            raise ValueError("a dynamic agent must move")

    def to_dict(self) -> dict[str, int]:
        return {
            "start_frame": self.start_frame, "end_frame": self.end_frame,
            "x": self.x, "y": self.y, "dx": self.dx, "dy": self.dy,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "DynamicAgentSpec":
        return cls(**dict(data))

    def cli_args(self) -> list[str]:
        return [
            "--moving-obstacle", str(self.start_frame), str(self.end_frame),
            str(self.x), str(self.y), str(self.dx), str(self.dy),
        ]


@dataclass(frozen=True)
class ValidationCase:
    case_id: str
    kind: str
    seed: int
    perturbation: PerturbationSpec = field(default_factory=PerturbationSpec)
    dynamic_agents: tuple[DynamicAgentSpec, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if not self.case_id.strip():
            raise ValueError("case_id must not be empty")
        if self.kind not in _CASE_KINDS:
            raise ValueError(f"unsupported validation case kind: {self.kind!r}")
        object.__setattr__(self, "seed", _non_negative_int("seed", self.seed))
        if self.kind == "tracking" and self.dynamic_agents:
            raise ValueError("tracking cases cannot contain dynamic agents")
        if self.kind == "dynamic_agents" and not self.dynamic_agents:
            raise ValueError("dynamic-agent cases require at least one agent")

    def to_dict(self) -> dict[str, Any]:
        return {
            "case_id": self.case_id,
            "kind": self.kind,
            "seed": self.seed,
            "perturbation": self.perturbation.to_dict(),
            "dynamic_agents": [agent.to_dict() for agent in self.dynamic_agents],
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ValidationCase":
        return cls(
            case_id=str(data["case_id"]), kind=str(data["kind"]),
            seed=data["seed"],
            perturbation=PerturbationSpec.from_dict(
                data.get("perturbation", {})),
            dynamic_agents=tuple(DynamicAgentSpec.from_dict(agent)
                                 for agent in data.get("dynamic_agents", ())),
        )


@dataclass(frozen=True)
class ValidationSuite:
    suite_id: str
    seed: int
    cases: tuple[ValidationCase, ...]

    def __post_init__(self) -> None:
        if not self.suite_id.strip():
            raise ValueError("suite_id must not be empty")
        object.__setattr__(self, "seed", _non_negative_int("seed", self.seed))
        if not self.cases:
            raise ValueError("a validation suite must contain cases")
        identifiers = [case.case_id for case in self.cases]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("validation case identifiers must be unique")

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "artifact_type": SUITE_TYPE,
            "suite_id": self.suite_id,
            "seed": self.seed,
            "cases": [case.to_dict() for case in self.cases],
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ValidationSuite":
        if data.get("schema_version") != SCHEMA_VERSION:
            raise ValueError("unsupported validation suite schema version")
        if data.get("artifact_type") != SUITE_TYPE:
            raise ValueError(f"expected artifact_type {SUITE_TYPE!r}")
        return cls(
            suite_id=str(data["suite_id"]), seed=data["seed"],
            cases=tuple(ValidationCase.from_dict(case)
                        for case in data["cases"]),
        )

    def save_json(self, path: str | Path) -> Path:
        output = Path(path).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as stream:
            json.dump(self.to_dict(), stream, indent=2, sort_keys=True,
                      allow_nan=False)
            stream.write("\n")
        return output

    @classmethod
    def load_json(cls, path: str | Path) -> "ValidationSuite":
        with Path(path).expanduser().open(encoding="utf-8") as stream:
            return cls.from_dict(json.load(stream))


def _is_free(grid: Sequence[str], x: int, y: int) -> bool:
    return (0 <= y < len(grid) and 0 <= x < len(grid[y]) and
            grid[y][x] not in "1#@")


def randomized_dynamic_agents(
        grid: Sequence[str], frames: int, count: int,
        seed: int) -> tuple[DynamicAgentSpec, ...]:
    """Choose seeded moving-agent tracks that remain inside free map cells."""

    if frames < 2 or count <= 0:
        raise ValueError("frames must be at least two and count must be positive")
    duration = min(frames - 1, 6)
    candidates: list[DynamicAgentSpec] = []
    directions = ((1, 0), (-1, 0), (0, 1), (0, -1))
    for y, row in enumerate(grid):
        for x in range(len(row)):
            for dx, dy in directions:
                if all(_is_free(grid, x + dx * step, y + dy * step)
                       for step in range(duration + 1)):
                    candidates.append(DynamicAgentSpec(
                        0, duration, x, y, dx, dy))
    if len(candidates) < count:
        raise ValueError("map has too few free straight tracks for agent suite")
    generator = random.Random(_non_negative_int("seed", seed))
    generator.shuffle(candidates)
    selected: list[DynamicAgentSpec] = []
    occupied_tracks: list[set[tuple[int, int]]] = []
    for candidate in candidates:
        track = {
            (candidate.x + candidate.dx * step,
             candidate.y + candidate.dy * step)
            for step in range(duration + 1)
        }
        if any(track & occupied for occupied in occupied_tracks):
            continue
        selected.append(candidate)
        occupied_tracks.append(track)
        if len(selected) == count:
            return tuple(selected)
    raise ValueError("map has too few non-overlapping tracks for agent suite")


def build_validation_suite(
        name: str, *, seed: int = 42, repeats: int = 1,
        grid: Sequence[str] | None = None, frames: int = 20,
        agent_count: int = 3) -> ValidationSuite:
    """Build one of the stable baseline/noise/latency/saturation/agent suites."""

    if name not in {"baseline", "noise", "latency", "saturation",
                    "dynamic_agents", "robustness"}:
        raise ValueError(f"unknown validation suite: {name!r}")
    if repeats <= 0:
        raise ValueError("repeats must be positive")
    base_seed = _non_negative_int("seed", seed)
    cases: list[ValidationCase] = []

    def append_tracking(label: str, spec: PerturbationSpec) -> None:
        for repeat in range(repeats):
            case_seed = base_seed + repeat
            cases.append(ValidationCase(
                f"{label}-r{repeat:02d}-s{case_seed}", "tracking",
                case_seed, spec))

    if name in {"baseline", "robustness"}:
        append_tracking("baseline", PerturbationSpec())
    if name in {"noise", "robustness"}:
        append_tracking("noise-mild", PerturbationSpec(
            position_noise_std=0.02, heading_noise_std=0.01,
            velocity_noise_std=0.02))
        append_tracking("noise-severe", PerturbationSpec(
            position_noise_std=0.10, heading_noise_std=0.05,
            velocity_noise_std=0.10))
    if name in {"latency", "robustness"}:
        for steps in (1, 3, 6):
            append_tracking(
                f"latency-{steps}",
                PerturbationSpec(observation_latency_steps=steps))
    if name in {"saturation", "robustness"}:
        for label, velocity, steering in (
                ("mild", 1.0, 0.50),
                ("medium", 0.60, 0.30),
                ("severe", 0.35, 0.18)):
            append_tracking(f"saturation-{label}", PerturbationSpec(
                command_velocity_limit=velocity,
                command_steering_limit=steering))
    if name in {"dynamic_agents", "robustness"}:
        if grid is None:
            raise ValueError("grid is required for dynamic-agent suites")
        for repeat in range(repeats):
            case_seed = base_seed + repeat
            agents = randomized_dynamic_agents(
                grid, frames, agent_count, case_seed)
            cases.append(ValidationCase(
                f"dynamic-agents-r{repeat:02d}-s{case_seed}",
                "dynamic_agents", case_seed,
                dynamic_agents=agents))
    return ValidationSuite(f"{name}-v1", base_seed, tuple(cases))
