"""Backend-neutral, deterministic world stepping for RobotNav experiments."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import random
from typing import Mapping, Sequence

from .control import (
    ControlCommand,
    RobotState,
    SimulationConfig,
    TwistCommand,
    _backend,
    steering_to_twist,
)
from .planning import BackendUnavailableError


@dataclass(frozen=True)
class SimulationClock:
    """Logical simulation clock; it advances only when ``World.step`` runs."""

    step_index: int = 0
    time: float = 0.0
    dt: float = 0.05


@dataclass(frozen=True)
class WorldConfig:
    """Shared execution and collision settings for a simulation world."""

    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    collision_check: bool = True
    seed: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.simulation, SimulationConfig):
            raise TypeError("simulation must be a SimulationConfig")
        if not isinstance(self.collision_check, bool):
            raise TypeError("collision_check must be a bool")
        if isinstance(self.seed, bool) or self.seed < 0:
            raise ValueError("seed must be a non-negative integer")


@dataclass(frozen=True)
class WorldEntity:
    """Initial state and collision radius for one simulated entity."""

    entity_id: str
    state: RobotState
    radius: float = 0.25

    def __post_init__(self) -> None:
        if not self.entity_id.strip():
            raise ValueError("entity_id must not be empty")
        if not isinstance(self.state, RobotState):
            raise TypeError("state must be a RobotState")
        if not math.isfinite(self.radius) or self.radius < 0.0:
            raise ValueError("radius must be finite and non-negative")


@dataclass(frozen=True)
class EntityObservation:
    entity_id: str
    state: RobotState
    radius: float = 0.25


@dataclass(frozen=True)
class WorldObservation:
    """Immutable snapshot returned after reset and every world step."""

    clock: SimulationClock
    entities: tuple[EntityObservation, ...]
    collisions: tuple[tuple[str, str], ...] = ()

    def state(self, entity_id: str) -> RobotState:
        for entity in self.entities:
            if entity.entity_id == entity_id:
                return entity.state
        raise KeyError(entity_id)


class SimulationWorld:
    """Run several identical-backend entities under one logical clock.

    The world deliberately owns only stepping and pairwise collision checks.
    Map collision layers and sensors can consume the immutable observation
    without being coupled to a particular native simulator backend.
    """

    def __init__(
        self,
        config: WorldConfig | None = None,
        entities: Sequence[WorldEntity] = (),
    ) -> None:
        self.config = config or WorldConfig()
        if not isinstance(self.config, WorldConfig):
            raise TypeError("config must be a WorldConfig")
        self._rng = random.Random(self.config.seed)
        self._entities: dict[str, WorldEntity] = {}
        self._simulators: dict[str, object] = {}
        self._clock = SimulationClock(dt=self.config.simulation.dt)
        self._last_observation = WorldObservation(self._clock, ())
        self.reset(entities)

    @property
    def clock(self) -> SimulationClock:
        return self._clock

    @property
    def observation(self) -> WorldObservation:
        return self._last_observation

    def reset(self, entities: Sequence[WorldEntity] = ()) -> WorldObservation:
        normalized = list(entities)
        ids = [entity.entity_id for entity in normalized]
        if len(set(ids)) != len(ids):
            raise ValueError("entity_id values must be unique")
        self._entities = {entity.entity_id: entity for entity in normalized}
        self._simulators = {
            entity.entity_id: self._make_simulator(entity.state)
            for entity in normalized
        }
        self._rng.seed(self.config.seed)
        self._clock = SimulationClock(dt=self.config.simulation.dt)
        self._last_observation = self._build_observation()
        return self._last_observation

    def step(
        self,
        commands: Mapping[str, ControlCommand | TwistCommand] | None = None,
    ) -> WorldObservation:
        commands = commands or {}
        unknown = set(commands) - set(self._entities)
        if unknown:
            raise KeyError(f"commands reference unknown entities: {sorted(unknown)}")
        backend = _backend()
        for entity_id in sorted(self._entities):
            command = commands.get(entity_id, ControlCommand(0.0, 0.0))
            simulator = self._simulators[entity_id]
            if self.config.simulation.execution_model == "differential_drive":
                if isinstance(command, ControlCommand):
                    command = steering_to_twist(
                        command, self.config.simulation.wheelbase)
                if not isinstance(command, TwistCommand):
                    raise TypeError(
                        "differential-drive commands must be ControlCommand or TwistCommand")
                native_command = backend.DifferentialDriveCommand(
                    command.linear_velocity, command.angular_velocity)
            else:
                if not isinstance(command, ControlCommand):
                    raise TypeError(
                        "bicycle commands must be ControlCommand")
                native_command = backend.Control(
                    command.velocity, command.steering)
            simulator.step(native_command)
        self._clock = SimulationClock(
            step_index=self._clock.step_index + 1,
            time=self._clock.time + self._clock.dt,
            dt=self._clock.dt,
        )
        self._last_observation = self._build_observation()
        return self._last_observation

    def _make_simulator(self, state: RobotState) -> object:
        backend = _backend()
        config = self.config.simulation
        native_state = state.to_native(backend)
        if config.execution_model == "differential_drive":
            if not hasattr(backend, "DifferentialDriveSimulator"):
                raise BackendUnavailableError(
                    "differential-drive execution is unavailable")
            return backend.DifferentialDriveSimulator(
                native_state, config.to_native(backend))
        if config.execution_model == "dynamic_bicycle":
            if not hasattr(backend, "DynamicBicycleSimulator"):
                raise BackendUnavailableError(
                    "dynamic-bicycle execution is unavailable")
            return backend.DynamicBicycleSimulator(
                native_state, config.to_native(backend))
        return backend.KinematicBicycleSimulator(
            native_state, config.to_native(backend))

    def _build_observation(self) -> WorldObservation:
        entities = tuple(
            EntityObservation(
                entity_id,
                RobotState.from_native(self._simulators[entity_id].state),
                self._entities[entity_id].radius,
            )
            for entity_id in sorted(self._entities)
        )
        collisions: list[tuple[str, str]] = []
        if self.config.collision_check:
            for index, first in enumerate(entities):
                first_radius = self._entities[first.entity_id].radius
                for second in entities[index + 1:]:
                    second_radius = self._entities[second.entity_id].radius
                    if math.hypot(
                        first.state.x - second.state.x,
                        first.state.y - second.state.y,
                    ) <= first_radius + second_radius:
                        collisions.append((first.entity_id, second.entity_id))
        return WorldObservation(
            clock=self._clock,
            entities=entities,
            collisions=tuple(collisions),
        )
