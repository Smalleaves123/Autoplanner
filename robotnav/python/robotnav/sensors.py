"""Deterministic 2-D sensor simulation for :mod:`robotnav.simulation`."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import random
from typing import Any, Sequence

from .control import RobotState
from .simulation import SimulationClock, WorldObservation


TimestampNs = int


@dataclass(frozen=True)
class SensorTiming:
    latency_steps: int = 0
    drop_probability: float = 0.0
    timestamp_jitter_ns: int = 0

    def __post_init__(self) -> None:
        if self.latency_steps < 0 or self.timestamp_jitter_ns < 0:
            raise ValueError("sensor timing values must be non-negative")
        if not 0.0 <= self.drop_probability <= 1.0:
            raise ValueError("drop_probability must be in [0, 1]")


@dataclass(frozen=True)
class Lidar2DConfig:
    sensor_id: str = "lidar"
    entity_id: str = "robot"
    num_beams: int = 181
    field_of_view: float = math.pi
    max_range: float = 20.0
    min_range: float = 0.05
    range_noise_std: float = 0.0
    ray_step: float = 0.1
    timing: SensorTiming = SensorTiming()

    def __post_init__(self) -> None:
        if not self.sensor_id.strip() or not self.entity_id.strip():
            raise ValueError("sensor_id and entity_id must not be empty")
        if self.num_beams < 2 or self.field_of_view <= 0.0:
            raise ValueError("lidar beam count and field of view are invalid")
        if self.min_range <= 0.0 or self.max_range < self.min_range:
            raise ValueError("lidar range limits are invalid")
        if self.range_noise_std < 0.0 or self.ray_step <= 0.0:
            raise ValueError("lidar noise and ray_step must be non-negative/positive")


@dataclass(frozen=True)
class OdometryConfig:
    sensor_id: str = "odom"
    entity_id: str = "robot"
    position_noise_std: float = 0.0
    heading_noise_std: float = 0.0
    velocity_noise_std: float = 0.0
    timing: SensorTiming = SensorTiming()

    def __post_init__(self) -> None:
        if not self.sensor_id.strip() or not self.entity_id.strip():
            raise ValueError("sensor_id and entity_id must not be empty")
        if min(self.position_noise_std, self.heading_noise_std,
               self.velocity_noise_std) < 0.0:
            raise ValueError("odometry noise must be non-negative")


@dataclass(frozen=True)
class ImuConfig:
    sensor_id: str = "imu"
    entity_id: str = "robot"
    acceleration_noise_std: float = 0.0
    angular_velocity_noise_std: float = 0.0
    timing: SensorTiming = SensorTiming()
    angular_velocity_bias: float = 0.0
    # rad/s / sqrt(s); evolves on physical time, even when packets drop.
    angular_velocity_bias_random_walk_std: float = 0.0
    acceleration_bias: float = 0.0
    # Forward acceleration units / sqrt(s); independent of gyro bias RNG.
    acceleration_bias_random_walk_std: float = 0.0

    def __post_init__(self) -> None:
        if not self.sensor_id.strip() or not self.entity_id.strip():
            raise ValueError("sensor_id and entity_id must not be empty")
        for value in (self.acceleration_noise_std, self.angular_velocity_noise_std,
                      self.angular_velocity_bias_random_walk_std,
                      self.acceleration_bias_random_walk_std):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("IMU noise must be finite and non-negative")
        if not math.isfinite(self.angular_velocity_bias):
            raise ValueError("IMU angular velocity bias must be finite")
        if not math.isfinite(self.acceleration_bias):
            raise ValueError("IMU acceleration bias must be finite")


@dataclass(frozen=True)
class DepthCameraConfig:
    sensor_id: str = "depth"
    entity_id: str = "robot"
    width: int = 64
    height: int = 48
    horizontal_field_of_view: float = math.radians(90.0)
    max_range: float = 20.0
    depth_noise_std: float = 0.0
    timing: SensorTiming = SensorTiming()

    def __post_init__(self) -> None:
        if not self.sensor_id.strip() or not self.entity_id.strip():
            raise ValueError("sensor_id and entity_id must not be empty")
        if self.width < 2 or self.height < 2:
            raise ValueError("depth image dimensions must be at least 2")
        if self.horizontal_field_of_view <= 0.0 or self.max_range <= 0.0:
            raise ValueError("depth camera field of view and range are invalid")
        if self.depth_noise_std < 0.0:
            raise ValueError("depth noise must be non-negative")


@dataclass(frozen=True)
class LidarScan:
    sensor_id: str
    entity_id: str
    sequence: int
    timestamp_ns: TimestampNs
    arrival_timestamp_ns: TimestampNs
    frame_id: str
    pose: RobotState
    angles: tuple[float, ...]
    ranges: tuple[float, ...]
    hits: tuple[bool, ...]
    # Empty keeps the pre-dynamic-return constructor compatible; scans
    # produced by SensorSuite always populate one semantic kind per beam.
    kinds: tuple[str, ...] = ()

    def _beam_kinds(self) -> tuple[str, ...]:
        if len(self.kinds) == len(self.ranges):
            return self.kinds
        return tuple("static" if hit else "unknown" for hit in self.hits)

    @property
    def points(self) -> tuple[tuple[float, float, bool], ...]:
        """Return absolute map-frame endpoints for perception adapters."""
        return tuple(
            (
                self.pose.x + distance * math.cos(self.pose.theta + angle),
                self.pose.y + distance * math.sin(self.pose.theta + angle),
                hit,
            )
            for angle, distance, hit in zip(self.angles, self.ranges, self.hits)
        )

    def to_dict(self) -> dict[str, Any]:
        return {"type": "lidar2d", "sensor_id": self.sensor_id,
                "entity_id": self.entity_id, "sequence": self.sequence,
                "timestamp_ns": self.timestamp_ns,
                "arrival_timestamp_ns": self.arrival_timestamp_ns,
                "frame_id": self.frame_id, "pose": self.pose.__dict__,
                "angles": list(self.angles), "ranges": list(self.ranges),
                "hits": list(self.hits), "kinds": list(self._beam_kinds())}

    def to_perception_frame(
        self, frame: int | None = None, *, pose: RobotState | None = None,
    ) -> dict[str, Any]:
        """Project local rays using a supplied estimate or the simulator pose.

        Sensor-driven consumers should supply the estimated capture-time pose
        explicitly; the default remains compatible with truth-based replay.
        """
        projection_pose = self.pose if pose is None else pose
        return {
            "frame": self.sequence if frame is None else frame,
            "sensor_x": projection_pose.x,
            "sensor_y": projection_pose.y,
            "sensor_theta": projection_pose.theta,
            "timestamp_ns": self.timestamp_ns,
            "arrival_timestamp_ns": self.arrival_timestamp_ns,
            "points": [
                {"x": projection_pose.x + distance * math.cos(projection_pose.theta + angle),
                 "y": projection_pose.y + distance * math.sin(projection_pose.theta + angle),
                 "hit": hit, "kind": kind}
                for angle, distance, hit, kind in zip(
                    self.angles, self.ranges, self.hits, self._beam_kinds())
            ],
        }


@dataclass(frozen=True)
class OdometryMeasurement:
    sensor_id: str
    entity_id: str
    sequence: int
    timestamp_ns: TimestampNs
    arrival_timestamp_ns: TimestampNs
    delta_x: float
    delta_y: float
    delta_theta: float
    velocity: float
    # Physical interval between successfully generated samples, unaffected by
    # timestamp jitter. Zero means the initial sample/no rate information.
    sample_dt: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        data = self.__dict__.copy()
        data["type"] = "odometry"
        return data


@dataclass(frozen=True)
class ImuMeasurement:
    sensor_id: str
    entity_id: str
    sequence: int
    timestamp_ns: TimestampNs
    arrival_timestamp_ns: TimestampNs
    linear_acceleration: float
    angular_velocity: float

    def to_dict(self) -> dict[str, Any]:
        data = self.__dict__.copy()
        data["type"] = "imu"
        return data


@dataclass(frozen=True)
class DepthImage:
    sensor_id: str
    entity_id: str
    sequence: int
    timestamp_ns: TimestampNs
    arrival_timestamp_ns: TimestampNs
    width: int
    height: int
    depths: tuple[float, ...]

    def to_dict(self) -> dict[str, Any]:
        data = self.__dict__.copy()
        data["type"] = "depth_camera"
        data["depths"] = list(self.depths)
        return data


@dataclass(frozen=True)
class SensorBatch:
    clock: SimulationClock
    ready: tuple[object, ...]
    dropped: tuple[str, ...]
    pending_count: int


class GridSensorMap:
    """Small occupancy-grid adapter used by ray-based sensors."""

    def __init__(self, source: str | Path | Sequence[str]) -> None:
        if isinstance(source, (str, Path)):
            rows = Path(source).expanduser().read_text(encoding="utf-8").splitlines()
        else:
            rows = list(source)
        self.rows = tuple(row.strip() for row in rows if row.strip())
        if not self.rows or any(set(row) - {"0", "1"} for row in self.rows):
            raise ValueError("map must contain only 0/1 occupancy rows")
        if any(len(row) != len(self.rows[0]) for row in self.rows):
            raise ValueError("map rows must have equal width")

    def occupied(self, x: float, y: float) -> bool:
        cell_x, cell_y = math.floor(x), math.floor(y)
        if cell_y < 0 or cell_y >= len(self.rows) or cell_x < 0 or cell_x >= len(self.rows[0]):
            return True
        return self.rows[cell_y][cell_x] == "1"


class SensorSuite:
    """Generate deterministic sensor packets from a world observation."""

    def __init__(
        self,
        occupancy_map: GridSensorMap | str | Path | Sequence[str],
        *,
        lidars: Sequence[Lidar2DConfig] = (),
        odometries: Sequence[OdometryConfig] = (),
        imus: Sequence[ImuConfig] = (),
        depth_cameras: Sequence[DepthCameraConfig] = (),
        seed: int = 0,
    ) -> None:
        self.map = occupancy_map if isinstance(occupancy_map, GridSensorMap) else GridSensorMap(occupancy_map)
        self.lidars = tuple(lidars)
        self.odometries = tuple(odometries)
        self.imus = tuple(imus)
        self.depth_cameras = tuple(depth_cameras)
        configs = [*self.lidars, *self.odometries, *self.imus, *self.depth_cameras]
        ids = [config.sensor_id for config in configs]
        if len(set(ids)) != len(ids):
            raise ValueError("sensor_id values must be unique")
        if seed < 0:
            raise ValueError("seed must be non-negative")
        self.seed = seed
        self._rng = random.Random(seed)
        self._sequences: dict[str, int] = {}
        self._pending: list[tuple[int, int, object]] = []
        self._previous: dict[str, tuple[RobotState, float]] = {}
        self._imu_bias: dict[str, tuple[float, float]] = {}
        self._bias_rng = {config.sensor_id: random.Random(
            f"{seed}:imu_bias:{config.sensor_id}") for config in self.imus}
        self._acceleration_bias: dict[str, tuple[float, float]] = {}
        self._acceleration_bias_rng = {config.sensor_id: random.Random(
            f"{seed}:imu_acceleration_bias:{config.sensor_id}") for config in self.imus}

    def reset(self) -> None:
        self._rng.seed(self.seed)
        self._sequences.clear()
        self._pending.clear()
        self._previous.clear()
        self._imu_bias.clear()
        self._acceleration_bias.clear()
        for sensor_id, rng in self._bias_rng.items():
            rng.seed(f"{self.seed}:imu_bias:{sensor_id}")
        for sensor_id, rng in self._acceleration_bias_rng.items():
            rng.seed(f"{self.seed}:imu_acceleration_bias:{sensor_id}")

    @property
    def imu_biases(self) -> dict[str, float]:
        """Current simulator truth for evaluation only, not sensor packets."""
        return {config.sensor_id: self._imu_bias.get(
            config.sensor_id, (config.angular_velocity_bias, 0.0))[0]
                for config in self.imus}

    @property
    def imu_acceleration_biases(self) -> dict[str, float]:
        """Current simulator acceleration bias truth for evaluation only."""
        return {config.sensor_id: self._acceleration_bias.get(
            config.sensor_id, (config.acceleration_bias, 0.0))[0]
                for config in self.imus}

    def step(self, observation: WorldObservation) -> SensorBatch:
        clock = observation.clock
        now_ns = int(round(clock.time * 1_000_000_000.0))
        dropped: list[str] = []
        for config in (*self.lidars, *self.odometries, *self.imus, *self.depth_cameras):
            if isinstance(config, ImuConfig):
                for store, rng, initial_bias, walk_std in (
                    (self._imu_bias, self._bias_rng[config.sensor_id],
                     config.angular_velocity_bias, config.angular_velocity_bias_random_walk_std),
                    (self._acceleration_bias, self._acceleration_bias_rng[config.sensor_id],
                     config.acceleration_bias, config.acceleration_bias_random_walk_std),
                ):
                    bias, previous_time = store.get(config.sensor_id, (initial_bias, clock.time))
                    elapsed = clock.time - previous_time
                    if elapsed < 0:
                        raise ValueError("IMU clock moved backwards; reset the sensor suite")
                    if elapsed > 0 and walk_std > 0:
                        bias += rng.gauss(0.0, walk_std * math.sqrt(elapsed))
                    store[config.sensor_id] = (bias, clock.time)
            sequence = self._sequences.get(config.sensor_id, 0)
            self._sequences[config.sensor_id] = sequence + 1
            if self._rng.random() < config.timing.drop_probability:
                dropped.append(config.sensor_id)
                continue
            state = observation.state(config.entity_id)
            timestamp_ns = max(0, now_ns + self._jitter(config.timing))
            arrival_ns = timestamp_ns + config.timing.latency_steps * int(round(clock.dt * 1e9))
            if isinstance(config, Lidar2DConfig):
                packet = self._lidar(
                    config, sequence, state, timestamp_ns, arrival_ns,
                    observation,
                )
            elif isinstance(config, OdometryConfig):
                packet = self._odometry(config, sequence, state, timestamp_ns, arrival_ns, clock)
            elif isinstance(config, ImuConfig):
                packet = self._imu(config, sequence, state, timestamp_ns, arrival_ns, clock)
            else:
                packet = self._depth(config, sequence, state, timestamp_ns, arrival_ns)
            self._pending.append((arrival_ns, sequence, packet))
        return self._drain_ready(observation, tuple(dropped))

    def drain(self, observation: WorldObservation) -> SensorBatch:
        """Release packets already due without generating a new sample.

        This is useful when an experiment ends with sensor latency enabled:
        callers can advance the world clock and drain the tail of the stream
        without accidentally adding extra sensor frames.
        """
        return self._drain_ready(observation)

    def _drain_ready(
        self,
        observation: WorldObservation,
        dropped: tuple[str, ...] = (),
    ) -> SensorBatch:
        clock = observation.clock
        now_ns = int(round(clock.time * 1_000_000_000.0))
        self._pending.sort(key=lambda item: (item[0], item[1]))
        ready: list[object] = []
        remaining: list[tuple[int, int, object]] = []
        for arrival_ns, sequence, packet in self._pending:
            if arrival_ns <= now_ns:
                ready.append(packet)
            else:
                remaining.append((arrival_ns, sequence, packet))
        self._pending = remaining
        return SensorBatch(clock, tuple(ready), dropped, len(self._pending))

    def _jitter(self, timing: SensorTiming) -> int:
        if timing.timestamp_jitter_ns == 0:
            return 0
        return int(round(self._rng.gauss(0.0, timing.timestamp_jitter_ns)))

    def _noise(self, value: float, standard_deviation: float) -> float:
        return value if standard_deviation == 0.0 else value + self._rng.gauss(0.0, standard_deviation)

    def _ray(
        self,
        state: RobotState,
        angle: float,
        max_range: float,
        step: float,
        dynamic_entities: Sequence[tuple[float, float, float]] = (),
    ) -> tuple[float, bool, str]:
        distance = 0.0
        while distance < max_range:
            distance += step
            x = state.x + distance * math.cos(state.theta + angle)
            y = state.y + distance * math.sin(state.theta + angle)
            if self.map.occupied(x, y):
                return distance, True, "static"
            if any(math.hypot(x - entity_x, y - entity_y) <= radius
                   for entity_x, entity_y, radius in dynamic_entities):
                return distance, True, "dynamic"
        return max_range, False, "unknown"

    def _lidar(
        self,
        config: Lidar2DConfig,
        sequence: int,
        state: RobotState,
        timestamp_ns: int,
        arrival_ns: int,
        observation: WorldObservation,
    ) -> LidarScan:
        angles = tuple(
            -config.field_of_view / 2.0 + config.field_of_view * index / (config.num_beams - 1)
            for index in range(config.num_beams))
        ranges, hits, kinds = [], [], []
        dynamic_entities = tuple(
            (entity.state.x, entity.state.y, entity.radius)
            for entity in observation.entities
            if entity.entity_id != config.entity_id and entity.radius > 0.0
        )
        for angle in angles:
            distance, hit, kind = self._ray(
                state, angle, config.max_range, config.ray_step,
                dynamic_entities,
            )
            ranges.append(min(config.max_range, max(config.min_range,
                         self._noise(distance, config.range_noise_std))))
            hits.append(hit)
            kinds.append(kind)
        return LidarScan(config.sensor_id, config.entity_id, sequence,
                         timestamp_ns, arrival_ns, "map", state,
                         angles, tuple(ranges), tuple(hits), tuple(kinds))

    def _previous_state(self, sensor_id: str, state: RobotState, clock: SimulationClock) -> tuple[RobotState, float]:
        previous = self._previous.get(sensor_id, (state, clock.time))
        self._previous[sensor_id] = (state, clock.time)
        return previous

    def _odometry(self, config: OdometryConfig, sequence: int, state: RobotState,
                  timestamp_ns: int, arrival_ns: int, clock: SimulationClock) -> OdometryMeasurement:
        previous, previous_time = self._previous_state(config.sensor_id, state, clock)
        heading_delta = math.atan2(math.sin(state.theta - previous.theta),
                                   math.cos(state.theta - previous.theta))
        return OdometryMeasurement(
            config.sensor_id, config.entity_id, sequence, timestamp_ns, arrival_ns,
            self._noise(state.x - previous.x, config.position_noise_std),
            self._noise(state.y - previous.y, config.position_noise_std),
            self._noise(heading_delta, config.heading_noise_std),
            self._noise(state.velocity, config.velocity_noise_std),
            max(0.0, clock.time - previous_time),
        )

    def _imu(self, config: ImuConfig, sequence: int, state: RobotState,
             timestamp_ns: int, arrival_ns: int, clock: SimulationClock) -> ImuMeasurement:
        previous, previous_time = self._previous_state(config.sensor_id, state, clock)
        dt = max(clock.time - previous_time, clock.dt)
        return ImuMeasurement(
            config.sensor_id, config.entity_id, sequence, timestamp_ns, arrival_ns,
            self._noise((state.velocity - previous.velocity) / dt +
                        self._acceleration_bias[config.sensor_id][0],
                        config.acceleration_noise_std),
            self._noise(math.atan2(math.sin(state.theta - previous.theta),
                                  math.cos(state.theta - previous.theta)) / dt +
                        self._imu_bias[config.sensor_id][0],
                        config.angular_velocity_noise_std),
        )

    def _depth(self, config: DepthCameraConfig, sequence: int, state: RobotState,
               timestamp_ns: int, arrival_ns: int) -> DepthImage:
        ranges = []
        for column in range(config.width):
            angle = (-config.horizontal_field_of_view / 2.0 +
                     config.horizontal_field_of_view * column / (config.width - 1))
            ranges.append(min(config.max_range, max(0.0, self._noise(
                self._ray(state, angle, config.max_range, 0.1)[0],
                config.depth_noise_std))))
        return DepthImage(
            config.sensor_id, config.entity_id, sequence, timestamp_ns,
            arrival_ns, config.width, config.height,
            tuple(ranges[column]
                  for _row in range(config.height)
                  for column in range(config.width)))
