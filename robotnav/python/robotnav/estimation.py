"""Dependency-free planar EKF and bounded event-time sensor history.

The state is [x, y, heading, forward velocity, yaw rate], optionally followed
by gyro and/or forward acceleration bias. Bias-corrected IMU acceleration
drives prediction; odometry velocity and IMU/odometry yaw rate are scalar
measurements. This is dead reckoning, not absolute localization: without
pose corrections or another absolute reference, position/heading drift remains.
World-frame odometry deltas and the LiDAR ground-truth pose are NOT used.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

from .control import RobotState
from .sensors import ImuMeasurement, OdometryMeasurement


def _angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _timestamp(value: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError("timestamp_ns must be a non-negative integer")


@dataclass(frozen=True)
class EkfConfig:
    entity_id: str = "robot"
    history_seconds: float = 5.0
    max_history_packets: int = 512
    odometry_velocity_std: float = 0.05
    odometry_yaw_rate_std: float = 0.1
    imu_yaw_rate_std: float = 0.03
    position_process_std: float = 0.02
    heading_process_std: float = 0.02
    velocity_process_std: float = 0.2
    yaw_rate_process_std: float = 0.1
    initial_position_std: float = 0.1
    initial_heading_std: float = 0.1
    initial_velocity_std: float = 0.5
    initial_yaw_rate_std: float = 0.5
    estimate_gyro_bias: bool = False
    initial_gyro_bias: float = 0.0
    initial_gyro_bias_std: float = 0.1
    gyro_bias_process_std: float = 0.001
    estimate_acceleration_bias: bool = False
    initial_acceleration_bias: float = 0.0
    initial_acceleration_bias_std: float = 0.2
    acceleration_bias_process_std: float = 0.001

    def __post_init__(self) -> None:
        if not self.entity_id.strip():
            raise ValueError("entity_id must not be empty")
        for bias in ("gyro", "acceleration"):
            if not isinstance(getattr(self, f"estimate_{bias}_bias"), bool):
                raise ValueError(f"estimate_{bias}_bias must be a boolean")
            if not math.isfinite(getattr(self, f"initial_{bias}_bias")):
                raise ValueError(f"initial_{bias}_bias must be finite")
            std = getattr(self, f"{bias}_bias_process_std")
            if not math.isfinite(std) or std < 0:
                raise ValueError(f"{bias}_bias_process_std must be finite and non-negative")
        if (isinstance(self.max_history_packets, bool) or
                not isinstance(self.max_history_packets, int) or
                self.max_history_packets < 1):
            raise ValueError("max_history_packets must be a positive integer")
        for name, value in self.__dict__.items():
            if name in {"entity_id", "max_history_packets", "estimate_gyro_bias",
                        "initial_gyro_bias", "gyro_bias_process_std",
                        "estimate_acceleration_bias", "initial_acceleration_bias",
                        "acceleration_bias_process_std"}:
                continue
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class StateEstimate:
    timestamp_ns: int
    state: RobotState
    angular_velocity: float
    covariance: tuple[tuple[float, ...], ...]
    gyro_bias: float = 0.0
    # None means the acceleration bias state is disabled. If present it is
    # the final covariance component, after gyro bias when both are enabled.
    acceleration_bias: float | None = None

    @property
    def gyro_bias_std(self) -> float | None:
        """None when gyro bias estimation is disabled."""
        index = _gyro_bias_index(self)
        return math.sqrt(max(0.0, self.covariance[index][index])) if index is not None else None

    @property
    def acceleration_bias_std(self) -> float | None:
        return (math.sqrt(max(0.0, self.covariance[-1][-1]))
                if self.acceleration_bias is not None else None)


def _gyro_bias_index(estimate: StateEstimate) -> int | None:
    return 5 if len(estimate.covariance) - int(estimate.acceleration_bias is not None) == 6 else None


@dataclass(frozen=True)
class PoseMeasurement:
    """Map-frame pose correction with conservative independent stds.

    For scan matching these are tuning parameters, not a claim of exact ICP
    covariance. Repeated scans share map/systematic errors; tune accordingly.
    """

    sensor_id: str
    entity_id: str
    sequence: int
    timestamp_ns: int
    arrival_timestamp_ns: int
    x: float
    y: float
    theta: float
    position_std: float = 0.15
    heading_std: float = 0.03


@dataclass(frozen=True)
class _Checkpoint:
    estimate: StateEstimate
    acceleration: float | None = None


def _predict(point: _Checkpoint, timestamp_ns: int, config: EkfConfig) -> _Checkpoint:
    """Midpoint nonlinear motion model with its analytic Jacobian."""
    dt = (timestamp_ns - point.estimate.timestamp_ns) * 1e-9
    if dt < 0:
        raise ValueError("cannot predict before the retained estimate")
    if dt == 0:
        return point
    state = point.estimate.state
    omega = point.estimate.angular_velocity
    acceleration = (0.0 if point.acceleration is None else
                    point.acceleration - (point.estimate.acceleration_bias or 0.0))
    distance = state.velocity * dt + 0.5 * acceleration * dt * dt
    heading = state.theta + 0.5 * omega * dt
    c, s = math.cos(heading), math.sin(heading)
    size = len(point.estimate.covariance)
    jacobian = [[float(i == j) for j in range(size)] for i in range(size)]
    jacobian[0][2], jacobian[1][2] = -distance * s, distance * c
    jacobian[0][3], jacobian[1][3] = dt * c, dt * s
    jacobian[0][4] = -0.5 * dt * distance * s
    jacobian[1][4] = 0.5 * dt * distance * c
    jacobian[2][4] = dt
    if point.estimate.acceleration_bias is not None and point.acceleration is not None:
        jacobian[0][-1] = -0.5 * dt * dt * c
        jacobian[1][-1] = -0.5 * dt * dt * s
        jacobian[3][-1] = -dt
    p = point.estimate.covariance
    fp = [[sum(jacobian[i][k] * p[k][j] for k in range(size))
           for j in range(size)] for i in range(size)]
    noise = (config.position_process_std, config.position_process_std,
             config.heading_process_std, config.velocity_process_std,
             config.yaw_rate_process_std)
    if config.estimate_gyro_bias:
        noise += (config.gyro_bias_process_std,)
    if config.estimate_acceleration_bias:
        noise += (config.acceleration_bias_process_std,)
    predicted = tuple(tuple(
        sum(fp[i][k] * jacobian[j][k] for k in range(size)) +
        (noise[i] ** 2 * dt if i == j else 0.0)
        for j in range(size)) for i in range(size))
    return _Checkpoint(StateEstimate(
        timestamp_ns,
        RobotState(state.x + distance * c, state.y + distance * s,
                   _angle(state.theta + omega * dt),
                   state.velocity + acceleration * dt),
        omega, predicted, point.estimate.gyro_bias, point.estimate.acceleration_bias,
    ), point.acceleration)


def _correct(point: _Checkpoint, index: int, value: float, std: float,
             *, gyro: bool = False) -> _Checkpoint:
    estimate = point.estimate
    state = estimate.state
    values = [state.x, state.y, state.theta, state.velocity,
              estimate.angular_velocity]
    p = estimate.covariance
    size = len(p)
    gyro_index = _gyro_bias_index(estimate)
    if gyro_index is not None:
        values.append(estimate.gyro_bias)
    if estimate.acceleration_bias is not None:
        values.append(estimate.acceleration_bias)
    h = [float(i == index) for i in range(size)]
    if gyro and gyro_index is not None:
        h[gyro_index] = 1.0  # z_gyro = yaw_rate + bias + noise
    variance = std * std
    ph = [sum(p[i][j] * h[j] for j in range(size)) for i in range(size)]
    innovation_variance = sum(h[i] * ph[i] for i in range(size)) + variance
    gain = [item / innovation_variance for item in ph]
    residual = value - sum(h[i] * values[i] for i in range(size))
    if index == 2:
        residual = _angle(residual)
    values = [item + gain[i] * residual for i, item in enumerate(values)]
    values[2] = _angle(values[2])
    # Joseph form maintains symmetry/positive covariance under roundoff.
    a = [[float(i == j) - gain[i] * h[j]
          for j in range(size)] for i in range(size)]
    ap = [[sum(a[i][k] * p[k][j] for k in range(size))
           for j in range(size)] for i in range(size)]
    covariance = tuple(tuple(
        sum(ap[i][k] * a[j][k] for k in range(size)) + gain[i] * gain[j] * variance
        for j in range(size)) for i in range(size))
    return _Checkpoint(StateEstimate(
        estimate.timestamp_ns, RobotState(*values[:4]), values[4], covariance,
        values[gyro_index] if gyro_index is not None else 0.0,
        values[-1] if estimate.acceleration_bias is not None else None,
    ), point.acceleration)


class PlanarEkf:
    """Fuse timestamped odometry/IMU, replaying late packets in a fixed lag.

    ``update`` returns False for duplicates, wrong-entity, or expired packets.
    Malformed packets raise before mutation. Event-time tie order is stable
    (IMU, odometry, pose correction, sensor id, sequence), independent of arrival order.
    ``estimate_at`` is a non-mutating historical query/extrapolation, useful
    for transforming delayed LiDAR at capture time instead of arrival time.
    """

    def __init__(self, initial: RobotState, config: EkfConfig | None = None,
                 *, timestamp_ns: int = 0) -> None:
        self.config = config or EkfConfig()
        if not isinstance(self.config, EkfConfig):
            raise TypeError("config must be an EkfConfig")
        self.reset(initial, timestamp_ns=timestamp_ns)

    def reset(self, initial: RobotState, *, timestamp_ns: int = 0) -> None:
        _timestamp(timestamp_ns)
        if not isinstance(initial, RobotState) or not all(
                math.isfinite(value) for value in initial.__dict__.values()):
            raise ValueError("initial state must be a finite RobotState")
        c = self.config
        std = (c.initial_position_std, c.initial_position_std,
               c.initial_heading_std, c.initial_velocity_std, c.initial_yaw_rate_std)
        if c.estimate_gyro_bias:
            std += (c.initial_gyro_bias_std,)
        if c.estimate_acceleration_bias:
            std += (c.initial_acceleration_bias_std,)
        p = tuple(tuple(std[i] ** 2 if i == j else 0.0 for j in range(len(std)))
                  for i in range(len(std)))
        self._anchor = _Checkpoint(StateEstimate(
            timestamp_ns, replace(initial, theta=_angle(initial.theta)), 0.0, p,
            c.initial_gyro_bias if c.estimate_gyro_bias else 0.0,
            c.initial_acceleration_bias if c.estimate_acceleration_bias else None))
        self._committed = False
        self._events: list[ImuMeasurement | OdometryMeasurement | PoseMeasurement] = []
        self._history: list[_Checkpoint] = []
        self.accepted_packets = 0
        self.accepted_motion_packets = 0
        self.accepted_pose_packets = 0
        self.rejected_packets = 0
        self.rejected_motion_packets = 0
        self.rejected_pose_packets = 0
        self.last_motion_timestamp_ns = timestamp_ns

    @staticmethod
    def _key(packet: ImuMeasurement | OdometryMeasurement | PoseMeasurement) -> tuple:
        priority = (0 if isinstance(packet, ImuMeasurement) else
                    1 if isinstance(packet, OdometryMeasurement) else 2)
        return (packet.timestamp_ns, priority,
                packet.sensor_id, packet.sequence)

    @property
    def estimate(self) -> StateEstimate:
        return (self._history[-1] if self._history else self._anchor).estimate

    @property
    def oldest_timestamp_ns(self) -> int:
        return max(self._anchor.estimate.timestamp_ns,
                   self.estimate.timestamp_ns - int(self.config.history_seconds * 1e9))

    def update(self, packet: ImuMeasurement | OdometryMeasurement | PoseMeasurement) -> bool:
        if not isinstance(packet, (ImuMeasurement, OdometryMeasurement, PoseMeasurement)):
            raise TypeError("EKF accepts only IMU, odometry, and pose measurements")
        _timestamp(packet.timestamp_ns)
        _timestamp(packet.arrival_timestamp_ns)
        if (not packet.sensor_id.strip() or isinstance(packet.sequence, bool) or
                not isinstance(packet.sequence, int) or packet.sequence < 0):
            raise ValueError("packet identity is invalid")
        if isinstance(packet, ImuMeasurement):
            fields = (packet.linear_acceleration, packet.angular_velocity)
        elif isinstance(packet, OdometryMeasurement):
            fields = (packet.delta_x, packet.delta_y, packet.delta_theta,
                      packet.velocity, packet.sample_dt)
        else:
            fields = (packet.x, packet.y, packet.theta, packet.position_std, packet.heading_std)
            if packet.position_std <= 0 or packet.heading_std <= 0:
                raise ValueError("pose standard deviations must be positive")
        if not all(math.isfinite(value) for value in fields):
            raise ValueError("measurement values must be finite")
        if isinstance(packet, OdometryMeasurement) and packet.sample_dt < 0:
            raise ValueError("sample_dt must be non-negative")
        expired = packet.timestamp_ns < self.estimate.timestamp_ns - int(
            self.config.history_seconds * 1e9)
        before_anchor = packet.timestamp_ns < self._anchor.estimate.timestamp_ns or (
            self._committed and packet.timestamp_ns == self._anchor.estimate.timestamp_ns)
        duplicate = any(type(item) is type(packet) and
                        (item.sensor_id, item.sequence) ==
                        (packet.sensor_id, packet.sequence) for item in self._events)
        if packet.entity_id != self.config.entity_id or expired or before_anchor or duplicate:
            self.rejected_packets += 1
            if isinstance(packet, PoseMeasurement):
                self.rejected_pose_packets += 1
            else:
                self.rejected_motion_packets += 1
            return False
        self._events.append(packet)
        self._events.sort(key=self._key)
        self._history = []
        point = self._anchor
        for item in self._events:
            point = _predict(point, item.timestamp_ns, self.config)
            if isinstance(item, ImuMeasurement):
                point = _correct(point, 4, item.angular_velocity,
                                 self.config.imu_yaw_rate_std, gyro=True)
                point = replace(point, acceleration=item.linear_acceleration)
            elif isinstance(item, OdometryMeasurement):
                point = _correct(point, 3, item.velocity,
                                 self.config.odometry_velocity_std)
                if item.sample_dt > 0:
                    point = _correct(point, 4, item.delta_theta / item.sample_dt,
                                     self.config.odometry_yaw_rate_std)
            else:
                point = _correct(point, 0, item.x, item.position_std)
                point = _correct(point, 1, item.y, item.position_std)
                point = _correct(point, 2, item.theta, item.heading_std)
            self._history.append(point)
        self.accepted_packets += 1
        if isinstance(packet, PoseMeasurement):
            self.accepted_pose_packets += 1
        else:
            self.accepted_motion_packets += 1
            self.last_motion_timestamp_ns = max(self.last_motion_timestamp_ns,
                                                packet.timestamp_ns)
        cutoff = self.estimate.timestamp_ns - int(self.config.history_seconds * 1e9)
        remove = sum(item.timestamp_ns < cutoff for item in self._events)
        remove = max(remove, len(self._events) - self.config.max_history_packets)
        # Commit all events at the boundary timestamp, avoiding ambiguous ties.
        if remove:
            boundary = self._events[remove - 1].timestamp_ns
            while remove < len(self._events) and self._events[remove].timestamp_ns == boundary:
                remove += 1
            self._anchor = self._history[remove - 1]
            self._committed = True
            del self._events[:remove]
            del self._history[:remove]
        return True

    def estimate_at(self, timestamp_ns: int) -> StateEstimate:
        _timestamp(timestamp_ns)
        if timestamp_ns < self.oldest_timestamp_ns:
            raise ValueError("requested pose is older than the retained history")
        point = self._anchor
        for candidate in self._history:
            if candidate.estimate.timestamp_ns > timestamp_ns:
                break
            point = candidate
        return _predict(point, timestamp_ns, self.config).estimate
