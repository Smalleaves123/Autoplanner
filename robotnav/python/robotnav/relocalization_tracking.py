"""Motion-consistent scan confirmations and explicit stationary recovery epochs.

Heuristic mode association, not a probabilistic hypothesis filter. The epoch
adapter owns only EKF/health state; mapping/control resets remain caller-owned.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

from .control import RobotState
from .estimation import EkfConfig, PlanarEkf, _angle, _timestamp
from .localization_health import LocalizationHealthConfig, LocalizationHealthMonitor
from .relocalization import RelocalizationResult
from .sensors import ImuMeasurement, LidarScan, OdometryMeasurement


@dataclass(frozen=True)
class RelocalizationTrackingConfig:
    confirmations: int = 3
    max_scan_gap: float = 1.0
    translation_tolerance: float = 0.3
    heading_tolerance: float = 0.1

    def __post_init__(self) -> None:
        if isinstance(self.confirmations, bool) or not isinstance(self.confirmations, int) or self.confirmations < 2:
            raise ValueError("confirmations must be an integer >= 2")
        for name in ("max_scan_gap", "translation_tolerance", "heading_tolerance"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class RelocalizationTrack:
    track_id: int
    pose: RobotState
    consistent_frames: int
    unique_frames: int


@dataclass(frozen=True)
class RelocalizationDecision:
    timestamp_ns: int
    status: str
    pose: RobotState | None
    tracks: tuple[RelocalizationTrack, ...]


def _finite_pose(pose: RobotState) -> None:
    if not isinstance(pose, RobotState) or not all(math.isfinite(v) for v in pose.__dict__.values()):
        raise ValueError("pose must be a finite RobotState")


def _transport(pose: RobotState, before: RobotState, after: RobotState) -> RobotState:
    """Transport a map hypothesis by the relative SE(2) motion estimate."""
    dx, dy = after.x - before.x, after.y - before.y
    c, s = math.cos(before.theta), math.sin(before.theta)
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    return RobotState(pose.x + c * lx - s * ly, pose.y + s * lx + c * ly,
                      _angle(pose.theta + _angle(after.theta - before.theta)), pose.velocity)


class RelocalizationTracker:
    """Associate per-scan modes using sensor-derived relative motion.

    ``motion_pose`` must describe capture-time dead-reckoning motion in a
    consistent frame, not simulation truth or a pose jumped by map corrections.
    Ambiguous scans can sustain diagnostic mode tracks, but cannot contribute
    to the consecutive unique confirmations required to publish a pose.
    Missing matches and long gaps break streaks; duplicate/older times do not
    mutate tracks. Greedy association is deterministic and one-to-one.
    """

    def __init__(self, config: RelocalizationTrackingConfig | None = None) -> None:
        self.config = config or RelocalizationTrackingConfig()
        if not isinstance(self.config, RelocalizationTrackingConfig):
            raise TypeError("config must be RelocalizationTrackingConfig")
        self.reset()

    def reset(self) -> None:
        self._timestamp: int | None = None
        self._motion: RobotState | None = None
        self._tracks: tuple[RelocalizationTrack, ...] = ()
        self._next_id = 0
        self.ignored_frames = 0

    def update(self, result: RelocalizationResult, *, timestamp_ns: int,
               motion_pose: RobotState) -> RelocalizationDecision:
        _timestamp(timestamp_ns)
        _finite_pose(motion_pose)
        if not isinstance(result, RelocalizationResult):
            raise TypeError("result must be RelocalizationResult")
        if not isinstance(result.accepted, bool):
            raise ValueError("result acceptance must be boolean")
        for hypothesis in result.hypotheses:
            _finite_pose(hypothesis.pose)
            if not math.isfinite(hypothesis.ray_score) or hypothesis.ray_score < 0:
                raise ValueError("hypothesis score must be finite and non-negative")
        if result.accepted:
            if result.reason != "localized" or not result.hypotheses or result.pose != result.hypotheses[0].pose:
                raise ValueError("accepted result must select its best localized hypothesis")
        elif result.pose is not None:
            raise ValueError("unaccepted result cannot select a pose")
        if self._timestamp is not None and timestamp_ns <= self._timestamp:
            self.ignored_frames += 1
            return RelocalizationDecision(timestamp_ns, "out_of_order", None, self._tracks)
        gap = self._timestamp is not None and (timestamp_ns - self._timestamp) * 1e-9 > self.config.max_scan_gap
        previous = () if gap else self._tracks
        predicted = {track.track_id: _transport(track.pose, self._motion, motion_pose)
                     for track in previous}
        available = {track.track_id: track for track in previous}
        tracks = []
        best_associated = False
        # A failed search is not treated as evidence for its diagnostic modes.
        modes = result.hypotheses if result.reason in {"localized", "ambiguous"} else ()
        for index, hypothesis in enumerate(modes):
            choices = []
            for track_id, track in available.items():
                pose = predicted[track_id]
                translation = math.hypot(pose.x - hypothesis.pose.x, pose.y - hypothesis.pose.y)
                heading = abs(_angle(pose.theta - hypothesis.pose.theta))
                if translation <= self.config.translation_tolerance and heading <= self.config.heading_tolerance:
                    choices.append((translation / self.config.translation_tolerance +
                                    heading / self.config.heading_tolerance, track_id, track))
            old = min(choices, key=lambda item: (item[0], item[1]))[2] if choices else None
            if old is not None:
                del available[old.track_id]
                track_id, consistent = old.track_id, old.consistent_frames + 1
            else:
                track_id, consistent = self._next_id, 1
                self._next_id += 1
            # Only the currently selected best mode gets unique evidence;
            # runner-up histories cannot turn an ambiguous scan into recovery.
            unique = (old.unique_frames + 1 if old else 1) if result.accepted and index == 0 else 0
            tracks.append(RelocalizationTrack(track_id, hypothesis.pose, consistent, unique))
            if index == 0:
                best_associated = old is not None
        self._timestamp, self._motion, self._tracks = timestamp_ns, motion_pose, tuple(tracks)
        if not tracks:
            status = "no_match"
        elif not result.accepted:
            status = "ambiguous"
        elif tracks[0].unique_frames >= self.config.confirmations:
            status = "confirmed"
        elif gap:
            status = "gap_reset"
        elif previous and not best_associated:
            status = "motion_inconsistent"
        else:
            status = "candidate_pending"
        return RelocalizationDecision(timestamp_ns, status,
                                      result.pose if status == "confirmed" else None, self._tracks)


@dataclass(frozen=True)
class RelocalizationEpochTransition:
    epoch_id: int
    start_timestamp_ns: int
    confirmation_timestamp_ns: int
    required_resets: tuple[str, ...] = ("motion_frame", "mapping", "obstacle_tracks", "planner", "control")


class RelocalizationEpoch:
    """Explicit stationary-only epoch boundary for an owned EKF/health pair.

    Packet epoch IDs must be tagged at capture/enqueue, not on arrival. This
    adapter does not clear external queues; it rejects old tags/timestamps even
    when they are delivered later. Do not route old data directly into ``ekf``.
    Caller must stop, establish stationarity, reset other components listed in
    the transition, and wait for fresh health confirmations before moving.
    """

    def __init__(self, initial: RobotState, config: EkfConfig | None = None,
                 health_config: LocalizationHealthConfig | None = None, *,
                 timestamp_ns: int = 0, max_confirmation_age: float = 0.5) -> None:
        if not math.isfinite(max_confirmation_age) or max_confirmation_age <= 0:
            raise ValueError("max_confirmation_age must be finite and positive")
        self.ekf = PlanarEkf(initial, config, timestamp_ns=timestamp_ns)
        self.health = LocalizationHealthMonitor(health_config, timestamp_ns=timestamp_ns)
        self.max_confirmation_age = max_confirmation_age
        self.epoch_id = 0
        self.start_timestamp_ns = timestamp_ns
        self._last_confirmation = -1
        self.rejected_epoch_packets = 0

    def apply(self, decision: RelocalizationDecision, *, now_ns: int, epoch_id: int,
              stationary: bool) -> RelocalizationEpochTransition:
        _timestamp(now_ns)
        _timestamp(decision.timestamp_ns)
        if isinstance(epoch_id, bool) or not isinstance(epoch_id, int) or epoch_id != self.epoch_id:
            raise ValueError("confirmation must belong to the current source epoch")
        if stationary is not True:
            raise ValueError("epoch recovery requires explicitly established stationarity")
        if decision.status != "confirmed" or decision.pose is None:
            raise ValueError("only confirmed decisions can start a recovery epoch")
        _finite_pose(decision.pose)
        if (decision.timestamp_ns <= self._last_confirmation or
                decision.timestamp_ns < self.start_timestamp_ns or
                now_ns < self.ekf.estimate.timestamp_ns or
                now_ns < self.health.latest_timestamp_ns or
                not 0 <= now_ns - decision.timestamp_ns <= self.max_confirmation_age * 1e9):
            raise ValueError("confirmation is stale, future, reused, or predates this epoch")
        # Preserve bias means, but reset cross-frame covariances and widen
        # bias uncertainty to at least the configured initial prior.
        estimate, config = self.ekf.estimate_at(now_ns), self.ekf.config
        config = replace(config, initial_gyro_bias=(estimate.gyro_bias if config.estimate_gyro_bias
                                                   else config.initial_gyro_bias),
                         initial_gyro_bias_std=max(config.initial_gyro_bias_std,
                                                   estimate.gyro_bias_std or 0.0),
                         initial_acceleration_bias=(estimate.acceleration_bias if
                             config.estimate_acceleration_bias else config.initial_acceleration_bias),
                         initial_acceleration_bias_std=max(config.initial_acceleration_bias_std,
                                                          estimate.acceleration_bias_std or 0.0))
        initial = replace(decision.pose, velocity=0.0)
        self.ekf = PlanarEkf(initial, config, timestamp_ns=now_ns)
        self.health.reset(timestamp_ns=now_ns)
        self.epoch_id += 1
        self.start_timestamp_ns = now_ns
        self._last_confirmation = decision.timestamp_ns
        return RelocalizationEpochTransition(self.epoch_id, now_ns, decision.timestamp_ns)

    def accept_packet(self, packet: ImuMeasurement | OdometryMeasurement | LidarScan,
                      *, epoch_id: int) -> bool:
        if not isinstance(packet, (ImuMeasurement, OdometryMeasurement, LidarScan)):
            raise TypeError("epoch adapter accepts IMU, odometry, and LiDAR packets")
        _timestamp(packet.timestamp_ns)
        if isinstance(epoch_id, bool) or not isinstance(epoch_id, int) or epoch_id < 0:
            raise ValueError("epoch_id must be a non-negative integer")
        boundary = packet.timestamp_ns < self.start_timestamp_ns or (
            self.epoch_id > 0 and packet.timestamp_ns == self.start_timestamp_ns)
        if epoch_id != self.epoch_id or boundary or packet.entity_id != self.ekf.config.entity_id:
            self.rejected_epoch_packets += 1
            return False
        return True

    def update_motion(self, packet: ImuMeasurement | OdometryMeasurement, *, epoch_id: int) -> bool:
        if not isinstance(packet, (ImuMeasurement, OdometryMeasurement)):
            raise TypeError("update_motion requires IMU or odometry")
        return self.accept_packet(packet, epoch_id=epoch_id) and self.ekf.update(packet)
