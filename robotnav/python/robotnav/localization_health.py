"""Observable localization health checks, not calibrated confidence or safety.

No simulator truth is consumed. Repeated structures can produce plausible but
wrong matches and small EKF covariance; this monitor cannot certify correctness.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .control import RobotState
from .estimation import StateEstimate, _timestamp
from .localization import ScanMatchResult


@dataclass(frozen=True)
class LocalizationHealthConfig:
    require_map_matching: bool = True
    max_motion_age: float = 0.5
    max_scan_age: float = 0.5
    max_match_age: float = 2.0
    max_position_std: float = 1.0
    max_heading_std: float = 0.5
    max_correction_translation: float = 0.6
    max_correction_heading: float = 0.15
    failures_before_lost: int = 3
    successes_before_recovery: int = 2
    degraded_velocity_scale: float = 0.35

    def __post_init__(self) -> None:
        if not isinstance(self.require_map_matching, bool):
            raise ValueError("require_map_matching must be a boolean")
        for name in ("failures_before_lost", "successes_before_recovery"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        for name, value in self.__dict__.items():
            if name in {"require_map_matching", "failures_before_lost",
                        "successes_before_recovery"}:
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.degraded_velocity_scale > 1:
            raise ValueError("degraded_velocity_scale must be in (0, 1]")


@dataclass(frozen=True)
class MatchAssessment:
    accepted: bool
    reason: str
    translation: float
    heading: float


@dataclass(frozen=True)
class LocalizationHealth:
    timestamp_ns: int
    status: str
    reasons: tuple[str, ...]
    safe_stop: bool
    velocity_scale: float
    motion_age: float | None
    scan_age: float | None
    match_age: float | None
    position_std: float
    heading_std: float
    consecutive_failures: int
    consecutive_successes: int
    last_match_reason: str | None


class LocalizationHealthMonitor:
    """Arrival-time supervisor with capture-time match freshness and hysteresis.

    Assess a proposed correction *before* EKF fusion, then record whether it
    was actually fused. Older/equal match timestamps are ignored, not allowed
    to restore health. Evaluate at monotonic supervisor time. This monitor is
    deliberately separate from EKF event replay: past health decisions are not
    rewritten by later packets. One monitor represents one robot/map stream.
    """

    def __init__(self, config: LocalizationHealthConfig | None = None,
                 *, timestamp_ns: int = 0) -> None:
        self.config = config or LocalizationHealthConfig()
        if not isinstance(self.config, LocalizationHealthConfig):
            raise TypeError("config must be a LocalizationHealthConfig")
        self.reset(timestamp_ns=timestamp_ns)

    def reset(self, *, timestamp_ns: int = 0) -> None:
        _timestamp(timestamp_ns)
        self._start = self._now = timestamp_ns
        self._last_match: int | None = None
        self._last_success: int | None = None
        self._last_reason: str | None = None
        self._failures = self._successes = 0
        self._ready = not self.config.require_map_matching
        self._lost = False
        self.ignored_matches = 0

    @property
    def latest_timestamp_ns(self) -> int:
        """Most recent supervisor evaluation (or reset) time."""
        return self._now

    def assess_match(self, match: ScanMatchResult, hint: RobotState) -> MatchAssessment:
        values = (match.pose.x, match.pose.y, match.pose.theta, hint.x, hint.y, hint.theta)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("match pose and hint must be finite")
        translation = math.hypot(match.pose.x - hint.x, match.pose.y - hint.y)
        delta = match.pose.theta - hint.theta
        heading = abs(math.atan2(math.sin(delta), math.cos(delta)))
        reason = match.reason
        accepted = match.accepted
        if accepted and (translation > self.config.max_correction_translation or
                         heading > self.config.max_correction_heading):
            accepted, reason = False, "correction_jump"
        return MatchAssessment(accepted, reason, translation, heading)

    def record_match(self, timestamp_ns: int, *, accepted: bool, reason: str) -> bool:
        _timestamp(timestamp_ns)
        if not isinstance(accepted, bool) or not isinstance(reason, str) or not reason.strip():
            raise ValueError("match acceptance and reason are invalid")
        if timestamp_ns < self._start or (self._last_match is not None and
                                         timestamp_ns <= self._last_match):
            self.ignored_matches += 1
            return False
        self._last_match, self._last_reason = timestamp_ns, reason
        if accepted:
            # A long gap breaks the recovery streak even if evaluate was not
            # called during that gap (e.g. a batch of delayed corrections).
            if (self._last_success is None or timestamp_ns - self._last_success >
                    self.config.max_match_age * 1e9):
                self._successes = 0
            self._last_success = timestamp_ns
            self._failures = 0
            self._successes += 1
        else:
            self._failures += 1
            self._successes = 0
            if self._failures >= self.config.failures_before_lost:
                self._lost = True
        return True

    def evaluate(self, timestamp_ns: int, estimate: StateEstimate, *,
                 motion_timestamp_ns: int | None,
                 scan_timestamp_ns: int | None) -> LocalizationHealth:
        _timestamp(timestamp_ns)
        if timestamp_ns < self._now:
            raise ValueError("health evaluation time must be monotonic")
        # EKF query must be for this control tick, not an old low-covariance
        # checkpoint. Sensor timestamps may be slightly ahead due to jitter.
        if estimate.timestamp_ns != timestamp_ns:
            raise ValueError("estimate must be evaluated at supervisor time")
        for stamp in (motion_timestamp_ns, scan_timestamp_ns):
            if stamp is not None:
                _timestamp(stamp)
        p = estimate.covariance
        if len(p) < 5 or any(len(row) != len(p) for row in p):
            raise ValueError("estimate covariance must be square with at least five states")
        if not all(math.isfinite(value) for row in p for value in row):
            raise ValueError("estimate covariance must be finite")
        # Largest eigenvalue of the XY block, not only diagonal marginal stds.
        a, b, d = p[0][0], (p[0][1] + p[1][0]) / 2, p[1][1]
        if min(a, d, p[2][2]) < 0 or a * d - b * b < -1e-12:
            raise ValueError("pose covariance must be positive semidefinite")
        position_std = math.sqrt(max(0, (a + d + math.hypot(a - d, 2 * b)) / 2))
        heading_std = math.sqrt(p[2][2])

        def age(stamp: int | None) -> float | None:
            return None if stamp is None else max(0.0, (timestamp_ns - stamp) * 1e-9)

        motion_age, scan_age, match_age = (age(motion_timestamp_ns), age(scan_timestamp_ns),
                                         age(self._last_success))
        c = self.config
        reasons: list[str] = []
        if motion_age is None or motion_age > c.max_motion_age:
            reasons.append("motion_stale")
        if scan_age is None or scan_age > c.max_scan_age:
            reasons.append("scan_stale")
        if position_std > c.max_position_std:
            reasons.append("position_uncertain")
        if heading_std > c.max_heading_std:
            reasons.append("heading_uncertain")
        if c.require_map_matching:
            if (match_age is not None and match_age > c.max_match_age or
                    match_age is None and timestamp_ns - self._start > c.max_match_age * 1e9):
                reasons.append("map_correction_stale")
            if self._failures >= c.failures_before_lost:
                reasons.append("repeated_match_failure")
        if reasons:
            self._lost = True
            # A stale/uncertain tick interrupts recovery; two fresh successes
            # must subsequently be observed, not a cached streak from earlier.
            self._successes = 0
            status = "lost"
        elif c.require_map_matching and self._successes < c.successes_before_recovery:
            if self._lost:
                status = "lost"
                reasons.append("recovery_pending")
            elif not self._ready:
                status = "initializing"
                reasons.append("map_confirmation_pending")
            else:
                status = "degraded"
                reasons.append("match_failure" if self._failures else "recovery_pending")
        else:
            self._ready, self._lost = True, False
            status = "tracking"
        self._now = timestamp_ns
        safe_stop = status in {"lost", "initializing"}
        scale = 0.0 if safe_stop else c.degraded_velocity_scale if status == "degraded" else 1.0
        return LocalizationHealth(timestamp_ns, status, tuple(reasons), safe_stop, scale,
                                  motion_age, scan_age, match_age, position_std, heading_std,
                                  self._failures, self._successes, self._last_reason)
