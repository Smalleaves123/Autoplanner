"""Bounded, deterministic multi-hypothesis localization in a known 2-D grid.

This is an approximate grid search, not a proof of uniqueness or a particle
filter. It never reads LidarScan.pose and never resets an estimator implicitly.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

from .control import RobotState
from .localization import GridScanMatcher, ScanMatcherConfig, ScanMatchResult
from .sensors import GridSensorMap, LidarScan


@dataclass(frozen=True)
class RelocalizationConfig:
    translation_step: float = 1.0
    heading_bins: int = 16
    coarse_beams: int = 24
    max_coarse_poses: int = 50000
    max_refinements: int = 24
    max_hypotheses: int = 8
    distinct_translation: float = 1.0
    distinct_heading: float = 0.4
    ambiguity_score_margin: float = 0.02
    max_ray_score: float = 0.08
    residual_cap: float = 2.0

    def __post_init__(self) -> None:
        for name in ("heading_bins", "coarse_beams", "max_coarse_poses",
                     "max_refinements", "max_hypotheses"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 2:
                raise ValueError(f"{name} must be an integer >= 2")
        for name in ("translation_step", "distinct_translation", "distinct_heading",
                     "ambiguity_score_margin", "max_ray_score", "residual_cap"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class RelocalizationHypothesis:
    pose: RobotState
    ray_score: float
    match: ScanMatchResult


@dataclass(frozen=True)
class RelocalizationResult:
    accepted: bool
    reason: str
    # None for ambiguity/failure: no silently selected winning pose.
    pose: RobotState | None
    hypotheses: tuple[RelocalizationHypothesis, ...]
    coarse_poses: int
    refinements: int
    score_gap: float | None


class GridRelocalizer:
    """Rank grid seeds by static-ray consistency, then refine multiple modes.

    No-hit rays are negative evidence; dynamic-labeled beams are ignored.
    Distinct refined modes within the score margin are reported as ambiguous.
    Ray scores are capped squared range errors, not probabilities. Large maps
    exceeding the configured grid budget are rejected without partial search.
    """

    def __init__(self, occupancy_map: GridSensorMap | str | Path | Sequence[str],
                 config: RelocalizationConfig | None = None,
                 matcher_config: ScanMatcherConfig | None = None) -> None:
        self.config = config or RelocalizationConfig()
        if not isinstance(self.config, RelocalizationConfig):
            raise TypeError("config must be a RelocalizationConfig")
        self.matcher = GridScanMatcher(occupancy_map, matcher_config or ScanMatcherConfig(
            translation_window=max(0.75, self.config.translation_step / math.sqrt(2)),
            rotation_window=max(0.2, math.pi / self.config.heading_bins),
            max_points=31))
        self.map = self.matcher.map

    def _range(self, pose: RobotState, angle: float, limit: float) -> float:
        """Exact grid DDA ray distance, bounded by the observed ray range."""
        dx, dy = math.cos(pose.theta + angle), math.sin(pose.theta + angle)
        cx, cy = math.floor(pose.x), math.floor(pose.y)
        sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        tx = ((cx + 1 - pose.x) / dx if dx > 0 else
              (cx - pose.x) / dx if dx < 0 else float("inf"))
        ty = ((cy + 1 - pose.y) / dy if dy > 0 else
              (cy - pose.y) / dy if dy < 0 else float("inf"))
        dtx, dty = (1 / abs(dx) if dx else float("inf")), (1 / abs(dy) if dy else float("inf"))
        while min(tx, ty) <= limit:
            distance = min(tx, ty)
            if tx < ty:
                cx, tx = cx + sx, tx + dtx
            elif ty < tx:
                cy, ty = cy + sy, ty + dty
            else:
                # Corner ties conservatively include either adjacent wall.
                if self.map.occupied(cx + sx + 0.5, cy + 0.5) or self.map.occupied(
                        cx + 0.5, cy + sy + 0.5):
                    return distance
                cx, cy, tx, ty = cx + sx, cy + sy, tx + dtx, ty + dty
            if self.map.occupied(cx + 0.5, cy + 0.5):
                return distance
        return limit

    def _score(self, rays: list[tuple[float, float, bool]], pose: RobotState) -> float:
        cap = self.config.residual_cap
        return sum(min(cap * cap, (distance - self._range(
            pose, angle, distance + cap if hit else distance)) ** 2)
                   for angle, distance, hit in rays) / len(rays)

    def _distinct(self, a: RobotState, b: RobotState) -> bool:
        delta = a.theta - b.theta
        return (math.hypot(a.x - b.x, a.y - b.y) > self.config.distinct_translation or
                abs(math.atan2(math.sin(delta), math.cos(delta))) > self.config.distinct_heading)

    def localize(self, scan: LidarScan) -> RelocalizationResult:
        # Reuse local matcher's validation and semantic point support check.
        points = self.matcher._points(scan)
        if len(points) < self.matcher.config.min_points:
            return RelocalizationResult(False, "insufficient_points", None, (), 0, 0, None)
        rays = [(a, d, hit) for a, d, hit, kind in zip(
            scan.angles, scan.ranges, scan.hits, scan._beam_kinds()) if kind != "dynamic"]
        coarse_rays = rays
        if len(rays) > self.config.coarse_beams:
            coarse_rays = [rays[round(i * (len(rays) - 1) / (self.config.coarse_beams - 1))]
                           for i in range(self.config.coarse_beams)]
        width, height = len(self.map.rows[0]), len(self.map.rows)
        step = self.config.translation_step
        nx, ny = (width - 0.5) / step, (height - 0.5) / step
        # Check the complete lattice upper bound before allocating seeds,
        # including extremely small positive user-specified spacing.
        if not math.isfinite(nx) or not math.isfinite(ny):
            return RelocalizationResult(False, "search_budget_exceeded", None, (),
                                        self.config.max_coarse_poses + 1, 0, None)
        nx, ny = math.ceil(nx), math.ceil(ny)
        lattice_count = nx * ny * self.config.heading_bins
        if lattice_count > self.config.max_coarse_poses:
            return RelocalizationResult(False, "search_budget_exceeded", None, (),
                                        lattice_count, 0, None)
        seeds = [RobotState(0.5 + ix * step, 0.5 + iy * step)
                 for iy in range(ny)
                 for ix in range(nx)
                 if not self.map.occupied(0.5 + ix * step, 0.5 + iy * step)]
        count = len(seeds) * self.config.heading_bins
        ranked = []
        for seed in seeds:
            for i in range(self.config.heading_bins):
                pose = RobotState(seed.x, seed.y, -math.pi + 2 * math.pi * i / self.config.heading_bins)
                ranked.append((self._score(coarse_rays, pose), pose))
        ranked.sort(key=lambda pair: (pair[0], pair[1].x, pair[1].y, pair[1].theta))
        selected: list[RobotState] = []
        for _, pose in ranked:
            if all(self._distinct(pose, other) for other in selected):
                selected.append(pose)
            if len(selected) == self.config.max_refinements:
                break
        modes: list[RelocalizationHypothesis] = []
        for pose in selected:
            match = self.matcher.match(scan, pose)
            if not match.accepted:
                continue
            normalized = RobotState(match.pose.x, match.pose.y,
                                   math.atan2(math.sin(match.pose.theta), math.cos(match.pose.theta)))
            score = self._score(rays, normalized)
            # Retain near-threshold alternatives for ambiguity checking;
            # a competitor just above the acceptance cutoff is still evidence
            # against selecting a barely acceptable winning mode.
            if score <= self.config.max_ray_score + self.config.ambiguity_score_margin:
                modes.append(RelocalizationHypothesis(normalized, score, match))
        modes.sort(key=lambda item: (item.ray_score, item.pose.x, item.pose.y, item.pose.theta))
        distinct: list[RelocalizationHypothesis] = []
        for mode in modes:
            if all(self._distinct(mode.pose, other.pose) for other in distinct):
                distinct.append(mode)
        hypotheses = tuple(distinct[:self.config.max_hypotheses])
        gap = distinct[1].ray_score - distinct[0].ray_score if len(distinct) > 1 else None
        if not distinct or distinct[0].ray_score > self.config.max_ray_score:
            return RelocalizationResult(False, "no_match", None, (), count, len(selected), None)
        if gap is not None and gap <= self.config.ambiguity_score_margin:
            return RelocalizationResult(False, "ambiguous", None, hypotheses, count, len(selected), gap)
        return RelocalizationResult(True, "localized", distinct[0].pose, hypotheses,
                                    count, len(selected), gap)
