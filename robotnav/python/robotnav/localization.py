"""Local 2-D scan-to-static-map matching, without simulator-pose access.

An initial correlative search is refined by point-to-line ICP against exposed
grid-cell boundaries. Sparse, poor, and geometrically degenerate matches are
rejected. This requires a known map and a nearby pose hint; it is not SLAM or
global relocalization. Only the Python standard library is required.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
from typing import Sequence

from .control import RobotState
from .sensors import GridSensorMap, LidarScan


@dataclass(frozen=True)
class ScanMatcherConfig:
    translation_window: float = 0.75
    rotation_window: float = 0.2
    max_correspondence_distance: float = 1.0
    robust_scale: float = 0.2
    max_rmse: float = 0.2
    min_inlier_ratio: float = 0.6
    min_information_ratio: float = 0.01
    min_points: int = 12
    max_points: int = 61
    search_levels: int = 3
    iterations: int = 10

    def __post_init__(self) -> None:
        for name in ("min_points", "max_points", "search_levels", "iterations"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.max_points < self.min_points:
            raise ValueError("max_points must be at least min_points")
        if self.min_points < 3:
            raise ValueError("min_points must be at least 3 for planar registration")
        for name in ("translation_window", "rotation_window",
                     "max_correspondence_distance", "robust_scale", "max_rmse"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("min_inlier_ratio", "min_information_ratio"):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError(f"{name} must be in (0, 1]")


@dataclass(frozen=True)
class ScanMatchResult:
    accepted: bool
    reason: str
    pose: RobotState
    initial_rmse: float | None
    rmse: float | None
    inliers: int
    points: int
    information_ratio: float


def _solve(matrix: list[list[float]], values: list[float]) -> list[float] | None:
    rows = [row[:] + [value] for row, value in zip(matrix, values)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda index: abs(rows[index][column]))
        if abs(rows[pivot][column]) < 1e-10:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]
        scale = rows[column][column]
        rows[column] = [value / scale for value in rows[column]]
        for index in range(3):
            if index == column:
                continue
            scale = rows[index][column]
            rows[index] = [a - scale * b for a, b in zip(rows[index], rows[column])]
    return [row[3] for row in rows]


def _information_ratio(matrix: list[list[float]]) -> float:
    diagonal = [matrix[index][index] for index in range(3)]
    if min(diagonal) <= 1e-10:
        return 0.0
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    return max(0.0, min(1.0, determinant / math.prod(diagonal)))


class GridScanMatcher:
    """Deterministic local registration to the fixed, known static map.

    Dynamic-labeled returns and no-hit beams are excluded. Unknown-labeled
    hit beams are robustly scored; reliable semantic labels are optional.
    ``match`` never uses ``scan.pose`` or absolute scan endpoints.
    """

    def __init__(self, occupancy_map: GridSensorMap | str | Path | Sequence[str],
                 config: ScanMatcherConfig | None = None) -> None:
        self.map = (occupancy_map if isinstance(occupancy_map, GridSensorMap)
                    else GridSensorMap(occupancy_map))
        self.config = config or ScanMatcherConfig()
        if not isinstance(self.config, ScanMatcherConfig):
            raise TypeError("config must be a ScanMatcherConfig")
        # One segment for every free/occupied boundary, including map edges.
        self._segments: dict[tuple[int, int], list[tuple[float, ...]]] = {}
        for y, row in enumerate(self.map.rows):
            for x, value in enumerate(row):
                if value != "0":
                    continue
                edges = ((-1, 0, x, y, x, y + 1),
                         (1, 0, x + 1, y, x + 1, y + 1),
                         (0, -1, x, y, x + 1, y),
                         (0, 1, x, y + 1, x + 1, y + 1))
                for dx, dy, ax, ay, bx, by in edges:
                    if self.map.occupied(x + 0.5 + dx, y + 0.5 + dy):
                        key = (math.floor((ax + bx) / 2), math.floor((ay + by) / 2))
                        self._segments.setdefault(key, []).append(
                            (ax, ay, bx, by, float(-dx), float(-dy)))
        self._radius = math.ceil(self.config.max_correspondence_distance) + 1

    def _visible(self, ox: float, oy: float, x: float, y: float) -> bool:
        """Grid DDA free-space check up to, but not beyond, the wall surface."""
        if self.map.occupied(ox, oy):
            return False
        dx, dy = x - ox, y - oy
        cx, cy = math.floor(ox), math.floor(oy)
        sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        tx = ((cx + 1 - ox) / dx if dx > 0 else
              (cx - ox) / dx if dx < 0 else float("inf"))
        ty = ((cy + 1 - oy) / dy if dy > 0 else
              (cy - oy) / dy if dy < 0 else float("inf"))
        dtx = 1 / abs(dx) if dx else float("inf")
        dty = 1 / abs(dy) if dy else float("inf")
        while min(tx, ty) < 1.0 - 1e-9:
            if tx < ty:
                cx, tx = cx + sx, tx + dtx
            elif ty < tx:
                cy, ty = cy + sy, ty + dty
            else:
                cx, cy, tx, ty = cx + sx, cy + sy, tx + dtx, ty + dty
            if self.map.occupied(cx + 0.5, cy + 0.5):
                return False
        return True

    def _nearest(self, x: float, y: float, ox: float, oy: float
                 ) -> tuple[float, float, float, float, float] | None:
        best = None
        best_distance = self.config.max_correspondence_distance ** 2
        cx, cy = math.floor(x), math.floor(y)
        for iy in range(cy - self._radius, cy + self._radius + 1):
            for ix in range(cx - self._radius, cx + self._radius + 1):
                for ax, ay, bx, by, nx, ny in self._segments.get((ix, iy), ()):
                    qx = max(ax, min(bx, x))
                    qy = max(ay, min(by, y))
                    # Normals point into free space: a scan cannot see the
                    # back side of a wall or a surface behind another wall.
                    if nx * (ox - qx) + ny * (oy - qy) < 0:
                        continue
                    distance = (x - qx) ** 2 + (y - qy) ** 2
                    if distance < best_distance and self._visible(ox, oy, qx, qy):
                        best_distance = distance
                        best = (qx, qy, nx, ny, math.sqrt(distance))
        return best

    def _evaluate(self, points: list[tuple[float, float]], pose: RobotState
                  ) -> tuple[float, float | None, list[tuple[float, list[float], float]]]:
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        matches = []
        score = 0.0
        squared = 0.0
        cap = self.config.max_correspondence_distance ** 2
        for lx, ly in points:
            rx, ry = c * lx - s * ly, s * lx + c * ly
            x, y = pose.x + rx, pose.y + ry
            nearest = self._nearest(x, y, pose.x, pose.y)
            if nearest is None:
                score += cap
                continue
            qx, qy, nx, ny, distance = nearest
            residual = nx * (x - qx) + ny * (y - qy)
            weight = 1 / (1 + (distance / self.config.robust_scale) ** 2)
            matches.append((residual, [nx, ny, -nx * ry + ny * rx], weight))
            squared += distance ** 2
            score += min(distance ** 2, cap)
        return score / len(points), (math.sqrt(squared / len(matches))
                                    if matches else None), matches

    @staticmethod
    def _normal_equations(matches: list[tuple[float, list[float], float]]
                          ) -> tuple[list[list[float]], list[float]]:
        matrix = [[0.0] * 3 for _ in range(3)]
        gradient = [0.0] * 3
        for residual, jacobian, weight in matches:
            for i in range(3):
                gradient[i] += weight * jacobian[i] * residual
                for j in range(3):
                    matrix[i][j] += weight * jacobian[i] * jacobian[j]
        return matrix, gradient

    def match(self, scan: LidarScan, hint: RobotState) -> ScanMatchResult:
        if not all(math.isfinite(value) for value in hint.__dict__.values()):
            raise ValueError("pose hint must be finite")
        if not (len(scan.angles) == len(scan.ranges) == len(scan.hits)):
            raise ValueError("scan arrays must have equal lengths")
        if scan.kinds and len(scan.kinds) != len(scan.ranges):
            raise ValueError("scan kinds must have one entry per beam")
        if not all(math.isfinite(angle) and math.isfinite(distance) and distance >= 0
                   for angle, distance in zip(scan.angles, scan.ranges)):
            raise ValueError("scan geometry must be finite with non-negative ranges")
        points = [(distance * math.cos(angle), distance * math.sin(angle))
                  for angle, distance, hit, kind in zip(
                      scan.angles, scan.ranges, scan.hits, scan._beam_kinds())
                  if hit and kind != "dynamic"]
        if len(points) > self.config.max_points:
            points = [points[round(index * (len(points) - 1) / (self.config.max_points - 1))]
                      for index in range(self.config.max_points)]
        if len(points) < self.config.min_points:
            return ScanMatchResult(False, "insufficient_points", hint, None, None,
                                   0, len(points), 0.0)
        score, initial_rmse, matches = self._evaluate(points, hint)
        best = hint

        def candidate(dx: float, dy: float, dtheta: float) -> RobotState | None:
            x = max(hint.x - self.config.translation_window,
                    min(hint.x + self.config.translation_window, dx))
            y = max(hint.y - self.config.translation_window,
                    min(hint.y + self.config.translation_window, dy))
            theta = max(hint.theta - self.config.rotation_window,
                        min(hint.theta + self.config.rotation_window, dtheta))
            if self.map.occupied(x, y):
                return None
            return replace(hint, x=x, y=y, theta=theta)

        for level in range(self.config.search_levels):
            translation = self.config.translation_window / (2 ** level)
            rotation = self.config.rotation_window / (2 ** level)
            center = best
            for ix in (-1, 0, 1):
                for iy in (-1, 0, 1):
                    for it in (-1, 0, 1):
                        pose = candidate(center.x + ix * translation,
                                         center.y + iy * translation,
                                         center.theta + it * rotation)
                        if pose is None:
                            continue
                        cost, _, _ = self._evaluate(points, pose)
                        if cost < score - 1e-12:
                            score, best = cost, pose
        for _ in range(self.config.iterations):
            _, _, matches = self._evaluate(points, best)
            matrix, gradient = self._normal_equations(matches)
            step = _solve(matrix, [-value for value in gradient])
            if step is None or max(abs(value) for value in step) < 1e-6:
                break
            improved = False
            for scale in (1.0, 0.5, 0.25, 0.125):
                pose = candidate(best.x + scale * step[0], best.y + scale * step[1],
                                 best.theta + scale * step[2])
                if pose is None:
                    continue
                cost, _, _ = self._evaluate(points, pose)
                if cost < score - 1e-12:
                    score, best, improved = cost, pose, True
                    break
            if not improved:
                break
        _, rmse, matches = self._evaluate(points, best)
        matrix, _ = self._normal_equations(matches)
        information = _information_ratio(matrix)
        reason = "matched"
        if (len(matches) < self.config.min_points or
                len(matches) / len(points) < self.config.min_inlier_ratio):
            reason = "insufficient_inliers"
        elif rmse is None or rmse > self.config.max_rmse:
            reason = "poor_fit"
        elif information < self.config.min_information_ratio:
            reason = "degenerate_geometry"
        elif self.map.occupied(best.x, best.y):
            reason = "occupied_pose"
        return ScanMatchResult(reason == "matched", reason, best, initial_rmse,
                               rmse, len(matches), len(points), information)
