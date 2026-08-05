"""ROS-free perception, mapping, tracking, and incremental replanning tools.

The module deliberately uses plain Python data structures so that a simulated
sensor can be replaced by a CSV/JSON producer or a small Python adapter.  A
typical loop is:

    sensor frame -> occupancy update -> dynamic detection/tracking
                 -> distance field -> incremental D* Lite replan

Coordinates are map-cell coordinates.  Sensor points are absolute map
coordinates and ``hit=False`` means that the beam reached its maximum range
without observing an obstacle.
"""

from __future__ import annotations

import csv
import heapq
import json
import math
import random
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence


Cell = tuple[int, int]
Grid = list[list[int]]
DynamicMapUpdate = tuple[int, int, int, bool]


def validate_grid(grid: Sequence[Sequence[int]]) -> Grid:
    """Return a copied rectangular binary grid or raise ``ValueError``."""
    rows = [list(row) for row in grid]
    if not rows or not rows[0] or any(len(row) != len(rows[0]) for row in rows):
        raise ValueError("grid must contain non-empty rectangular rows")
    if any(cell not in (0, 1, False, True) for row in rows for cell in row):
        raise ValueError("grid cells must be binary")
    return [[int(cell) for cell in row] for row in rows]


def load_grid(path: Path) -> Grid:
    rows = [line.strip() for line in path.read_text().splitlines() if line.strip()]
    if not rows or any(len(row) != len(rows[0]) for row in rows):
        raise ValueError(f"map must contain non-empty rectangular rows: {path}")
    if any(cell not in "01.#@" for row in rows for cell in row):
        raise ValueError(f"map contains an unsupported cell: {path}")
    return [[1 if cell in "1#@" else 0 for cell in row] for row in rows]


def save_grid(path: Path, grid: Sequence[Sequence[int]]) -> None:
    checked = validate_grid(grid)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join("".join(str(cell) for cell in row)
                             for row in checked) + "\n")


def in_bounds(grid: Sequence[Sequence[int]], cell: Cell) -> bool:
    x, y = cell
    return 0 <= y < len(grid) and 0 <= x < len(grid[0])


@dataclass(frozen=True)
class SensorPoint:
    x: float
    y: float
    hit: bool = True
    kind: str = "unknown"


@dataclass(frozen=True)
class SensorFrame:
    frame: int
    sensor_x: float
    sensor_y: float
    points: tuple[SensorPoint, ...]
    sensor_theta: float = 0.0


@dataclass(frozen=True)
class SimulatedObstacle:
    obstacle_id: str
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    radius: float = 0.5

    def __post_init__(self) -> None:
        if self.radius < 0.0 or not math.isfinite(self.radius):
            raise ValueError("simulated obstacle radius must be finite and non-negative")

    def position(self, frame: int) -> tuple[float, float]:
        return self.x + frame * self.vx, self.y + frame * self.vy


@dataclass(frozen=True)
class LidarConfig:
    beams: int = 72
    max_range: float = 15.0
    ray_step: float = 0.25
    noise_std: float = 0.0
    seed: int = 0

    def __post_init__(self) -> None:
        if self.beams < 4 or self.max_range <= 0.0 or self.ray_step <= 0.0:
            raise ValueError("invalid lidar configuration")
        if self.noise_std < 0.0 or not math.isfinite(self.noise_std):
            raise ValueError("lidar noise must be finite and non-negative")


class LidarSimulator:
    """Generate deterministic 2-D lidar endpoint observations."""

    def __init__(self, static_grid: Sequence[Sequence[int]],
                 obstacles: Iterable[SimulatedObstacle] = (),
                 config: LidarConfig | None = None) -> None:
        self.static_grid = validate_grid(static_grid)
        self.obstacles = tuple(obstacles)
        self.config = config or LidarConfig()
        self._random = random.Random(self.config.seed)

    def _static_hit(self, x: float, y: float) -> bool:
        cell = (math.floor(x), math.floor(y))
        return not in_bounds(self.static_grid, cell) or bool(
            self.static_grid[cell[1]][cell[0]])

    def _dynamic_hit(self, x: float, y: float, frame: int) -> bool:
        for obstacle in self.obstacles:
            ox, oy = obstacle.position(frame)
            if math.hypot(x - ox, y - oy) <= obstacle.radius:
                return True
        return False

    def _dynamic_kind(self, x: float, y: float, frame: int) -> str:
        for obstacle in self.obstacles:
            ox, oy = obstacle.position(frame)
            if math.hypot(x - ox, y - oy) <= obstacle.radius:
                return "dynamic"
        return "static"

    def scan(self, frame: int, sensor_x: float, sensor_y: float,
             sensor_theta: float = 0.0) -> SensorFrame:
        points: list[SensorPoint] = []
        for beam in range(self.config.beams):
            angle = (sensor_theta + 2.0 * math.pi * beam / self.config.beams)
            endpoint_x = sensor_x + self.config.max_range * math.cos(angle)
            endpoint_y = sensor_y + self.config.max_range * math.sin(angle)
            hit = False
            kind = "unknown"
            distance = self.config.ray_step
            while distance <= self.config.max_range:
                x = sensor_x + distance * math.cos(angle)
                y = sensor_y + distance * math.sin(angle)
                if self._static_hit(x, y) or self._dynamic_hit(x, y, frame):
                    endpoint_x, endpoint_y = x, y
                    hit = True
                    kind = self._dynamic_kind(x, y, frame)
                    break
                distance += self.config.ray_step
            if self.config.noise_std > 0.0:
                endpoint_x += self._random.gauss(0.0, self.config.noise_std)
                endpoint_y += self._random.gauss(0.0, self.config.noise_std)
            points.append(SensorPoint(endpoint_x, endpoint_y, hit, kind))
        return SensorFrame(frame, sensor_x, sensor_y, tuple(points), sensor_theta)


class OccupancyGrid:
    """Online log-odds occupancy grid with an explicit unknown state."""

    def __init__(self, width: int, height: int, *, known_threshold: float = 0.5,
                 free_update: float = -0.7, occupied_update: float = 1.2,
                 prior_strength: float = 0.0) -> None:
        if width <= 0 or height <= 0 or known_threshold <= 0.0:
            raise ValueError("invalid occupancy-grid dimensions or threshold")
        self.width = width
        self.height = height
        self.known_threshold = known_threshold
        self.free_update = free_update
        self.occupied_update = occupied_update
        self.log_odds = [[0.0 for _ in range(width)] for _ in range(height)]
        self.observed = [[False for _ in range(width)] for _ in range(height)]
        if prior_strength > 0.0:
            self.prior_strength = prior_strength
        else:
            self.prior_strength = 0.0

    @classmethod
    def from_grid(cls, grid: Sequence[Sequence[int]], *,
                  prior_strength: float = 1.0) -> "OccupancyGrid":
        checked = validate_grid(grid)
        result = cls(len(checked[0]), len(checked),
                     prior_strength=prior_strength)
        for y, row in enumerate(checked):
            for x, occupied in enumerate(row):
                result.observed[y][x] = True
                result.log_odds[y][x] = (
                    prior_strength if occupied else -prior_strength)
        return result

    def _cell_state(self, x: int, y: int, unknown_policy: str) -> int:
        if self.log_odds[y][x] >= self.known_threshold:
            return 1
        if not self.observed[y][x]:
            if unknown_policy == "occupied":
                return 1
            if unknown_policy == "free":
                return 0
            raise ValueError("unknown policy must be 'occupied' or 'free'")
        return 0

    def to_grid(self, unknown_policy: str = "occupied") -> Grid:
        return [[self._cell_state(x, y, unknown_policy)
                 for x in range(self.width)] for y in range(self.height)]

    def unknown_cells(self) -> list[Cell]:
        return [(x, y) for y in range(self.height) for x in range(self.width)
                if not self.observed[y][x]]

    def _update_cell(self, cell: Cell, occupied: bool) -> None:
        x, y = cell
        if not (0 <= x < self.width and 0 <= y < self.height):
            return
        delta = self.occupied_update if occupied else self.free_update
        self.log_odds[y][x] = max(-6.0, min(6.0, self.log_odds[y][x] + delta))
        self.observed[y][x] = True

    @staticmethod
    def _ray_cells(origin: tuple[float, float], endpoint: tuple[float, float]) -> list[Cell]:
        ox, oy = origin
        ex, ey = endpoint
        steps = max(1, int(math.ceil(max(abs(ex - ox), abs(ey - oy)) * 4.0)))
        cells: list[Cell] = []
        for index in range(steps + 1):
            ratio = index / steps
            cell = (math.floor(ox + ratio * (ex - ox)),
                    math.floor(oy + ratio * (ey - oy)))
            if not cells or cells[-1] != cell:
                cells.append(cell)
        return cells

    def integrate_scan(self, scan: SensorFrame,
                       unknown_policy: str = "occupied") -> set[Cell]:
        before = [row[:] for row in self.to_grid(unknown_policy)]
        observed_before = [row[:] for row in self.observed]
        origin = (scan.sensor_x, scan.sensor_y)
        for point in scan.points:
            cells = self._ray_cells(origin, (point.x, point.y))
            if point.hit and cells:
                for cell in cells[:-1]:
                    self._update_cell(cell, False)
                self._update_cell(cells[-1], True)
            else:
                for cell in cells:
                    self._update_cell(cell, False)
        after = self.to_grid(unknown_policy)
        changed = {
            (x, y)
            for y in range(self.height)
            for x in range(self.width)
            if before[y][x] != after[y][x] or
            observed_before[y][x] != self.observed[y][x]
        }
        # Newly observed free cells are useful diagnostics even when the
        # planner's unknown policy already treated them as free.
        for y in range(self.height):
            for x in range(self.width):
                if before[y][x] != after[y][x]:
                    changed.add((x, y))
        return changed


@dataclass(frozen=True)
class DetectedObstacle:
    frame: int
    x: float
    y: float
    radius: float
    point_count: int


def _fit_circle_center(points: Sequence[SensorPoint]) -> tuple[float, float] | None:
    """Estimate a circular obstacle center from a visible hit-point arc."""
    if len(points) < 3:
        return None

    # Solve x^2 + y^2 = 2*cx*x + 2*cy*y + c using normal equations.
    matrix = [[0.0 for _ in range(4)] for _ in range(3)]
    for point in points:
        row = (2.0 * point.x, 2.0 * point.y, 1.0)
        value = point.x * point.x + point.y * point.y
        for row_index in range(3):
            for column in range(3):
                matrix[row_index][column] += row[row_index] * row[column]
            matrix[row_index][3] += row[row_index] * value

    for pivot in range(3):
        pivot_row = max(
            range(pivot, 3),
            key=lambda row_index: abs(matrix[row_index][pivot]))
        if abs(matrix[pivot_row][pivot]) <= 1e-9:
            return None
        matrix[pivot], matrix[pivot_row] = matrix[pivot_row], matrix[pivot]
        pivot_value = matrix[pivot][pivot]
        for column in range(pivot, 4):
            matrix[pivot][column] /= pivot_value
        for row_index in range(3):
            if row_index == pivot:
                continue
            factor = matrix[row_index][pivot]
            for column in range(pivot, 4):
                matrix[row_index][column] -= factor * matrix[pivot][column]

    center_x, center_y = matrix[0][3], matrix[1][3]
    circle_constant = matrix[2][3]
    radius_squared = center_x * center_x + center_y * center_y + circle_constant
    if radius_squared <= 0.0 or not all(
            math.isfinite(value)
            for value in (center_x, center_y, radius_squared)):
        return None
    radius = math.sqrt(radius_squared)
    residual = math.sqrt(sum(
        (math.hypot(point.x - center_x, point.y - center_y) - radius) ** 2
        for point in points) / len(points))
    if residual > max(0.25, radius * 0.5):
        return None
    return center_x, center_y


def detect_dynamic_obstacles(scan: SensorFrame,
                             static_grid: Sequence[Sequence[int]],
                             cluster_distance: float = 1.5) -> list[DetectedObstacle]:
    """Cluster hit points that do not coincide with known static obstacles."""
    grid = validate_grid(static_grid)
    candidates = [
        point for point in scan.points
        if point.hit and in_bounds(grid, (math.floor(point.x), math.floor(point.y))
                                   ) and not grid[math.floor(point.y)][math.floor(point.x)]
        and point.kind != "static"
    ]
    clusters: list[list[SensorPoint]] = []
    for point in candidates:
        for cluster in clusters:
            cx = sum(item.x for item in cluster) / len(cluster)
            cy = sum(item.y for item in cluster) / len(cluster)
            if math.hypot(point.x - cx, point.y - cy) <= cluster_distance:
                cluster.append(point)
                break
        else:
            clusters.append([point])
    detections: list[DetectedObstacle] = []
    for cluster in clusters:
        center = _fit_circle_center(cluster)
        if center is None:
            x = sum(point.x for point in cluster) / len(cluster)
            y = sum(point.y for point in cluster) / len(cluster)
        else:
            x, y = center
        radius = max(0.25, max(math.hypot(point.x - x, point.y - y)
                               for point in cluster))
        detections.append(DetectedObstacle(scan.frame, x, y, radius,
                                           len(cluster)))
    return detections


@dataclass
class ObstacleTrack:
    track_id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    radius: float = 0.5
    age: int = 1
    hits: int = 1
    missed: int = 0
    last_frame: int = 0
    history: list[tuple[int, float, float]] | None = None

    def __post_init__(self) -> None:
        if self.history is None:
            self.history = [(self.last_frame, self.x, self.y)]

    def predict(self, frame: int) -> tuple[float, float]:
        delta = frame - self.last_frame
        return self.x + self.vx * delta, self.y + self.vy * delta

    def as_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["history"] = [list(item) for item in self.history or []]
        return data

    def snapshot(self) -> "ObstacleTrack":
        """Return an immutable-in-practice frame snapshot of this track."""
        return ObstacleTrack(
            self.track_id, self.x, self.y, self.vx, self.vy, self.radius,
            self.age, self.hits, self.missed, self.last_frame,
            list(self.history or []),
        )


class ObstacleTracker:
    """Nearest-neighbour constant-velocity tracker for dynamic clusters."""

    def __init__(self, association_distance: float = 3.0,
                 max_missed: int = 3, position_alpha: float = 0.65,
                 velocity_alpha: float = 0.5) -> None:
        self.association_distance = association_distance
        self.max_missed = max_missed
        self.position_alpha = position_alpha
        self.velocity_alpha = velocity_alpha
        self._next_id = 1
        self.tracks: dict[int, ObstacleTrack] = {}

    def update(self, detections: Sequence[DetectedObstacle],
               frame: int) -> list[ObstacleTrack]:
        unmatched_tracks = set(self.tracks)
        matched: set[int] = set()
        for detection in detections:
            candidates = []
            for track_id in unmatched_tracks:
                track = self.tracks[track_id]
                px, py = track.predict(frame)
                candidates.append((math.hypot(detection.x - px,
                                              detection.y - py), track_id))
            candidates.sort()
            if candidates and candidates[0][0] <= self.association_distance:
                track_id = candidates[0][1]
                track = self.tracks[track_id]
                previous_x, previous_y = track.x, track.y
                dt = max(1, frame - track.last_frame)
                measured_vx = (detection.x - previous_x) / dt
                measured_vy = (detection.y - previous_y) / dt
                track.x = (self.position_alpha * detection.x +
                           (1.0 - self.position_alpha) * track.predict(frame)[0])
                track.y = (self.position_alpha * detection.y +
                           (1.0 - self.position_alpha) * track.predict(frame)[1])
                track.vx = (self.velocity_alpha * measured_vx +
                            (1.0 - self.velocity_alpha) * track.vx)
                track.vy = (self.velocity_alpha * measured_vy +
                            (1.0 - self.velocity_alpha) * track.vy)
                track.radius = max(track.radius, detection.radius)
                track.age += 1
                track.hits += 1
                track.missed = 0
                track.last_frame = frame
                track.history = (track.history or []) + [(frame, track.x, track.y)]
                unmatched_tracks.remove(track_id)
                matched.add(track_id)
            else:
                track_id = self._next_id
                self._next_id += 1
                self.tracks[track_id] = ObstacleTrack(
                    track_id, detection.x, detection.y,
                    radius=detection.radius, last_frame=frame)
                matched.add(track_id)
        for track_id in list(unmatched_tracks):
            track = self.tracks[track_id]
            track.missed += 1
            track.age += 1
            if track.missed > self.max_missed:
                del self.tracks[track_id]
        return [self.tracks[track_id].snapshot()
                for track_id in sorted(self.tracks)
                if track_id in matched or
                self.tracks[track_id].missed <= self.max_missed]

    def predictions(self, frames_ahead: int = 5) -> list[dict[str, Any]]:
        return [{
            "track_id": track.track_id,
            "x": track.predict(track.last_frame + frames_ahead)[0],
            "y": track.predict(track.last_frame + frames_ahead)[1],
            "vx": track.vx,
            "vy": track.vy,
            "radius": track.radius,
        } for track in self.tracks.values()]


def compute_distance_field(grid: Sequence[Sequence[int]]) -> list[list[float]]:
    """Distance to the nearest occupied cell; occupied cells are zero."""
    checked = validate_grid(grid)
    height, width = len(checked), len(checked[0])
    infinity = float("inf")
    distance = [[infinity for _ in range(width)] for _ in range(height)]
    queue: list[tuple[float, int, int]] = []
    for y, row in enumerate(checked):
        for x, occupied in enumerate(row):
            if occupied:
                distance[y][x] = 0.0
                heapq.heappush(queue, (0.0, x, y))
    directions = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0),
                  (0, -1, 1.0), (1, 1, math.sqrt(2.0)),
                  (1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
                  (-1, -1, math.sqrt(2.0)))
    while queue:
        current, x, y = heapq.heappop(queue)
        if current != distance[y][x]:
            continue
        for dx, dy, cost in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                candidate = current + cost
                if candidate < distance[ny][nx]:
                    distance[ny][nx] = candidate
                    heapq.heappush(queue, (candidate, nx, ny))
    return distance


def path_clearance(distance_field: Sequence[Sequence[float]],
                   path: Sequence[Cell], robot_radius: float = 0.0) -> float:
    if not path:
        return 0.0
    values = [distance_field[y][x] - robot_radius for x, y in path]
    return min(values)


@dataclass(frozen=True)
class ReplanResult:
    path: tuple[Cell, ...]
    replanned: bool
    changed_cells: tuple[Cell, ...]
    expanded_nodes: int
    planning_time_ms: float


class IncrementalDStarLite:
    """Small Python D* Lite implementation for sensor-loop experiments."""

    def __init__(self, allow_diagonal: bool = True) -> None:
        self.allow_diagonal = allow_diagonal
        self.grid: Grid | None = None
        self.start: Cell | None = None
        self.goal: Cell | None = None
        self.last_start: Cell | None = None
        self.km = 0.0
        self.g: list[float] = []
        self.rhs: list[float] = []
        self.open: list[tuple[float, float, int]] = []
        self.queued: dict[int, tuple[float, float]] = {}
        self.replan_count = 0
        self.total_expanded_nodes = 0

    def _index(self, cell: Cell) -> int:
        assert self.grid is not None
        return cell[1] * len(self.grid[0]) + cell[0]

    def _cell(self, index: int) -> Cell:
        assert self.grid is not None
        width = len(self.grid[0])
        return index % width, index // width

    def _heuristic(self, left: Cell, right: Cell) -> float:
        return math.hypot(left[0] - right[0], left[1] - right[1])

    def _neighbors(self, cell: Cell) -> list[Cell]:
        assert self.grid is not None
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if self.allow_diagonal:
            directions += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        result = []
        for dx, dy in directions:
            candidate = (cell[0] + dx, cell[1] + dy)
            if not in_bounds(self.grid, candidate):
                continue
            if dx and dy:
                if (self.grid[cell[1]][cell[0]] or
                        self.grid[cell[1]][cell[0] + dx] or
                        self.grid[cell[1] + dy][cell[0]]):
                    continue
            result.append(candidate)
        return result

    def _edge_cost(self, left: Cell, right: Cell) -> float:
        assert self.grid is not None
        if (self.grid[left[1]][left[0]] or self.grid[right[1]][right[0]]):
            return float("inf")
        return math.sqrt(2.0) if left[0] != right[0] and left[1] != right[1] else 1.0

    def _calculate_key(self, cell: Cell) -> tuple[float, float]:
        assert self.start is not None
        best = min(self.g[self._index(cell)], self.rhs[self._index(cell)])
        return best + self._heuristic(self.start, cell) + self.km, best

    def _push(self, index: int, key: tuple[float, float]) -> None:
        self.queued[index] = key
        heapq.heappush(self.open, (key[0], key[1], index))

    def _update_vertex(self, cell: Cell) -> None:
        assert self.goal is not None
        index = self._index(cell)
        self.queued.pop(index, None)
        if cell != self.goal:
            best = float("inf")
            for successor in self._neighbors(cell):
                best = min(best, self._edge_cost(cell, successor) +
                           self.g[self._index(successor)])
            self.rhs[index] = best
        if self.g[index] != self.rhs[index]:
            self._push(index, self._calculate_key(cell))

    def _compute_shortest_path(self) -> int:
        assert self.start is not None
        expanded = 0
        start_index = self._index(self.start)
        limit = max(1, len(self.g) * 100)
        while self.open and expanded < limit:
            top = heapq.heappop(self.open)
            index = top[2]
            queued_key = self.queued.get(index)
            if queued_key is None or queued_key != (top[0], top[1]):
                continue
            cell = self._cell(index)
            current_key = self._calculate_key(cell)
            self.queued.pop(index, None)
            if top[:2] < current_key:
                self._push(index, current_key)
                continue
            if not (top[:2] < self._calculate_key(self.start) or
                    self.rhs[start_index] != self.g[start_index]):
                break
            expanded += 1
            if self.g[index] > self.rhs[index]:
                self.g[index] = self.rhs[index]
                for predecessor in self._neighbors(cell):
                    self._update_vertex(predecessor)
            else:
                self.g[index] = float("inf")
                self._update_vertex(cell)
                for predecessor in self._neighbors(cell):
                    self._update_vertex(predecessor)
        self.total_expanded_nodes += expanded
        return expanded

    def _extract_path(self) -> tuple[Cell, ...]:
        assert self.start is not None and self.goal is not None
        if not in_bounds(self.grid or [], self.start) or not in_bounds(self.grid or [], self.goal):
            return ()
        if (self.grid[self.start[1]][self.start[0]] or
                self.grid[self.goal[1]][self.goal[0]] or
                not math.isfinite(self.g[self._index(self.start)])):
            return ()
        path = [self.start]
        current = self.start
        visited = {current}
        while current != self.goal and len(path) <= len(self.g) + 1:
            candidates = [
                (self._edge_cost(current, candidate) + self.g[self._index(candidate)], candidate)
                for candidate in self._neighbors(current)
                if self._edge_cost(current, candidate) < float("inf") and candidate not in visited
            ]
            if not candidates:
                return ()
            _, current = min(candidates, key=lambda item: (item[0], item[1][1], item[1][0]))
            path.append(current)
            visited.add(current)
        return tuple(path) if current == self.goal else ()

    def _initialize(self, grid: Grid, start: Cell, goal: Cell) -> None:
        self.grid = validate_grid(grid)
        self.start = start
        self.last_start = start
        self.goal = goal
        size = len(self.grid) * len(self.grid[0])
        self.g = [float("inf")] * size
        self.rhs = [float("inf")] * size
        self.open = []
        self.queued = {}
        self.km = 0.0
        if in_bounds(self.grid, goal):
            goal_index = self._index(goal)
            self.rhs[goal_index] = 0.0
            self._push(goal_index, self._calculate_key(goal))

    def update(self, grid: Sequence[Sequence[int]], start: Cell,
               goal: Cell | None = None,
               changed_cells: Iterable[Cell] | None = None) -> ReplanResult:
        checked = validate_grid(grid)
        requested_goal = goal or self.goal
        if requested_goal is None:
            raise ValueError("goal is required for the first incremental plan")
        initialized = self.grid is not None
        if (not initialized or len(self.grid or []) != len(checked) or
                len((self.grid or [[]])[0]) != len(checked[0]) or
                requested_goal != self.goal):
            self._initialize(checked, start, requested_goal)
            dirty = tuple(sorted(set(changed_cells or [])))
            began = time.perf_counter()
            expanded = self._compute_shortest_path()
            elapsed = (time.perf_counter() - began) * 1000.0
            self.replan_count += 1
            return ReplanResult(self._extract_path(), True, dirty, expanded, elapsed)

        assert self.grid is not None and self.start is not None
        previous = self.grid
        if changed_cells is None:
            dirty_set = {
                (x, y) for y in range(len(checked)) for x in range(len(checked[0]))
                if previous[y][x] != checked[y][x]
            }
        else:
            dirty_set = set(changed_cells)
        start_changed = start != self.start
        self.km += self._heuristic(self.last_start or start, start)
        self.last_start = start
        self.start = start
        self.grid = checked
        for cell in dirty_set:
            if in_bounds(self.grid, cell):
                self._update_vertex(cell)
                for predecessor in self._neighbors(cell):
                    self._update_vertex(predecessor)
        began = time.perf_counter()
        expanded = self._compute_shortest_path()
        elapsed = (time.perf_counter() - began) * 1000.0
        should_report_replan = bool(dirty_set or start_changed or expanded)
        if should_report_replan:
            self.replan_count += 1
        return ReplanResult(self._extract_path(), should_report_replan,
                            tuple(sorted(dirty_set)), expanded, elapsed)


@dataclass(frozen=True)
class PerceptionStep:
    frame: int
    robot: Cell
    changed_cells: tuple[Cell, ...]
    detections: tuple[DetectedObstacle, ...]
    tracks: tuple[ObstacleTrack, ...]
    path: tuple[Cell, ...]
    replanned: bool
    expanded_nodes: int
    planning_time_ms: float
    minimum_path_clearance: float
    occupied_cells: int
    unknown_cells: int


class PerceptionMappingPipeline:
    """Connect sensor integration, tracking, safety, and D* Lite updates."""

    def __init__(self, static_grid: Sequence[Sequence[int]], start: Cell,
                 goal: Cell, *, unknown_policy: str = "occupied",
                 initial_map_known: bool = True, robot_radius: float = 0.0,
                 cluster_distance: float = 1.5,
                 tracker: ObstacleTracker | None = None) -> None:
        self.static_grid = validate_grid(static_grid)
        self.unknown_policy = unknown_policy
        self.robot_radius = robot_radius
        self.cluster_distance = cluster_distance
        if initial_map_known:
            self.occupancy = OccupancyGrid.from_grid(self.static_grid,
                                                     prior_strength=1.0)
        else:
            self.occupancy = OccupancyGrid(len(self.static_grid[0]),
                                           len(self.static_grid))
        self.tracker = tracker or ObstacleTracker()
        self.planner = IncrementalDStarLite()
        self.path: tuple[Cell, ...] = ()
        self.robot = start
        self.goal = goal
        self.distance_field = compute_distance_field(
            self.occupancy.to_grid(self.unknown_policy))

    def step(self, scan: SensorFrame, robot: Cell | None = None) -> PerceptionStep:
        if robot is not None:
            self.robot = robot
        changed = self.occupancy.integrate_scan(scan, self.unknown_policy)
        planning_grid = self.occupancy.to_grid(self.unknown_policy)
        detections = detect_dynamic_obstacles(scan, self.static_grid,
                                               self.cluster_distance)
        tracks = self.tracker.update(detections, scan.frame)
        needs_first_plan = self.planner.grid is None
        result = self.planner.update(
            planning_grid, self.robot, self.goal, changed
        ) if (needs_first_plan or changed or self.path) else ReplanResult(
            self.path, False, (), 0, 0.0)
        self.path = result.path
        self.distance_field = compute_distance_field(planning_grid)
        clearance = path_clearance(self.distance_field, self.path,
                                   self.robot_radius)
        return PerceptionStep(
            scan.frame, self.robot, result.changed_cells,
            tuple(detections), tuple(tracks), self.path, result.replanned,
            result.expanded_nodes, result.planning_time_ms, clearance,
            sum(sum(row) for row in planning_grid),
            len(self.occupancy.unknown_cells()),
        )


def tracked_obstacle_updates(
        steps: Iterable[PerceptionStep]) -> list[DynamicMapUpdate]:
    """Convert tracked obstacle motion into C++ dynamic-map updates.

    A clear is emitted before an occupied update when a track moves to a new
    cell. Missed tracks clear their last known cell so replayed frames do not
    leave stale obstacles in the C++ map.
    """
    previous_cells: dict[int, Cell] = {}
    updates: list[DynamicMapUpdate] = []
    for step in steps:
        active_cells: dict[int, Cell] = {}
        for track in step.tracks:
            if track.missed != 0:
                continue
            cell = (math.floor(track.x + 0.5), math.floor(track.y + 0.5))
            active_cells[track.track_id] = cell
            previous = previous_cells.get(track.track_id)
            if previous is not None and previous != cell:
                updates.append((step.frame, previous[0], previous[1], False))
            if previous != cell:
                updates.append((step.frame, cell[0], cell[1], True))
        for track_id, previous in previous_cells.items():
            if track_id not in active_cells:
                updates.append((step.frame, previous[0], previous[1], False))
        previous_cells = active_cells
    return updates


def _parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "hit"}


def load_sensor_frames(path: Path) -> dict[int, SensorFrame]:
    """Load the documented CSV or JSON sensor interchange format."""
    if path.suffix.lower() == ".csv":
        grouped: dict[int, list[SensorPoint]] = {}
        origins: dict[int, tuple[float, float, float]] = {}
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                frame = int(row["frame"])
                grouped.setdefault(frame, []).append(SensorPoint(
                    float(row["point_x"]), float(row["point_y"]),
                    _parse_bool(row.get("hit", True)), row.get("kind", "unknown")))
                origins[frame] = (
                    float(row.get("sensor_x", 0.0)),
                    float(row.get("sensor_y", 0.0)),
                    float(row.get("sensor_theta", 0.0)),
                )
        result: dict[int, SensorFrame] = {}
        for frame, points in sorted(grouped.items()):
            sensor_x, sensor_y, sensor_theta = origins.get(
                frame, (0.0, 0.0, 0.0))
            result[frame] = SensorFrame(
                frame, sensor_x, sensor_y, tuple(points), sensor_theta)
        return result
    data = json.loads(path.read_text())
    frames = data.get("frames", data) if isinstance(data, dict) else data
    if not isinstance(frames, list):
        raise ValueError("sensor JSON must be a list or contain a frames list")
    result: dict[int, SensorFrame] = {}
    for item in frames:
        frame = int(item["frame"])
        points = tuple(SensorPoint(
            float(point["x"]), float(point["y"]),
            _parse_bool(point.get("hit", True)), point.get("kind", "unknown"))
            for point in item.get("points", []))
        result[frame] = SensorFrame(
            frame, float(item.get("sensor_x", 0.0)),
            float(item.get("sensor_y", 0.0)), points,
            float(item.get("sensor_theta", 0.0)))
    return result


def save_sensor_csv(path: Path, frames: Iterable[SensorFrame]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        fields = ("frame", "sensor_x", "sensor_y", "sensor_theta",
                  "point_x", "point_y", "hit", "kind")
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for frame in frames:
            for point in frame.points:
                writer.writerow({
                    "frame": frame.frame, "sensor_x": frame.sensor_x,
                    "sensor_y": frame.sensor_y, "sensor_theta": frame.sensor_theta,
                    "point_x": point.x, "point_y": point.y,
                    "hit": int(point.hit), "kind": point.kind,
                })


def save_sensor_json(path: Path, frames: Iterable[SensorFrame]) -> None:
    """Write the same sensor interchange format as a JSON frame list."""
    data = {"frames": [{
        "frame": frame.frame,
        "sensor_x": frame.sensor_x,
        "sensor_y": frame.sensor_y,
        "sensor_theta": frame.sensor_theta,
        "points": [asdict(point) for point in frame.points],
    } for frame in frames]}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def save_dynamic_updates(path: Path,
                         updates: Iterable[DynamicMapUpdate]) -> None:
    """Write the CSV contract consumed by dynamic_navigation_pipeline_cli."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        fields = ("frame", "cell_x", "cell_y", "occupied")
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for frame, cell_x, cell_y, occupied in updates:
            writer.writerow({
                "frame": frame,
                "cell_x": cell_x,
                "cell_y": cell_y,
                "occupied": int(occupied),
            })
