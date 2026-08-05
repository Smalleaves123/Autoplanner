"""Tests for the ROS-free perception and incremental planning loop."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from perception_mapping import (  # noqa: E402
    DetectedObstacle,
    IncrementalDStarLite,
    LidarConfig,
    LidarSimulator,
    OccupancyGrid,
    ObstacleTracker,
    ObstacleTrack,
    PerceptionStep,
    PerceptionMappingPipeline,
    SensorFrame,
    SensorPoint,
    SimulatedObstacle,
    compute_distance_field,
    detect_dynamic_obstacles,
    load_sensor_frames,
    save_dynamic_updates,
    save_sensor_csv,
    save_sensor_json,
    tracked_obstacle_updates,
)


class PerceptionMappingTests(unittest.TestCase):
    def test_unknown_policy_and_distance_field(self) -> None:
        occupancy = OccupancyGrid(4, 3)
        self.assertEqual(sum(sum(row) for row in occupancy.to_grid("occupied")), 12)
        self.assertEqual(sum(sum(row) for row in occupancy.to_grid("free")), 0)
        occupancy.integrate_scan(
            SensorFrame(0, 1.5, 1.5, (SensorPoint(3.5, 1.5, False),)),
            "occupied")
        occupied_grid = occupancy.to_grid("occupied")
        field = compute_distance_field(occupied_grid)
        self.assertEqual(occupied_grid[1][1], 0)
        self.assertGreater(field[1][1], 0.0)
        self.assertTrue(occupancy.unknown_cells())

    def test_lidar_detects_dynamic_obstacle_and_updates_map(self) -> None:
        grid = [[0 for _ in range(16)] for _ in range(16)]
        simulator = LidarSimulator(
            grid, [SimulatedObstacle("box", 6.0, 4.0, radius=0.8)],
            LidarConfig(beams=72, max_range=10.0))
        scan = simulator.scan(0, 2.5, 4.5)
        detections = detect_dynamic_obstacles(scan, grid)
        self.assertTrue(detections)
        self.assertAlmostEqual(detections[0].x, 6.0, delta=0.5)
        occupancy = OccupancyGrid.from_grid(grid)
        occupancy.integrate_scan(scan)
        for _ in range(2):
            occupancy.integrate_scan(scan)
        self.assertTrue(any(
            occupancy.to_grid()[y][x]
            for x in range(16) for y in range(16)
            if 5 <= x <= 7 and 3 <= y <= 5))

    def test_tracker_estimates_constant_velocity(self) -> None:
        tracker = ObstacleTracker()
        first = tracker.update([DetectedObstacle(0, 2.0, 2.0, 0.5, 3)], 0)
        second = tracker.update([DetectedObstacle(1, 3.0, 2.0, 0.5, 3)], 1)
        self.assertEqual(first[0].track_id, second[0].track_id)
        self.assertGreater(second[0].vx, 0.0)
        self.assertEqual(len(tracker.predictions()), 1)

    def test_tracked_obstacles_export_cpp_updates(self) -> None:
        def step(frame: int, track: ObstacleTrack) -> PerceptionStep:
            return PerceptionStep(
                frame=frame, robot=(0, 0), changed_cells=(), detections=(),
                tracks=(track,), path=(), replanned=False, expanded_nodes=0,
                planning_time_ms=0.0, minimum_path_clearance=0.0,
                occupied_cells=0, unknown_cells=0)

        updates = tracked_obstacle_updates([
            step(0, ObstacleTrack(1, 2.2, 3.1, last_frame=0)),
            step(1, ObstacleTrack(1, 3.2, 3.1, last_frame=1)),
        ])
        self.assertEqual(updates, [
            (0, 2, 3, True),
            (1, 2, 3, False),
            (1, 3, 3, True),
        ])
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "cpp_updates.csv"
            save_dynamic_updates(path, updates)
            self.assertEqual(path.read_text().splitlines()[0],
                             "frame,cell_x,cell_y,occupied")

    def test_sensor_csv_and_json_round_trip(self) -> None:
        frames = [SensorFrame(
            0, 1.5, 2.5,
            (SensorPoint(3.0, 2.5, True, "dynamic"),),
        )]
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "scan.csv"
            json_path = Path(temp_dir) / "scan.json"
            save_sensor_csv(csv_path, frames)
            save_sensor_json(json_path, frames)
            csv_loaded = load_sensor_frames(csv_path)
            json_loaded = load_sensor_frames(json_path)
        self.assertEqual(csv_loaded[0].points, frames[0].points)
        self.assertEqual(json_loaded[0].points, frames[0].points)
        self.assertEqual(json_loaded[0].sensor_x, 1.5)

    def test_incremental_replan_avoids_new_local_obstacle(self) -> None:
        grid = [[0 for _ in range(9)] for _ in range(7)]
        planner = IncrementalDStarLite(allow_diagonal=False)
        first = planner.update(grid, (0, 3), (8, 3))
        self.assertTrue(first.path)
        grid[3][4] = 1
        second = planner.update(grid, (0, 3), (8, 3), {(4, 3)})
        self.assertTrue(second.replanned)
        self.assertNotIn((4, 3), second.path)
        self.assertGreater(second.expanded_nodes, 0)
        self.assertEqual(planner.replan_count, 2)

    def test_pipeline_connects_mapping_tracking_and_replanning(self) -> None:
        grid = [[0 for _ in range(20)] for _ in range(20)]
        simulator = LidarSimulator(
            grid, [SimulatedObstacle("moving", 7.0, 4.5, 0.0, 0.5, 0.6)],
            LidarConfig(beams=48, max_range=12.0))
        pipeline = PerceptionMappingPipeline(grid, (1, 1), (15, 15))
        steps = []
        robot = (1, 1)
        for frame in range(5):
            step = pipeline.step(simulator.scan(frame, robot[0] + 0.5,
                                                robot[1] + 0.5), robot)
            steps.append(step)
            if len(step.path) > 1:
                robot = step.path[1]
        self.assertTrue(any(step.replanned for step in steps))
        self.assertTrue(any(step.detections for step in steps))
        self.assertTrue(any(step.tracks for step in steps))
        self.assertGreaterEqual(pipeline.planner.replan_count, 1)


if __name__ == "__main__":
    unittest.main()
