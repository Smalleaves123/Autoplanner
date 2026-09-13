#!/usr/bin/env python3
"""Tests for complete validation result and latency reporting."""

from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from validation_reporting import (  # noqa: E402
    build_report,
    percentile,
    result_rows,
    save_report,
)


def write_json(path: Path, data: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def write_trace(path: Path, values: list[float]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("compute_latency_ms",))
        writer.writeheader()
        for value in values:
            writer.writerow({"compute_latency_ms": value})


class ValidationReportingTests(unittest.TestCase):
    def test_percentile_uses_linear_interpolation(self) -> None:
        self.assertEqual(percentile([0.0, 10.0], 0.50), 5.0)
        self.assertEqual(percentile([], 0.95), None)

    def test_tracking_manifest_expands_every_backend(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            write_trace(trace, [1.0, 2.0, 3.0])
            manifest = root / "manifest.json"
            write_json(manifest, {"runs": [{
                "scenario": {"backend": {"name": backend}},
                "metrics": {
                    "run_success": True, "goal_reached": backend == "kinematic",
                    "collision_steps": 0, "safe_stop": False,
                    "safe_stop_steps": 0, "goal_time_s": 2.0,
                    "control_effort": 4.0,
                    "compute_latency_ms": {"p50": 2.0, "p95": 2.9,
                                           "p99": 2.98},
                },
                "artifacts": {"trace_csv": str(trace)},
            } for backend in ("kinematic", "mujoco")]})
            rows = result_rows([{
                "case_id": "case", "kind": "tracking", "attempted": True,
                "artifact": str(manifest),
            }])
            report = build_report(rows)
        self.assertEqual(len(rows), 2)
        self.assertEqual(report["attempted_runs"], 2)
        self.assertEqual(report["successful_runs"], 1)
        self.assertEqual(report["compute_latency_ms"]["samples"], 6)
        self.assertEqual(report["compute_latency_ms"]["p50"], 2.0)

    def test_dynamic_and_missing_runs_are_not_dropped(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            metrics = root / "dynamic" / "metrics.json"
            write_json(metrics, {
                "success": False, "goal_reached": False,
                "collision_steps": 1, "safe_stop": True,
                "safe_stop_steps": 3, "goal_time_s": 0.0,
                "control_effort": 1.5, "compute_latency_p50_ms": 0.2,
                "compute_latency_p95_ms": 0.5,
                "compute_latency_p99_ms": 0.7,
                "compute_latency_samples": 3,
            })
            rows = result_rows([
                {"case_id": "dynamic", "kind": "dynamic_agents",
                 "attempted": True, "artifact": str(metrics)},
                {"case_id": "missing", "kind": "tracking",
                 "attempted": True, "artifact": str(root / "missing.json")},
            ])
            report = build_report(rows)
        self.assertEqual(len(rows), 2)
        self.assertEqual(report["attempted_runs"], 2)
        self.assertEqual(report["completed_runs"], 1)
        self.assertEqual(report["collision_runs"], 1)
        self.assertEqual(report["safe_stop_runs"], 1)
        self.assertIsNone(rows[0]["goal_time_s"])

    def test_save_report_writes_csv_and_json(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            csv_path, json_path, report = save_report(root, [{
                "case_id": "dry", "kind": "tracking", "attempted": False,
                "artifact": str(root / "none.json"),
            }])
            restored = json.loads(json_path.read_text())
        self.assertTrue(csv_path.name.endswith(".csv"))
        self.assertEqual(report["attempted_runs"], 0)
        self.assertEqual(restored["artifact_type"],
                         "robotnav.validation.report")


if __name__ == "__main__":
    unittest.main()
