#!/usr/bin/env python3
"""Aggregate validation artifacts without hiding failed or incomplete runs."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping


REPORT_TYPE = "robotnav.validation.report"
RESULT_FIELDS = (
    "case_id", "kind", "backend", "attempted", "completed",
    "run_success", "success", "goal_reached", "collision",
    "collision_steps", "safe_stop", "safe_stop_steps", "goal_time_s",
    "control_effort", "compute_latency_p50_ms", "compute_latency_p95_ms",
    "compute_latency_p99_ms", "artifact", "trace_csv",
)


def percentile(values: Iterable[float], quantile: float) -> float | None:
    ordered = sorted(float(value) for value in values if math.isfinite(value))
    if not ordered:
        return None
    index = quantile * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    fraction = index - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def _load_object(path: str | Path) -> dict[str, Any] | None:
    source = Path(path)
    if not source.exists():
        return None
    try:
        with source.open(encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _tracking_rows(result: Mapping[str, Any], manifest: Mapping[str, Any]) \
        -> list[dict[str, Any]]:
    rows = []
    for run in manifest.get("runs", []):
        metrics = run.get("metrics", {})
        scenario = run.get("scenario", {})
        backend = scenario.get("backend", {}).get("name", "")
        trace_csv = run.get("artifacts", {}).get("trace_csv", "")
        goal_reached = bool(metrics.get("goal_reached", False))
        run_success = bool(metrics.get("run_success", False))
        collision_steps = int(metrics.get("collision_steps", 0))
        latency = metrics.get("compute_latency_ms", {})
        rows.append({
            "case_id": result["case_id"], "kind": result["kind"],
            "backend": backend, "attempted": bool(result["attempted"]),
            "completed": True, "run_success": run_success,
            "success": run_success and goal_reached,
            "goal_reached": goal_reached,
            "collision": collision_steps > 0,
            "collision_steps": collision_steps,
            "safe_stop": bool(metrics.get("safe_stop", False)),
            "safe_stop_steps": int(metrics.get("safe_stop_steps", 0)),
            "goal_time_s": metrics.get("goal_time_s"),
            "control_effort": float(metrics.get("control_effort", 0.0)),
            "compute_latency_p50_ms": latency.get("p50"),
            "compute_latency_p95_ms": latency.get("p95"),
            "compute_latency_p99_ms": latency.get("p99"),
            "artifact": result["artifact"], "trace_csv": trace_csv,
        })
    return rows


def _dynamic_row(result: Mapping[str, Any], metrics: Mapping[str, Any]) \
        -> dict[str, Any]:
    goal_reached = bool(metrics.get("goal_reached", False))
    run_success = bool(metrics.get("success", False))
    collision_steps = int(metrics.get("collision_steps", 0))
    goal_time = metrics.get("goal_time_s") if goal_reached else None
    has_latency = int(metrics.get("compute_latency_samples", 0)) > 0
    return {
        "case_id": result["case_id"], "kind": result["kind"],
        "backend": "kinematic", "attempted": bool(result["attempted"]),
        "completed": True, "run_success": run_success,
        "success": run_success and goal_reached,
        "goal_reached": goal_reached,
        "collision": collision_steps > 0,
        "collision_steps": collision_steps,
        "safe_stop": bool(metrics.get("safe_stop", False)),
        "safe_stop_steps": int(metrics.get("safe_stop_steps", 0)),
        "goal_time_s": goal_time,
        "control_effort": float(metrics.get("control_effort", 0.0)),
        "compute_latency_p50_ms": (
            metrics.get("compute_latency_p50_ms") if has_latency else None),
        "compute_latency_p95_ms": (
            metrics.get("compute_latency_p95_ms") if has_latency else None),
        "compute_latency_p99_ms": (
            metrics.get("compute_latency_p99_ms") if has_latency else None),
        "artifact": result["artifact"],
        "trace_csv": str(Path(result["artifact"]).with_name("trace.csv")),
    }


def result_rows(execution_results: Iterable[Mapping[str, Any]]) \
        -> list[dict[str, Any]]:
    """Expand every attempted case/backend into a non-dropping result ledger."""

    rows: list[dict[str, Any]] = []
    for result in execution_results:
        artifact = _load_object(result["artifact"])
        if artifact is not None and result["kind"] == "tracking":
            expanded = _tracking_rows(result, artifact)
            if expanded:
                rows.extend(expanded)
                continue
        elif artifact is not None:
            rows.append(_dynamic_row(result, artifact))
            continue
        rows.append({
            "case_id": result["case_id"], "kind": result["kind"],
            "backend": "", "attempted": bool(result["attempted"]),
            "completed": False, "run_success": False, "success": False,
            "goal_reached": False, "collision": False,
            "collision_steps": 0, "safe_stop": False,
            "safe_stop_steps": 0, "goal_time_s": None,
            "control_effort": 0.0, "compute_latency_p50_ms": None,
            "compute_latency_p95_ms": None,
            "compute_latency_p99_ms": None,
            "artifact": result["artifact"], "trace_csv": "",
        })
    return rows


def trace_latencies(rows: Iterable[Mapping[str, Any]]) -> list[float]:
    samples: list[float] = []
    for row in rows:
        trace_path = row.get("trace_csv")
        if not trace_path or not Path(str(trace_path)).exists():
            continue
        try:
            with Path(str(trace_path)).open(newline="", encoding="utf-8") as stream:
                for sample in csv.DictReader(stream):
                    value = sample.get("compute_latency_ms")
                    if value not in (None, ""):
                        latency = float(value)
                        if math.isfinite(latency) and latency >= 0.0:
                            samples.append(latency)
        except (OSError, ValueError):
            continue
    return samples


def build_report(rows: list[dict[str, Any]]) -> dict[str, Any]:
    attempted = [row for row in rows if row["attempted"]]
    completed = [row for row in attempted if row["completed"]]
    goal_times = [float(row["goal_time_s"]) for row in completed
                  if row["goal_time_s"] is not None]
    efforts = [float(row["control_effort"]) for row in completed]
    latencies = trace_latencies(completed)
    denominator = len(attempted)
    return {
        "schema_version": 1,
        "artifact_type": REPORT_TYPE,
        "attempted_runs": denominator,
        "completed_runs": len(completed),
        "successful_runs": sum(bool(row["success"]) for row in attempted),
        "success_rate": (sum(bool(row["success"]) for row in attempted) /
                         denominator if denominator else 0.0),
        "collision_runs": sum(bool(row["collision"]) for row in attempted),
        "collision_rate": (sum(bool(row["collision"]) for row in attempted) /
                           denominator if denominator else 0.0),
        "safe_stop_runs": sum(bool(row["safe_stop"]) for row in attempted),
        "safe_stop_rate": (sum(bool(row["safe_stop"]) for row in attempted) /
                           denominator if denominator else 0.0),
        "goal_time_s": {
            "samples": len(goal_times),
            "mean": sum(goal_times) / len(goal_times) if goal_times else None,
            "p50": percentile(goal_times, 0.50),
            "p95": percentile(goal_times, 0.95),
            "p99": percentile(goal_times, 0.99),
        },
        "control_effort": {
            "samples": len(efforts),
            "total": sum(efforts),
            "mean": sum(efforts) / len(efforts) if efforts else None,
        },
        "compute_latency_ms": {
            "samples": len(latencies),
            "p50": percentile(latencies, 0.50),
            "p95": percentile(latencies, 0.95),
            "p99": percentile(latencies, 0.99),
        },
        "runs": rows,
    }


def save_report(output_dir: str | Path,
                execution_results: Iterable[Mapping[str, Any]]) \
        -> tuple[Path, Path, dict[str, Any]]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    rows = result_rows(execution_results)
    csv_path = output / "validation_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=RESULT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    report = build_report(rows)
    json_path = output / "validation_report.json"
    with json_path.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return csv_path, json_path, report
