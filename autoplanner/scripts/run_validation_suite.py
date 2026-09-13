#!/usr/bin/env python3
"""Generate and execute deterministic RobotNav robustness validation suites."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

from validation_suites import ValidationCase, build_validation_suite
from validation_reporting import save_report
from failure_replay import minimize_failure_case, preserve_failure_bundle


EXECUTION_TYPE = "robotnav.validation.execution"


def resolve(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (root / path).resolve()


def tracking_command(
        root: Path, build_dir: Path, map_path: Path, output_dir: Path,
        case: ValidationCase, args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        str(root / "autoplanner/scripts/physics_tracking_benchmark.py"),
        "--backend", args.backend,
        "--controller", args.controller,
        "--planner", args.planner,
        "--build-dir", str(build_dir),
        "--map", str(map_path),
        "--start", str(args.start[0]), str(args.start[1]),
        "--goal", str(args.goal[0]), str(args.goal[1]),
        "--velocity", str(args.velocity),
        "--output-dir", str(output_dir),
        "--steps", str(args.steps),
        "--scenario-id", case.case_id,
        "--seed", str(case.seed),
    ]
    if args.path is not None:
        command += ["--path", str(resolve(root, args.path))]
    command += case.perturbation.cli_args()
    return command


def dynamic_command(
        build_dir: Path, map_path: Path, output_dir: Path,
        case: ValidationCase, args: argparse.Namespace) -> list[str]:
    command = [
        str(build_dir / "apps/dynamic_navigation_pipeline_cli"),
        "--map", str(map_path),
        "--planner", args.planner,
        "--controller", args.controller,
        "--local-planner", args.local_planner,
        "--start", str(args.start[0]), str(args.start[1]),
        "--goal", str(args.goal[0]), str(args.goal[1]),
        "--velocity", str(args.velocity),
        "--frames", str(args.frames),
        "--steps-per-frame", str(args.steps_per_frame),
        "--no-auto-obstacles",
        "--output-dir", str(output_dir),
    ]
    for agent in case.dynamic_agents:
        command += agent.cli_args()
    return command


def read_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        with path.open(encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def execute_case(
        root: Path, build_dir: Path, map_path: Path, output_root: Path,
        case: ValidationCase, args: argparse.Namespace) -> dict[str, Any]:
    case_dir = output_root / "cases" / case.case_id
    if case.kind == "tracking":
        command = tracking_command(
            root, build_dir, map_path, case_dir, case, args)
        artifact_path = case_dir / "experiment_manifest.json"
    else:
        command = dynamic_command(build_dir, map_path, case_dir, case, args)
        artifact_path = case_dir / "metrics.json"

    print(f"[{case.kind}] {case.case_id}", flush=True)
    if args.dry_run:
        return {
            "case_id": case.case_id,
            "kind": case.kind,
            "attempted": False,
            "completed": False,
            "return_code": None,
            "outcome_success": None,
            "artifact": str(artifact_path),
            "command": command,
        }

    case_dir.mkdir(parents=True, exist_ok=True)
    completed = subprocess.run(command, cwd=root, text=True)
    artifact = read_json(artifact_path)
    if case.kind == "tracking":
        runs = artifact.get("runs", []) if artifact else []
        outcome_success = bool(runs) and all(
            bool(run.get("metrics", {}).get("goal_reached", False))
            for run in runs)
    else:
        outcome_success = bool(artifact and artifact.get("success", False))
    return {
        "case_id": case.case_id,
        "kind": case.kind,
        "attempted": True,
        "completed": artifact is not None,
        "return_code": completed.returncode,
        "outcome_success": outcome_success,
        "artifact": str(artifact_path),
        "command": command,
    }


def preserve_failures(
        root: Path, build_dir: Path, map_path: Path, output_root: Path,
        cases: tuple[ValidationCase, ...], results: list[dict[str, Any]],
        args: argparse.Namespace) -> list[str]:
    """Minimize failed outcomes and save self-contained replay bundles."""

    if args.dry_run:
        return []
    case_by_id = {case.case_id: case for case in cases}
    bundles = []
    for result in results:
        if (not result["attempted"] or not result["completed"] or
                result["outcome_success"] is not False):
            continue
        original = case_by_id[result["case_id"]]
        minimized = original
        attempts = 0
        final_result = result
        if args.minimize_failures:
            candidate_index = 0

            def failure_persists(candidate: ValidationCase) -> bool:
                nonlocal candidate_index
                candidate_index += 1
                execution_case = replace(
                    candidate,
                    case_id=f"{original.case_id}-candidate-{candidate_index:02d}")
                candidate_result = execute_case(
                    root, build_dir, map_path,
                    output_root / "minimization" / original.case_id,
                    execution_case, args)
                return (candidate_result["completed"] and
                        candidate_result["outcome_success"] is False)

            minimized, attempts = minimize_failure_case(
                original, failure_persists)
            if minimized != original:
                minimized = replace(
                    minimized, case_id=f"{original.case_id}-minimized")
                final_result = execute_case(
                    root, build_dir, map_path,
                    output_root / "minimized" / original.case_id,
                    minimized, args)
                if (not final_result["completed"] or
                        final_result["outcome_success"] is not False):
                    minimized = original
                    final_result = result
        bundle_path = preserve_failure_bundle(
            output_root / "failures" / original.case_id,
            original, minimized, attempts, final_result)
        bundles.append(str(bundle_path))
    return bundles


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--suite",
        choices=("baseline", "noise", "latency", "saturation",
                 "dynamic_agents", "robustness"),
        default="robustness")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--agent-count", type=int, default=3)
    parser.add_argument("--backend",
                        choices=("kinematic", "mujoco", "pybullet",
                                 "both", "all"),
                        default="kinematic")
    parser.add_argument("--controller", choices=("stanley", "mpc"),
                        default="stanley")
    parser.add_argument("--planner", default="improved_astar")
    parser.add_argument("--local-planner", choices=("none", "dwa", "mppi"),
                        default="none")
    parser.add_argument("--build-dir", default="build")
    parser.add_argument("--map",
                        default="autoplanner/data/maps/simple_50x50.txt")
    parser.add_argument("--path", default=None,
                        help="optional existing path CSV for tracking cases")
    parser.add_argument("--start", nargs=2, type=int, default=(1, 1))
    parser.add_argument("--goal", nargs=2, type=int, default=(48, 48))
    parser.add_argument("--velocity", type=float, default=1.0)
    parser.add_argument("--steps", type=int, default=2200)
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--steps-per-frame", type=int, default=40)
    parser.add_argument("--output-dir",
                        default="autoplanner/results/validation_suite")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--fail-on-outcome", action="store_true",
        help="return non-zero when a completed navigation case misses its goal")
    parser.add_argument(
        "--minimize-failures", action=argparse.BooleanOptionalAction,
        default=True,
        help="delta-debug failed perturbations/agents before preserving them")
    args = parser.parse_args()
    if args.seed < 0 or args.repeats <= 0 or args.agent_count <= 0:
        parser.error("seed must be non-negative; repeats and agent-count positive")
    if args.steps <= 0 or args.frames < 2 or args.steps_per_frame <= 0:
        parser.error("steps/steps-per-frame must be positive and frames >= 2")

    root = Path(__file__).resolve().parents[2]
    build_dir = resolve(root, args.build_dir)
    map_path = resolve(root, args.map)
    if not map_path.exists():
        parser.error(f"map does not exist: {map_path}")
    grid = tuple(line.strip() for line in map_path.read_text().splitlines()
                 if line.strip())
    try:
        suite = build_validation_suite(
            args.suite, seed=args.seed, repeats=args.repeats,
            grid=grid, frames=args.frames, agent_count=args.agent_count)
    except ValueError as error:
        parser.error(str(error))

    output_root = resolve(root, args.output_dir)
    output_root.mkdir(parents=True, exist_ok=True)
    suite_path = suite.save_json(output_root / "validation_suite.json")
    results = [execute_case(
        root, build_dir, map_path, output_root, case, args)
        for case in suite.cases]
    results_csv, report_json, report = save_report(output_root, results)
    failure_bundles = preserve_failures(
        root, build_dir, map_path, output_root, suite.cases, results, args)
    execution = {
        "schema_version": 1,
        "artifact_type": EXECUTION_TYPE,
        "suite": str(suite_path),
        "dry_run": args.dry_run,
        "cases": results,
        "attempted_cases": sum(bool(result["attempted"]) for result in results),
        "completed_cases": sum(bool(result["completed"]) for result in results),
        "successful_outcomes": sum(
            result["outcome_success"] is True for result in results),
        "results_csv": str(results_csv),
        "report_json": str(report_json),
        "failure_bundles": failure_bundles,
    }
    execution_path = output_root / "validation_execution.json"
    with execution_path.open("w", encoding="utf-8") as stream:
        json.dump(execution, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(f"Suite: {suite_path}")
    print(f"Execution: {execution_path}")
    print(
        f"Report: {report_json} "
        f"({report['successful_runs']}/{report['attempted_runs']} successful)")
    if failure_bundles:
        print(f"Failure bundles: {len(failure_bundles)}")

    if not args.dry_run and any(not result["completed"] for result in results):
        return 2
    if args.fail_on_outcome and any(
            result["outcome_success"] is False for result in results):
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
