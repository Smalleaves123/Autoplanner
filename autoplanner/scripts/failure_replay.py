#!/usr/bin/env python3
"""Minimize, preserve, and replay RobotNav validation failures."""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any, Callable, Mapping

from validation_suites import ValidationCase


SCHEMA_VERSION = 1
FAILURE_TYPE = "robotnav.validation.failure"


@dataclass(frozen=True)
class FailureBundle:
    failure_id: str
    original_case: ValidationCase
    minimized_case: ValidationCase
    minimization_attempts: int
    source_artifact: str
    captured_artifacts: str
    replay_command: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.failure_id.strip():
            raise ValueError("failure_id must not be empty")
        if self.minimization_attempts < 0:
            raise ValueError("minimization_attempts must be non-negative")

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "artifact_type": FAILURE_TYPE,
            "failure_id": self.failure_id,
            "expected_outcome": "failure",
            "original_case": self.original_case.to_dict(),
            "minimized_case": self.minimized_case.to_dict(),
            "minimization_attempts": self.minimization_attempts,
            "source_artifact": self.source_artifact,
            "captured_artifacts": self.captured_artifacts,
            "replay_command": list(self.replay_command),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "FailureBundle":
        if data.get("schema_version") != SCHEMA_VERSION:
            raise ValueError("unsupported failure bundle schema version")
        if data.get("artifact_type") != FAILURE_TYPE:
            raise ValueError(f"expected artifact_type {FAILURE_TYPE!r}")
        return cls(
            failure_id=str(data["failure_id"]),
            original_case=ValidationCase.from_dict(data["original_case"]),
            minimized_case=ValidationCase.from_dict(data["minimized_case"]),
            minimization_attempts=int(data["minimization_attempts"]),
            source_artifact=str(data["source_artifact"]),
            captured_artifacts=str(data["captured_artifacts"]),
            replay_command=tuple(str(value) for value in data["replay_command"]),
        )

    def save_json(self, path: str | Path) -> Path:
        output = Path(path).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as stream:
            json.dump(self.to_dict(), stream, indent=2, sort_keys=True,
                      allow_nan=False)
            stream.write("\n")
        return output

    @classmethod
    def load_json(cls, path: str | Path) -> "FailureBundle":
        with Path(path).expanduser().open(encoding="utf-8") as stream:
            return cls.from_dict(json.load(stream))


def minimize_failure_case(
        case: ValidationCase,
        failure_persists: Callable[[ValidationCase], bool]) \
        -> tuple[ValidationCase, int]:
    """Greedily remove independent causes while the failure still reproduces."""

    current = case
    attempts = 0
    if case.kind == "dynamic_agents":
        changed = True
        while changed and len(current.dynamic_agents) > 1:
            changed = False
            for index in range(len(current.dynamic_agents)):
                candidate = replace(
                    current,
                    dynamic_agents=(current.dynamic_agents[:index] +
                                    current.dynamic_agents[index + 1:]))
                attempts += 1
                if failure_persists(candidate):
                    current = candidate
                    changed = True
                    break
        for index, agent in enumerate(current.dynamic_agents):
            if agent.end_frame == agent.start_frame:
                continue
            shortened = replace(agent, end_frame=agent.start_frame)
            candidate = replace(
                current,
                dynamic_agents=(current.dynamic_agents[:index] +
                                (shortened,) +
                                current.dynamic_agents[index + 1:]))
            attempts += 1
            if failure_persists(candidate):
                current = candidate
                continue
            lower = agent.start_frame + 1
            upper = agent.end_frame
            while lower < upper:
                middle = (lower + upper) // 2
                candidate_agent = replace(agent, end_frame=middle)
                candidate = replace(
                    current,
                    dynamic_agents=(current.dynamic_agents[:index] +
                                    (candidate_agent,) +
                                    current.dynamic_agents[index + 1:]))
                attempts += 1
                if failure_persists(candidate):
                    current = candidate
                    agent = candidate_agent
                    upper = middle
                else:
                    lower = middle + 1
        return current, attempts

    neutral_values: tuple[tuple[str, float | int | None], ...] = (
        ("position_noise_std", 0.0),
        ("heading_noise_std", 0.0),
        ("velocity_noise_std", 0.0),
        ("observation_latency_steps", 0),
        ("command_velocity_limit", None),
        ("command_steering_limit", None),
    )
    for field_name, neutral in neutral_values:
        if getattr(current.perturbation, field_name) == neutral:
            continue
        candidate_spec = replace(
            current.perturbation, **{field_name: neutral})
        candidate = replace(current, perturbation=candidate_spec)
        attempts += 1
        if failure_persists(candidate):
            current = candidate
    for field_name in (
            "position_noise_std", "heading_noise_std", "velocity_noise_std"):
        value = float(getattr(current.perturbation, field_name))
        if value <= 0.0:
            continue
        lower = 0.0
        upper = value
        for _ in range(6):
            middle = 0.5 * (lower + upper)
            candidate = replace(
                current,
                perturbation=replace(
                    current.perturbation, **{field_name: middle}))
            attempts += 1
            if failure_persists(candidate):
                current = candidate
                upper = middle
            else:
                lower = middle
    latency = current.perturbation.observation_latency_steps
    if latency > 1:
        lower = 1
        upper = latency
        while lower < upper:
            middle = (lower + upper) // 2
            candidate = replace(
                current,
                perturbation=replace(
                    current.perturbation,
                    observation_latency_steps=middle))
            attempts += 1
            if failure_persists(candidate):
                current = candidate
                upper = middle
            else:
                lower = middle + 1
    return current, attempts


def rewrite_output_dir(command: tuple[str, ...] | list[str],
                       output_dir: str | Path) -> list[str]:
    rewritten = list(command)
    try:
        index = rewritten.index("--output-dir")
    except ValueError as error:
        raise ValueError("replay command has no --output-dir argument") from error
    if index + 1 >= len(rewritten):
        raise ValueError("replay command has an incomplete --output-dir argument")
    rewritten[index + 1] = str(Path(output_dir).expanduser().resolve())
    return rewritten


def preserve_failure_bundle(
        output_dir: str | Path, original_case: ValidationCase,
        minimized_case: ValidationCase, minimization_attempts: int,
        execution_result: Mapping[str, Any]) -> Path:
    """Copy one minimized reproducer and its complete case artifact directory."""

    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    artifact = Path(str(execution_result["artifact"])).expanduser().resolve()
    captured = output / "captured_artifacts"
    if artifact.parent.exists():
        shutil.copytree(artifact.parent, captured, dirs_exist_ok=True)
    replay_command = [str(value) for value in execution_result["command"]]
    inputs = output / "inputs"
    for option in ("--map", "--path"):
        if option not in replay_command:
            continue
        index = replay_command.index(option)
        if index + 1 >= len(replay_command):
            continue
        source = Path(replay_command[index + 1]).expanduser()
        if not source.is_file():
            continue
        inputs.mkdir(parents=True, exist_ok=True)
        destination = inputs / f"{option[2:]}_{source.name}"
        shutil.copy2(source, destination)
        replay_command[index + 1] = str(destination)
    bundle = FailureBundle(
        failure_id=original_case.case_id,
        original_case=original_case,
        minimized_case=minimized_case,
        minimization_attempts=minimization_attempts,
        source_artifact=str(artifact),
        captured_artifacts=str(captured),
        replay_command=tuple(replay_command),
    )
    return bundle.save_json(output / "failure_case.json")


def replay_bundle(bundle: FailureBundle, output_dir: str | Path,
                  cwd: str | Path | None = None) -> tuple[int, bool, Path]:
    """Execute a bundle and return process code, failure reproduction, artifact."""

    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    command = rewrite_output_dir(bundle.replay_command, output)
    if command and Path(command[0]).name.startswith("python"):
        command[0] = sys.executable
    completed = subprocess.run(command, cwd=cwd, text=True)
    if bundle.minimized_case.kind == "tracking":
        artifact = output / "experiment_manifest.json"
    else:
        artifact = output / "metrics.json"
    if not artifact.exists():
        return completed.returncode, False, artifact
    try:
        with artifact.open(encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, json.JSONDecodeError):
        return completed.returncode, False, artifact
    if bundle.minimized_case.kind == "tracking":
        runs = data.get("runs", [])
        reproduced = bool(runs) and any(
            not bool(run.get("metrics", {}).get("goal_reached", False))
            for run in runs)
    else:
        reproduced = not bool(data.get("success", False))
    return completed.returncode, reproduced, artifact
