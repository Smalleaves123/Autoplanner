#!/usr/bin/env python3
"""Tests for minimized validation failure bundles and replay."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from failure_replay import (  # noqa: E402
    FailureBundle,
    minimize_failure_case,
    preserve_failure_bundle,
    replay_bundle,
    rewrite_output_dir,
)
from validation_suites import (  # noqa: E402
    DynamicAgentSpec,
    PerturbationSpec,
    ValidationCase,
)


class FailureReplayTests(unittest.TestCase):
    def test_tracking_minimizer_removes_unnecessary_dimensions(self) -> None:
        case = ValidationCase(
            "mixed", "tracking", 3,
            PerturbationSpec(
                position_noise_std=0.1, heading_noise_std=0.2,
                observation_latency_steps=3))

        minimized, attempts = minimize_failure_case(
            case,
            lambda candidate:
                candidate.perturbation.observation_latency_steps > 0)

        self.assertGreater(attempts, 0)
        self.assertEqual(minimized.perturbation.position_noise_std, 0.0)
        self.assertEqual(minimized.perturbation.heading_noise_std, 0.0)
        self.assertEqual(minimized.perturbation.observation_latency_steps, 1)

    def test_dynamic_minimizer_keeps_only_required_agent(self) -> None:
        required = DynamicAgentSpec(0, 5, 2, 3, 1, 0)
        other = DynamicAgentSpec(0, 5, 8, 3, -1, 0)
        case = ValidationCase(
            "agents", "dynamic_agents", 2,
            dynamic_agents=(other, required))

        minimized, attempts = minimize_failure_case(
            case,
            lambda candidate: any(agent.x == 2
                                  for agent in candidate.dynamic_agents))

        self.assertGreater(attempts, 0)
        self.assertEqual(len(minimized.dynamic_agents), 1)
        self.assertEqual(minimized.dynamic_agents[0].x, 2)
        self.assertEqual(minimized.dynamic_agents[0].end_frame, 0)

    def test_output_rewrite_requires_complete_option(self) -> None:
        self.assertEqual(
            rewrite_output_dir(["runner", "--output-dir", "old"], "/new"),
            ["runner", "--output-dir", "/new"])
        with self.assertRaises(ValueError):
            rewrite_output_dir(["runner"], "/new")

    def test_bundle_captures_artifacts_and_round_trips(self) -> None:
        case = ValidationCase("case", "tracking", 1)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            artifact = root / "run" / "experiment_manifest.json"
            artifact.parent.mkdir()
            artifact.write_text("{}")
            map_path = root / "map.txt"
            map_path.write_text("000\n")
            bundle_path = preserve_failure_bundle(
                root / "bundle", case, case, 0, {
                    "artifact": str(artifact),
                    "command": [
                        "runner", "--map", str(map_path),
                        "--output-dir", str(artifact.parent)],
                })
            restored = FailureBundle.load_json(bundle_path)
            captured = (
                Path(restored.captured_artifacts) /
                "experiment_manifest.json")
            self.assertTrue(captured.exists())
            self.assertTrue(Path(restored.replay_command[2]).exists())
        self.assertEqual(restored.original_case, case)
        self.assertEqual(restored.minimized_case, case)

    def test_replay_executes_in_new_directory_and_confirms_failure(self) -> None:
        case = ValidationCase(
            "agent", "dynamic_agents", 1,
            dynamic_agents=(DynamicAgentSpec(0, 1, 1, 1, 1, 0),))
        script = (
            "import json,pathlib,sys;"
            "p=pathlib.Path(sys.argv[sys.argv.index('--output-dir')+1]);"
            "p.mkdir(parents=True,exist_ok=True);"
            "(p/'metrics.json').write_text(json.dumps({'success':False}))")
        bundle = FailureBundle(
            "agent", case, case, 0, "", "",
            (sys.executable, "-c", script, "--output-dir", "unused"))
        with tempfile.TemporaryDirectory() as temporary:
            return_code, reproduced, artifact = replay_bundle(
                bundle, Path(temporary) / "replay")
            self.assertTrue(artifact.exists())
        self.assertEqual(return_code, 0)
        self.assertTrue(reproduced)


if __name__ == "__main__":
    unittest.main()
