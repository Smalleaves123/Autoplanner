#!/usr/bin/env python3
"""Replay a minimized RobotNav validation failure bundle."""

from __future__ import annotations

import argparse
from pathlib import Path

from failure_replay import FailureBundle, replay_bundle


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", help="path to failure_case.json")
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args()

    case_path = Path(args.case).expanduser().resolve()
    try:
        bundle = FailureBundle.load_json(case_path)
    except (OSError, KeyError, TypeError, ValueError) as error:
        parser.error(str(error))
    output = (Path(args.output_dir).expanduser().resolve()
              if args.output_dir else case_path.parent / "replay")
    return_code, reproduced, artifact = replay_bundle(
        bundle, output, Path(__file__).resolve().parents[2])
    print(f"Artifact: {artifact}")
    print(f"Failure reproduced: {'yes' if reproduced else 'no'}")
    if reproduced:
        return 0
    return return_code if return_code != 0 else 3


if __name__ == "__main__":
    raise SystemExit(main())
