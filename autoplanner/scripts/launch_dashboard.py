#!/usr/bin/env python3
"""Launch the local RobotNav experiment dashboard with no arguments."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DASHBOARD = SCRIPT_DIR / "navigation_dashboard.py"


def main() -> int:
    command = [sys.executable, "-m", "streamlit", "run", str(DASHBOARD)]
    return subprocess.run(command, cwd=REPO_ROOT).returncode


if __name__ == "__main__":
    raise SystemExit(main())
