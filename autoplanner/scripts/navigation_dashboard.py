#!/usr/bin/env python3
"""Local Streamlit experiment dashboard for the ROS-free RobotNav stack."""

from __future__ import annotations

import json
import subprocess
import sys
import uuid
from pathlib import Path
from typing import Any

import streamlit as st


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
PIPELINE_SCRIPT = SCRIPT_DIR / "run_navigation_pipeline.py"
MAP_DIR = REPO_ROOT / "autoplanner" / "data" / "maps"
RESULTS_DIR = REPO_ROOT / "autoplanner" / "results" / "dashboard"

PLANNERS = (
    "improved_astar",
    "astar",
    "dijkstra",
    "jps",
    "weighted_astar",
    "dstar_lite",
    "rrt",
    "rrt_star",
    "bi_rrt",
    "informed_rrt_star",
    "hybrid_astar",
)
CONTROLLERS = ("stanley", "pure_pursuit", "pid", "mpc")
ENGINES = ("unified", "dynamic", "legacy")


def map_choices() -> dict[str, Path]:
    maps = sorted(MAP_DIR.glob("*.txt"))
    return {path.stem: path for path in maps}


def map_dimensions(path: Path) -> tuple[int, int]:
    rows = [line.strip() for line in path.read_text().splitlines() if line.strip()]
    if not rows or any(len(row) != len(rows[0]) for row in rows):
        raise ValueError(f"invalid rectangular map: {path}")
    return len(rows[0]), len(rows)


def output_directory() -> Path:
    return RESULTS_DIR / f"run_{uuid.uuid4().hex[:8]}"


def build_command(settings: dict[str, Any], output_dir: Path) -> list[str]:
    command = [
        sys.executable,
        str(PIPELINE_SCRIPT),
        "--build_dir", "build",
        "--engine", settings["engine"],
        "--map", str(settings["map"]),
        "--planner", settings["planner"],
        "--controller", settings["controller"],
        "--start", str(settings["start_x"]), str(settings["start_y"]),
        "--goal", str(settings["goal_x"]), str(settings["goal_y"]),
        "--velocity", str(settings["velocity"]),
        "--robot-radius", str(settings["robot_radius"]),
        "--smooth", settings["smooth"],
        "--output_dir", str(output_dir),
        "--plot",
    ]
    if settings["controller"] == "mpc":
        command.extend(("--mpc-horizon", str(settings["mpc_horizon"])))

    if settings["engine"] == "dynamic":
        command.extend((
            "--frames", str(settings["frames"]),
            "--steps-per-frame", str(settings["steps_per_frame"]),
            "--obstacle-ahead", str(settings["obstacle_ahead"]),
        ))
        if not settings["auto_obstacles"]:
            command.append("--no-auto-obstacles")
        if settings["use_prediction"]:
            command.extend((
                "--moving-obstacle",
                str(settings["prediction_start"]),
                str(settings["prediction_end"]),
                str(settings["prediction_x"]),
                str(settings["prediction_y"]),
                str(settings["prediction_dx"]),
                str(settings["prediction_dy"]),
            ))
    return command


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text())


def metric_cards(summary: dict[str, Any]) -> None:
    metrics = summary.get("pipeline", summary.get("tracking", summary))
    if not isinstance(metrics, dict):
        return
    candidates = (
        ("goal_reached", "Goal reached"),
        ("success", "Success"),
        ("replanning_count", "Replans"),
        ("collision_steps", "Collision steps"),
        ("planning_time_ms", "Planning ms"),
        ("total_time_ms", "Total ms"),
    )
    values = [(label, metrics[key]) for key, label in candidates if key in metrics]
    if not values:
        return
    columns = st.columns(min(3, len(values)))
    for column, (label, value) in zip(columns, values):
        column.metric(label, value)


def render_results(output_dir: Path, completed: subprocess.CompletedProcess[str]) -> None:
    if completed.stdout:
        with st.expander("Pipeline output"):
            st.code(completed.stdout)
    if completed.stderr:
        with st.expander("Pipeline diagnostics"):
            st.code(completed.stderr)

    if completed.returncode != 0:
        st.error(f"Pipeline exited with code {completed.returncode}.")
        return

    summary = read_json(output_dir / "summary.json")
    plot_path = Path(summary.get("plot_png", output_dir / "navigation.png"))
    st.success(f"Experiment complete: {output_dir.relative_to(REPO_ROOT)}")
    metric_cards(summary)
    if plot_path.exists():
        st.image(str(plot_path), caption="Navigation trace and tracking error")
    st.subheader("Machine-readable result")
    st.json(summary)


def main() -> None:
    st.set_page_config(page_title="RobotNav Lab", page_icon="🧭", layout="wide")
    st.title("RobotNav Lab")
    st.caption("ROS-free path planning, tracking, dynamic replanning, and visualization.")

    choices = map_choices()
    if not choices:
        st.error(f"No maps found in {MAP_DIR}")
        return

    with st.sidebar:
        st.header("Experiment")
        engine = st.selectbox("Pipeline", ENGINES, index=0)
        map_name = st.selectbox("Map", tuple(choices), index=0)
        planner = st.selectbox("Planner", PLANNERS, index=0)
        controller = st.selectbox("Controller", CONTROLLERS, index=0)
        smooth = st.selectbox("Path smoothing", ("shortcut", "none"), index=0)
        velocity = st.slider("Target velocity", 0.2, 2.0, 1.0, 0.1)
        robot_radius = st.slider("Robot radius", 0.0, 2.0, 1.0, 0.1)
        mpc_horizon = st.slider("MPC horizon", 5, 30, 15)

    map_path = choices[map_name]
    width, height = map_dimensions(map_path)
    st.info(f"Map: {map_name} · {width} × {height} cells")

    control_column, prediction_column = st.columns(2)
    with control_column:
        st.subheader("Start and goal")
        start_x = st.number_input("Start x", 0, width - 1, 1)
        start_y = st.number_input("Start y", 0, height - 1, 1)
        goal_x = st.number_input("Goal x", 0, width - 1, width - 2)
        goal_y = st.number_input("Goal y", 0, height - 1, height - 2)

    settings: dict[str, Any] = {
        "engine": engine,
        "map": map_path,
        "planner": planner,
        "controller": controller,
        "smooth": smooth,
        "velocity": velocity,
        "robot_radius": robot_radius,
        "mpc_horizon": mpc_horizon,
        "start_x": start_x,
        "start_y": start_y,
        "goal_x": goal_x,
        "goal_y": goal_y,
        "frames": 20,
        "steps_per_frame": 40,
        "obstacle_ahead": 5,
        "auto_obstacles": True,
        "use_prediction": False,
        "prediction_start": 3,
        "prediction_end": 15,
        "prediction_x": width // 2,
        "prediction_y": height // 2,
        "prediction_dx": 0,
        "prediction_dy": 1,
    }

    if engine == "dynamic":
        with prediction_column:
            st.subheader("Predicted obstacle trajectory")
            st.caption("The scheduled motion is supplied to the D* Lite replanning pipeline.")
            settings["auto_obstacles"] = st.checkbox("Enable automatic path obstacle", True)
            settings["use_prediction"] = st.checkbox("Add predicted moving obstacle", True)
            settings["frames"] = st.slider("Update frames", 2, 60, 20)
            settings["steps_per_frame"] = st.slider("Control steps per frame", 10, 120, 40)
            settings["obstacle_ahead"] = st.slider("Auto-obstacle path lookahead", 1, 20, 5)
            if settings["use_prediction"]:
                frame_limit = settings["frames"]
                settings["prediction_start"] = st.number_input(
                    "Prediction start frame", 0, frame_limit - 1,
                    min(3, frame_limit - 1)
                )
                settings["prediction_end"] = st.number_input(
                    "Prediction end frame", 1, frame_limit,
                    min(15, frame_limit)
                )
                settings["prediction_x"] = st.number_input("Prediction x", 0, width - 1, width // 2)
                settings["prediction_y"] = st.number_input("Prediction y", 0, height - 1, height // 2)
                settings["prediction_dx"] = st.number_input("Prediction Δx / frame", -3, 3, 0)
                settings["prediction_dy"] = st.number_input("Prediction Δy / frame", -3, 3, 1)
    else:
        with prediction_column:
            st.subheader("Dynamic obstacle mode")
            st.caption("Select the Dynamic pipeline to supply a predicted obstacle trajectory and observe replanning.")

    build_binary = REPO_ROOT / "build" / "apps" / "navigation_pipeline_cli"
    if not build_binary.exists():
        st.warning("Build outputs are missing. Run `cmake -S . -B build` and `cmake --build build -j` first.")

    if st.button("Run experiment", type="primary", use_container_width=True):
        if (start_x, start_y) == (goal_x, goal_y):
            st.error("Start and goal must be different.")
            return
        if settings["prediction_end"] < settings["prediction_start"]:
            st.error("Prediction end frame must not precede its start frame.")
            return
        if settings["use_prediction"]:
            duration = settings["prediction_end"] - settings["prediction_start"]
            end_x = settings["prediction_x"] + duration * settings["prediction_dx"]
            end_y = settings["prediction_y"] + duration * settings["prediction_dy"]
            if not (0 <= end_x < width and 0 <= end_y < height):
                st.error("Predicted obstacle trajectory leaves the selected map.")
                return
        output_dir = output_directory()
        command = build_command(settings, output_dir)
        with st.spinner("Running C++ navigation pipeline..."):
            completed = subprocess.run(
                command, cwd=REPO_ROOT, text=True, capture_output=True
            )
        render_results(output_dir, completed)


if __name__ == "__main__":
    main()
