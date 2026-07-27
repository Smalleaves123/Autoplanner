#!/usr/bin/env python3
"""Local Streamlit experiment dashboard for the ROS-free RobotNav stack."""

from __future__ import annotations

import json
import math
import re
import subprocess
import sys
import uuid
from pathlib import Path
from typing import Any

import plotly.graph_objects as go
import streamlit as st

from dashboard_support import (
    MovingObstaclePrediction,
    RiskDecision,
    decide_prediction_risk,
    load_grid,
    load_scene,
    save_scene,
    write_grid,
)

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
PIPELINE_SCRIPT = SCRIPT_DIR / "run_navigation_pipeline.py"
MAP_DIR = REPO_ROOT / "autoplanner" / "data" / "maps"
RESULTS_DIR = REPO_ROOT / "autoplanner" / "results" / "dashboard"
SCENES_DIR = RESULTS_DIR / "scenes"
EDITABLE_CELL_LIMIT = 20_000

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


def relative_to_root(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def scene_choices() -> dict[str, Path]:
    return {path.stem: path for path in sorted(SCENES_DIR.glob("*.yaml"))}


def reset_grid(map_path: Path) -> None:
    st.session_state["editor_grid"] = load_grid(map_path)
    st.session_state["editor_base_map"] = str(map_path.resolve())


def ensure_grid(map_path: Path) -> list[list[int]]:
    if st.session_state.get("editor_base_map") != str(map_path.resolve()):
        reset_grid(map_path)
    return st.session_state["editor_grid"]


def prediction_from_settings(settings: dict[str, Any]) -> MovingObstaclePrediction | None:
    if not settings.get("use_prediction", False):
        return None
    return MovingObstaclePrediction(
        start_frame=int(settings["prediction_start"]),
        end_frame=int(settings["prediction_end"]),
        x=int(settings["prediction_x"]),
        y=int(settings["prediction_y"]),
        dx=int(settings["prediction_dx"]),
        dy=int(settings["prediction_dy"]),
    )


def selected_cell(selection: Any) -> tuple[int, int] | None:
    try:
        points = selection["selection"]["points"]
        if not points:
            return None
        point = points[-1]
        return int(point["x"]), int(point["y"])
    except (KeyError, TypeError, ValueError):
        return None


def grid_figure(grid: list[list[int]], start: tuple[int, int],
                goal: tuple[int, int],
                prediction: MovingObstaclePrediction | None) -> go.Figure:
    height = len(grid)
    width = len(grid[0])
    x_values = [x for y in range(height) for x in range(width)]
    y_values = [y for y in range(height) for _ in range(width)]
    colors = ["#202124" if grid[y][x] else "#f8f9fa"
              for y in range(height) for x in range(width)]
    marker_size = max(4, min(16, 700 / max(width, height)))
    figure = go.Figure()
    figure.add_trace(go.Scatter(
        x=x_values, y=y_values, mode="markers", name="grid",
        marker={"size": marker_size, "color": colors,
                "line": {"width": 0.25, "color": "#c7c7c7"}},
        customdata=[[x, y] for x, y in zip(x_values, y_values)],
        hovertemplate="cell (%{x}, %{y})<extra></extra>",
    ))
    figure.add_trace(go.Scatter(
        x=[start[0]], y=[start[1]], mode="markers", name="start",
        marker={"size": 14, "color": "#2ca02c", "symbol": "circle"},
    ))
    figure.add_trace(go.Scatter(
        x=[goal[0]], y=[goal[1]], mode="markers", name="goal",
        marker={"size": 14, "color": "#1f77b4", "symbol": "diamond"},
    ))
    if prediction is not None:
        positions = prediction.positions()
        figure.add_trace(go.Scatter(
            x=[point[0] for point in positions],
            y=[point[1] for point in positions], mode="lines+markers",
            name="predicted obstacle",
            marker={"size": 9, "color": "#ff7f0e"},
            line={"width": 2, "dash": "dot", "color": "#ff7f0e"},
        ))
    figure.update_layout(
        height=min(720, max(420, 9 * max(width, height))),
        margin={"l": 10, "r": 10, "t": 30, "b": 10},
        legend={"orientation": "h", "y": 1.03},
        dragmode="select",
    )
    figure.update_xaxes(range=[-1, width], dtick=max(1, width // 10),
                         showgrid=False, zeroline=False)
    figure.update_yaxes(range=[height, -1], dtick=max(1, height // 10),
                         showgrid=False, zeroline=False, scaleanchor="x")
    return figure


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
        "--velocity", str(settings["effective_velocity"]),
        "--robot-radius", str(settings["robot_radius"]),
        "--smooth", settings["smooth"],
        "--output_dir", str(output_dir),
        "--plot",
    ]
    if settings["controller"] == "mpc":
        command.extend(("--mpc-horizon", str(settings["mpc_horizon"])))

    if settings["engine"] == "dynamic":
        command.extend((
            "--frames", str(settings["effective_frames"]),
            "--steps-per-frame", str(settings["steps_per_frame"]),
            "--obstacle-ahead", str(settings["obstacle_ahead"]),
        ))
        if not settings["auto_obstacles"]:
            command.append("--no-auto-obstacles")
        prediction = settings.get("prediction")
        if prediction is not None:
            command.extend((
                "--moving-obstacle",
                *(str(value) for value in prediction.cli_values()),
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


def scene_setting_values(settings: dict[str, Any]) -> dict[str, Any]:
    keys = (
        "engine", "planner", "controller", "smooth", "velocity",
        "robot_radius", "mpc_horizon", "start_x", "start_y", "goal_x",
        "goal_y", "frames", "steps_per_frame", "obstacle_ahead",
        "auto_obstacles", "use_prediction",
    )
    return {key: settings[key] for key in keys if key in settings}


def apply_scene(scene: dict[str, Any], choices: dict[str, Path]) -> None:
    base_map = (REPO_ROOT / scene["base_map"]).resolve()
    matching_name = next(
        (name for name, path in choices.items() if path.resolve() == base_map),
        None,
    )
    if matching_name is None:
        raise ValueError(f"scene base map is not available: {scene['base_map']}")
    base_grid = load_grid(base_map)
    if (len(scene["grid"]) != len(base_grid)
            or len(scene["grid"][0]) != len(base_grid[0])):
        raise ValueError("scene grid dimensions do not match its base map")
    st.session_state["map_selector"] = matching_name
    st.session_state["editor_grid"] = scene["grid"]
    st.session_state["editor_base_map"] = str(base_map)
    for key, value in scene["settings"].items():
        st.session_state[key] = value
    prediction = scene["prediction"]
    if prediction is not None:
        st.session_state["use_prediction"] = True
        st.session_state["prediction_start"] = prediction.start_frame
        st.session_state["prediction_end"] = prediction.end_frame
        st.session_state["prediction_x"] = prediction.x
        st.session_state["prediction_y"] = prediction.y
        st.session_state["prediction_dx"] = prediction.dx
        st.session_state["prediction_dy"] = prediction.dy


def apply_editor_action(action: str, cell: tuple[int, int],
                        grid: list[list[int]]) -> None:
    x, y = cell
    if action == "Add obstacle":
        grid[y][x] = 1
    elif action == "Remove obstacle":
        grid[y][x] = 0
    elif action == "Set start":
        st.session_state["start_x"] = x
        st.session_state["start_y"] = y
    elif action == "Set goal":
        st.session_state["goal_x"] = x
        st.session_state["goal_y"] = y


def decision_text(decision: RiskDecision) -> str:
    action = {
        "monitor": "Monitor",
        "replan_slow": "Slow down and replan",
        "safe_stop": "Safe stop",
    }[decision.action]
    return (
        f"**Decision: {action}** · minimum corridor clearance: "
        f"{decision.minimum_clearance:.2f} cells · {decision.reason}"
    )


def main() -> None:
    st.set_page_config(page_title="RobotNav Lab", page_icon="🧭", layout="wide")
    st.title("RobotNav Lab")
    st.caption("ROS-free scenario editing, planning, tracking, and dynamic safety decisions.")

    choices = map_choices()
    if not choices:
        st.error(f"No maps found in {MAP_DIR}")
        return

    with st.sidebar:
        st.header("Scenes")
        saved_scenes = scene_choices()
        scene_name = st.selectbox(
            "Saved scene", ("", *saved_scenes), format_func=lambda name: name or "Select a scene"
        )
        if st.button("Load selected scene", disabled=not scene_name):
            try:
                apply_scene(load_scene(saved_scenes[scene_name]), choices)
            except (OSError, ValueError) as error:
                st.error(f"Could not load scene: {error}")
            else:
                st.rerun()

        st.header("Experiment")
        engine = st.selectbox("Pipeline", ENGINES, index=0, key="engine")
        map_name = st.selectbox("Map", tuple(choices), index=0, key="map_selector")
        planner = st.selectbox("Planner", PLANNERS, index=0, key="planner")
        controller = st.selectbox("Controller", CONTROLLERS, index=0, key="controller")
        smooth = st.selectbox("Path smoothing", ("shortcut", "none"), index=0, key="smooth")
        velocity = st.slider("Target velocity", 0.2, 2.0, 1.0, 0.1, key="velocity")
        robot_radius = st.slider("Robot radius", 0.0, 2.0, 1.0, 0.1, key="robot_radius")
        mpc_horizon = st.slider("MPC horizon", 5, 30, 15, key="mpc_horizon")

    map_path = choices[map_name]
    width, height = map_dimensions(map_path)
    grid = ensure_grid(map_path)
    st.info(f"Map: {map_name} · {width} × {height} cells · {sum(map(sum, grid))} occupied cells")

    control_column, prediction_column = st.columns(2)
    with control_column:
        st.subheader("Start and goal")
        start_x = st.number_input("Start x", 0, width - 1, 1, key="start_x")
        start_y = st.number_input("Start y", 0, height - 1, 1, key="start_y")
        goal_x = st.number_input("Goal x", 0, width - 1, width - 2, key="goal_x")
        goal_y = st.number_input("Goal y", 0, height - 1, height - 2, key="goal_y")

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
            st.subheader("Prediction and safety decision")
            st.caption("A constant-velocity forecast is screened before the D* Lite dynamic pipeline runs.")
            settings["auto_obstacles"] = st.checkbox("Enable automatic path obstacle", True, key="auto_obstacles")
            settings["use_prediction"] = st.checkbox("Enable predicted moving obstacle", True, key="use_prediction")
            settings["frames"] = st.slider("Update frames", 2, 60, 20, key="frames")
            settings["steps_per_frame"] = st.slider("Control steps per frame", 10, 120, 40, key="steps_per_frame")
            settings["obstacle_ahead"] = st.slider("Auto-obstacle path lookahead", 1, 20, 5, key="obstacle_ahead")
            if settings["use_prediction"]:
                frame_limit = settings["frames"]
                settings["prediction_start"] = st.number_input(
                    "Prediction start frame", 0, frame_limit - 1,
                    min(3, frame_limit - 1), key="prediction_start"
                )
                settings["prediction_end"] = st.number_input(
                    "Prediction end frame", 1, frame_limit,
                    min(15, frame_limit), key="prediction_end"
                )
                settings["prediction_x"] = st.number_input("Prediction x", 0, width - 1, width // 2, key="prediction_x")
                settings["prediction_y"] = st.number_input("Prediction y", 0, height - 1, height // 2, key="prediction_y")
                settings["prediction_dx"] = st.number_input("Prediction Δx / frame", -3, 3, 0, key="prediction_dx")
                settings["prediction_dy"] = st.number_input("Prediction Δy / frame", -3, 3, 1, key="prediction_dy")
    else:
        with prediction_column:
            st.subheader("Dynamic obstacle mode")
            st.caption("Select the Dynamic pipeline to supply a predicted obstacle trajectory and observe replanning.")

    prediction = prediction_from_settings(settings)
    prediction_is_valid = (
        prediction is not None and prediction.end_frame >= prediction.start_frame
    )
    if prediction is not None and not prediction_is_valid:
        st.error("Prediction end frame must not precede its start frame.")
    decision = (
        decide_prediction_risk(prediction, (start_x, start_y), (goal_x, goal_y), robot_radius)
        if prediction_is_valid and engine == "dynamic" else None
    )
    if decision is not None:
        if decision.action == "safe_stop":
            st.error(decision_text(decision))
        elif decision.action == "replan_slow":
            st.warning(decision_text(decision))
        else:
            st.success(decision_text(decision))
    settings["prediction"] = (
        prediction if decision is not None and decision.action == "replan_slow" else None
    )
    velocity_scale = decision.velocity_scale if decision else 1.0
    settings["effective_velocity"] = velocity * velocity_scale
    settings["effective_frames"] = min(
        60, math.ceil(settings["frames"] / max(velocity_scale, 0.1))
    )
    if decision is not None and decision.action == "replan_slow":
        st.caption(
            f"Dynamic run uses {settings['effective_velocity']:.2f} target velocity "
            f"and {settings['effective_frames']} frames to retain the control-time budget."
        )

    st.subheader("Scene editor")
    if width * height > EDITABLE_CELL_LIMIT:
        st.warning("This map is too large for point-and-click editing. Choose a map with at most 20,000 cells.")
    else:
        editor_column, action_column = st.columns((4, 1))
        with editor_column:
            selection = st.plotly_chart(
                grid_figure(grid, (start_x, start_y), (goal_x, goal_y), prediction),
                key="grid_editor", on_select="rerun", selection_mode="points",
                width="stretch",
            )
        with action_column:
            action = st.selectbox(
                "Selected-cell action",
                ("Add obstacle", "Remove obstacle", "Set start", "Set goal"),
            )
            cell = selected_cell(selection)
            st.caption(f"Selected: {cell}" if cell else "Click a cell on the map.")
            if st.button("Apply cell action", disabled=cell is None):
                apply_editor_action(action, cell, grid)
                st.rerun()

    with st.sidebar:
        st.header("Save scene")
        new_scene_name = st.text_input("Scene name", value="navigation_scene")
        if st.button("Save current scene"):
            safe_name = re.sub(r"[^A-Za-z0-9_-]+", "_", new_scene_name).strip("_")
            if not safe_name:
                st.error("Use a scene name containing letters, numbers, _ or -.")
            else:
                scene_prediction = prediction_from_settings(settings)
                try:
                    save_scene(
                        SCENES_DIR / f"{safe_name}.yaml",
                        relative_to_root(map_path), grid,
                        scene_setting_values(settings), scene_prediction,
                    )
                except (OSError, ValueError) as error:
                    st.error(f"Could not save scene: {error}")
                else:
                    st.success(f"Saved {safe_name}.yaml")

    build_binary = REPO_ROOT / "build" / "apps" / "navigation_pipeline_cli"
    if not build_binary.exists():
        st.warning("Build outputs are missing. Run `cmake -S . -B build` and `cmake --build build -j` first.")

    if st.button("Run experiment", type="primary", width="stretch"):
        if (start_x, start_y) == (goal_x, goal_y):
            st.error("Start and goal must be different.")
            return
        if grid[start_y][start_x] or grid[goal_y][goal_x]:
            st.error("Start and goal must be free cells in the edited map.")
            return
        if prediction is not None and prediction.end_frame < prediction.start_frame:
            st.error("Prediction end frame must not precede its start frame.")
            return
        if prediction is not None:
            end_x, end_y = prediction.positions()[-1]
            if not (0 <= end_x < width and 0 <= end_y < height):
                st.error("Predicted obstacle trajectory leaves the selected map.")
                return
        if decision is not None and decision.action == "safe_stop":
            st.error("Experiment not started because the prediction policy selected a safe stop.")
            return
        output_dir = output_directory()
        write_grid(output_dir / "scenario_map.txt", grid)
        settings["map"] = output_dir / "scenario_map.txt"
        command = build_command(settings, output_dir)
        with st.spinner("Running C++ navigation pipeline..."):
            completed = subprocess.run(
                command, cwd=REPO_ROOT, text=True, capture_output=True
            )
        render_results(output_dir, completed)


if __name__ == "__main__":
    main()
