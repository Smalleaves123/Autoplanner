# RobotNav

Monorepo for mobile robot navigation algorithms — path planning + trajectory tracking.

## Projects

| Project | Description |
|---------|-------------|
| [autoplanner](autoplanner/) | C++ path planning library: A\*, Dijkstra, RRT, RRT\*, JPS, Hybrid A\* and more |
| [autompc](autompc/) | C++ trajectory tracking: PID, Pure Pursuit, Stanley, LQR |

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# Path planning
./build/autoplanner/apps/autoplanner_cli --planner astar \
    --map autoplanner/data/maps/maze_100x100.txt --start 1 1 --goal 98 98

# Trajectory tracking
./build/autompc/examples/circle_tracking
```

To build/install the Python package:

```bash
python -m pip install -e '.[tools]'
```

Optional physics validation backends are used for slower, contact-aware
validation; the C++ kinematic simulator remains the default benchmark backend.

```bash
python -m pip install -e '.[simulation]'
python autoplanner/scripts/physics_backend_smoke.py \
    --backend both --output autoplanner/results/physics_smoke.json
```

The physics smoke harness is headless and ROS-free. It uses the same velocity
and steering command convention in both engines and preserves the raw JSON
state trace for comparison.

For closed-loop validation, run the planner, C++ trajectory generator, and C++
controller against the physics backend:

```bash
python autoplanner/scripts/physics_tracking_benchmark.py \
    --backend both --controller mpc \
    --output-dir autoplanner/results/physics_tracking
```

PyBullet uses the bundled four-wheel racecar model by default. The planar
force model remains available for controlled ablation with
`--pybullet-model planar`.

This records one CSV and one JSON report per backend. `goal_reached` is kept
separate from `run_success`, so a physically unstable or incomplete run is
preserved as evidence rather than being presented as a successful demo.

Dynamic physical replanning uses the same occupancy grid for collision geometry
and D* Lite updates:

```bash
python autoplanner/scripts/physics_dynamic_replanning.py \
    --controller mpc --frames 30 --steps-per-frame 80 \
    --output-dir autoplanner/results/physics_dynamic
```

## Python Experiment Workflow

Python is used for experiment orchestration and analysis; the planning and
tracking core remains C++. From the repository root:

### Zero-argument utility demos

Every Python utility has runnable defaults. Plotting and reporting helpers use
the versioned samples in `autoplanner/data/demos`, while generated outputs are
written below `autoplanner/results`.

```bash
python autoplanner/scripts/visualize_path.py
python autoplanner/scripts/visualize_navigation_trace.py
python autoplanner/scripts/plot_benchmark.py
python autoplanner/scripts/make_gif.py
python autompc/scripts/plot_tracking.py
```

### Local interactive experiment lab

The ROS-free Streamlit dashboard selects maps, planners, controllers, start
and goal cells, then runs the existing C++ pipeline and displays its trace,
metrics, and plot. Its point-and-click editor adds/removes occupancy cells or
sets start/goal cells; saved YAML scenes contain the complete grid and remain
portable across machines. Dynamic mode converts a predicted constant-velocity
obstacle trajectory into a monitor, slower D* Lite replan, or pre-flight safe
stop decision.

Install the optional dashboard dependency once:

```bash
python -m pip install -e '.[tools,dashboard]'
```

```bash
python autoplanner/scripts/launch_dashboard.py
```

It opens at `http://localhost:8501`.

### Reproducible scenario regression

Dashboard YAML scenes can be executed without the web UI. The zero-argument
command runs a bundled dynamic prediction example and writes a CSV ledger with
the decision, velocity scale, replanning count, safety result, and output path.

```bash
python autoplanner/scripts/run_saved_scenarios.py
```

Run one or more saved scenes repeatedly for a local regression batch:

```bash
python autoplanner/scripts/run_saved_scenarios.py \
    --scene autoplanner/results/dashboard/scenes/loading_bay.yaml \
    --scene autoplanner/results/dashboard/scenes/crossing.yaml \
    --repeat 3 --plot
```

```bash
python autoplanner/scripts/run_all_experiments.py \
    --build_dir build \
    --data_dir autoplanner/data \
    --output_dir autoplanner/results/benchmark \
    --repeat 3 --controllers stanley pure_pursuit mpc

python autoplanner/scripts/compare_results.py \
    autoplanner/results/benchmark

# End-to-end planning and tracking
python autoplanner/scripts/run_navigation_pipeline.py \
    --build_dir build \
    --map autoplanner/data/maps/simple_50x50.txt \
    --planner improved_astar \
    --controller stanley

# Finite-horizon MPC tracking
python autoplanner/scripts/run_navigation_pipeline.py \
    --build_dir build \
    --map autoplanner/data/maps/simple_50x50.txt \
    --planner improved_astar \
    --controller mpc --mpc-horizon 15

# Rectangle robot with conservative footprint inflation
python autoplanner/scripts/run_navigation_pipeline.py \
    --build_dir build \
    --map autoplanner/data/maps/simple_50x50.txt \
    --planner astar \
    --controller pure_pursuit \
    --footprint rectangle --robot-length 2.0 --robot-width 1.0 --inflate
```

The batch runner invokes the C++ planner CLI and collects the machine-readable
`metrics.json` output, making it easy to add maps, planners, repeats, and plots
without changing the C++ benchmark code.

The navigation pipeline applies collision-safe shortcut smoothing by default;
pass `--smooth none` when the raw planner path is required.

## Python C++ Backend

Python bindings are optional. Python handles orchestration and analysis while
the planning and MPC kernels execute in C++ with the GIL released:

```bash
cmake -S . -B build-python -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF -DBUILD_PYTHON_BINDINGS=ON
cmake --build build-python -j

PYTHONPATH=build-python/python python - <<'PY'
import autoplanner
import autompc

grid = autoplanner.GridMap()
grid.load_from_txt("autoplanner/data/maps/simple_50x50.txt")
result = autoplanner.plan(
    "astar", grid,
    autoplanner.Point2i(1, 1), autoplanner.Point2i(48, 48))
print(result.success, result.path_length)
PY
```

## Test

```bash
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

For reproducible local configurations, use the CMake presets:

```bash
cmake --preset release
cmake --build --preset release
ctest --preset release

# Optional address/undefined-behaviour sanitizer run
cmake --preset asan-ubsan
cmake --build --preset asan-ubsan
ctest --preset asan-ubsan
```

The repository also exposes `format-check`, `format`, and `quality-check`
targets when `clang-format` is installed. Static analysis is opt-in so the
normal build remains portable across developer machines:

```bash
cmake -S . -B build-quality -DROBOTNAV_ENABLE_CLANG_TIDY=ON
cmake --build build-quality
cmake --build build-quality --target quality-check
```

Every CI run keeps the C++ test result, sanitizer result, and Python package
smoke result as separate gates. A passing demo is not treated as a substitute
for these checks.

## RobotNav Pipeline

```
autoplanner (path planning) → reference trajectory → autompc (tracking) → control commands
```

## Unified ROS-Free Navigation Pipeline

The top-level `robotnav_pipeline` library combines planning, trajectory
generation, controller execution, safety checks, and machine-readable trace
output in one reusable C++ API. It is exposed through a standalone CLI and
does not require ROS:

```bash
cmake --build build -j
./build/apps/navigation_pipeline_cli \
    --scenario autoplanner/data/configs/navigation_pipeline.yaml \
    --output-dir autoplanner/results/navigation_pipeline
```

The output directory contains `trace.csv` with state and command samples and
`metrics.json` with status, goal, tracking, and safety metrics. Use
`--planner`, `--controller`, `--start`, and `--goal` to override scenario
values for a quick local experiment.

The existing Python orchestrator can select this unified C++ engine while
keeping its legacy planner/tracker mode available:

```bash
python autoplanner/scripts/run_navigation_pipeline.py \
    --engine unified \
    --build_dir build \
    --map autoplanner/data/maps/simple_50x50.txt \
    --planner astar --controller stanley \
    --smooth none --plot
```

`--plot` integrates visualization into the existing pipeline: it writes
`navigation.png` beside `trace.csv`, `metrics.json`, and `summary.json`; the
summary records `plot_png`. The legacy pipeline supports the same switch and
also overlays its planner path and controller reference trajectory.

### Dynamic replanning

The dynamic pipeline adds automatic obstacle updates, D* Lite incremental
replanning, optional A* timing comparison, controller reset/reference
continuation, safety supervision, and `trace.csv`/`metrics.json` artifacts.
It is also ROS-free. Run it through the same Python entry point to create
`navigation.png` automatically:

```bash
python autoplanner/scripts/run_navigation_pipeline.py \
    --engine dynamic --build_dir build \
    --map autoplanner/data/maps/simple_50x50.txt \
    --start 1 1 --goal 20 20 --planner astar --controller stanley \
    --smooth shortcut --frames 20 --steps-per-frame 40 \
    --obstacle-ahead 15 --obstacle-margin 1 --max-auto-obstacles 1 \
    --output_dir /tmp/robotnav-dynamic --plot
```

The expected report has `status_code: "success"`,
`replanning_count >= 1`, `goal_reached: true`, and `safe_stop: false`.
Automatic obstacles use a conservative safety envelope and a reachability
check so the demonstration does not intentionally create an unsolvable map.
To model a simple moving obstacle, use `--moving-obstacle START END X Y DX DY`;
the obstacle occupies `(X, Y)` at `START` and then moves by `(DX, DY)` cells
per frame through `END`.
For externally driven updates, repeat `--obstacle FRAME X Y` (or use
`--clear-obstacle FRAME X Y`) and disable the demo generator with
`--no-auto-obstacles`:

```bash
python autoplanner/scripts/run_navigation_pipeline.py \
    --engine dynamic --build_dir build \
    --map autoplanner/data/maps/simple_50x50.txt \
    --start 1 1 --goal 20 20 --controller stanley --smooth none \
    --frames 20 --steps-per-frame 40 --no-auto-obstacles \
    --obstacle 1 3 10 --output_dir /tmp/robotnav-external-dynamic --plot
```

When invoking the lower-level dynamic C++ CLI directly, use the same
visualizer on its existing trace artifacts:

```bash
python autoplanner/scripts/visualize_navigation_trace.py \
    --map autoplanner/data/maps/simple_50x50.txt \
    --trace /tmp/robotnav-dynamic/trace.csv \
    --metrics /tmp/robotnav-dynamic/metrics.json \
    --output /tmp/robotnav-dynamic/trace.png
```
