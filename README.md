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
./build/apps/autoplanner_cli --planner astar \
    --map autoplanner/data/maps/maze_100x100.txt --start 1 1 --goal 98 98

# Trajectory tracking
./build/examples/circle_tracking
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
portable across machines. Dynamic mode converts a predicted kinematic
obstacle trajectory into a monitor, slower D* Lite replan, or pre-flight safe
stop decision. A prediction can carry constant acceleration, a 2D position
covariance with per-frame growth, a confidence scale, and a moving-obstacle
radius. The dashboard exposes these together with the Space-Time A* risk
weight and clearance band. Existing six-field prediction scenes remain
loadable with zero-valued safety extensions.

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
Each run's `summary.json` also stores the prediction footprint, uncertainty,
and risk parameters used to construct the C++ command.

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

To compare the reusable local-planner pipeline, run the ROS-free DWA/MPPI
benchmark. It evaluates the same global plan with the nominal controller,
DWA, and MPPI, then repeats the combinations with a moving obstacle:

```bash
python autoplanner/scripts/benchmark_local_planners.py \
    --build_dir build \
    --output_dir autoplanner/results/local_planner_benchmark \
    --repeat 2

python autoplanner/scripts/compare_results.py \
    autoplanner/results/local_planner_benchmark
```

The benchmark writes `local_planner_results.csv`,
`dynamic_local_planner_results.csv`, the raw per-run pipeline artifacts, and
`local_planner_report.txt`. Metrics distinguish run success, goal reach,
safe-stop, control effort, MPPI rollout count, collision rejections, and
minimum predicted dynamic-obstacle clearance.

The navigation pipeline applies collision-safe shortcut smoothing by default;
pass `--smooth none` when the raw planner path is required.

MPPI reports `local_planner_time_ms` separately from global planning time. Its
rollout evaluator uses deterministic noise pre-generation and can use OpenMP
when an OpenMP C++ runtime is available; builds without OpenMP remain supported.

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
cmake --preset test
cmake --build --preset test
ctest --preset test
```

The `test` preset builds the C++ tests and registers the standard-library
Python regression tests with CTest. Dashboard tests are added automatically
when PyYAML, Plotly, and Streamlit are installed. To use a minimal C++-only
configuration, set `BUILD_PYTHON_TESTS=OFF`; to require all test dependencies,
keep the default `ROBOTNAV_REQUIRE_TEST_DEPENDENCIES=ON` in the test preset.

For an isolated manual build, use a dedicated binary directory rather than a
shared `build` directory:

```bash
cmake -S . -B build/manual-test -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=ON -DBUILD_PYTHON_TESTS=ON
cmake --build build/manual-test -j
ctest --test-dir build/manual-test --output-on-failure
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

For vehicle-executable paths, the pipeline also supports a curvature-constrained
smoother. It iteratively relaxes sharp turns while preserving collision-free
segments and fixed endpoints:

```bash
./build/apps/navigation_pipeline_cli \
    --scenario autoplanner/data/configs/navigation_pipeline.yaml \
    --start 1 1 --goal 20 20 \
    --smooth curvature --smooth-max-curvature 0.3 \
    --output-dir autoplanner/results/robotnav-curvature
```

An optional Dynamic Window Approach local planner can be inserted between the
controller and simulator. It samples velocity/steering commands, rolls them out
against the active collision checker, and executes the best collision-free
candidate:

```bash
./build/apps/navigation_pipeline_cli \
    --scenario autoplanner/data/configs/navigation_pipeline.yaml \
    --start 1 1 --goal 20 20 \
    --local-planner dwa --dwa-prediction-time 0.8 \
    --output-dir autoplanner/results/robotnav-dwa
```

For sampling-based model-predictive control, select the MPPI local planner.
It samples Gaussian control sequences around the controller command, rolls
each sequence through the kinematic bicycle model, scores path/heading/speed
tracking and control smoothness, and aggregates the first command with
temperature-weighted costs. The zero-noise nominal sequence is always kept as
a deterministic baseline; `--mppi-*` options expose the prediction horizon,
rollout count, temperature, and exploration noise:

```bash
./build/apps/navigation_pipeline_cli \
    --scenario autoplanner/data/configs/navigation_pipeline.yaml \
    --start 1 1 --goal 20 20 --local-planner mppi \
    --mppi-prediction-time 0.8 --mppi-horizon 12 \
    --mppi-rollouts 64 --mppi-temperature 0.5 \
    --mppi-velocity-noise 0.35 --mppi-steering-noise 0.12 \
    --output-dir autoplanner/results/robotnav-mppi
```

In the dynamic pipeline, MPPI additionally rejects static and predicted
dynamic collisions during every rollout and adds a configurable dynamic
clearance cost (`local_planner.mppi.dynamic_clearance`, default `0.5`) before
the hard collision threshold. This makes the optimizer react to moving
obstacles before they occupy the nominal path:

```bash
./build/apps/dynamic_navigation_pipeline_cli \
    --scenario autoplanner/data/configs/navigation_pipeline.yaml \
    --start 1 1 --goal 20 20 --controller stanley \
    --local-planner mppi --mppi-prediction-time 0.8 \
    --mppi-horizon 12 --mppi-rollouts 64 \
    --frames 20 --steps-per-frame 40 --no-auto-obstacles \
    --moving-obstacle 1 20 3 10 1 0 \
    --output-dir autoplanner/results/robotnav-dynamic-mppi
```

In the dynamic pipeline, the same DWA rollout also checks predicted moving
obstacle occupancy at intermediate times. Its safety margin and temporal
sampling can be configured from YAML or overridden at the CLI:

```bash
./build/apps/dynamic_navigation_pipeline_cli \
    --scenario autoplanner/data/configs/navigation_pipeline.yaml \
    --planner astar --controller stanley --local-planner dwa \
    --dwa-dynamic-obstacle-margin 0.25 \
    --dwa-dynamic-collision-samples 5 \
    --moving-obstacle 1 5 3 10 1 0 \
    --output-dir autoplanner/results/robotnav-dynamic-dwa
```

Dynamic metrics include the number of rejected local candidates and the
minimum predicted obstacle clearance, so replanning success is not the only
signal used to evaluate a run.

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
    --output_dir autoplanner/results/robotnav-dynamic --plot
```

The expected report has `status_code: "success"`,
`replanning_count >= 1`, `goal_reached: true`, and `safe_stop: false`.
Automatic obstacles use a conservative safety envelope and a reachability
check so the demonstration does not intentionally create an unsolvable map.
To model a simple moving obstacle, use `--moving-obstacle START END X Y DX DY`;
the obstacle occupies `(X, Y)` at `START` and then moves by `(DX, DY)` cells
per frame through `END`.
For a conservative footprint and prediction-aware Space-Time A* cost, add a
radius, uncertainty growth, and a soft risk band:

```bash
./build/apps/dynamic_navigation_pipeline_cli \
    --scenario autoplanner/data/configs/navigation_pipeline.yaml \
    --planner space_time_astar --prediction-horizon 80 \
    --prediction-risk-weight 2.0 --prediction-risk-clearance 1.5 \
    --moving-obstacle 1 20 3 10 1 0 \
    --moving-obstacle-radius 0.5 \
    --moving-obstacle-uncertainty-growth 0.05 \
    --output-dir autoplanner/results/robotnav-risk-aware
```

The hard collision envelope remains authoritative; the risk band adds cost to
otherwise feasible cells that pass too close to the predicted obstacle. The
same parameters are available from `benchmark_local_planners.py`.

For acceleration and covariance-aware prediction, append the following options
to the same command:

```bash
    --moving-obstacle-acceleration 0.1 0.0 \
    --moving-obstacle-covariance 0.25 0.0 0.25 \
    --moving-obstacle-covariance-growth 0.05 0.0 0.05 \
    --moving-obstacle-confidence-scale 2.0
```

The collision envelope shared by Space-Time A*, DWA, MPPI, and the dynamic
state supervisor is `radius + linear_growth + confidence_scale *
sqrt(lambda_max(covariance))`, with covariance evaluated at the queried frame.
Covariance and covariance growth must be finite positive-semidefinite 2D
matrices.
For prediction-aware planning, set the dynamic C++ planner to
`space_time_astar`; it plans in `(x, y, t)` and treats the moving obstacle
model as future occupancy:

```bash
./build/apps/dynamic_navigation_pipeline_cli \
    --scenario autoplanner/data/configs/navigation_pipeline.yaml \
    --planner space_time_astar --prediction-horizon 80 \
    --start 1 1 --goal 20 20 --controller stanley \
    --frames 20 --steps-per-frame 40 --no-auto-obstacles \
    --moving-obstacle 1 3 3 10 1 0 \
    --output-dir autoplanner/results/robotnav-spacetime
```

### Perception replay into the C++ dynamic pipeline

The ROS-free perception runner can replay simulated, CSV, or JSON sensor
frames, then export tracked obstacle motion as a stable CSV contract for the
C++ dynamic pipeline. The bridge emits `frame,cell_x,cell_y,occupied` updates;
real LiDAR adapters only need to produce the same `SensorFrame` interchange
format.

```bash
python autoplanner/scripts/run_perception_pipeline.py \
    --map autoplanner/data/maps/simple_50x50.txt \
    --start 1 1 --goal 20 20 --frames 20 \
    --dynamic-obstacle moving 7 4.5 0 0.5 0.6 \
    --cpp-build-dir build --cpp-planner space_time_astar \
    --cpp-local-planner dwa \
    --output-dir autoplanner/results/perception_pipeline
```

The runner writes `cpp_dynamic_updates.csv` and, when `--cpp-build-dir` is
provided, invokes `dynamic_navigation_pipeline_cli` and records its trace and
metrics below `cpp_dynamic/`. Use `--sensor-csv` or `--sensor-json` to replay
external sensor data instead of the built-in simulator.

For externally driven updates, repeat `--obstacle FRAME X Y` (or use
`--clear-obstacle FRAME X Y`) and disable the demo generator with
`--no-auto-obstacles`:

```bash
python autoplanner/scripts/run_navigation_pipeline.py \
    --engine dynamic --build_dir build \
    --map autoplanner/data/maps/simple_50x50.txt \
    --start 1 1 --goal 20 20 --controller stanley --smooth none \
    --frames 20 --steps-per-frame 40 --no-auto-obstacles \
    --obstacle 1 3 10 --output_dir autoplanner/results/robotnav-external-dynamic --plot
```

When invoking the lower-level dynamic C++ CLI directly, use the same
visualizer on its existing trace artifacts:

```bash
./build/apps/dynamic_navigation_pipeline_cli \
    --scenario autoplanner/data/configs/navigation_pipeline.yaml \
    --start 1 1 --goal 20 20 --controller stanley \
    --frames 20 --steps-per-frame 40 --no-auto-obstacles \
    --moving-obstacle 1 3 3 10 1 0 \
    --local-planner dwa --dwa-prediction-time 0.8 \
    --output-dir autoplanner/results/robotnav-dynamic

python autoplanner/scripts/visualize_navigation_trace.py \
    --map autoplanner/data/maps/simple_50x50.txt \
    --trace autoplanner/results/robotnav-dynamic/trace.csv \
    --metrics autoplanner/results/robotnav-dynamic/metrics.json \
    --output autoplanner/results/robotnav-dynamic/trace.png
```
