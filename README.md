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
python3 -m pip install -e .
```

Optional physics validation backends can be installed in the local `CV`
environment. They are used for slower, contact-aware validation; the C++
kinematic simulator remains the default benchmark backend.

```bash
conda run -n CV python -m pip install 'mujoco>=3.3,<4' 'pybullet>=3.2.5,<3.3' 'gymnasium>=0.29'
conda run -n CV python autoplanner/scripts/physics_backend_smoke.py \
    --backend both --output autoplanner/results/physics_smoke.json
```

The physics smoke harness is headless and ROS-free. It uses the same velocity
and steering command convention in both engines and preserves the raw JSON
state trace for comparison.

For closed-loop validation, run the planner, C++ trajectory generator, and C++
controller against the physics backend:

```bash
PYTHONPATH=/tmp/robotnav-build-cv/python \
conda run --no-capture-output -n CV python -u \
    autoplanner/scripts/physics_tracking_benchmark.py \
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
PYTHONPATH=/tmp/robotnav-build-cv/python \
conda run --no-capture-output -n CV python -u \
    autoplanner/scripts/physics_dynamic_replanning.py \
    --controller mpc --frames 30 --steps-per-frame 80 \
    --output-dir autoplanner/results/physics_dynamic
```

## Python Experiment Workflow

Python is used for experiment orchestration and analysis; the planning and
tracking core remains C++. From the repository root:

```bash
python3 autoplanner/scripts/run_all_experiments.py \
    --build_dir build \
    --data_dir autoplanner/data \
    --output_dir autoplanner/results/benchmark \
    --repeat 3 --controllers stanley pure_pursuit mpc

python3 autoplanner/scripts/compare_results.py \
    autoplanner/results/benchmark

# End-to-end planning and tracking
python3 autoplanner/scripts/run_navigation_pipeline.py \
    --build_dir build \
    --map autoplanner/data/maps/simple_50x50.txt \
    --planner improved_astar \
    --controller stanley

# Finite-horizon MPC tracking
python3 autoplanner/scripts/run_navigation_pipeline.py \
    --build_dir build \
    --map autoplanner/data/maps/simple_50x50.txt \
    --planner improved_astar \
    --controller mpc --mpc-horizon 15

# Rectangle robot with conservative footprint inflation
python3 autoplanner/scripts/run_navigation_pipeline.py \
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

PYTHONPATH=build-python/python python3 - <<'PY'
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
python3 autoplanner/scripts/run_navigation_pipeline.py \
    --engine unified \
    --build_dir build \
    --map autoplanner/data/maps/simple_50x50.txt \
    --planner astar --controller stanley \
    --smooth none
```
