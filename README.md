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

## Python Experiment Workflow

Python is used for experiment orchestration and analysis; the planning and
tracking core remains C++. From the repository root:

```bash
python3 autoplanner/scripts/run_all_experiments.py \
    --build_dir build \
    --output_dir autoplanner/results/benchmark

python3 autoplanner/scripts/compare_results.py \
    autoplanner/results/benchmark/all_results.csv

# End-to-end planning and tracking
python3 autoplanner/scripts/run_navigation_pipeline.py \
    --build_dir build \
    --map autoplanner/data/maps/simple_50x50.txt \
    --planner improved_astar \
    --controller stanley

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

## Test

```bash
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## RobotNav Pipeline

```
autoplanner (path planning) → reference trajectory → autompc (tracking) → control commands
```
