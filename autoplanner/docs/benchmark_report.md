# Benchmark Report

## Purpose

RobotNav benchmarks the complete navigation workflow using measured outputs:

```text
planner -> collision-checked path -> reference trajectory -> controller
```

The benchmark does not encode expected algorithm rankings. Each run stores the
raw path, trajectory, tracking CSV, per-run JSON metrics, and combined CSV
ledgers so that conclusions can be regenerated from the actual data.

## Metrics

### Planning

| Field | Meaning |
|---|---|
| `success` | Planner returned a valid result |
| `collision_free` | Final path passed the selected footprint checker |
| `planning_time_ms` | Measured planning time |
| `path_length` | Euclidean length of the final path |
| `minimum_obstacle_distance` | Minimum distance from path points to occupied cells |
| `turning_count` | Number of non-collinear direction changes |
| `average_curvature` | Mean absolute discrete curvature |
| `smoothness_score` | Straight-line distance divided by path length |

### Tracking

Both Stanley and MPC are evaluated on the same saved planner path and the same
generated trajectory. The benchmark records maximum and mean cross-track error,
maximum and mean heading error, goal distance, and whether the goal was reached.

### Dynamic replanning

The `dynamic_navigation` executable inserts an obstacle ahead of the robot on
several frames. It automatically checks whether the current path is invalid,
triggers D* Lite, runs a fresh A* plan on the same map as a baseline, updates
the reference trajectory, and continues control from the previous command.
The per-step CSV and JSON summary record replan count, D*/A* timing, steering
and velocity jumps, and replan-time control continuity.
If incremental replanning fails, the summary marks `safe_stop=true` and keeps
the run as a failed dynamic-navigation case.

## Reproduce

From the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

python autoplanner/scripts/run_all_experiments.py \
    --build_dir build \
    --data_dir autoplanner/data \
    --output_dir autoplanner/results/benchmark \
    --repeat 3 \
    --controllers stanley pure_pursuit mpc \
    --smooth shortcut

python autoplanner/scripts/compare_results.py \
    autoplanner/results/benchmark \
    --output autoplanner/results/benchmark/report.txt
```

The runner writes:

```text
autoplanner/results/benchmark/
├── planning_results.csv
├── tracking_results.csv
├── dynamic_replanning_results.csv
├── benchmark_manifest.json
└── <planner>_<map>_<repeat>/
    ├── planning/path.csv
    ├── planning/metrics.json
    ├── tracking/<controller>/tracking.csv
    ├── tracking/<controller>/trajectory.csv
    └── tracking/<controller>/tracking_metrics.json
```

Use `--repeat 1` for a quick smoke benchmark. Sampling-based planners should
use multiple repeats because their measured results are stochastic.

## Interpretation rules

- Report success rates over all attempted runs.
- Report path and timing averages over successful planning runs only.
- Report tracking metrics by planner and controller separately.
- Keep failed runs in the CSV ledger; do not replace them with expected values.
- Compare planners under the same map, start/goal, footprint, smoothing mode,
  and controller configuration.
