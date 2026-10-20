# RobotNav

Monorepo for mobile robot navigation algorithms — path planning + trajectory tracking.

RobotNav is intentionally ROS-free. Its integration surfaces are the C++ API,
Python package, command-line tools, versioned replay files, and simulation
backends. See the [development roadmap](autoplanner/docs/roadmap.md).

## Projects

| Project | Description |
|---------|-------------|
| [autoplanner](autoplanner/) | C++ path planning library: A\*, Dijkstra, RRT, RRT\*, JPS, Hybrid A\* and more |
| [autompc](autompc/) | C++ trajectory tracking and actuator-limited bicycle/differential-drive simulation |

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
controller against every execution backend:

```bash
python autoplanner/scripts/physics_tracking_benchmark.py \
    --backend all --controller mpc \
    --output-dir autoplanner/results/physics_tracking
```

`all` executes the C++ constrained kinematic bicycle, MuJoCo planar model, and
PyBullet model. Use `--backend kinematic` for the fast dependency-free
execution backend, or the backward-compatible `both` alias for only MuJoCo and
PyBullet.

PyBullet uses the bundled four-wheel racecar model by default. The planar
force model remains available for controlled ablation with
`--pybullet-model planar`.

This records one CSV trace and one versioned, backend-neutral JSON run artifact
per backend, plus `experiment_manifest.json` for comparison and replay. Each
artifact carries the complete route, component selection, simulator model,
actuator limits, seed, and common metrics. `goal_reached` is kept separate from
`run_success`, so a physically unstable or incomplete run is preserved as
evidence rather than being presented as a successful demo.

Deterministic robustness suites exercise observation noise, delayed state
feedback, command saturation, and map-valid randomized moving agents. The
default uses the dependency-free kinematic backend; select `all` after
installing the optional simulation dependencies.

```bash
python autoplanner/scripts/run_validation_suite.py \
    --suite robustness --backend kinematic --repeats 3 \
    --seed 42 --build-dir build \
    --output-dir autoplanner/results/validation_suite
```

The runner writes the exact generated cases to `validation_suite.json` before
execution and records every command, return code, outcome, and artifact path in
`validation_execution.json`. Navigation failures are results rather than
runner failures, so the remaining cases still execute; add `--fail-on-outcome`
when using the suite as a pass/fail gate. `--dry-run` generates a fully
inspectable plan without launching a backend.

Every execution also produces `validation_results.csv` and
`validation_report.json`. The ledger retains incomplete runs and reports each
case/backend's success, collision, safe-stop, goal time, integrated squared
control effort, and control-compute P50/P95/P99 latency. The JSON report adds
aggregate rates and percentiles using raw per-cycle latency samples from all
available traces.

Failed outcomes are minimized by default. The runner greedily removes
unnecessary perturbation dimensions or moving agents, verifies that the
smaller case still fails, and stores a versioned bundle under `failures/` with
captured map/path inputs and run artifacts. Replay a bundle into a clean output
directory with:

```bash
python autoplanner/scripts/replay_failure.py \
    autoplanner/results/validation_suite/failures/CASE/failure_case.json \
    --output-dir autoplanner/results/failure_replay
```

The replay command returns success only when the expected failure is reproduced.
Use `--no-minimize-failures` on the suite runner when only exact preservation is
needed and extra minimization executions are undesirable.

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

The recommended user-facing entry point is the pure-Python `robotnav` facade.
It accepts paths or native map objects, validates common inputs, and returns a
structured `PlanResult` while keeping the algorithm execution in C++:

```python
from robotnav import PlannerConfig, plan

result = plan(
    "autoplanner/data/maps/simple_50x50.txt",
    start=(1, 1), goal=(20, 20),
    config=PlannerConfig(
        planner="improved_astar",
        robot_radius=0.0,
    ),
)
print(result.success, result.status_code, result.path_length)
```

The same result can be passed directly to AutoMPC trajectory generation and
closed-loop simulation:

```python
import robotnav

trajectory = robotnav.generate_trajectory(
    result, robotnav.TrajectoryConfig(target_velocity=1.0))
controller = robotnav.Controller(
    robotnav.ControllerConfig(controller="stanley"))
simulation = robotnav.simulate(
    robotnav.RobotState(1.0, 1.0), trajectory, controller, max_time=5.0)
print(simulation.metrics.max_cross_track)
```

For multi-entity experiments, `SimulationWorld` provides one deterministic
clock, reset/step semantics, backend selection, immutable observations, and
pairwise collision reports:

```python
world = robotnav.SimulationWorld(
    robotnav.WorldConfig(
        simulation=robotnav.SimulationConfig(
            execution_model="differential_drive"),
        seed=7,
    ),
    [robotnav.WorldEntity("robot", robotnav.RobotState(1.0, 1.0))],
)
observation = world.step({
    "robot": robotnav.TwistCommand(0.8, 0.0),
})
print(observation.clock.time, observation.collisions)
```

The world facade is intentionally small: map layers, sensors, and higher-level
controllers can consume its immutable snapshots without depending on the
selected native execution backend.

Sensor simulation is available through `SensorSuite`. It currently provides
2-D LiDAR, odometry, IMU, and a 2.5-D depth image over the same world clock.
Every packet includes a sequence, event timestamp, arrival timestamp, and
sensor-specific payload; timing can inject deterministic latency, drops, and
timestamp jitter:

```python
sensors = robotnav.SensorSuite(
    "autoplanner/data/maps/simple_50x50.txt",
    lidars=[robotnav.Lidar2DConfig(
        timing=robotnav.SensorTiming(latency_steps=1),)],
    odometries=[robotnav.OdometryConfig()],
    imus=[robotnav.ImuConfig()],
    depth_cameras=[robotnav.DepthCameraConfig()],
    seed=42,
)
batch = sensors.step(world.observation)
```

Generate a replayable JSON sensor stream with:

```bash
PYTHONPATH=build/python-build/python \
python autoplanner/scripts/simulate_sensor_stream.py \
  --steps 100 --latency-steps 1 --drop-probability 0.05 \
  --output autoplanner/results/sensors/sensor_stream.json
```

The stream writer flushes delayed packets at the end of a run.  Replay the
same artifact directly through the occupancy/tracking/replanning pipeline:

```bash
python autoplanner/scripts/run_perception_pipeline.py \
  --map autoplanner/data/maps/simple_50x50.txt \
  --sensor-json autoplanner/results/sensors/sensor_stream.json \
  --start 2 2 --goal 12 12 \
  --output-dir autoplanner/results/sensors/perception-replay
```

The loader still accepts the older `{"frames": [...]}` JSON and CSV formats,
while packet streams retain `timestamp_ns` and `arrival_timestamp_ns` on each
LiDAR frame.  Every replay run also writes `experiment_run.json`, a standard
`robotnav.experiment.run` artifact containing sensor parameters and references
to the stream, trace, and replay manifest; pass `--experiment-artifact` to
store it at a custom path.

`LidarScan.to_perception_frame()` emits the endpoint format already consumed
by the Python perception mapping and replay helpers.  Other simulated world
entities are ray-cast as `dynamic` returns (their configured collision radius
is carried in the immutable world observation), so the same packet can feed
obstacle tracking and prediction.

Run the complete ROS-free feedback loop with:

```bash
PYTHONPATH=build/python-build/python \
python autoplanner/scripts/simulate_sensor_navigation.py \
  --steps 100 --latency-steps 1 --drop-probability 0.05 \
  --output build/sensor-navigation/trace.json
```

This runner advances `World -> SensorSuite -> PlanarEkf -> OccupancyGrid/tracker ->
D* Lite -> control -> World`. Odometry velocity and yaw increments are fused
with IMU yaw rate; IMU acceleration drives prediction. Planning and control
use the estimate, and LiDAR rays are projected using the estimated **capture-time**
pose, not the ground-truth pose embedded in a simulation scan. The initial
pose hint and static map are assumed known. World truth is retained for evaluation;
dynamic-return labels are still provided by the simulator.

```python
ekf = robotnav.PlanarEkf(
    robotnav.RobotState(2.5, 2.5), robotnav.EkfConfig(history_seconds=5.0))
for packet in batch.ready:
    if isinstance(packet, (robotnav.OdometryMeasurement, robotnav.ImuMeasurement)):
        ekf.update(packet)
current = ekf.estimate_at(int(round(batch.clock.time * 1e9)))
# For a delayed LiDAR packet:
# frame = scan.to_perception_frame(pose=ekf.estimate_at(scan.timestamp_ns).state)
```

`PlanarEkf` uses only the standard library, maintains a 5-by-5 covariance
(6-by-6 with either IMU bias state, 7-by-7 with both),
and deterministically replays late observations within bounded history. It
rejects duplicates, wrong-entity, and expired packets. Without pose corrections
it is a dead-reckoning baseline. The demo now defaults to `--localization
scan_matching`: bounded local search and point-to-line ICP align raw LiDAR
rays to exposed static-map boundaries, then feed `PoseMeasurement` into the
same EKF history. Dynamic-labeled returns, invisible wall back faces, occluded
surfaces, and geometrically degenerate fits are excluded/rejected. Set
`--localization dead_reckoning` for the previous baseline.

This is local known-map localization, **not** SLAM or global relocalization.
Isolated failed matches fall back to dead reckoning with reduced speed;
repeated failures or stale corrections stop the guarded demo. Pose measurement standard deviations
are conservative tuning parameters, not an exact ICP covariance; repeated
scans can share systematic map/ray errors. The demo enables gyro bias estimation
by default (`--no-estimate-gyro-bias` retains the five-state baseline unless
acceleration bias estimation is enabled separately). IMU yaw
rate is modeled as true rate plus bias; unbiased odometry or map pose corrections
are needed to distinguish the two. `--estimate-acceleration-bias` also estimates
forward acceleration bias using odometry velocity or map-position references.
This does not fix shared systematic reference errors or eliminate absolute
dead-reckoning drift, and is not a full 3-D inertial navigation model.
The demo emits stop commands when motion
or LiDAR data exceeds `--max-estimate-age` / `--max-scan-age` (seconds). This
freshness check is not a complete collision-safety supervisor.

`LocalizationHealthMonitor` adds observable health rules without reading truth:
map-match confirmation, correction-jump gating, failure/recovery streaks,
capture-time freshness, and EKF position/heading uncertainty. The demo enables
`--localization-health` by default: `initializing`/`lost` stop, `degraded`
scales velocity to 0.35, and `tracking` uses nominal speed. Two consecutive
fused matches initialize/recover; three consecutive failures mark map tracking
lost. Default age limits are 0.5 s motion/LiDAR and 2 s map correction; default
correction-jump limits are 0.6 map units / 0.15 rad. Tune with
`--matches-before-recovery`, `--match-failures-before-lost`,
`--max-map-correction-age`, `--max-position-std`, `--max-heading-std`,
`--max-correction-translation`, and `--max-correction-heading`.
The map-correction age must accommodate match frequency and delivery latency.
`--no-localization-health` retains the earlier freshness-only control and
ungated map fusion while still recording health diagnostics. Dead-reckoning
mode does not require map corrections. The JSON includes per-step diagnostics,
transition reasons, correction assessments, and gate counts.

Run deterministic healthy, dynamic-only, single-wall, teleport, LiDAR-loss,
motion-loss, and repeated-room cases:

```bash
PYTHONPATH=build/python-build/python \
python autoplanner/scripts/benchmark_localization_health.py \
  --output build/sensor-navigation/localization-health-benchmark.json
```

This benchmark uses an open-loop stationary trajectory, introduces faults after
two initial samples, and removes non-alias faults before recovery checks.
Returning the teleport case to its original pose is **not** global relocalization.
The repeated-room case intentionally demonstrates wrong-but-healthy tracking:
small covariance and a good local match cannot resolve identical rooms. Health
states are heuristics, not correctness probabilities or a safety guarantee.

`GridRelocalizer` adds a separate known-map global-search prototype. It scores
free-cell/heading lattice seeds against static rays, refines multiple modes
with `GridScanMatcher`, and reports competing hypotheses. No-hit rays penalize
unexpected walls; dynamic beams are ignored. A close competing mode returns
`ambiguous` with **no selected pose**, rather than resetting to an arbitrary
room. Oversized search lattices return `search_budget_exceeded` before allocation.

```python
relocalizer = robotnav.GridRelocalizer(map_path)
result = relocalizer.localize(scan)  # does not read scan.pose or reset EKF
if result.accepted:
    candidate = result.pose  # candidate among searched modes, not proven unique
```

Run persistent-teleport and ambiguity baselines:

```bash
PYTHONPATH=build/python-build/python \
python autoplanner/scripts/benchmark_relocalization.py \
  --output build/sensor-navigation/relocalization-benchmark.json
```

This benchmark keeps the new pose after teleport: unlike the health benchmark,
recovery does not depend on returning truth to its original location. It
explicitly resets EKF only for an accepted global candidate; ambiguous/unsupported
scans leave the estimator unchanged. `RelocalizationConfig` controls lattice
spacing/headings, CPU budgets, mode clustering, ray-score cutoff, and ambiguity
margin. Finite grid sampling and top-seed refinement can miss alternatives;
scores are not probabilities, and one scan does not establish temporal stability.
Global search is **not yet automatically connected** to the navigation runner.
The ordinary local correction gate remains in force. This is not SLAM, AMCL,
or a real-time global-search guarantee.

`RelocalizationTracker` associates successive global modes using capture-time
relative SE(2) motion from a separate dead-reckoning stream. It requires three
consecutive **unique**, motion-consistent scan results before publishing a
`confirmed` pose. Duplicates/out-of-order timestamps cannot count as evidence;
search failures, gaps over 1 s, motion inconsistency and ambiguity interrupt
unique confirmation. Ambiguous modes retain diagnostic track IDs but do not
become unique simply through repeated observation.

`RelocalizationEpoch` provides an explicit stationary-only handoff for its
owned EKF/health monitor. `apply(decision, now_ns=..., epoch_id=...,
stationary=True)` requires a fresh confirmed candidate in the current epoch,
preserves enabled IMU bias means, resets pose/cross-frame covariance and health,
and returns required caller resets (motion frame, mapping, obstacle tracks,
planner, control). Stationarity is a **caller-established condition**, not
inferred from a candidate's zero velocity. It rejects confirmations older than
0.5 s, reused/future decisions and epoch mismatches. Moving candidates may be
confirmed diagnostically but are not supported for this handoff.

Tag each packet with its epoch at capture/enqueue. Route motion through
`update_motion(packet, epoch_id=tag)` and check delayed scans through
`accept_packet(scan, epoch_id=tag)` before other consumers; do not relabel old
queued packets at arrival. After a switch, old tags and capture timestamps at
or before the new boundary are rejected. External queues and components are
still caller-owned; automatic navigation recovery is not wired in yet, and
health must collect fresh observations before motion resumes.

```bash
PYTHONPATH=build/python-build/python \
python autoplanner/scripts/benchmark_relocalization_tracking.py \
  --output build/sensor-navigation/relocalization-tracking-benchmark.json
```

This benchmark uses independent global searches on four noisy scans per case,
sensor-derived relative motion, stationary recovery with delayed-packet
rejection, moving candidate confirmation without application, and persistent
room ambiguity without arbitrary reset.

For a noisy, delayed straight-route example:

```bash
PYTHONPATH=build/python-build/python \
python autoplanner/scripts/simulate_sensor_navigation.py \
  --steps 100 --goal 8 2 --latency-steps 2 --drop-probability 0.15 \
  --timestamp-jitter-ns 5000000 --initial-estimate-offset 0.3 0.2 0.05 \
  --localization scan_matching --output build/sensor-navigation/localized.json
```

The deterministic JSON trace includes truth/estimated trajectories, covariance,
position RMSE, heading errors, stale-input stops, dropped/rejected packets,
tracks, replans, configuration, match quality/rejection reasons, pose correction
counts, and goal status. `--scan-match-every` controls correction frequency;
`--scan-match-translation-window` / `--scan-match-rotation-window` bound local
search, and `--scan-match-position-std` / `--scan-match-heading-std` tune EKF
correction strength. Complex routes can still
reach the step limit with the demo's simple waypoint controller.

IMU simulation supports signed `ImuConfig.angular_velocity_bias` (rad/s) and
`angular_velocity_bias_random_walk_std` (rad/s/sqrt(s)). The seeded random walk
uses physical simulation time, evolves through dropped packets, and resets
reproducibly. `SensorSuite.imu_biases` is evaluation-only truth, never included
in measurement packets or consumed by the EKF. Both sensor runners expose
`--imu-gyro-bias` / `--imu-gyro-bias-random-walk`; navigation additionally exposes
`--ekf-gyro-bias-process-std`, estimated bias and uncertainty in JSON.

`ImuConfig.acceleration_bias` (map units/s²) and
`acceleration_bias_random_walk_std` add independent forward-acceleration
offset/random walk with the same timing/reset guarantees. Simulator truth is
available only via `SensorSuite.imu_acceleration_biases`. Enable
`EkfConfig(estimate_acceleration_bias=True)` independently of gyro estimation;
`StateEstimate.acceleration_bias` / `acceleration_bias_std` are `None` when
disabled. Navigation exposes `--imu-acceleration-bias`,
`--imu-acceleration-bias-random-walk`, `--estimate-acceleration-bias`, and
`--ekf-acceleration-bias-process-std`; acceleration bias estimation is opt-in.

For an identical-observation comparison (open-loop varying turns, noisy
unbiased odometry, latency and drops; no map pose corrections):

```bash
PYTHONPATH=build/python-build/python \
python autoplanner/scripts/benchmark_imu_bias.py --bias 0.08 \
  --output build/sensor-navigation/imu-bias-benchmark.json
```

The benchmark stores delivered packets, shared truth trajectory, both estimates
and drift metrics. Pending packets at the horizon are reported, not flushed
into the evaluated estimates. Add `--random-walk-std 0.002` for drifting bias.
The bias-aware benchmark enables both bias states. To assess learned acceleration
bias during an odometry interruption on the same stream:

```bash
PYTHONPATH=build/python-build/python \
python autoplanner/scripts/benchmark_imu_bias.py --bias 0 \
  --acceleration-bias 0.15 --steps 120 --odometry-outage-start 80 \
  --output build/sensor-navigation/acceleration-bias-benchmark.json
```

`--acceleration-random-walk-std` enables drifting acceleration bias. The outage
withholds odometry captured at/after the specified step from both estimators;
withheld packets are recorded separately. Overall RMSE includes startup latency.
Additional outage metrics start at `outage_start + latency_steps`, after older
odometry deliveries have cleared. This is an estimation benchmark, not the
navigation runner's stale-input safety behavior.

Select the differential-drive execution model without changing the planner or
controller API:

```python
simulation = robotnav.simulate(
    robotnav.RobotState(1.0, 1.0),
    trajectory,
    controller,
    config=robotnav.SimulationConfig(
        execution_model="differential_drive",
        track_width=0.55,
        max_wheel_velocity=2.0,
    ),
    max_time=5.0,
)
```

Controllers continue to emit velocity and equivalent steering angle. For the
differential-drive backend, `robotnav.steering_to_twist()` preserves signed
curvature with `angular_velocity = velocity * tan(steering) / wheelbase` before
the native backend applies twist and wheel limits. The default remains
`kinematic_bicycle` for backward compatibility.

Run the deterministic cross-model benchmark to emit one manifest and one
trace per execution model:

```bash
python autoplanner/scripts/cross_model_tracking_benchmark.py \
    --output-dir autoplanner/results/cross_model_tracking
```

The generated artifacts use the same versioned experiment schema as the
physics validation runs, including actuator limits, control-compute latency,
and comparable state/twist traces. The default `--model both` compares the
kinematic bicycle, dynamic bicycle, and differential-drive execution models.
In addition to `experiment_manifest.json` and per-model traces, the command
writes `comparison.json` and `comparison.csv` with directly comparable error,
effort, goal, and latency metrics.
For a dynamic bicycle run in Python, select
`SimulationConfig(execution_model="dynamic_bicycle")` and configure
`front_axle`, `rear_axle`, `mass`, `yaw_inertia`, and cornering stiffnesses.

For a deterministic parameter sweep over mass, yaw inertia, and tire
stiffness, use:

```bash
PYTHONPATH=build/python-build/python:autoplanner/scripts \
python autoplanner/scripts/dynamic_bicycle_sweep.py
```

The sweep writes one trace per case plus `sweep.csv` and `sweep.json`.

The C++ API includes `robotnav::CooperativeAStarPlanner` with two deterministic
multi-robot modes: the default prioritized Cooperative A* baseline and an
optional Conflict-Based Search (CBS) mode. Both enforce vertex and edge
reservations; CBS branches on conflicts and can resolve cases where a fixed
priority order gets stuck.

Run the bundled multi-agent benchmark with:

```bash
PYTHONPATH=build/python-build/python \
python autoplanner/scripts/multi_agent_benchmark.py --algorithm both
```

For reproducible randomized stress scenarios, append seeded assignments to
the same run:

```bash
PYTHONPATH=build/python-build/python \
python autoplanner/scripts/multi_agent_benchmark.py \
  --algorithm both --random-scenarios 20 --random-agents 4 \
  --seed 1234 --max-cbs-nodes 5000
```

The JSON ledger stores the seed, starts, goals, per-algorithm summary
(success rate, successful-run mean makespan/path length, and all-run means),
and every individual trace so a failed case can be replayed exactly.
`--max-cbs-nodes` bounds high-level CBS expansion, making large stress runs
predictable even when a scenario is difficult or unsolvable.

For a reusable scenario corpus with connectivity and shortest-path difficulty
metadata, generate a manifest first and pass it to the benchmark:

```bash
python autoplanner/scripts/multi_agent_scenarios.py \
  --map autoplanner/data/maps/simple_50x50.txt \
  --output autoplanner/results/multi_agent/scenarios.json \
  --scenarios 20 --agents 4 --seed 1234 --min-distance 10

PYTHONPATH=build/python-build/python \
python autoplanner/scripts/multi_agent_benchmark.py \
  --scenario-manifest autoplanner/results/multi_agent/scenarios.json \
  --algorithm both
```

The manifest stores every start/goal assignment and difficulty summary, so
different planners and later experiments consume the same tasks.

For a matrix sweep over agent counts, seeds, and CBS budgets, use:

```bash
PYTHONPATH=build/python-build/python:autoplanner/scripts \
python autoplanner/scripts/multi_agent_sweep.py \
  --seeds 1234,1235 \
  --agent-counts 2,3,4 \
  --cbs-node-budgets 1000,5000,10000 \
  --scenarios-per-seed 10
```

The sweep records success rate, successful-run path metrics, mean/p95 runtime,
and conflict counts in `multi_agent_sweep.csv` and `.json`.

To render a compact four-panel report from the JSON output:

```bash
python autoplanner/scripts/plot_multi_agent_sweep.py \
  --input autoplanner/results/multi_agent_sweep/multi_agent_sweep.json \
  --output autoplanner/results/multi_agent_sweep/sweep.png
```

It writes `multi_agent_benchmark.csv` and `.json` with success rate, makespan,
algorithm, priority order, wait steps, reservation conflicts, total path length, and the padded
per-timestep `agent_paths` for crossing, four-way, and converging cases. Each
algorithm/scenario pair gets a long-form `trace_<algorithm>_<scenario>.csv`, which can be rendered
without changing the planner:

```bash
python autoplanner/scripts/visualize_multi_agent.py \
  --map autoplanner/data/maps/simple_50x50.txt \
  --trace autoplanner/results/multi_agent/trace_cbs_crossing.csv \
  --output autoplanner/results/multi_agent/crossing.png
```

The trace records each agent's position at every time step, including padded
goal occupancy and wait actions, so it can also be consumed by replay or
collision-analysis tooling.

Validate a generated trace against the map and the multi-agent collision
rules with:

```bash
python autoplanner/scripts/validate_multi_agent_trace.py \
  --map autoplanner/data/maps/simple_50x50.txt \
  --trace autoplanner/results/multi_agent/trace_cbs_crossing.csv \
  --output autoplanner/results/multi_agent/crossing_validation.json
```

Trace rows include an `active` flag, so goal padding does not create false
collisions after an agent has completed. Use `--allow-shared-goals` when the
scenario intentionally permits simultaneous completion at a shared target.

The Python API keeps the prioritized baseline as its default; select CBS with
`plan_cooperative(..., algorithm="cbs")`. For co-located destinations,
`allow_shared_goals=True` models agents as complete when they reach the shared
goal at the same time.

Prioritized planning accepts `priority_order=[...]` in the Python API (or
`--priority-order 2,0,1` in the benchmark); the returned plan list remains in
the original input-agent order. CBS ignores this setting by design.

See `examples/python/quick_plan.py` and `examples/python/track_path.py` for
complete runnable examples.

Dynamic navigation is also available without invoking the CLI:

```python
result = robotnav.run_dynamic(
    "autoplanner/data/maps/simple_50x50.txt",
    (1, 1), (12, 12),
    robotnav.DynamicConfig(
        planner="space_time_astar",
        controller="stanley",
        frames=12,
        steps_per_frame=30,
        max_replanning_retries=2,
        replanning_cooldown_frames=1,
        recovery_stop_steps=10,
        moving_obstacles=(robotnav.MovingObstacle(
            0, 10, 7, 4, 0, 1, radius=0.5),),
    ),
)
print(result.status_code, result.metrics["goal_reached"])
print(result.metrics["final_state"], result.state_transitions)
```

Failed dynamic replans use a bounded recovery policy. The retry count,
replanning cooldown, and stop duration are configurable through
`DynamicConfig` or the CLI flags `--max-replanning-retries`,
`--replanning-cooldown-frames`, and `--recovery-stop-steps`. Per-sample state
is stored in `trace`, while timestamped transitions and their reasons are
stored in `state_transitions`.

Dynamic prediction uses elapsed simulation time as its canonical clock. A
prediction frame spans `steps_per_frame * simulation dt`; Space-Time A*, DWA,
MPPI, and runtime collision supervision all convert through that same period.
The original integer-frame `DynamicObstacleContext` construction remains
supported for application-defined local planners.

See `examples/python/dynamic_nav.py` for a complete scenario.

Dynamic inputs and outputs can be stored as dependency-free JSON replay
artifacts for batch evaluation or another machine:

```python
scenario = robotnav.DynamicScenario(
    "autoplanner/data/maps/simple_50x50.txt", (1, 1), (12, 12), config)
scenario.save_json("results/scenario.json")
result = robotnav.run_dynamic_scenario(
    robotnav.DynamicScenario.load_json("results/scenario.json"))
result.save_json("results/result.json")
```

Both formats carry `schema_version` and are intended to remain stable across
patch releases.

For source-tree binding builds, include `build/python-build/python` in
`PYTHONPATH`. The facade is also included in the Python wheel.

```bash
cmake --preset python
cmake --build --preset python

PYTHONPATH=build/python-build/python python - <<'PY'
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
smoke result as separate gates on Linux and macOS. A passing demo is not
treated as a substitute for these checks.

### Binary Python wheels

The wheel workflow builds and tests CPython 3.10–3.13 binary wheels for
manylinux x86_64 and macOS x86_64 with the full Eigen-backed MPC API. Manual
workflow runs retain the wheels as GitHub artifacts. A `v*` tag additionally
publishes them through PyPI trusted publishing after both platform jobs pass.

Build and verify the current platform wheel locally with:

```bash
python -m pip install --upgrade build
python -m build --wheel --outdir build/local-wheel
python -m pip install --force-reinstall build/local-wheel/robotnav-*.whl
python .github/scripts/wheel_smoke.py \
    autoplanner/data/maps/simple_50x50.txt
```

The source distribution explicitly includes all CMake, C++, binding, and
Python sources required for isolated builds. Publishing requires a protected
GitHub `pypi` environment configured as a PyPI trusted publisher; no API token
is stored in the workflow.

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
rollout count, temperature, and exploration noise. After the first cycle,
the optimized sequence is shifted and blended with the current controller
command to warm-start the next cycle. Use `--mppi-warm-start-blend` to tune
that blend or `--no-mppi-warm-start` to restore independent sampling:

```bash
./build/apps/navigation_pipeline_cli \
    --scenario autoplanner/data/configs/navigation_pipeline.yaml \
    --start 1 1 --goal 20 20 --local-planner mppi \
    --mppi-prediction-time 0.8 --mppi-horizon 12 \
    --mppi-rollouts 64 --mppi-temperature 0.5 \
    --mppi-velocity-noise 0.35 --mppi-steering-noise 0.12 \
    --mppi-warm-start-blend 0.25 \
    --output-dir autoplanner/results/robotnav-mppi
```

Adaptive sampling is enabled by default. MPPI compares the effective sample
size (ESS) with `local_planner.mppi.target_effective_sample_ratio` and adjusts
the next cycle's velocity and steering noise within the configured minimum and
maximum scales. Pipeline metrics report the mean ESS, mean ESS ratio, and most
recent sampling-noise scale for tuning and regression analysis.

MPPI also keeps its control-sequence and rollout-result buffers between
control cycles. Pipeline metrics expose the number of MPPI-owned workspace
expansions and the number of cycles that reused all those buffers. To measure
the steady-state controller latency and verify that fixed-size cycles do not
expand this workspace after a warm-up cycle, run:

```bash
cmake --build --preset test --target mppi_benchmark -j4
./build/test/apps/mppi_benchmark \
    --iterations 100 --rollouts 64 --horizon 20
```

The benchmark reports mean, P50, P95, and P99 latency together with ESS,
sampling-noise, workspace-expansion, and workspace-reuse diagnostics. A
non-zero post-warm-up workspace-expansion count makes it fail. This check is
limited to the rollout buffers owned by MPPI; it does not claim to intercept
allocations made by the C++ runtime, OpenMP, or other dependencies.
It also reports whether the build is using the serial or OpenMP rollout
backend and its maximum worker count. See the
[MPPI performance baseline](autoplanner/docs/mppi_performance.md) for the
recorded CPU measurements and the SIMD/GPU adoption criteria.

For tracked obstacles with covariance, MPPI can also add an occupancy-risk
cost using `local_planner.mppi.dynamic_probability_weight`. The probability
term is computed independently of the collision gate: increasing or disabling
the soft cost never relaxes static, footprint, or predicted hard-collision
rejection. The maximum evaluated probability is included in pipeline metrics.

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
