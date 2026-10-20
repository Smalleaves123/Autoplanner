# RobotNav ROS-Free Roadmap

RobotNav is a standalone C++17 navigation and experiment platform. ROS and
Nav2 integration are intentionally out of scope. The public C++ API, Python
facade, CLI tools, replay formats, and simulation backends are the supported
integration surfaces.

## Working principles

- Keep planning, prediction, control, and safety kernels in C++.
- Keep Python focused on orchestration, replay, analysis, and visualization.
- Preserve deterministic, machine-readable experiment artifacts.
- Treat collision constraints as hard safety rules and learned/probabilistic
  outputs as costs or predictions.
- Maintain backward compatibility for public CLI, C++, Python, and scenario
  interfaces unless a versioned migration is provided.

## Phase 1: Extensible core

- [x] Replace the hard-coded global planner factory with a thread-safe runtime
  registry while preserving `createPlanner()`.
- [x] Expose registered planner discovery to the native Python backend.
- [x] Introduce a common trajectory-controller adapter and controller registry.
- [x] Introduce local-planner and path-smoother registries.
- [x] Centralize component selection validation and expose available component
  names through the C++ catalog.
- [x] Add examples showing an application-defined planner and controller.

## Phase 2: Dynamic navigation resilience

- [x] Add an explicit navigation state machine: tracking, yielding, replanning,
  recovery, safe stop, and terminal failure.
- [x] Add bounded retry and cooldown policies for global replanning.
- [x] Record state transitions and failure causes in trace and JSON artifacts.
- [x] Add deterministic crossing, overtaking, blocked-corridor, and disappearing
  obstacle regression scenarios.
- [x] Enforce one time base across prediction, local rollout, simulation, and
  safety supervision.

## Phase 3: Risk-aware MPPI

- [x] Warm-start each control sequence from the previous optimal rollout.
- [x] Add adaptive sampling variance and effective-sample diagnostics.
- [x] Add configurable probability/risk costs without weakening hard collision
  rejection.
- [x] Reuse rollout buffers and benchmark allocation-free control cycles.
- [x] Evaluate SIMD and optional GPU acceleration after a CPU profiling baseline.

## Phase 4: Middleware-independent perception

- [x] Define a versioned C++ `SensorFrame` and timestamped obstacle-track API.
- [x] Move occupancy integration and tracking kernels from the Python prototype
  into reusable C++ modules.
- [x] Keep CSV and JSON replay adapters and add stream/callback adapters.
- [x] Add Kalman and interacting-multiple-model prediction baselines.
- [x] Model delayed, missing, and out-of-order observations explicitly.

## Phase 5: Reproducible validation

- [x] Unify MuJoCo, PyBullet, and kinematic runs under one scenario/result schema.
- [x] Add noise, latency, actuator saturation, and randomized dynamic-agent suites.
- [x] Report success, collision, safe-stop, goal-time, control effort, and
  P50/P95/P99 compute latency across all attempted runs.
- [x] Preserve and replay minimized failure cases.
- [x] Add Linux/macOS CI and distributable Python wheels.

## Phase 6: Multi-kinematic execution

- [x] Add an actuator-limited differential-drive simulator with native and
  Python APIs.
- [x] Add selectable bicycle/differential-drive execution to the high-level
  RobotNav simulation facade.
- [x] Add controller-command adaptation with explicit steering/twist
  conversion contracts.
- [x] Add deterministic cross-model tracking benchmarks and schema fields.
- [x] Add a dynamic bicycle model after establishing cross-model baselines.

## Phase 7: Simulation experiment tooling

- [x] Add deterministic prioritized Cooperative A* and CBS multi-agent
  baselines with seeded benchmarks.
- [x] Add reusable connected scenario manifests with shortest-path difficulty
  metadata.
- [x] Add a backend-neutral Python World/Clock/Entity observation facade with
  reset, step, and pairwise collision semantics.
- [x] Add deterministic 2-D LiDAR, odometry, IMU, and 2.5-D depth sensor
  simulation with latency, drops, noise, and timestamp perturbations.
- [x] Ray-cast other world entities as dynamic LiDAR returns and preserve
  return semantics through the perception adapter.
- [x] Flush and replay packet-oriented multi-sensor streams without losing
  LiDAR timing metadata or compatibility with the legacy frame format.
- [x] Record sensor replay parameters and output references in the unified
  versioned experiment run artifact.
- [x] Add a replayable sensor-to-perception-to-planning-to-control closed-loop
  runner with deterministic JSON traces.
- [x] Add a planar odometry/IMU EKF dead-reckoning baseline with covariance,
  fixed-lag event replay, capture-time LiDAR projection, and estimated-state
  planning/control (known initial pose).
- [x] Add known-map scan matching with visibility/degeneracy rejection and
  delayed EKF pose corrections; compare drift on identical biased observations.
- [x] Add seeded gyro bias/random-walk simulation through packet drops, optional
  six-state EKF bias estimation, and identical-stream drift comparison tooling.
- [x] Add independent forward-acceleration bias/random-walk simulation, optional
  acceleration bias estimation, and identical-stream odometry-outage benchmarks.
- [x] Add observable localization health, correction-jump gates, failure/recovery
  hysteresis, and deterministic fault benchmarks including an unresolved
  repeated-room aliasing limitation.
- [x] Add bounded multi-hypothesis known-map global search and persistent-teleport,
  repeated-room, symmetric-room, and unsupported-scan recovery benchmarks.
- [x] Add multi-scan unique-candidate confirmation, motion-consistent mode
  association, and a stationary-only EKF/health epoch adapter with old-packet
  rejection and caller-owned component reset contracts.
- [ ] Integrate confirmed recovery into navigation with stationarity checks,
  mapping/tracker/planner resets and a stopped recovery state machine.
- [ ] Expand noisy/partial-view localization comparisons and scalability benchmarks.
- [x] Add multi-agent parameter sweeps, trace visualization, and collision
  validation tools.
- [ ] Store multi-agent runs in the versioned experiment manifest schema used
  by single-robot and physics experiments.
- [ ] Add resumable batch execution, per-case timeouts, and failure bundles for
  multi-agent sweeps.
- [ ] Connect discrete multi-agent plans to continuous dynamic bicycle and
  differential-drive execution simulation.
- [x] Build a replayable perception-to-prediction-to-planning closed-loop
  scenario runner.

## Later research tracks

- Semantic and multi-resolution costmaps.
- Multi-modal learned obstacle prediction with classical safety supervision.
- Residual dynamics learning for MPPI.
- Anytime and risk-bounded kinodynamic planning.
- Conflict-based multi-agent planning with bounded suboptimality (ECBS).

These tracks should start only after the deterministic dynamic-navigation
baseline and evaluation contracts are stable.
