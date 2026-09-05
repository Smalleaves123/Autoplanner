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
- [ ] Add examples showing an application-defined planner and controller.

## Phase 2: Dynamic navigation resilience

- [x] Add an explicit navigation state machine: tracking, yielding, replanning,
  recovery, safe stop, and terminal failure.
- [x] Add bounded retry and cooldown policies for global replanning.
- [x] Record state transitions and failure causes in trace and JSON artifacts.
- [ ] Add deterministic crossing, overtaking, blocked-corridor, and disappearing
  obstacle regression scenarios.
- [ ] Enforce one time base across prediction, local rollout, simulation, and
  safety supervision.

## Phase 3: Risk-aware MPPI

- [ ] Warm-start each control sequence from the previous optimal rollout.
- [ ] Add adaptive sampling variance and effective-sample diagnostics.
- [ ] Add configurable probability/risk costs without weakening hard collision
  rejection.
- [ ] Reuse rollout buffers and benchmark allocation-free control cycles.
- [ ] Evaluate SIMD and optional GPU acceleration after a CPU profiling baseline.

## Phase 4: Middleware-independent perception

- [ ] Define a versioned C++ `SensorFrame` and timestamped obstacle-track API.
- [ ] Move occupancy integration and tracking kernels from the Python prototype
  into reusable C++ modules.
- [ ] Keep CSV and JSON replay adapters and add stream/callback adapters.
- [ ] Add Kalman and interacting-multiple-model prediction baselines.
- [ ] Model delayed, missing, and out-of-order observations explicitly.

## Phase 5: Reproducible validation

- [ ] Unify MuJoCo, PyBullet, and kinematic runs under one scenario/result schema.
- [ ] Add noise, latency, actuator saturation, and randomized dynamic-agent suites.
- [ ] Report success, collision, safe-stop, goal-time, control effort, and
  P50/P95/P99 compute latency across all attempted runs.
- [ ] Preserve and replay minimized failure cases.
- [ ] Add Linux/macOS CI and distributable Python wheels.

## Later research tracks

- Differential-drive and dynamic bicycle execution models.
- Semantic and multi-resolution costmaps.
- Multi-modal learned obstacle prediction with classical safety supervision.
- Residual dynamics learning for MPPI.
- Anytime and risk-bounded kinodynamic planning.

These tracks should start only after the deterministic dynamic-navigation
baseline and evaluation contracts are stable.
