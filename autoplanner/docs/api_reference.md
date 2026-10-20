# API Reference

## Namespace

All types and functions live in the `autoplanner` namespace.

```cpp
using namespace autoplanner;
```

---

## Core Types

### Point2i / Point2d

```cpp
struct Point2i { int x = 0; int y = 0; };
struct Point2d { double x = 0.0; double y = 0.0; };

double distance(const Point2i& a, const Point2i& b);
double distance(const Point2d& a, const Point2d& b);
```

| Type | Coordinate space | Usage |
|------|-----------------|-------|
| `Point2i` | Integer grid cells | Map indexing, graph search nodes |
| `Point2d` | Continuous metres | Planner output, smoothing, collision |

### Pose2d

```cpp
struct Pose2d {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;  // radians
};
```

### Path2d

```cpp
using Path2d = std::vector<Point2d>;

double computePathLength(const Path2d& path);
bool savePathCsv(const Path2d& path, const std::string& file_path);
```

---

## GridMap

```cpp
class GridMap {
public:
    bool loadFromTxt(const std::string& file_path);

    bool isInside(int x, int y) const;
    bool isFree(int x, int y) const;
    bool isOccupied(int x, int y) const;

    int width() const;
    int height() const;
    int index(int x, int y) const;

    void setResolution(double resolution);
    double resolution() const;
};
```

**Map file format:** plain text with `0`/`.` for free cells and `1`/`#`/`@`
for obstacles. All rows must have identical width.

---

## PlannerBase (Interface)

```cpp
class PlannerBase {
public:
    virtual ~PlannerBase() = default;
    virtual PlannerResult plan(const GridMap& map,
                               const Point2i& start,
                               const Point2i& goal) = 0;
    virtual std::string name() const = 0;
};
```

All planners implement this interface.

---

## Planner Registry

Built-in and application-defined planners share one process-local registry.
The legacy `createPlanner()` function delegates to this registry, so existing
callers remain compatible.

```cpp
auto& registry = PlannerRegistry::instance();
registry.registerPlanner(
    "my_planner",
    [](const PlannerFactoryOptions& options, const Costmap2D* costmap) {
        return std::make_unique<MyPlanner>(options, costmap);
    });

auto planner = createPlanner("my_planner");
const auto names = availablePlanners();
```

Registration and discovery are thread-safe. A factory is invoked after the
registry lock is released, so it may safely perform non-trivial construction.
Names are listed in deterministic lexicographic order. Registering a duplicate
name fails unless `replace=true` is supplied.

---

## Navigation Controller Registry

The top-level RobotNav pipeline adapts PID, Pure Pursuit, Stanley, and optional
MPC controllers to one interface:

```cpp
class MyController : public robotnav::TrajectoryController {
public:
    autompc::Control compute(
        const autompc::State& state,
        const autompc::Trajectory& trajectory,
        const autompc::TrajectoryPoint& nearest_reference) override;
};

robotnav::ControllerRegistry::instance().registerController(
    "my_controller",
    [](const autompc::SimulationOptions&) {
        return std::make_unique<MyController>();
    });
```

Both `NavigationPipeline` and `DynamicNavigationPipeline` resolve controllers
through this registry. Stateful controllers may override `reset()` and
`onTrajectoryChanged()` lifecycle hooks.

AutoMPC also provides two standalone, actuator-limited execution models. The
existing `KinematicBicycleSimulator` consumes velocity and steering commands;
`DifferentialDriveSimulator` consumes linear and angular velocity commands and
reports the constrained body twist plus left/right wheel velocities. Wheel
saturation scales both components together so commanded curvature is retained.
Neither simulator depends on middleware.

```cpp
autompc::DifferentialDriveOptions options;
options.track_width = 0.55;
autompc::DifferentialDriveSimulator simulator(
    {0.0, 0.0, 0.0, 0.0}, options);
const auto next = simulator.step({1.0, 0.4});
```

The high-level Python facade selects the backend through
`SimulationConfig.execution_model`, which accepts `kinematic_bicycle` or
`differential_drive`. Existing controllers remain model-independent:

```python
config = robotnav.SimulationConfig(
    execution_model="differential_drive",
    track_width=0.55,
    max_wheel_velocity=2.0,
)
result = robotnav.simulate(initial, trajectory, controller, config=config)
```

`robotnav.steering_to_twist()` is the public command-adaptation contract. It
converts the controller's velocity and equivalent steering angle to a signed
body twist using bicycle curvature; actuator clipping occurs only afterward in
the selected native simulator.

`autoplanner/scripts/cross_model_tracking_benchmark.py` runs the same fixed
polyline and controller configuration through both models. It writes one
`ExperimentManifest` containing a constrained-bicycle and a
constrained-differential-drive `RunArtifact`, with matching state/twist CSV
traces and the controller latency percentiles used by the validation report.
Repeated runs produce identical state traces; timing samples remain
observational metrics.

The buildable `custom_components_example` demonstrates an application-owned
planner and controller together, including registration, discovery through
the existing factory helpers, execution, and cleanup:

```bash
cmake --preset release
cmake --build --preset release --target custom_components_example -j4
./build/release/examples/custom_components_example
```

---

## Local Planner and Smoother Registries

DWA and MPPI implement the common `robotnav::LocalPlanner` interface. A local
planner receives the nominal controller command plus the current dynamic
obstacle context and returns a normalized decision containing feasibility,
the adjusted command, rollout count, collision rejections, and clearance.

Applications register additional implementations through
`robotnav::LocalPlannerRegistry`; both navigation pipelines accept registered
names without source changes.

Collision-safe path smoothers use `autoplanner::SmootherRegistry`:

```cpp
SmootherFactoryOptions options;
options.max_iterations = 100;
options.max_curvature = 0.4;
auto smoother = createSmoother("curvature", checker, options);
```

The reserved name `none` means that the corresponding optional stage is
disabled and cannot be replaced by a registry entry.

The top-level catalog provides deterministic discovery and shared selection
validation before map preparation begins:

```cpp
const auto catalog = robotnav::availableComponents();
const auto validation = robotnav::validateComponentSelection(config);
if (!validation.valid) {
    std::cerr << validation.message << '\n';
}
```

---

## Planar Python state estimation

`robotnav.PlanarEkf(initial, EkfConfig(), timestamp_ns=0)` maintains the state
`[x, y, heading, forward velocity, yaw rate]` and a 5-by-5 covariance using only
the Python standard library. It accepts `OdometryMeasurement` (velocity and
`delta_theta / sample_dt`) and `ImuMeasurement` (yaw rate measurement and
acceleration prediction input) and optional `PoseMeasurement` (map-frame x/y/
heading), but never integrates truth-frame odometry
position deltas or reads `LidarScan.pose`.

- `update(packet)` returns whether the observation was accepted; invalid
  geometry raises, while duplicate, wrong-entity, and expired packets are
  counted in `rejected_packets`.
- `estimate` exposes the latest event-time `StateEstimate`.
- `estimate_at(timestamp_ns)` queries retained historical poses or predicts
  forward without changing filter state; queries before retained history raise.
- `reset(initial, timestamp_ns=0)` clears observations and counters.

Delayed observations are replayed in deterministic event order within
`history_seconds` and `max_history_packets` bounds. Process standard deviations
define diagonal covariance growth per square root of second; measurement
standard deviations are in velocity/yaw-rate units. Prediction acceleration
white-noise uncertainty is approximated by velocity process noise; IMU bias
states can be enabled separately as described below.
The initial pose hint is assumed known. Without pose measurements, no absolute
drift correction is made. Pose measurements are replayed after motion events
at equal timestamps, use wrapped heading innovations, and expose separate
`accepted_pose_packets` / `accepted_motion_packets` counters.
Consumers must impose their own extrapolation-age limits.

`EkfConfig(estimate_gyro_bias=True)` augments the state with gyro bias at index
5 and uses a 6-by-6 covariance. The IMU observation is `yaw_rate + gyro_bias`;
odometry observes yaw rate alone. `initial_gyro_bias` (signed rad/s),
`initial_gyro_bias_std` (positive rad/s), and `gyro_bias_process_std`
(non-negative rad/s/sqrt(s), zero for constant bias) configure the prior and
random walk. `StateEstimate.gyro_bias` and `gyro_bias_std` expose estimate and
uncertainty. Five-state mode remains the library default, reports bias zero and
std `None`; the navigation demo defaults to six-state mode. Reset restores the
configured bias prior, and fixed-lag replay/pose corrections retain bias cross
covariances. IMU alone cannot separately observe rate and bias; an unbiased
odometry or pose reference is required. Common odometry/IMU systematic errors
remain unresolved.

`ImuConfig.angular_velocity_bias` adds a signed constant gyro offset.
`angular_velocity_bias_random_walk_std` controls seeded Brownian increments
with std `sigma * sqrt(physical_dt)`, independent of timestamp jitter and
delivery drops/latency. The initial sample uses the configured offset; subsequent
sensor steps evolve it even if dropped. Measurement packets retain their schema;
`SensorSuite.imu_biases` exposes simulator-only truth for evaluation. Bias RNGs
are independent of measurement/drop RNGs, and `reset()` resets both. Gyro truth
rate is the wrapped heading difference over the successful sample interval,
with bias evaluated at the current capture time. This is a planar simplified
IMU, not a 3-D inertial sensor.

`ImuConfig.acceleration_bias` adds a signed forward-acceleration offset
(map units/s²); `acceleration_bias_random_walk_std` adds an independent seeded
random walk (map units/s²/sqrt(s)) with the same physical-time, drop, jitter,
and reset guarantees. `SensorSuite.imu_acceleration_biases` provides evaluation
truth only. Neither bias is placed in measurement packets. Acceleration truth
is the change in forward speed over the successful sample interval, not a
gravity-inclusive body-frame specific force.

`EkfConfig(estimate_acceleration_bias=True)` adds a forward-acceleration bias
state. Prediction holds the most recent raw IMU acceleration and subtracts
the current estimated bias, with analytic bias derivatives in the position
and velocity Jacobian. Before the first IMU observation prediction assumes
zero acceleration, regardless of the bias prior. Unbiased odometry velocity
or sufficiently informative map-position observations can estimate this bias
through cross covariance; IMU alone cannot observe it.
`initial_acceleration_bias`, `initial_acceleration_bias_std` (positive), and
`acceleration_bias_process_std` (non-negative, per sqrt(s)) configure its prior
and diffusion. `StateEstimate.acceleration_bias` / `acceleration_bias_std` are
`None` when disabled. Gyro bias remains zero/std `None` when disabled.

The five core state components stay in their original order; enabled bias
states are appended in gyro-then-acceleration order:

| Gyro bias | Acceleration bias | Covariance | Bias indices |
| --- | --- | --- | --- |
| off | off | 5×5 | none |
| on | off | 6×6 | gyro: 5 |
| off | on | 6×6 | acceleration: 5 |
| on | on | 7×7 | gyro: 5, acceleration: 6 |

Reset restores both configured priors; delayed-event replay and pose correction
retain both biases and their cross covariances. All bias estimation is optional
in the library. Navigation retains gyro-on/acceleration-off defaults, with
independent `--[no-]estimate-gyro-bias` / `--[no-]estimate-acceleration-bias`
switches. No gravity, roll/pitch, 3-D accelerometer calibration, or shared
reference-error estimation is implied.

`scan.to_perception_frame(pose=estimated_capture_pose)` transforms local LiDAR
angles/ranges with the supplied estimate, preserving event/arrival timestamps
and return semantics. Without `pose=`, it retains the previous truth-based
projection behavior for backwards-compatible replay.

### Known-map scan matching

`robotnav.GridScanMatcher(map, ScanMatcherConfig()).match(scan, hint)` registers
local beam angles/ranges to exposed 0/1 grid-cell boundaries. It never reads
`scan.pose`. Bounded multilevel search is followed by robust point-to-line ICP;
free-space ray visibility and surface normals prevent correspondence to a
wall's back face or a surface occluded by another wall. Dynamic-labeled beams
and no-hit beams are ignored; unknown-labeled hits can be matched.

`ScanMatchResult` reports the proposed `pose`, `accepted`, `reason`, initial/
final RMSE, inlier/point counts, and a normalized normal-matrix determinant
(`information_ratio`) used to reject degenerate geometry. Missing support,
poor fit, and geometrically ambiguous fits are not fed to the EKF. This is a
local baseline: bounded search cannot resolve globally repeated structures
or recover an arbitrarily wrong initial pose.

### Localization health supervision

`LocalizationHealthMonitor(LocalizationHealthConfig(), timestamp_ns=0)` is an
arrival-time, single-robot/map supervisor, separate from EKF replay. Its inputs
are estimated state/covariance, sensor timestamps, and match outcomes, never
simulator truth. `assess_match(match, hint)` rejects matcher failures and
accepted corrections whose Euclidean translation or wrapped heading jump
exceeds configured limits; the returned `MatchAssessment` includes reason and
jump magnitudes. Call it **before** pose fusion.
`record_match(capture_timestamp_ns, accepted=fused, reason=...)` records actual
fusion success, not just registration success. Older/equal capture timestamps
are ignored and counted in `ignored_matches`; one monitor expects one logical
map correction stream. It does not rewrite past control decisions after late
EKF updates.

`evaluate(now_ns, estimate, motion_timestamp_ns=..., scan_timestamp_ns=...)`
requires a current `estimate_at(now_ns)` and monotonic evaluation time. `None`
means no packet received. Timestamp ages are clamped at zero for forward
jitter; map freshness uses the last successfully fused capture timestamp, not
arrival time. Position uncertainty is the square root of the largest XY-block
covariance eigenvalue; heading std is the heading marginal. These are model
uncertainties, not calibrated localization confidence.

- `tracking`: required confirmations and freshness/uncertainty checks pass.
- `degraded`: a previously initialized map tracker has isolated failed matches
  or incomplete recovery streak; suggested velocity scale defaults to 0.35.
- `initializing`: map confirmation is incomplete; suggests stop.
- `lost`: stale motion/scan/map correction, excessive pose uncertainty, repeated
  failure, or pending recovery after loss; suggests stop.

Default `failures_before_lost=3`, `successes_before_recovery=2` apply to distinct
capture-time matches. Stale/uncertain ticks reset recovery streaks; long gaps
between successful captures also interrupt them. Defaults are positive
`max_motion_age=0.5`, `max_scan_age=0.5`, `max_match_age=2.0` seconds,
`max_position_std=1.0` map units, `max_heading_std=0.5` rad,
`max_correction_translation=0.6` map units, and `max_correction_heading=0.15`
rad. `require_map_matching=False` omits map confirmation/freshness/streak
requirements for dead reckoning. `reset(timestamp_ns=0)` clears all health
history, counters, and recovery state.

`LocalizationHealth` exposes status, reasons, stop/velocity-scale suggestion,
nullable ages, uncertainty, streaks, and last match reason. Navigation applies
these suggestions by default; it still applies basic freshness checks when
`--no-localization-health` is selected. Tune map age relative to scan-match
frequency and latency. This does not certify collision-free operation, resolve
identical-room aliases, or perform global relocalization. Fault benchmark
artifacts explicitly report large-error `tracking` steps in repeated rooms;
truth appears only in sensor generation/evaluation. Recovery in the teleport
benchmark occurs after the simulated pose returns, not through relocalization.

### Scan-match pose corrections

Accepted results can be sent as `PoseMeasurement` with the original LiDAR
sequence and capture/arrival timestamps. Its `position_std` / `heading_std`
are conservative independent uncertainty assumptions, not calibrated full
ICP covariance. The runner defaults to matching every five generated scans,
records quality diagnostics, and can disable matching with
`--localization dead_reckoning`. Online matches are not re-registered when a
later motion observation revises their original hint; accepted pose events
are replayed as fixed measurements within EKF history.

### Multi-hypothesis known-map relocalization

`GridRelocalizer(map, RelocalizationConfig(), matcher_config=None).localize(scan)`
performs deterministic coarse global search then bounded local refinement,
without an initial pose hint or any use of `scan.pose`. The local matcher uses
the same geometry validation, dynamic-return exclusion, visibility, and
degeneracy checks as ordinary scan matching. Unknown hit beams are supported;
all-dynamic/no-hit scans cannot establish a global pose.

Coarse scoring uses grid-DDA static ray ranges. Static hit rays compare
predicted/observed range; no-hit rays penalize walls closer than the observed
range. Range residuals are capped at `residual_cap=2.0` map units and squared;
the mean squared residual is a heuristic score, not a likelihood. Coarse rays
are evenly subsampled, while refined scores use all non-dynamic rays.

Default controls:

- `translation_step=1.0`, `heading_bins=16`: lattice poses at coordinates
  `0.5 + i*step`, headings uniformly around the full circle; occupied origins
  are excluded. Local refinement defaults to windows covering half the grid
  interval in position/heading, with 31 endpoint samples.
- `coarse_beams=24`, `max_coarse_poses=50000`, `max_refinements=24`:
  deterministic work budgets. The full lattice upper bound (including occupied
  cells) is checked before seed allocation; exceeding it rejects the request
  rather than scoring only an arbitrary portion of the map.
- `distinct_translation=1.0`, `distinct_heading=0.4`: poses are distinct if
  either Euclidean position or wrapped heading separation exceeds its threshold.
  These suppress nearby coarse seeds and merge refined modes.
- `max_ray_score=0.08`, `ambiguity_score_margin=0.02` (map units squared):
  require an adequate best score and reject a competing distinct mode within
  the margin. Alternatives up to cutoff plus margin are retained for this
  check, so a near-threshold runner-up is not silently discarded.
- `max_hypotheses=8`: limits returned hypotheses, not ambiguity checks on all
  successfully refined distinct modes.

`RelocalizationResult` contains `accepted`, `reason`, nullable selected `pose`,
ordered `hypotheses`, `coarse_poses`, `refinements`, and nullable `score_gap`.
Reasons are `localized`, `ambiguous`, `no_match`, `insufficient_points`, or
`search_budget_exceeded`. `pose` is present only for `localized`;
`RelocalizationHypothesis` carries pose, full-ray score and local-match diagnostics.
`score_gap` is the second distinct score minus the best score, not confidence.
`coarse_poses` counts scored free-cell headings, or the refused lattice bound
on budget failure (a sentinel above budget for non-finite spacing arithmetic).
Malformed geometry/configuration raises without mutating an estimator.

The utility never resets/fuses into an EKF implicitly. An accepted candidate is
only non-ambiguous among the explored/refined modes, not a proof of global
uniqueness: lattice gaps, top-seed limits, clustering, sensor/map error and
partial views may hide alternatives. The standalone benchmark compares far-hint local registration to
global recovery, explicitly resets only accepted candidates, and records
unresolved symmetric/repeated-room hypotheses. Its stationary post-teleport
epoch has no pending motion packets. Online consumers must confirm candidates,
preserve stop behavior during ambiguity, and explicitly handle coordinate-frame
changes, estimator/health reset, mapping/tracker state, delayed packet queues,
and epoch boundaries. Large global corrections must not simply bypass the
local health gate. Automatic navigation integration remains pending.

### Temporal global confirmation and recovery epochs

`RelocalizationTracker(RelocalizationTrackingConfig()).update(result,
timestamp_ns=scan_capture_ns, motion_pose=dead_reckoning_at_capture)` returns a
`RelocalizationDecision`. `motion_pose` must remain in one consistent frame;
map-corrected pose jumps or simulator truth must not supply relative motion.
Previous map hypotheses are transported using relative SE(2) motion, then
greedily associated one-to-one under translation/heading tolerances (defaults
0.3 map units / 0.1 rad). This is deterministic heuristic association, not a
probabilistic multi-target filter or calibrated confidence model.

Defaults require `confirmations=3` distinct increasing scan timestamps and
`max_scan_gap=1.0` s. Only a globally accepted best mode earns consecutive
`unique_frames`; ambiguous results reset unique counts even when multiple
tracks remain motion-consistent. Missing search results discard tracks, large
gaps start new tracks, and an unassociated best mode restarts at one frame.
Duplicate/older timestamps return `out_of_order`, increment `ignored_frames`,
and leave state unchanged. `reset()` clears association and counters.

`RelocalizationDecision` contains capture timestamp, status, nullable pose and
current `RelocalizationTrack` diagnostics (ID, pose, consistent/unique counts).
Statuses are `candidate_pending`, `motion_inconsistent`, `gap_reset`,
`ambiguous`, `no_match`, `out_of_order`, and `confirmed`. Only `confirmed` has
a selected pose. Shared systematic errors or missing search alternatives can
still appear stable; temporal confirmation does not prove correctness.

`RelocalizationEpoch(initial, EkfConfig(), health_config=None,
timestamp_ns=0, max_confirmation_age=0.5)` owns `ekf` and `health`, with initial
`epoch_id=0`. `apply(decision, now_ns=..., epoch_id=source_epoch,
stationary=True)` starts a stationary recovery epoch. Caller must establish
that the robot is stopped throughout capture-to-application: the adapter does
not detect stationarity or extrapolate a moving global pose. It rejects
non-confirmed, stale/future, reused, pre-epoch or wrong-epoch decisions and
backward application times relative to EKF/health.

On handoff, EKF is recreated at application time with candidate map pose and
zero velocity/rate. Enabled bias means are preserved; bias std is at least the
configured initial prior. Pose/cross-state covariance and EKF event history/
counters reset. Health resets to require fresh motion/scans and map confirmation.
`epoch_id` increments; `RelocalizationEpochTransition` records application/
confirmation time and caller-required resets: motion frame, mapping, obstacle
tracks, planner, control. Reacquire `epoch.ekf` after handoff; an old reference
does not address the new estimator.

Tag packets/confirmation batches with their source epoch at capture/enqueue,
not arrival. `update_motion(packet, epoch_id=tag)` filters IMU/odometry before
fusion; `accept_packet(scan, epoch_id=tag)` filters LiDAR before other consumers.
Wrong epoch/entity and old captures are counted in `rejected_epoch_packets`.
The initial epoch accepts its starting timestamp; after handoff, captures at
or before the new application boundary are rejected. Correct tags remain
necessary even for old packets with future/jittered timestamps. The adapter
does not own queues, sensor schedules, maps or planners; callers must discard/
reinitialize these and not feed the underlying EKF around the filter. The
multiframe benchmark tests real noisy global search, stopped switching and
delayed packets, moving confirmation without switching, and persistent room
ambiguity. Automatic navigation integration and stationarity verification
remain pending.

## Middleware-independent perception types

`robotnav/perception/types.h` defines the stable C++ boundary between sensor
adapters, mapping, tracking, prediction, and navigation. It uses plain C++17
types and does not depend on a transport or middleware runtime.

- `SensorFrame` carries a schema version, sequence number, nanosecond
  timestamp, coordinate frame, sensor source, pose, and 2-D beam endpoints.
- `ObstacleTrack` carries a timestamped kinematic state, last-observation
  timestamp, radius, position covariance, confidence, counters, and lifecycle
  state.
- `ObstacleTrackFrame` is a coherent snapshot whose estimates share a
  timestamp and coordinate frame.
- `validate()` rejects unsupported schema versions, non-finite geometry,
  invalid covariance, inconsistent timestamps, duplicate track IDs, and
  malformed lifecycle counters using stable `ValidationError` values.

Adapters should validate at ingestion and preserve `source_id`, `sequence`,
`timestamp_ns`, and `frame_id` in replay artifacts. Version 1 coordinates are
2-D world values in the named frame and timestamps are signed nanoseconds from
the adapter's documented clock domain.

`robotnav/perception/occupancy_grid.h` provides bounded log-odds ray
integration with explicit unknown/free/occupied states and configurable
unknown-cell export policy. `robotnav/perception/obstacle_tracker.h` provides
a deterministic nearest-neighbour tracker with constant-velocity prediction,
tentative/confirmed/coasting lifecycle states, missed-observation expiry, and
timestamp-regression rejection. Both kernels reject invalid or mismatched
frames before mutating state.

`robotnav/perception/frame_stream.h` supplies synchronous validated callback
dispatchers and bounded thread-safe frame streams for both sensor and track
frames. Dispatchers copy their subscriber set before invocation, permitting
reentrant subscription changes. Buffers make overload behavior explicit with
either drop-oldest or reject-newest policies and expose both counters.

`robotnav/perception/replay.h` reads and writes sensor-frame and track-frame
streams in CSV or newline-delimited JSON. Both encodings use the same
versioned record schema: a replay preamble, one frame header, then the exact
number of point or track records declared by that header. Writers validate all
input before producing bytes; readers report the failing line and invoke the
callback only for complete, validated frames. This lets replay feed a
`FrameDispatcher` or `BufferedFrameStream` without an intermediate container.

`robotnav/perception/track_prediction.h` provides a dependency-free
constant-velocity Kalman baseline and a two-mode interacting multiple model
(IMM) baseline. The IMM mixes smooth and maneuvering model state/covariance
before prediction, evaluates measurement likelihoods, and reports normalized
mode probabilities with the combined position covariance. Both predictors
reject invalid or non-monotonic observations and support non-mutating future
queries plus explicit reset.

`robotnav/perception/observation_buffer.h` models transport timing explicitly.
Its event-time watermark reorders delayed frames within a configured window,
then emits a deterministic timestamp/source/sequence order. Per-source
sequence tracking reports missing, duplicate, and stale observations;
post-watermark arrivals are rejected as too late. Arrival delay, out-of-order,
capacity, and delivery counters remain available for experiment artifacts.

---

## Execution experiment schema

`autoplanner/scripts/experiment_schema.py` defines the versioned JSON contract
shared by the C++ kinematic bicycle, MuJoCo, and PyBullet validation backends.
It uses only the Python standard library, so stored scenarios and results can
be inspected without any simulator installed.

- `BackendSpec` identifies `kinematic/constrained_bicycle`, `mujoco/planar`,
  or `pybullet/planar|racecar`.
- `SimulationSpec` stores the common time step, wheelbase, and actuator limits.
- `ScenarioSpec` stores the route, component selection, run bounds, seed, and
  metadata.
- `RunMetrics` stores execution status, goal/collision/safe-stop results,
  tracking error, path length, control effort, and latency percentile slots.
- `RunArtifact` combines one scenario, its metrics, planner metadata, and CSV
  trace reference. `ExperimentManifest` groups comparable backend runs.

Every top-level object contains `schema_version` and `artifact_type`. Loaders
reject unknown versions, non-finite values, invalid backend/model pairs, and
invalid limits. `artifact_from_legacy_summary()` upgrades the earlier physics
tracking summary format to a v1 `RunArtifact`. Run artifacts also retain the
old flat summary keys as compatibility aliases for existing analysis scripts;
new consumers should read `scenario`, `metrics`, and `artifacts`.

`physics_tracking_benchmark.py --backend all` writes one run artifact per
backend and an `experiment_manifest.json` containing the same structures.

### Robustness validation suites

`autoplanner/scripts/validation_suites.py` provides versioned, seeded case
generation for four perturbation families. `PerturbedSimulator` wraps the same
`reset/observe/step/close` protocol used by every execution backend and adds:

- independent Gaussian position, heading, and velocity observation noise;
- observation delay measured in control steps;
- applied velocity and steering command saturation.

Controllers consume the perturbed observation while run metrics continue to
use simulator truth. Tracking CSV files store both state views and both
requested/applied commands. `randomized_dynamic_agents()` enumerates straight
free-space tracks from the selected occupancy map, chooses non-overlapping
tracks with a local seeded generator, and emits native dynamic-pipeline CLI
arguments.

`run_validation_suite.py` supports the `baseline`, `noise`, `latency`,
`saturation`, `dynamic_agents`, and aggregate `robustness` suites. Generated
case definitions are saved before execution, and the execution ledger keeps
failed navigation outcomes distinct from harness/invocation failures.

`autoplanner/scripts/validation_reporting.py` expands tracking manifests to
one ledger row per backend and retains a placeholder row when an attempted
case has no readable artifact. `validation_results.csv` reports success,
collision, safe-stop, goal time, control effort, and per-run compute latency.
`validation_report.json` aggregates success/collision/safe-stop rates, reached
goal-time distributions, control effort, and P50/P95/P99 across the raw
per-control-cycle latency samples in every available trace.

The dynamic C++ trace includes `compute_latency_ms` for every normal control
decision. `DynamicPipelineMetrics` exposes `goal_time_s`, `control_effort`,
`safe_stop_steps`, `compute_latency_samples`, and compute latency P50/P95/P99;
stop/recovery rows without a control decision are excluded from latency
percentiles.

### Minimized failure replay

`autoplanner/scripts/failure_replay.py` defines the versioned
`robotnav.validation.failure` bundle. For tracking failures, minimization
removes independent position/heading/velocity noise, latency, and saturation
dimensions while the fixed-seed failure persists. For dynamic failures it
uses greedy delta debugging to remove agents and then shortens the remaining
agent lifetimes. Every candidate is executed through the same runner command.

The preserved bundle contains the original and minimized `ValidationCase`,
the minimization attempt count, the replay command, copied map/path inputs, and
the complete artifact directory from the verified minimized run.
`replay_failure.py` redirects the stored command to a fresh output directory
and returns zero only when the new artifact reproduces the expected failed
outcome.

---

## CI and Python wheel distribution

`.github/workflows/ci.yml` runs the C++/CTest suite and an isolated wheel
build/install/API smoke test on Ubuntu and macOS. Sanitizer testing remains a
separate Linux gate so platform failures are distinguishable from memory or
undefined-behavior failures.

`.github/workflows/wheels.yml` uses cibuildwheel to build CPython 3.10–3.13
manylinux x86_64 and macOS x86_64 wheels. Each repaired wheel is installed and
executes `.github/scripts/wheel_smoke.py`, which imports all three native
packages, plans a path, advances the constrained bicycle, checks the Eigen MPC
surface, and runs the dynamic facade. The source-distribution job also rebuilds
and tests a wheel using only the generated archive. Manual runs upload
artifacts; `v*` tags publish through PyPI trusted publishing after every wheel
job succeeds.

`pyproject.toml` explicitly includes the monorepo's CMake files, C++ sources,
headers, bindings, and Python packages in the source distribution. Wheel builds
remain ROS-free and use only header-only Eigen as a build dependency; the
resulting wheel has no Eigen runtime dependency.

---

## PlannerResult

```cpp
struct PlannerResult {
    bool success = false;
    std::string planner_name;

    Path2d path;

    double path_length = 0.0;
    double planning_time_ms = 0.0;
    int expanded_nodes = 0;
    int iterations = 0;

    bool collision_free = false;
    int turning_count = 0;
    double total_turning = 0.0;
    double average_curvature = 0.0;
    double smoothness_score = 0.0;
    double minimum_obstacle_distance = 0.0;

    std::string message;
};

bool saveMetricsJson(const PlannerResult& result, const std::string& file_path);
```

---

## Planners

### AStarPlanner

```cpp
class AStarPlanner : public PlannerBase {
public:
    explicit AStarPlanner(bool allow_diagonal = true);
    void setHeuristic(std::unique_ptr<Heuristic> heuristic);
};
```

### WeightedAStarPlanner

```cpp
class WeightedAStarPlanner : public PlannerBase {
public:
    WeightedAStarPlanner(double weight = 1.5, bool allow_diagonal = true);
    void setHeuristic(std::unique_ptr<Heuristic> heuristic);
};
```

### ImprovedAStarPlanner

```cpp
class ImprovedAStarPlanner : public PlannerBase {
public:
    ImprovedAStarPlanner(double heuristic_weight = 1.0,
                         double obstacle_weight = 2.0,
                         double turning_weight = 0.5,
                         bool allow_diagonal = true);
    void setHeuristic(std::unique_ptr<Heuristic> heuristic);
    void setCostmap(const Costmap2D* costmap);
};
```

### DijkstraPlanner

```cpp
class DijkstraPlanner : public PlannerBase {
public:
    explicit DijkstraPlanner(bool allow_diagonal = true);
};
```

### JPSPlanner

```cpp
class JPSPlanner : public PlannerBase {
public:
    explicit JPSPlanner(bool allow_diagonal = true);
    void setHeuristic(std::unique_ptr<Heuristic> heuristic);
};
```

### DStarLitePlanner

```cpp
class DStarLitePlanner : public PlannerBase {
public:
    explicit DStarLitePlanner(bool allow_diagonal = true);
};
```

### RRTPlanner

```cpp
class RRTPlanner : public PlannerBase {
public:
    RRTPlanner(double step_size = 2.0,
               int max_iter = 5000,
               double goal_sample_rate = 0.1,
               double goal_tolerance = 2.0);
    const std::vector<std::pair<Point2d, Point2d>>& treeEdges() const;
};
```

### RRTStarPlanner

```cpp
class RRTStarPlanner : public PlannerBase {
public:
    RRTStarPlanner(double step_size = 2.0,
                   int max_iter = 8000,
                   double goal_sample_rate = 0.1,
                   double goal_tolerance = 2.0,
                   double rewire_radius = 5.0);
    const std::vector<std::pair<Point2d, Point2d>>& treeEdges() const;
};
```

### InformedRRTStarPlanner

```cpp
class InformedRRTStarPlanner : public PlannerBase {
public:
    InformedRRTStarPlanner(double step_size = 2.0,
                           int max_iter = 8000,
                           double goal_sample_rate = 0.1,
                           double goal_tolerance = 2.0,
                           double rewire_radius = 5.0);
    const std::vector<std::pair<Point2d, Point2d>>& treeEdges() const;
};
```

### BiRRTPlanner

```cpp
class BiRRTPlanner : public PlannerBase {
public:
    BiRRTPlanner(double step_size = 2.0,
                 int max_iter = 5000,
                 double goal_tolerance = 2.0);
};
```

### HybridAStarPlanner

```cpp
class HybridAStarPlanner : public PlannerBase {
public:
    HybridAStarPlanner(double step_size = 1.0,
                       double turning_radius = 3.0,
                       int max_iter = 10000,
                       double goal_tolerance = 1.0);
};
```

---

## Heuristics

```cpp
class Heuristic {
public:
    virtual ~Heuristic() = default;
    virtual double compute(const Point2i& current, const Point2i& goal) const = 0;
    virtual std::string name() const = 0;
};
```

**Concrete heuristics:** `ManhattanHeuristic`, `EuclideanHeuristic`, `DiagonalHeuristic`.

---

## Costmap2D

```cpp
class Costmap2D {
public:
    void buildFromGridMap(const GridMap& map);
    void inflateObstacles(double robot_radius);

    double getCost(int x, int y) const;
    bool isFree(int x, int y) const;

    int width() const;
    int height() const;
    double resolution() const;
};
```

### ObstacleInflation (utility)

```cpp
namespace ObstacleInflation {
    Costmap2D createInflatedCostmap(const GridMap& map, double robot_radius);
}
```

---

## CollisionChecker

```cpp
class CollisionChecker {
public:
    virtual ~CollisionChecker() = default;
    virtual bool isStateValid(const Point2d& p) const = 0;
    virtual bool isSegmentValid(const Point2d& p1, const Point2d& p2) const = 0;
    virtual bool isPathValid(const std::vector<Point2d>& path) const = 0;
};
```

**Concrete implementations:** `GridCollisionChecker`, `LineCollisionChecker`.

---

## Path Smoothing

### PathSmoother (Interface)

```cpp
class PathSmoother {
public:
    virtual ~PathSmoother() = default;
    virtual std::vector<Point2d> smooth(const std::vector<Point2d>& path) = 0;
    virtual std::string name() const = 0;
};
```

### ShortcutSmoother

```cpp
class ShortcutSmoother : public PathSmoother {
public:
    explicit ShortcutSmoother(const CollisionChecker& checker,
                              int max_iterations = 100);
};
```

### BezierSmoother

```cpp
class BezierSmoother : public PathSmoother {
public:
    explicit BezierSmoother(int samples_per_segment = 10);
};
```

### BSplineSmoother

```cpp
class BSplineSmoother : public PathSmoother {
public:
    explicit BSplineSmoother(int degree = 3, int samples_per_segment = 10);
};
```

---

## Metrics

```cpp
// From <autoplanner/metrics/path_metrics.h>
int    computeTurningCount(const std::vector<Point2d>& path);
double computeTotalTurning(const std::vector<Point2d>& path);
double computeAverageCurvature(const std::vector<Point2d>& path);
double computeSmoothnessScore(const std::vector<Point2d>& path);
double computeMinObstacleDistance(const std::vector<Point2d>& path,
                                  const std::vector<Point2i>& obstacles);
```

### BenchmarkMetrics

```cpp
class BenchmarkMetrics {
public:
    void addResult(const std::string& planner, const std::string& map,
                   const PlannerResult& result);
    std::string toCsv() const;
};
```

---

## CLI Apps

### autoplanner_cli

General-purpose planning CLI.

```
$ ./build/apps/autoplanner_cli --help
  --planner <name>   astar|dijkstra|weighted_astar|improved_astar|jps|rrt|rrt_star|bi_rrt
  --map <path>       grid map file
  --start <x> <y>    start coordinates
  --goal <x> <y>     goal coordinates
  --output <dir>     output directory

$ ./build/apps/autoplanner_cli \
    --planner astar \
    --map data/maps/maze_100x100.txt \
    --start 1 1 \
    --goal 98 98 \
    --output results/astar_maze
```

### compare_planners

Runs all planners on a single map and prints a comparison table.

```
$ ./build/apps/compare_planners \
    --map data/maps/warehouse_100x100.txt \
    --start 1 1 \
    --goal 98 98
```

### benchmark_all

Runs all planners on all maps and writes a CSV summary.

```
$ ./build/benchmark/benchmark_all
```
