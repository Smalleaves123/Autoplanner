# MPPI CPU performance baseline

This note records the profiling baseline used to decide whether RobotNav
should add explicit SIMD or a GPU rollout backend. It is an engineering
decision record, not a cross-platform performance guarantee.

## Reproducing the benchmark

Build the release benchmark and run at the controller scale of interest:

```bash
cmake --preset release
cmake --build --preset release --target mppi_benchmark -j4
./build/release/apps/mppi_benchmark \
    --iterations 1000 --rollouts 64 --horizon 20
```

The executable performs one warm-up cycle, times only subsequent cycles, and
fails if any timed cycle expands an MPPI-owned rollout workspace. It reports
the actual `parallel_backend` and `maximum_worker_threads`, so results do not
silently mix serial and OpenMP builds.

OpenMP remains an optional CPU optimization. Use
`-DROBOTNAV_ENABLE_OPENMP=OFF` for a forced-serial comparison; the default is
`ON`, with an automatic serial fallback when the compiler/runtime is absent:

```bash
cmake -S . -B build/mppi-serial \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF \
    -DROBOTNAV_ENABLE_OPENMP=OFF
cmake --build build/mppi-serial --target mppi_benchmark -j4
./build/mppi-serial/apps/mppi_benchmark \
    --iterations 1000 --rollouts 64 --horizon 20
```

## 2026-09-06 baseline

Environment: Apple M1 MacBook Air (`MacBookAir10,1`, 8 CPU cores), macOS arm64,
Apple Clang 17, Release build. CMake did not find an OpenMP runtime, so these
are serial CPU measurements. Each row is a separate process with one untimed
warm-up cycle.

| Rollouts | Horizon | Iterations | Mean (ms) | P50 (ms) | P95 (ms) | P99 (ms) | M rollout steps/s |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 64 | 20 | 1000 | 0.361 | 0.344 | 0.461 | 0.573 | 3.546 |
| 256 | 20 | 300 | 1.414 | 1.409 | 1.460 | 1.475 | 3.620 |
| 1024 | 40 | 120 | 11.341 | 11.331 | 11.375 | 11.418 | 3.612 |

Every measured cycle reused its workspace and the post-warm-up expansion
count was zero. The nearly constant throughput across the two larger cases
shows predictable linear scaling with rollout count and horizon on this CPU.

A three-second sampling profile of the 1024-by-40 case placed the dominant
cost inside `MppiLocalPlanner::computeCommand`. Visible leaf costs included
normal-distribution generation and its logarithm, pose-segment checking, and
the trigonometric functions used by rollout dynamics and trajectory scoring.
This agrees with the expected rollout and cost-evaluation hot path; workspace
management was not visible as a material cost.

## Acceleration decision

Explicit SIMD is not added at this baseline. The rollout loop currently has
per-rollout collision early exits, virtual collision-checker calls, nearest
trajectory queries, and branch-dependent dynamic-obstacle risk evaluation.
Those operations make a direct vector rewrite intrusive, while the normal
64-by-20 controller case is already below one millisecond on the measured
serial CPU. A future SIMD experiment should first isolate batched kinematics,
random-number generation, and distance scoring behind the existing planner
interface, then require a repeatable end-to-end improvement rather than only
a micro-kernel speedup.

A GPU backend is also deferred. At the default scale, launch, synchronization,
and transfer overhead are unlikely to recover their integration cost. Large
1024-by-40 workloads reach roughly 11 ms and are the first plausible target,
but the lower-complexity next experiment is the existing OpenMP rollout
backend on hardware with a supported runtime.

Revisit SIMD or GPU acceleration when a production scenario meets both of
these conditions:

1. The benchmark or an application trace shows MPPI consuming at least 30%
   of the control-cycle budget at the required rollout and horizon settings.
2. OpenMP at the target worker count either remains over budget or cannot be
   deployed, and an acceleration prototype demonstrates at least a 1.5x
   end-to-end speedup without changing deterministic seeds, collision gates,
   or numerical acceptance tests.

This keeps CPU/OpenMP as the portable implementation while preserving a
quantitative trigger for adding a more specialized backend later.
