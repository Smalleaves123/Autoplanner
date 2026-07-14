# AutoMPC

A C++ Model Predictive Control library for mobile robot trajectory tracking.

## Overview

AutoMPC takes a reference trajectory and robot state, computes optimal control
inputs (velocity + steering) to keep the robot on track.

## Controllers

| Controller | Type | Key Property |
|-----------|------|-------------|
| **PID** | Feedback | Dual PID loops for velocity and steering |
| **Pure Pursuit** | Geometric | Follows a lookahead point on the path |
| **Stanley** | Geometric | Cross-track error + heading correction |
| **LQR** | Optimal | Linear Quadratic Regulator (requires Eigen3) |
| **MPC** | Predictive | Finite-horizon receding-horizon control (requires Eigen3) |

## Quick Start

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# Run circle tracking demo (compares all controllers)
./build/examples/circle_tracking

# Visualize results
python scripts/plot_tracking.py results/circle_pid.csv results/circle_pure_pursuit.csv results/circle_stanley.csv
```

### CLI

```bash
./build/apps/autompc_cli --controller pid --trajectory circle --radius 5.0 --steps 500
./build/apps/autompc_cli --controller stanley --trajectory line --steps 200
./build/apps/autompc_cli --controller mpc --trajectory line --steps 200 --mpc-horizon 15
```

### With Eigen3 (LQR and MPC controllers)

```bash
brew install eigen  # macOS
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
# LQR controller will be automatically included
```

## Project Structure

```
AutoMPC/
├── CMakeLists.txt
├── README.md
├── include/autompc/
│   ├── autompc.h              # Top-level header
│   ├── core/                   # State, Control, Trajectory, kinematics
│   ├── controllers/            # PID, Pure Pursuit, Stanley, LQR
│   └── trajectory/             # Tracker, error metrics
├── src/                        # Implementations
├── apps/                       # CLI tool
├── examples/                   # Circle tracking demo
├── tests/                      # Unit tests
├── scripts/                    # Python plotting
└── docs/                       # Generated images
```

## API

```cpp
#include "autompc/autompc.h"

using namespace autompc;

// Create a reference trajectory
auto ref = makeCircle(5.0, 1.0, 500);  // radius 5m, 1m/s

// Set up a controller
PIDController pid(1.0, 0.0, 0.0,  2.0, 0.0, 0.5);
State initial{5.0, 0.0, M_PI_2, 0.0};

// Simulate
auto actual = simulate(initial, ref, pid, 0.05, 25.0);

// Evaluate
auto errors = computeErrors(actual, ref);
```

## Integration with AutoPlanner

```
AutoPlanner (path planning) → Trajectory → AutoMPC (tracking) → Control commands
```

Use AutoPlanner's path output as AutoMPC's reference trajectory input.

AutoMPC also exposes the C++ controllers to Python when Eigen3 is available:

```python
import autompc

reference = autompc.make_straight_line(0, 0, 20, 0, 1.0, 41)
controller = autompc.MPCController(horizon=15)
control = controller.compute(
    autompc.State(0, 1, 0, 0), reference, 1.0)
```

MPC input and rate constraints can be configured from Python:

```python
controller = autompc.MPCController(
    horizon=15,
    max_velocity=2.0,
    max_acceleration=1.5,
    max_deceleration=2.0,
    max_steering_rate=1.5,
)
```

The CLI also accepts an AutoPlanner waypoint CSV directly:

```bash
./build/apps/autompc_cli \
    --controller stanley \
    --trajectory path \
    --path ../autoplanner/results/path.csv \
    --velocity 1.0 \
    --output results/path_tracking.csv
```
