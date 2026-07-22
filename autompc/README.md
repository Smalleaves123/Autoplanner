# AutoMPC

A C++ Model Predictive Control library for mobile robot trajectory tracking.

## Overview

AutoMPC takes a reference trajectory and robot state, computes optimal control
inputs (velocity + steering) to keep the robot on track.

The finite-horizon MPC also supports a configurable terminal state cost,
bounded tracking-error state, and a soft steering-rate regularization term in
addition to velocity, steering, acceleration, deceleration, and steering-rate
constraints.

For AutoPlanner waypoint CSVs, AutoMPC first builds a controller-ready
trajectory: waypoints are resampled by arc length, headings and curvature are
computed, and velocity is limited by curvature, acceleration, and braking
constraints. The generated reference is written as `trajectory.csv` by the
CLI when tracking a path.

The local execution model is a constrained kinematic bicycle simulator. It
limits velocity, acceleration, braking, steering angle, and steering rate, and
keeps steering as actuator state between control updates. This ROS-free
backend is used by the CLI and dynamic-navigation benchmark; the legacy
instantaneous `step` function remains available for compatibility.

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

# Configure the local vehicle execution model
./build/apps/autompc_cli --controller stanley --trajectory line \
    --wheelbase 1.2 --max-acceleration 1.0 --max-steering-rate 1.0

# Generate a constrained reference trajectory from an AutoPlanner path
./build/apps/autompc_cli \
    --controller mpc --trajectory path \
    --path ../autoplanner/results/path.csv \
    --velocity 1.0 --sample-spacing 0.25 \
    --max-lateral-acceleration 1.0 \
    --trajectory-output results/reference_trajectory.csv
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
│   ├── trajectory/             # Tracker, error metrics
│   └── simulation/             # Constrained kinematic bicycle execution
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

sim_options = autompc.SimulationOptions()
sim_options.wheelbase = 1.2
sim_options.max_acceleration = 1.0
simulator = autompc.KinematicBicycleSimulator(
    autompc.State(0, 0, 0, 0), sim_options)
state = simulator.step(autompc.Control(1.0, 0.2))
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
