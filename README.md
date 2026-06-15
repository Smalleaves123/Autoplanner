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
