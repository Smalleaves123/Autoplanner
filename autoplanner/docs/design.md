# Design Document

## Overview

AutoPlanner is a modular C++ path planning library for 2D mobile robot navigation.
It provides a unified interface for multiple planning algorithms, a costmap
pipeline for safety-aware navigation, collision checking, path smoothing, and a
benchmarking framework.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     CLI Apps / Examples                  │
├─────────────────────────────────────────────────────────┤
│  autoplanner_cli  │  run_single_planner  │  compare_...  │
└──────────────────────┬──────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────┐
│                   PlannerBase (interface)                │
├──────────┬──────────┬──────────┬──────────┬─────────────┤
│ Dijkstra │   A*     │  JPS     │   RRT    │  RRT*       │
│ Weighted │ Improved │ D* Lite  │ Bi-RRT   │  Informed   │
│   A*     │   A*     │          │          │  RRT*       │
│          │          │          │          │  Hybrid A*  │
└──────────┴──────────┴──────────┴──────────┴─────────────┘
                       │
┌──────────────────────▼──────────────────────────────────┐
│                   Supporting Modules                     │
├──────────┬──────────┬──────────┬──────────┬─────────────┤
│ Costmap  │ Collision│ Smoothing│ Heuristics│  Metrics   │
│   2D     │ Checker  │          │           │            │
└──────────┴──────────┴──────────┴──────────┴─────────────┘
                       │
┌──────────────────────▼──────────────────────────────────┐
│                   Core Data Types                        │
├──────────┬──────────┬──────────┬──────────┬─────────────┤
│ Point2i  │ Point2d  │ Pose2d   │ GridMap  │ PlannerRes. │
│   Path   │  Node    │  Types   │          │             │
└──────────┴──────────┴──────────┴──────────┴─────────────┘
```

## Key Design Decisions

### 1. Unified Planner Interface

All planners inherit from `PlannerBase` and implement a single `plan()` method:

```cpp
PlannerResult plan(const GridMap& map, const Point2i& start, const Point2i& goal);
```

This makes it trivial to swap algorithms, run benchmarks, and add new planners.

### 2. Separation of Concerns

- **GridMap** handles map storage and queries — planners do not parse files.
- **Costmap2D** computes safety cost layers independent of any planner.
- **CollisionChecker** is an abstract interface, decoupled from specific planners.
- **PathSmoother** is a post-processing step applied after planning.
- **Metrics** are computed separately and can be used by any planner result.

### 3. Two Coordinate Spaces

| Type | Domain | Use |
|------|--------|-----|
| `Point2i` | Integer grid cells | Map queries, graph search nodes |
| `Point2d` | Continuous metric world | Sampling planners, output path, smoothing |

This avoids confusion between grid indices and world coordinates.

### 4. Graph Search vs Sampling

- **Graph search** planners (Dijkstra, A*, JPS, D* Lite) operate on grid cells
  and guarantee optimality under their cost models.
- **Sampling-based** planners (RRT, RRT*, Bi-RRT) operate in continuous space
  and are suited for high-dimensional or complex constraint spaces.
- **Hybrid A\*** bridges both worlds by searching in (x, y, θ) state space.

## Module Relationships

### Planner depends on

- `GridMap` — the occupancy grid to plan on.
- `Heuristic` — for graph search planners (A*, JPS, etc.).
- `Costmap2D` — for Improved A\* to incorporate obstacle proximity.
- `CollisionChecker` — for sampling planners and Hybrid A\*.

### Costmap depends on

- `GridMap` — source data to build from.
- `ObstacleInflation` — utility to expand obstacle regions.

### Smoothing depends on

- `CollisionChecker` — to ensure smoothed paths remain valid.

## Build System

- **C++17** standard, CMake ≥ 3.16.
- Single static library `libautoplanner.a` containing all modules.
- CLI apps and benchmarks link against it.
- Unit tests use a separate `tests/CMakeLists.txt` with `BUILD_TESTS` option.

## Extensibility

To add a new planner:
1. Create a class inheriting `PlannerBase`.
2. Implement `plan()` and `name()`.
3. Add the source file to `CMakeLists.txt`.
4. Register it in the CLI and benchmark apps.

No other code changes are needed — the architecture is open/closed.

## Python Integration

Python is used for auxiliary tasks:
- Map generation (`generate_random_map.py`, `generate_warehouse_map.py`)
- Path visualization (`visualize_path.py`)
- Benchmark plotting (`plot_benchmark.py`)
- Costmap visualization (`visualize_costmap.py`)

The C++ core remains pure C++ with no Python dependency at runtime.
