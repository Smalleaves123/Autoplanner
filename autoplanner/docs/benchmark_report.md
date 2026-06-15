# Benchmark Report

## Methodology

All planners are evaluated on four map types with fixed start/goal pairs.
Timing measurements exclude map loading and I/O overhead.

### Maps

| Map | Size | Type |
|-----|------|------|
| `simple_50x50.txt` | 50×50 | Open area with scattered obstacles |
| `maze_100x100.txt` | 100×100 | Maze-like corridors |
| `warehouse_100x100.txt` | 100×100 | Warehouse with shelf aisles |
| `random_100x100_density_20.txt` | 100×100 | Random obstacles at 20% density |

### Planners Compared

- **Dijkstra** — baseline shortest-path (no heuristic)
- **A\*** — standard with Euclidean heuristic
- **Weighted A\*** — A\* with weight 1.5
- **Improved A\*** — A\* with obstacle + turning costs
- **JPS** — jump point search acceleration
- **RRT** — sampling-based with goal bias
- **RRT\*** — asymptotically optimal sampling

### Metrics

| Metric | Description |
|--------|-------------|
| `planning_time_ms` | Wall-clock planning time |
| `path_length` | Euclidean length of the output path |
| `expanded_nodes` | Nodes visited during search |
| `turning_count` | Number of direction changes in the path |
| `smoothness_score` | Ratio of straight-line distance to path length (1.0 = straight) |
| `minimum_obstacle_distance` | Closest approach to any obstacle |

---

## Expected Results

Based on algorithm properties, the benchmark is expected to show the following
trends:

### Planning Time

```
RRT ≈ RRT* < JPS < A* < Weighted A* < Improved A* < Dijkstra
```

- Dijkstra expands in all directions → slowest.
- JPS skips intermediate nodes → faster than A\* on open maps.
- RRT/RRT\* sample sparsely → fast to find a first path.

### Path Length

```
Dijkstra ≈ A* ≈ JPS < Weighted A* < Improved A* < RRT* < RRT
```

- Dijkstra/A\*/JPS produce optimal or near-optimal lengths.
- Weighted A\* may inflate length slightly.
- Improved A\* may be longer but safer.
- RRT paths are typically longer and noisier.

### Expanded Nodes

```
JPS ≪ A* < Weighted A* < Improved A* < Dijkstra
```

- JPS dramatically reduces expanded nodes on uniform grids.
- The heuristic in A\* directs search, reducing nodes vs Dijkstra.
- Weighted A\* tightens the search further.

### Safety (Min Obstacle Distance)

```
Improved A* ≫ A* ≈ Dijkstra ≈ JPS
```

- Improved A\* explicitly penalizes obstacle proximity.
- Standard planners have no safety margin beyond grid connectivity.

### Path Smoothness

```
Improved A* > A* (after smoothing) > RRT* > RRT
```

- Improved A\*'s turning penalty produces inherently smoother paths.
- Post-smoothing (shortcut, Bezier) dramatically improves all planners.

---

## Ablation Study: Improved A\*

The contribution of each term in Improved A\*'s cost function can be isolated:

| Configuration | Obstacle cost | Turning cost | Expected effect |
|---------------|---------------|--------------|-----------------|
| A\* | off | off | baseline |
| A\* + obstacle | on | off | greater safety distance |
| A\* + turning | off | on | fewer turns |
| Improved A\* | on | on | best safety + smoothness |

---

## Regenerating Results

```bash
# Build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# Run full benchmark
./build/benchmark/benchmark_all

# Compare planners on one map
./build/apps/compare_planners --map data/maps/warehouse_100x100.txt --start 1 1 --goal 98 98

# Plot results (requires Python with matplotlib)
python scripts/plot_benchmark.py results/benchmark/benchmark_all.csv
```
