# Algorithms

AutoPlanner implements three families of path planning algorithms: graph search,
sampling-based, and kinematic planning.

---

## Graph Search Algorithms

All graph search algorithms operate on a grid graph where each cell is a node
and edges connect to 4 or 8 neighbours.

### Dijkstra

The simplest shortest-path algorithm. It expands nodes in order of their
distance from the start, without any goal-directed heuristic.

**Cost function:** `f(n) = g(n)` (actual cost from start)

**Properties:**
- Guarantees the shortest path on the grid.
- Explores in all directions equally — slow on large maps.
- Serves as a baseline for evaluating heuristics.

### A\*

Combines Dijkstra's actual cost with a heuristic estimate to focus search.

**Cost function:** `f(n) = g(n) + h(n)`

where `g(n)` is the cost from start to node `n`, and `h(n)` is the estimated
cost from `n` to the goal.

**Properties:**
- Optimal if `h(n)` is admissible (never overestimates).
- Much faster than Dijkstra on open maps.
- Path may hug obstacles with naive heuristics.

**Heuristic options:**

| Heuristic | Formula | Admissible for |
|-----------|---------|----------------|
| Manhattan | `|dx| + |dy|` | 4-connected grids |
| Euclidean | `sqrt(dx² + dy²)` | any grid |
| Diagonal | `max(|dx|,|dy|) + (√2-1)·min(|dx|,|dy|)` | 8-connected grids |

### Weighted A\*

Inflates the heuristic to bias search more aggressively toward the goal.

**Cost function:** `f(n) = g(n) + w · h(n)`

**Properties:**
- Weight `w > 1` trades optimality for speed.
- Useful when planning time matters more than path optimality.
- Typical weights: 1.2, 1.5, 2.0.

### Improved A\*

Extends A\* with obstacle proximity cost and turning penalty to produce paths
that are safer and smoother for real robots.

**Cost function:**

```
f(n) = g(n) + w_h·h(n) + w_obs·obstacle_cost(n) + w_turn·turning_cost(n)
```

| Term | Purpose |
|------|---------|
| `g(n)` | Actual cost from start |
| `w_h·h(n)` | Weighted heuristic |
| `w_obs·obstacle_cost(n)` | Penalizes cells near obstacles |
| `w_turn·turning_cost(n)` | Penalizes direction changes |

**Expected behavior vs standard A\*:**
- Slightly longer path length (tradeoff for safety).
- Greater minimum obstacle distance.
- Fewer turns → easier for robot tracking.
- Works best with a pre-computed `Costmap2D`.

### Jump Point Search (JPS)

Accelerates A\* on uniform-cost grids by skipping intermediate nodes and
jumping directly to "interesting" decision points.

**Properties:**
- Optimal, same paths as A\*.
- Dramatically fewer expanded nodes on open maps.
- Best on large grids with uniform traversal cost.

### D\* Lite

An incremental replanning algorithm for dynamic environments where obstacles
may appear or disappear after the initial plan.

**Properties:**
- Efficiently repairs paths when the map changes locally.
- Useful for online robot navigation.
- Heavier to implement than A\*, but much faster for replanning.

---

## Sampling-Based Algorithms

Sampling planners operate in continuous space and grow a tree of configurations.

### RRT (Rapidly-exploring Random Tree)

Grows a tree from the start by repeatedly sampling random points and extending
the nearest node toward the sample.

**Parameters:**

| Parameter | Typical | Description |
|-----------|---------|-------------|
| `step_size` | 2.0 | Maximum extension per iteration |
| `max_iterations` | 5000 | Stopping condition |
| `goal_sample_rate` | 0.1 | Probability of sampling the goal directly |
| `goal_tolerance` | 2.0 | Distance to consider goal reached |

**Properties:**
- Probabilistically complete — will find a path if one exists, given enough time.
- Not optimal; path quality varies between runs.
- Works well in high-dimensional or complex spaces.

### RRT\*

An asymptotically optimal variant of RRT. After growing a node, it reconsiders
parents in a local neighbourhood and rewires the tree to reduce path cost.

**Additional parameter:** `rewire_radius` — neighbourhood search radius.

**Properties:**
- Path cost monotonically decreases with more iterations.
- Slower per iteration than RRT due to rewiring overhead.
- Converges to the optimal path as iterations → ∞.

### Informed RRT\*

Further improves RRT\* by restricting the sampling space to an ellipse defined
by the start, goal, and current best path length — once a feasible path is found.

**Properties:**
- Faster convergence than RRT\*.
- Same asymptotic optimality guarantee.

### Bi-RRT

Grows two trees simultaneously: one from the start and one from the goal.
Attempts to connect them at each iteration.

**Properties:**
- Often finds paths faster than unidirectional RRT.
- Particularly effective in narrow passages.

---

## Kinodynamic Planning

### Hybrid A\*

Extends A\* to a 3D state space `(x, y, θ)` and applies kinematic motion
primitives instead of grid steps.

**Motion primitives:**
- Forward: left, straight, right
- Backward: left, straight, right

**Properties:**
- Produces paths that respect vehicle kinematics (minimum turning radius).
- Suitable for car-like or differential-drive robots.
- More computationally expensive than grid A\*.

---

## Path Smoothing

Raw planner outputs often contain redundant waypoints and sharp turns.
Smoothing post-processes these paths.

### Shortcut Smoothing

Randomly picks two points on the path. If the straight line between them is
collision-free, removes all intermediate points. Repeats for `max_iterations`.

- Fast and effective.
- Guarantees no new collisions.

### Bezier Smoothing

Fits cubic Bezier curves through path segments and resamples at higher density.

- Produces visually smooth curves.
- Does not guarantee collision freedom — pair with a collision checker.

### B-Spline Smoothing

Uses B-spline interpolation for locally-controlled smooth curves.

- Better local control than Bezier.
- Suitable for trajectory generation.

---

## Costmap & Safety Layers

### Obstacle Inflation

Expands obstacle boundaries by the robot radius so that planning treats
near-obstacle cells as high-cost rather than free.

### Distance Transform

Computes the distance from each free cell to the nearest obstacle.
Used by `Costmap2D` to assign graduated costs.

### Obstacle Cost Layer

Assigns a cost based on proximity:

```
obstacle_cost(cell) = exp(-distance(cell) / sigma)
```

Closer to obstacle → higher cost → planners avoid it.
