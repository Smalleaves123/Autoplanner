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
