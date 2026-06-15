#include "autoplanner/planners/graph_search/jps.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>

#include "autoplanner/heuristics/euclidean.h"

namespace autoplanner {

namespace {

struct QueueNode {
    int index = -1;
    double f = 0.0;
    bool operator>(const QueueNode& o) const { return f > o.f; }
};

Path2d reconstructPath(const GridMap& map, const std::vector<int>& parent,
                       int si, int gi) {
    Path2d path;
    int cur = gi;
    while (cur != -1) {
        path.emplace_back(static_cast<double>(cur % map.width()),
                          static_cast<double>(cur / map.width()));
        if (cur == si) break;
        cur = parent[static_cast<std::size_t>(cur)];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Try to "jump" in direction (dx,dy) from (x,y). Returns the index of
// a jump point, or -1 if none found.
int jump(const GridMap& map, int x, int y, int dx, int dy,
         int goal_x, int goal_y, bool allow_diag) {
    int nx = x + dx;
    int ny = y + dy;
    if (!map.isFree(nx, ny)) return -1;
    if (nx == goal_x && ny == goal_y) return map.index(nx, ny);

    // Diagonal jump: check both cardinal directions for forced neighbours.
    if (dx != 0 && dy != 0) {
        if (allow_diag) {
            if ((map.isFree(nx - dx, ny) && !map.isFree(nx - dx, ny - dy)) ||
                (map.isFree(nx, ny - dy) && !map.isFree(nx - dx, ny - dy)))
                return map.index(nx, ny);
            if (jump(map, nx, ny, dx, 0, goal_x, goal_y, allow_diag) != -1 ||
                jump(map, nx, ny, 0, dy, goal_x, goal_y, allow_diag) != -1)
                return map.index(nx, ny);
        }
    } else {
        // Cardinal jump: scan for forced neighbours.
        if (dx != 0) {
            if (allow_diag) {
                if ((map.isFree(nx, ny - 1) && !map.isFree(nx - dx, ny - 1)) ||
                    (map.isFree(nx, ny + 1) && !map.isFree(nx - dx, ny + 1)))
                    return map.index(nx, ny);
            }
        } else {
            if (allow_diag) {
                if ((map.isFree(nx - 1, ny) && !map.isFree(nx - 1, ny - dy)) ||
                    (map.isFree(nx + 1, ny) && !map.isFree(nx + 1, ny - dy)))
                    return map.index(nx, ny);
            }
        }
    }

    // Continue jumping in same direction.
    return allow_diag || (dx == 0 || dy == 0)
        ? jump(map, nx, ny, dx, dy, goal_x, goal_y, allow_diag)
        : -1;
}

std::vector<std::pair<int,int>> successors(bool allow_diag) {
    std::vector<std::pair<int,int>> dirs = {{1,0},{-1,0},{0,1},{0,-1}};
    if (allow_diag)
        for (int dx : {1,-1})
            for (int dy : {1,-1})
                dirs.emplace_back(dx, dy);
    return dirs;
}

}  // namespace

JPSPlanner::JPSPlanner(bool allow_diagonal)
    : allow_diagonal_(allow_diagonal)
    , heuristic_(std::make_unique<EuclideanHeuristic>()) {}

void JPSPlanner::setHeuristic(std::unique_ptr<Heuristic> h) {
    heuristic_ = std::move(h);
}

std::string JPSPlanner::name() const { return "jps"; }

PlannerResult JPSPlanner::plan(const GridMap& map, const Point2i& start,
                               const Point2i& goal) {
    PlannerResult result;
    result.planner_name = name();
    auto t0 = std::chrono::steady_clock::now();

    if (!map.isFree(start.x, start.y)) {
        result.message = "Start is occupied."; return result;
    }
    if (!map.isFree(goal.x, goal.y)) {
        result.message = "Goal is occupied."; return result;
    }

    int w = map.width(), h = map.height(), n = w * h;
    int si = map.index(start.x, start.y);
    int gi = map.index(goal.x, goal.y);

    std::vector<double> g(static_cast<std::size_t>(n),
                          std::numeric_limits<double>::infinity());
    std::vector<int> parent(static_cast<std::size_t>(n), -1);
    std::vector<unsigned char> closed(static_cast<std::size_t>(n), 0);

    std::priority_queue<QueueNode, std::vector<QueueNode>,
                        std::greater<QueueNode>> open;

    g[static_cast<std::size_t>(si)] = 0.0;
    open.push({si, heuristic_->compute(start, goal)});

    auto dirs = successors(allow_diagonal_);

    while (!open.empty()) {
        auto cur = open.top(); open.pop();
        int ci = cur.index;
        if (closed[static_cast<std::size_t>(ci)]) continue;
        closed[static_cast<std::size_t>(ci)] = 1;
        result.expanded_nodes++;

        if (ci == gi) { result.success = true; break; }

        int cx = ci % w, cy = ci / w;

        for (auto [dx, dy] : dirs) {
            int jp = jump(map, cx, cy, dx, dy, goal.x, goal.y,
                          allow_diagonal_);
            if (jp < 0) continue;
            if (closed[static_cast<std::size_t>(jp)]) continue;

            int jx = jp % w, jy = jp / w;
            double step = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
            double dist = std::sqrt(static_cast<double>(
                (jx - cx) * (jx - cx) + (jy - cy) * (jy - cy)));
            double tg = g[static_cast<std::size_t>(ci)] + step * (dist / std::sqrt(dx*dx + dy*dy));

            if (tg < g[static_cast<std::size_t>(jp)]) {
                g[static_cast<std::size_t>(jp)] = tg;
                parent[static_cast<std::size_t>(jp)] = ci;
                Point2i jp_pt(jx, jy);
                open.push({jp, tg + heuristic_->compute(jp_pt, goal)});
            }
        }
    }

    if (result.success) {
        result.path = reconstructPath(map, parent, si, gi);
        result.path_length = computePathLength(result.path);
        result.message = "Path found.";
    } else {
        result.message = "No path found.";
    }

    auto t1 = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.iterations = result.expanded_nodes;
    return result;
}

}  // namespace autoplanner
