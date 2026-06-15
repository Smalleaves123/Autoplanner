#include "autoplanner/planners/graph_search/astar.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "autoplanner/heuristics/euclidean.h"

namespace autoplanner {

namespace {

// Lightweight node for the open priority queue — ordered by f = g + h.
struct QueueNode {
    int index = -1;
    double f = 0.0;

    bool operator>(const QueueNode& other) const {
        return f > other.f;
    }
};

// Backtrack from goal to start using parent pointers, then reverse.
Path2d reconstructPath(
    const GridMap& map,
    const std::vector<int>& parent,
    int start_index,
    int goal_index
) {
    Path2d path;
    int current = goal_index;

    while (current != -1) {
        const int x = current % map.width();
        const int y = current / map.width();
        path.emplace_back(static_cast<double>(x), static_cast<double>(y));

        if (current == start_index) {
            break;
        }

        current = parent[static_cast<std::size_t>(current)];
    }

    std::reverse(path.begin(), path.end());
    return path;
}

// Diagonal moves must not cut corners — both adjacent cardinal cells
// must be free before a diagonal step is allowed.
bool canMoveDiagonal(const GridMap& map, int x, int y, int dx, int dy) {
    if (dx == 0 || dy == 0) {
        return true;
    }
    return map.isFree(x + dx, y) && map.isFree(x, y + dy);
}

}  // namespace

AStarPlanner::AStarPlanner(bool allow_diagonal)
    : allow_diagonal_(allow_diagonal)
    , heuristic_(std::make_unique<EuclideanHeuristic>()) {}

std::string AStarPlanner::name() const {
    return "astar";
}

void AStarPlanner::setHeuristic(std::unique_ptr<Heuristic> heuristic) {
    heuristic_ = std::move(heuristic);
}

PlannerResult AStarPlanner::plan(
    const GridMap& map,
    const Point2i& start,
    const Point2i& goal
) {
    PlannerResult result;
    result.planner_name = name();

    const auto time_begin = std::chrono::steady_clock::now();

    // Validate start and goal are traversable.
    if (!map.isFree(start.x, start.y)) {
        result.message = "Start is invalid or occupied.";
        return result;
    }

    if (!map.isFree(goal.x, goal.y)) {
        result.message = "Goal is invalid or occupied.";
        return result;
    }

    const int width = map.width();
    const int height = map.height();
    const int node_count = width * height;

    const int start_index = map.index(start.x, start.y);
    const int goal_index = map.index(goal.x, goal.y);

    // g_score[node] = cheapest known cost from start to node.
    std::vector<double> g_score(
        static_cast<std::size_t>(node_count),
        std::numeric_limits<double>::infinity()
    );

    // parent[node] = predecessor index for path reconstruction (-1 = none).
    std::vector<int> parent(static_cast<std::size_t>(node_count), -1);

    // closed[node] = 1 once the node has been expanded.
    std::vector<unsigned char> closed(static_cast<std::size_t>(node_count), 0);

    std::priority_queue<
        QueueNode,
        std::vector<QueueNode>,
        std::greater<QueueNode>
    > open;

    g_score[static_cast<std::size_t>(start_index)] = 0.0;
    const double h_start = heuristic_->compute(start, goal);
    open.push({start_index, h_start});

    // Movement directions: 4-connected or 8-connected.
    const std::vector<int> dx4 = {1, -1, 0, 0};
    const std::vector<int> dy4 = {0, 0, 1, -1};
    const std::vector<int> dx8 = {1, -1, 0, 0, 1, 1, -1, -1};
    const std::vector<int> dy8 = {0, 0, 1, -1, 1, -1, 1, -1};

    const auto& dxs = allow_diagonal_ ? dx8 : dx4;
    const auto& dys = allow_diagonal_ ? dy8 : dy4;

    while (!open.empty()) {
        const QueueNode current_node = open.top();
        open.pop();

        const int current_index = current_node.index;

        // Skip if already expanded — duplicates may exist in the PQ.
        if (closed[static_cast<std::size_t>(current_index)] != 0) {
            continue;
        }

        closed[static_cast<std::size_t>(current_index)] = 1;
        result.expanded_nodes++;

        // Goal reached.
        if (current_index == goal_index) {
            result.success = true;
            break;
        }

        const int current_x = current_index % width;
        const int current_y = current_index / width;

        for (std::size_t i = 0; i < dxs.size(); ++i) {
            const int nx = current_x + dxs[i];
            const int ny = current_y + dys[i];

            if (!map.isFree(nx, ny)) {
                continue;
            }

            if (!canMoveDiagonal(map, current_x, current_y, dxs[i], dys[i])) {
                continue;
            }

            const int neighbor_index = map.index(nx, ny);

            if (closed[static_cast<std::size_t>(neighbor_index)] != 0) {
                continue;
            }

            // Diagonal steps cost sqrt(2); cardinal steps cost 1.
            const double step_cost = (dxs[i] != 0 && dys[i] != 0)
                ? std::sqrt(2.0)
                : 1.0;

            const double tentative_g =
                g_score[static_cast<std::size_t>(current_index)] + step_cost;

            // Found a cheaper path to neighbor — relax edge.
            if (tentative_g < g_score[static_cast<std::size_t>(neighbor_index)]) {
                g_score[static_cast<std::size_t>(neighbor_index)] = tentative_g;
                parent[static_cast<std::size_t>(neighbor_index)] = current_index;

                const Point2i neighbor_point(nx, ny);
                const double h = heuristic_->compute(neighbor_point, goal);
                const double f = tentative_g + h;

                open.push({neighbor_index, f});
            }
        }
    }

    if (result.success) {
        result.path = reconstructPath(map, parent, start_index, goal_index);
        result.path_length = computePathLength(result.path);
        result.message = "Path found.";
    } else {
        result.message = "No path found.";
    }

    const auto time_end = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(time_end - time_begin).count();

    result.iterations = result.expanded_nodes;

    return result;
}

}  // namespace autoplanner
