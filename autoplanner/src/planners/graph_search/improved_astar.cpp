#include "autoplanner/planners/graph_search/improved_astar.h"

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
    bool operator>(const QueueNode& other) const { return f > other.f; }
};

Path2d reconstructPath(
    const GridMap& map, const std::vector<int>& parent,
    int start_index, int goal_index
) {
    Path2d path;
    int current = goal_index;
    while (current != -1) {
        path.emplace_back(static_cast<double>(current % map.width()),
                          static_cast<double>(current / map.width()));
        if (current == start_index) break;
        current = parent[static_cast<std::size_t>(current)];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

bool canMoveDiagonal(const GridMap& map, int x, int y, int dx, int dy) {
    if (dx == 0 || dy == 0) return true;
    return map.isFree(x + dx, y) && map.isFree(x, y + dy);
}

int moveDirection(int px, int py, int cx, int cy) {
    int dx = cx - px;
    int dy = cy - py;
    if (dx == 0 && dy == 0) return -1;
    return (dx > 0 ? 1 : dx < 0 ? -1 : 0) * 3 + (dy > 0 ? 1 : dy < 0 ? -1 : 0);
}

}  // namespace

ImprovedAStarPlanner::ImprovedAStarPlanner(double heuristic_weight,
                                           double obstacle_weight,
                                           double turning_weight,
                                           bool allow_diagonal)
    : heuristic_weight_(heuristic_weight)
    , obstacle_weight_(obstacle_weight)
    , turning_weight_(turning_weight)
    , allow_diagonal_(allow_diagonal)
    , heuristic_(std::make_unique<EuclideanHeuristic>()) {}

void ImprovedAStarPlanner::setHeuristic(std::unique_ptr<Heuristic> heuristic) {
    heuristic_ = std::move(heuristic);
}

void ImprovedAStarPlanner::setCostmap(const Costmap2D* costmap) {
    costmap_ = costmap;
}

std::string ImprovedAStarPlanner::name() const {
    return "improved_astar";
}

PlannerResult ImprovedAStarPlanner::plan(
    const GridMap& map, const Point2i& start, const Point2i& goal
) {
    PlannerResult result;
    result.planner_name = name();
    const auto time_begin = std::chrono::steady_clock::now();

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

    std::vector<double> g_score(static_cast<std::size_t>(node_count),
                                std::numeric_limits<double>::infinity());
    std::vector<int> parent(static_cast<std::size_t>(node_count), -1);
    std::vector<unsigned char> closed(static_cast<std::size_t>(node_count), 0);

    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;

    g_score[static_cast<std::size_t>(start_index)] = 0.0;
    open.push({start_index, heuristic_weight_ * heuristic_->compute(start, goal)});

    const std::vector<int> dx4 = {1, -1, 0, 0};
    const std::vector<int> dy4 = {0, 0, 1, -1};
    const std::vector<int> dx8 = {1, -1, 0, 0, 1, 1, -1, -1};
    const std::vector<int> dy8 = {0, 0, 1, -1, 1, -1, 1, -1};
    const auto& dxs = allow_diagonal_ ? dx8 : dx4;
    const auto& dys = allow_diagonal_ ? dy8 : dy4;

    while (!open.empty()) {
        QueueNode current_node = open.top();
        open.pop();
        int current_index = current_node.index;

        if (closed[static_cast<std::size_t>(current_index)] != 0) continue;
        closed[static_cast<std::size_t>(current_index)] = 1;
        result.expanded_nodes++;

        if (current_index == goal_index) {
            result.success = true;
            break;
        }

        int current_x = current_index % width;
        int current_y = current_index / width;
        int parent_index = parent[static_cast<std::size_t>(current_index)];
        int parent_x = (parent_index >= 0) ? parent_index % width : current_x;
        int parent_y = (parent_index >= 0) ? parent_index / width : current_y;
        int prev_dir = moveDirection(parent_x, parent_y, current_x, current_y);

        for (std::size_t i = 0; i < dxs.size(); ++i) {
            int nx = current_x + dxs[i];
            int ny = current_y + dys[i];
            if (!map.isFree(nx, ny)) continue;
            if (!canMoveDiagonal(map, current_x, current_y, dxs[i], dys[i])) continue;

            int neighbor_index = map.index(nx, ny);
            if (closed[static_cast<std::size_t>(neighbor_index)] != 0) continue;

            double step_cost = (dxs[i] != 0 && dys[i] != 0) ? std::sqrt(2.0) : 1.0;

            double obs_cost = 0.0;
            if (costmap_ != nullptr && obstacle_weight_ > 0.0) {
                obs_cost = costmap_->getCost(nx, ny);
            }

            double turn_cost = 0.0;
            if (turning_weight_ > 0.0 && prev_dir >= 0) {
                int new_dir = moveDirection(current_x, current_y, nx, ny);
                if (new_dir != prev_dir) turn_cost = 1.0;
            }

            double edge_cost = step_cost + obstacle_weight_ * obs_cost
                               + turning_weight_ * turn_cost;
            double tentative_g = g_score[static_cast<std::size_t>(current_index)] + edge_cost;

            if (tentative_g < g_score[static_cast<std::size_t>(neighbor_index)]) {
                g_score[static_cast<std::size_t>(neighbor_index)] = tentative_g;
                parent[static_cast<std::size_t>(neighbor_index)] = current_index;
                Point2i neighbor_point(nx, ny);
                double h = heuristic_->compute(neighbor_point, goal);
                open.push({neighbor_index, tentative_g + heuristic_weight_ * h});
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
