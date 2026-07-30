#include "robotnav/space_time_astar.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace robotnav {
namespace {

struct QueueNode {
    int x = 0;
    int y = 0;
    std::size_t time = 0;
    double f = 0.0;

    bool operator>(const QueueNode& other) const {
        return f > other.f;
    }
};

struct Parent {
    int x = -1;
    int y = -1;
    std::size_t time = 0;
};

double heuristic(const autoplanner::Point2i& cell,
                 const autoplanner::Point2i& goal) {
    return autoplanner::distance(cell, goal);
}

std::size_t stateIndex(const autoplanner::GridMap& map,
                       int x,
                       int y,
                       std::size_t time) {
    const auto layer_size = static_cast<std::size_t>(
        map.width() * map.height());
    return time * layer_size +
           static_cast<std::size_t>(map.index(x, y));
}

bool canMoveDiagonal(const autoplanner::GridMap& map,
                     int x,
                     int y,
                     int dx,
                     int dy) {
    if (dx == 0 || dy == 0) return true;
    return map.isFree(x + dx, y) && map.isFree(x, y + dy);
}

bool predictedFree(const std::vector<MovingObstacle>& obstacles,
                   const autoplanner::Point2i& cell,
                   std::size_t absolute_frame) {
    return !isPredictedOccupied(obstacles, cell, absolute_frame);
}

autoplanner::Path2d reconstructPath(
    const autoplanner::GridMap& map,
    const std::vector<Parent>& parents,
    const autoplanner::Point2i& start,
    const QueueNode& goal_node) {
    std::vector<autoplanner::Point2i> cells;
    int x = goal_node.x;
    int y = goal_node.y;
    std::size_t time = goal_node.time;

    while (x >= 0 && y >= 0) {
        cells.push_back({x, y});
        if (x == start.x && y == start.y && time == 0) break;
        const auto parent = parents[stateIndex(map, x, y, time)];
        x = parent.x;
        y = parent.y;
        time = parent.time;
    }
    std::reverse(cells.begin(), cells.end());

    autoplanner::Path2d path;
    path.reserve(cells.size());
    for (const auto& cell : cells) {
        path.emplace_back(static_cast<double>(cell.x),
                          static_cast<double>(cell.y));
    }
    return path;
}

}  // namespace

SpaceTimeAStarPlanner::SpaceTimeAStarPlanner(
    SpaceTimeAStarOptions options)
    : options_(options) {}

autoplanner::PlannerResult SpaceTimeAStarPlanner::plan(
    const autoplanner::GridMap& map,
    const autoplanner::Point2i& start,
    const autoplanner::Point2i& goal,
    const std::vector<MovingObstacle>& moving_obstacles,
    std::size_t start_frame) const {
    autoplanner::PlannerResult result;
    result.planner_name = "space_time_astar";
    const auto time_begin = std::chrono::steady_clock::now();

    if (map.isEmpty() || options_.max_time_steps == 0) {
        result.message = "Space-time planner is not initialized.";
        return result;
    }
    if (!map.isFree(start.x, start.y) ||
        !predictedFree(moving_obstacles, start, start_frame)) {
        result.message = "Start is invalid or occupied.";
        return result;
    }
    if (!map.isFree(goal.x, goal.y)) {
        result.message = "Goal is invalid or occupied.";
        return result;
    }

    const auto layer_size = static_cast<std::size_t>(
        map.width() * map.height());
    const auto state_count =
        layer_size * (options_.max_time_steps + 1);
    std::vector<double> g_score(
        state_count, std::numeric_limits<double>::infinity());
    std::vector<Parent> parents(state_count);
    std::vector<unsigned char> closed(state_count, 0);

    std::priority_queue<
        QueueNode,
        std::vector<QueueNode>,
        std::greater<QueueNode>> open;

    const auto start_index = stateIndex(map, start.x, start.y, 0);
    g_score[start_index] = 0.0;
    open.push({start.x, start.y, 0, heuristic(start, goal)});

    const std::vector<int> dx4 = {1, -1, 0, 0};
    const std::vector<int> dy4 = {0, 0, 1, -1};
    const std::vector<int> dx8 = {1, -1, 0, 0, 1, 1, -1, -1};
    const std::vector<int> dy8 = {0, 0, 1, -1, 1, -1, 1, -1};
    std::vector<int> dxs = options_.allow_diagonal ? dx8 : dx4;
    std::vector<int> dys = options_.allow_diagonal ? dy8 : dy4;
    if (options_.allow_wait) {
        dxs.push_back(0);
        dys.push_back(0);
    }

    QueueNode reached;
    while (!open.empty()) {
        const auto current = open.top();
        open.pop();
        const auto current_index = stateIndex(
            map, current.x, current.y, current.time);
        if (closed[current_index] != 0) continue;
        closed[current_index] = 1;
        ++result.expanded_nodes;

        if (current.x == goal.x && current.y == goal.y) {
            result.success = true;
            reached = current;
            break;
        }
        if (current.time >= options_.max_time_steps) continue;

        for (std::size_t index = 0; index < dxs.size(); ++index) {
            const int nx = current.x + dxs[index];
            const int ny = current.y + dys[index];
            const std::size_t next_time = current.time + 1;
            if (!map.isFree(nx, ny)) continue;
            if (!canMoveDiagonal(map, current.x, current.y,
                                 dxs[index], dys[index])) {
                continue;
            }
            const autoplanner::Point2i next_cell{nx, ny};
            if (!predictedFree(
                    moving_obstacles, next_cell, start_frame + next_time)) {
                continue;
            }

            const auto next_index = stateIndex(map, nx, ny, next_time);
            if (closed[next_index] != 0) continue;
            const double step_cost =
                dxs[index] == 0 && dys[index] == 0
                    ? 0.5
                    : ((dxs[index] != 0 && dys[index] != 0)
                           ? std::sqrt(2.0)
                           : 1.0);
            const double tentative_g = g_score[current_index] + step_cost;
            if (tentative_g < g_score[next_index]) {
                g_score[next_index] = tentative_g;
                parents[next_index] = {current.x, current.y, current.time};
                const double f = tentative_g + heuristic(next_cell, goal);
                open.push({nx, ny, next_time, f});
            }
        }
    }

    if (result.success) {
        result.path = reconstructPath(map, parents, start, reached);
        result.path_length = autoplanner::computePathLength(result.path);
        result.message = "Space-time path found.";
    } else {
        result.message = "No space-time path found within max time steps.";
    }

    const auto time_end = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(
            time_end - time_begin).count();
    result.iterations = result.expanded_nodes;
    return result;
}

}  // namespace robotnav
