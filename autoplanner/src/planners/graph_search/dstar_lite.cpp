#include "autoplanner/planners/graph_search/dstar_lite.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/heuristics/euclidean.h"

namespace autoplanner {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct Key {
    double first = 0.0;
    double second = 0.0;
};

bool keyLess(const Key& lhs, const Key& rhs) {
    if (lhs.first != rhs.first) return lhs.first < rhs.first;
    return lhs.second < rhs.second;
}

bool keyEqual(const Key& lhs, const Key& rhs) {
    return lhs.first == rhs.first && lhs.second == rhs.second;
}

struct QueueNode {
    int index = -1;
    Key key;

    bool operator>(const QueueNode& other) const {
        if (key.first != other.key.first) return key.first > other.key.first;
        return key.second > other.key.second;
    }
};

}  // namespace

struct DStarLitePlanner::SearchState {
    GridMap map;
    Point2i start;
    Point2i last_start;
    Point2i goal;
    int width = 0;
    int height = 0;
    double km = 0.0;
    std::vector<double> g;
    std::vector<double> rhs;
    std::set<std::tuple<double, double, int>> open;
    std::map<int, Key> queued_keys;
};

DStarLitePlanner::DStarLitePlanner(bool allow_diagonal)
    : allow_diagonal_(allow_diagonal)
    , heuristic_(std::make_unique<EuclideanHeuristic>())
    , state_(std::make_unique<SearchState>()) {}

DStarLitePlanner::~DStarLitePlanner() = default;

void DStarLitePlanner::setHeuristic(std::unique_ptr<Heuristic> h) {
    heuristic_ = std::move(h);
}

std::string DStarLitePlanner::name() const { return "dstar_lite"; }

namespace {

std::vector<std::pair<int, int>> directions(bool allow_diagonal) {
    std::vector<std::pair<int, int>> result = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };
    if (allow_diagonal) {
        result.insert(result.end(), {
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        });
    }
    return result;
}

bool diagonalAllowed(const GridMap& map, int x, int y, int dx, int dy) {
    return dx == 0 || dy == 0 ||
           (map.isFree(x + dx, y) && map.isFree(x, y + dy));
}

}  // namespace

PlannerResult DStarLitePlanner::plan(const GridMap& map,
                                     const Point2i& start,
                                     const Point2i& goal) {
    state_ = std::make_unique<SearchState>();
    state_->map = map;
    state_->start = start;
    state_->last_start = start;
    state_->goal = goal;
    state_->width = map.width();
    state_->height = map.height();
    const std::size_t count = static_cast<std::size_t>(
        std::max(0, state_->width * state_->height));
    state_->g.assign(count, kInfinity);
    state_->rhs.assign(count, kInfinity);
    state_->km = 0.0;

    if (map.isInside(goal.x, goal.y)) {
        const int goal_index = map.index(goal.x, goal.y);
        state_->rhs[static_cast<std::size_t>(goal_index)] = 0.0;
        const Key initial_key{heuristic_->compute(start, goal), 0.0};
        state_->open.insert({initial_key.first, initial_key.second, goal_index});
        state_->queued_keys[goal_index] = initial_key;
    }
    return replan(map, start);
}

PlannerResult DStarLitePlanner::replan(const GridMap& map,
                                       const Point2i& start) {
    PlannerResult result;
    result.planner_name = name();
    const auto time_begin = std::chrono::steady_clock::now();

    if (!state_ || state_->width == 0) {
        result.message = "Planner is not initialized; call plan first.";
        return result;
    }
    if (state_->width != map.width() || state_->height != map.height()) {
        return plan(map, start, state_->goal);
    }
    if (!map.isFree(start.x, start.y)) {
        result.message = "Start is occupied.";
        return result;
    }
    if (!map.isFree(state_->goal.x, state_->goal.y)) {
        result.message = "Goal is occupied.";
        return result;
    }

    const int width = state_->width;
    const auto dirs = directions(allow_diagonal_);
    auto heuristic = [&](int lhs, int rhs) {
        const Point2i a(lhs % width, lhs / width);
        const Point2i b(rhs % width, rhs / width);
        return heuristic_->compute(a, b);
    };
    auto validNeighbor = [&](int index, int neighbor) {
        const int x = index % width;
        const int y = index / width;
        const int nx = neighbor % width;
        const int ny = neighbor / width;
        const int dx = nx - x;
        const int dy = ny - y;
        return state_->map.isFree(x, y) && state_->map.isFree(nx, ny) &&
               diagonalAllowed(state_->map, x, y, dx, dy);
    };
    auto neighbors = [&](int index) {
        std::vector<int> result_indices;
        const int x = index % width;
        const int y = index / width;
        for (const auto& direction : dirs) {
            const int nx = x + direction.first;
            const int ny = y + direction.second;
            if (state_->map.isInside(nx, ny)) {
                result_indices.push_back(state_->map.index(nx, ny));
            }
        }
        return result_indices;
    };
    auto edgeCost = [&](int from, int to) {
        if (!validNeighbor(from, to)) return kInfinity;
        const int fx = from % width;
        const int fy = from / width;
        const int tx = to % width;
        const int ty = to / width;
        return (fx != tx && fy != ty) ? std::sqrt(2.0) : 1.0;
    };
    auto calculateKey = [&](int index) {
        const double best = std::min(
            state_->g[static_cast<std::size_t>(index)],
            state_->rhs[static_cast<std::size_t>(index)]);
        const int start_index = state_->map.index(start.x, start.y);
        return Key{best + heuristic(start_index, index) + state_->km, best};
    };
    auto updateVertex = [&](int index) {
        if (index < 0) return;

        const auto old_key = state_->queued_keys.find(index);
        if (old_key != state_->queued_keys.end()) {
            state_->open.erase({old_key->second.first,
                                old_key->second.second, index});
            state_->queued_keys.erase(old_key);
        }

        if (index != state_->map.index(state_->goal.x, state_->goal.y)) {
            double best_rhs = kInfinity;
            for (const int successor : neighbors(index)) {
                best_rhs = std::min(
                    best_rhs,
                    edgeCost(index, successor) * 1.0 +
                        state_->g[static_cast<std::size_t>(successor)]);
            }
            state_->rhs[static_cast<std::size_t>(index)] = best_rhs;
        }
        if (state_->g[static_cast<std::size_t>(index)] !=
            state_->rhs[static_cast<std::size_t>(index)]) {
            const Key key = calculateKey(index);
            state_->open.insert({key.first, key.second, index});
            state_->queued_keys[index] = key;
        }
    };

    // Detect changed occupancy cells and update their incident vertices.
    if (state_->map.width() == map.width() && state_->map.height() == map.height()) {
        std::vector<int> changed;
        for (int y = 0; y < map.height(); ++y) {
            for (int x = 0; x < map.width(); ++x) {
                if (state_->map.isOccupied(x, y) != map.isOccupied(x, y)) {
                    changed.push_back(map.index(x, y));
                }
            }
        }
        state_->km += heuristic(
            state_->map.index(state_->last_start.x, state_->last_start.y),
            map.index(start.x, start.y));
        state_->last_start = start;
        state_->map = map;
        for (const int changed_index : changed) {
            updateVertex(changed_index);
            for (const int neighbor : neighbors(changed_index)) updateVertex(neighbor);
        }
    }
    state_->start = start;

    const int start_index = map.index(start.x, start.y);
    const int goal_index = map.index(state_->goal.x, state_->goal.y);
    if (start_index < 0 || goal_index < 0) {
        result.message = "Start or goal is outside the map.";
        return result;
    }

    const int max_iterations = std::max(1, width * state_->height * 100);
    int search_iterations = 0;
    while (!state_->open.empty() && search_iterations < max_iterations) {
        ++search_iterations;
        const auto top_entry = *state_->open.begin();
        const QueueNode top{
            std::get<2>(top_entry),
            {std::get<0>(top_entry), std::get<1>(top_entry)}};
        const Key start_key = calculateKey(start_index);
        if (!keyLess(top.key, start_key) &&
            state_->rhs[static_cast<std::size_t>(start_index)] ==
                state_->g[static_cast<std::size_t>(start_index)]) {
            break;
        }
        state_->open.erase(state_->open.begin());
        state_->queued_keys.erase(top.index);
        const Key new_key = calculateKey(top.index);
        if (keyLess(top.key, new_key)) {
            state_->open.insert({new_key.first, new_key.second, top.index});
            state_->queued_keys[top.index] = new_key;
            continue;
        }
        // A node can have multiple queued keys because removal is lazy. An
        // entry that no longer matches the current key must not toggle g/rhs
        // back and forth after the node has already been repaired.
        if (!keyEqual(top.key, new_key)) continue;

        ++result.expanded_nodes;
        if (state_->g[static_cast<std::size_t>(top.index)] >
            state_->rhs[static_cast<std::size_t>(top.index)]) {
            state_->g[static_cast<std::size_t>(top.index)] =
                state_->rhs[static_cast<std::size_t>(top.index)];
            for (const int predecessor : neighbors(top.index)) {
                updateVertex(predecessor);
            }
        } else {
            state_->g[static_cast<std::size_t>(top.index)] = kInfinity;
            updateVertex(top.index);
            for (const int predecessor : neighbors(top.index)) {
                updateVertex(predecessor);
            }
        }
    }

    if (search_iterations >= max_iterations) {
        result.message = "D* Lite search iteration limit reached.";
    }

    const double start_cost = state_->g[static_cast<std::size_t>(start_index)];
    if (std::isfinite(start_cost)) {
        Path2d path;
        int current = start_index;
        std::vector<unsigned char> visited(static_cast<std::size_t>(
            width * state_->height), 0);
        while (current != goal_index) {
            if (current < 0 || visited[static_cast<std::size_t>(current)] != 0) {
                path.clear();
                break;
            }
            visited[static_cast<std::size_t>(current)] = 1;
            path.emplace_back(static_cast<double>(current % width),
                              static_cast<double>(current / width));

            int best_next = -1;
            double best_value = kInfinity;
            for (const int successor : neighbors(current)) {
                const double cost = edgeCost(current, successor);
                const double candidate =
                    cost + state_->g[static_cast<std::size_t>(successor)];
                if (candidate < best_value) {
                    best_value = candidate;
                    best_next = successor;
                }
            }
            if (best_next < 0 || !std::isfinite(best_value)) {
                path.clear();
                break;
            }
            current = best_next;
        }
        if (!path.empty() || current == goal_index) {
            path.emplace_back(static_cast<double>(state_->goal.x),
                              static_cast<double>(state_->goal.y));
            result.success = true;
            result.path = std::move(path);
            result.path_length = computePathLength(result.path);
            result.message = "Path found.";
        }
    }

    if (!result.success) result.message = "No path found.";
    result.iterations = result.expanded_nodes;
    const auto time_end = std::chrono::steady_clock::now();
    result.planning_time_ms = std::chrono::duration<double, std::milli>(
        time_end - time_begin).count();
    return result;
}

}  // namespace autoplanner
