#include "robotnav/cooperative_astar.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace robotnav {
namespace {

struct State {
    int x = 0;
    int y = 0;
    int t = 0;
    bool operator==(const State& other) const {
        return x == other.x && y == other.y && t == other.t;
    }
};

struct StateHash {
    std::size_t operator()(const State& state) const noexcept {
        std::size_t hash = static_cast<std::size_t>(state.x + 4096);
        hash = hash * 10007u + static_cast<std::size_t>(state.y + 4096);
        return hash * 10007u + static_cast<std::size_t>(state.t);
    }
};

struct Edge {
    int from_x = 0;
    int from_y = 0;
    int to_x = 0;
    int to_y = 0;
    int t = 0;
    bool operator==(const Edge& other) const {
        return from_x == other.from_x && from_y == other.from_y &&
               to_x == other.to_x && to_y == other.to_y && t == other.t;
    }
};

struct EdgeHash {
    std::size_t operator()(const Edge& edge) const noexcept {
        std::size_t hash = static_cast<std::size_t>(edge.from_x + 4096);
        hash = hash * 10007u + static_cast<std::size_t>(edge.from_y + 4096);
        hash = hash * 10007u + static_cast<std::size_t>(edge.to_x + 4096);
        hash = hash * 10007u + static_cast<std::size_t>(edge.to_y + 4096);
        return hash * 10007u + static_cast<std::size_t>(edge.t);
    }
};

struct QueueNode {
    State state;
    double cost = 0.0;
    double estimate = 0.0;
    bool operator<(const QueueNode& other) const {
        return estimate > other.estimate;
    }
};

struct ConstraintTable {
    std::unordered_set<State, StateHash> vertices;
    std::unordered_set<Edge, EdgeHash> edges;

    bool vertexReserved(int x, int y, int t) const {
        return vertices.count({x, y, t}) != 0;
    }

    bool edgeReserved(int from_x, int from_y, int to_x, int to_y,
                      int t) const {
        return edges.count({from_x, from_y, to_x, to_y, t}) != 0;
    }
};

struct ReservationTable {
    std::unordered_set<State, StateHash> vertices;
    std::unordered_set<Edge, EdgeHash> edges;
    int max_time = 0;

    bool vertexReserved(int x, int y, int t) const {
        return vertices.count({x, y, t}) != 0;
    }

    bool edgeReserved(int from_x, int from_y, int to_x, int to_y,
                      int t) const {
        return edges.count({from_x, from_y, to_x, to_y, t}) != 0;
    }

    void reserve(const std::vector<State>& path) {
        if (path.empty()) return;
        for (std::size_t index = 0; index < path.size(); ++index) {
            const auto& state = path[index];
            vertices.insert(state);
            if (index > 0) {
                const auto& previous = path[index - 1];
                edges.insert({previous.x, previous.y, state.x, state.y,
                              state.t});
            }
        }
        const auto& goal = path.back();
        for (int t = goal.t + 1; t <= max_time; ++t) {
            vertices.insert({goal.x, goal.y, t});
        }
    }
};

std::vector<autoplanner::Point2i> neighbors(
    const autoplanner::Point2i& point, bool diagonal, bool wait) {
    std::vector<autoplanner::Point2i> result;
    const int limit = diagonal ? 8 : 4;
    static constexpr int directions[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    for (int index = 0; index < limit; ++index) {
        result.emplace_back(point.x + directions[index][0],
                            point.y + directions[index][1]);
    }
    if (wait) result.push_back(point);
    return result;
}

double heuristic(const State& state, const autoplanner::Point2i& goal) {
    return std::hypot(static_cast<double>(state.x - goal.x),
                      static_cast<double>(state.y - goal.y));
}

bool samePoint(const State& state, const autoplanner::Point2i& point) {
    return state.x == point.x && state.y == point.y;
}

std::vector<State> lowLevelSearch(
    const autoplanner::GridMap& map,
    const autoplanner::Point2i& start,
    const autoplanner::Point2i& goal,
    const CooperativeAStarOptions& options,
    const ConstraintTable& constraints) {
    const State initial{start.x, start.y, 0};
    if (constraints.vertexReserved(initial.x, initial.y, initial.t)) {
        return {};
    }
    std::priority_queue<QueueNode> open;
    std::unordered_map<State, double, StateHash> costs;
    std::unordered_map<State, State, StateHash> parents;
    open.push({initial, 0.0, heuristic(initial, goal)});
    costs[initial] = 0.0;
    State final_state{};
    bool found = false;
    while (!open.empty()) {
        const auto current = open.top();
        open.pop();
        if (samePoint(current.state, goal)) {
            final_state = current.state;
            found = true;
            break;
        }
        if (current.state.t >= static_cast<int>(options.max_time_steps)) {
            continue;
        }
        for (const auto& next_point : neighbors(
                 {current.state.x, current.state.y}, options.allow_diagonal,
                 options.allow_wait)) {
            const State next{next_point.x, next_point.y,
                             current.state.t + 1};
            if (!map.isFree(next.x, next.y) ||
                constraints.vertexReserved(next.x, next.y, next.t) ||
                constraints.edgeReserved(current.state.x, current.state.y,
                                         next.x, next.y, next.t)) {
                continue;
            }
            const double next_cost = current.cost +
                ((next.x == current.state.x && next.y == current.state.y)
                     ? 1.25
                     : 1.0);
            const auto found_cost = costs.find(next);
            if (found_cost != costs.end() && found_cost->second <= next_cost) {
                continue;
            }
            costs[next] = next_cost;
            parents[next] = current.state;
            open.push({next, next_cost,
                       next_cost + heuristic(next, goal)});
        }
    }
    if (!found) {
        return {};
    }
    std::vector<State> path;
    for (State cursor = final_state;; cursor = parents.at(cursor)) {
        path.push_back(cursor);
        if (cursor == initial) break;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

struct Conflict {
    std::size_t first = 0;
    std::size_t second = 0;
    int time = 0;
    bool edge = false;
    State first_from{};
    State first_to{};
    State second_from{};
    State second_to{};
};

bool activeAt(const std::vector<State>& path, int time) {
    return time >= 0 && time < static_cast<int>(path.size());
}

bool isSharedGoalConflict(
    const Conflict& conflict,
    const std::vector<std::vector<State>>& paths,
    const std::vector<autoplanner::Point2i>& goals,
    bool allow_shared_goals) {
    if (!allow_shared_goals || conflict.edge) return false;
    const auto& first = paths[conflict.first][conflict.time];
    const auto& second = paths[conflict.second][conflict.time];
    return samePoint(first, goals[conflict.first]) &&
           samePoint(second, goals[conflict.second]) &&
           goals[conflict.first].x == goals[conflict.second].x &&
           goals[conflict.first].y == goals[conflict.second].y;
}

bool findConflict(
    const std::vector<std::vector<State>>& paths,
    const std::vector<autoplanner::Point2i>& goals,
    bool allow_shared_goals,
    Conflict* conflict) {
    for (std::size_t first = 0; first < paths.size(); ++first) {
        for (std::size_t second = first + 1; second < paths.size(); ++second) {
            const int horizon = static_cast<int>(std::max(
                paths[first].size(), paths[second].size()));
            for (int time = 0; time < horizon; ++time) {
                if (activeAt(paths[first], time) &&
                    activeAt(paths[second], time) &&
                    paths[first][time].x == paths[second][time].x &&
                    paths[first][time].y == paths[second][time].y) {
                    Conflict candidate;
                    candidate.first = first;
                    candidate.second = second;
                    candidate.time = time;
                    candidate.first_to = paths[first][time];
                    candidate.second_to = paths[second][time];
                    if (!isSharedGoalConflict(candidate, paths, goals,
                                              allow_shared_goals)) {
                        *conflict = candidate;
                        return true;
                    }
                }
                if (time == 0 || !activeAt(paths[first], time - 1) ||
                    !activeAt(paths[second], time - 1) ||
                    !activeAt(paths[first], time) ||
                    !activeAt(paths[second], time)) {
                    continue;
                }
                const auto& first_from = paths[first][time - 1];
                const auto& second_from = paths[second][time - 1];
                const auto& first_to = paths[first][time];
                const auto& second_to = paths[second][time];
                if (first_from.x == second_to.x &&
                    first_from.y == second_to.y &&
                    second_from.x == first_to.x &&
                    second_from.y == first_to.y &&
                    (first_from.x != first_to.x || first_from.y != first_to.y)) {
                    conflict->first = first;
                    conflict->second = second;
                    conflict->time = time;
                    conflict->edge = true;
                    conflict->first_from = first_from;
                    conflict->first_to = first_to;
                    conflict->second_from = second_from;
                    conflict->second_to = second_to;
                    return true;
                }
            }
        }
    }
    return false;
}

struct CBSNode {
    std::vector<std::vector<State>> paths;
    std::vector<ConstraintTable> constraints;
    double cost = 0.0;
    std::size_t serial = 0;
};

struct CBSNodeOrder {
    bool operator()(const CBSNode& first, const CBSNode& second) const {
        if (first.cost != second.cost) return first.cost > second.cost;
        return first.serial > second.serial;
    }
};

autoplanner::PlannerResult plannerResultFromPath(
    const std::vector<State>& states) {
    autoplanner::PlannerResult result;
    result.planner_name = "cooperative_astar_cbs";
    for (const auto& state : states) {
        result.path.emplace_back(static_cast<double>(state.x),
                                 static_cast<double>(state.y));
    }
    result.path_length = autoplanner::computePathLength(result.path);
    result.success = !states.empty();
    result.message = result.success ? "CBS path planned" : "CBS path failed";
    return result;
}

MultiAgentPlanResult planCBS(
    const autoplanner::GridMap& map,
    const std::vector<autoplanner::Point2i>& starts,
    const std::vector<autoplanner::Point2i>& goals,
    const CooperativeAStarOptions& options) {
    MultiAgentPlanResult result;
    std::vector<ConstraintTable> root_constraints(starts.size());
    CBSNode root;
    root.constraints = root_constraints;
    root.paths.reserve(starts.size());
    for (std::size_t agent = 0; agent < starts.size(); ++agent) {
        if (!map.isFree(starts[agent].x, starts[agent].y) ||
            !map.isFree(goals[agent].x, goals[agent].y)) {
            result.message = "invalid start or goal for agent " +
                             std::to_string(agent);
            return result;
        }
        auto path = lowLevelSearch(map, starts[agent], goals[agent], options,
                                   root.constraints[agent]);
        if (path.empty()) {
            result.message = "agent " + std::to_string(agent) + " has no path";
            return result;
        }
        root.paths.push_back(std::move(path));
    }
    for (const auto& path : root.paths) root.cost += path.size();

    std::priority_queue<CBSNode, std::vector<CBSNode>, CBSNodeOrder> open;
    open.push(std::move(root));
    std::size_t serial = 1;
    std::size_t expanded = 0;
    while (!open.empty() && expanded++ < options.max_cbs_nodes) {
        CBSNode node = open.top();
        open.pop();
        Conflict conflict;
        if (!findConflict(node.paths, goals, options.allow_shared_goals,
                          &conflict)) {
            result.success = true;
            result.planned_agents = starts.size();
            result.message = "all agents planned with CBS";
            result.plans.reserve(node.paths.size());
            for (const auto& path : node.paths) {
                result.plans.push_back(plannerResultFromPath(path));
            }
            return result;
        }
        ++result.vertex_conflicts;
        if (conflict.edge) ++result.edge_conflicts;
        for (const std::size_t agent : {conflict.first, conflict.second}) {
            CBSNode child = node;
            ConstraintTable& table = child.constraints[agent];
            if (conflict.edge) {
                const auto& from = agent == conflict.first
                    ? conflict.first_from : conflict.second_from;
                const auto& to = agent == conflict.first
                    ? conflict.first_to : conflict.second_to;
                table.edges.insert({from.x, from.y, to.x, to.y,
                                    conflict.time});
            } else {
                const auto& state = agent == conflict.first
                    ? conflict.first_to : conflict.second_to;
                table.vertices.insert({state.x, state.y, conflict.time});
            }
            auto replacement = lowLevelSearch(
                map, starts[agent], goals[agent], options, table);
            if (replacement.empty()) continue;
            child.paths[agent] = std::move(replacement);
            child.cost = 0.0;
            for (const auto& path : child.paths) child.cost += path.size();
            child.serial = serial++;
            open.push(std::move(child));
        }
    }
    result.message = "CBS search exhausted conflict branches";
    return result;
}

}  // namespace

CooperativeAStarPlanner::CooperativeAStarPlanner(
    CooperativeAStarOptions options) : options_(options) {}

MultiAgentPlanResult CooperativeAStarPlanner::plan(
    const autoplanner::GridMap& map,
    const std::vector<autoplanner::Point2i>& starts,
    const std::vector<autoplanner::Point2i>& goals) const {
    MultiAgentPlanResult result;
    if (starts.empty() || starts.size() != goals.size() ||
        options_.max_time_steps == 0) {
        result.message = "starts, goals, and time horizon are invalid";
        return result;
    }
    if (options_.use_cbs) {
        return planCBS(map, starts, goals, options_);
    }
    ReservationTable reservations;
    reservations.max_time = static_cast<int>(options_.max_time_steps);
    result.plans.resize(starts.size());
    std::vector<std::size_t> order = options_.priority_order;
    if (order.empty()) {
        order.resize(starts.size());
        std::iota(order.begin(), order.end(), 0);
    }
    std::vector<bool> seen(starts.size(), false);
    if (order.size() != starts.size()) {
        result.message = "priority order must contain every agent exactly once";
        return result;
    }
    for (const auto agent : order) {
        if (agent >= starts.size() || seen[agent]) {
            result.message = "priority order must contain every agent exactly once";
            return result;
        }
        seen[agent] = true;
    }

    for (const std::size_t agent : order) {
        const auto start = starts[agent];
        const auto goal = goals[agent];
        autoplanner::PlannerResult single;
        single.planner_name = "cooperative_astar";
        if (!map.isFree(start.x, start.y) || !map.isFree(goal.x, goal.y)) {
            single.message = "Start or Goal is outside the free map";
            result.plans[agent] = std::move(single);
            result.message = "invalid start or goal for agent " +
                             std::to_string(agent);
            return result;
        }

        const State initial{start.x, start.y, 0};
        std::priority_queue<QueueNode> open;
        std::unordered_map<State, double, StateHash> costs;
        std::unordered_map<State, State, StateHash> parents;
        open.push({initial, 0.0, heuristic(initial, goal)});
        costs[initial] = 0.0;
        State final_state{};
        bool found = false;
        while (!open.empty()) {
            const auto current = open.top();
            open.pop();
            if (current.state.x == goal.x && current.state.y == goal.y) {
                final_state = current.state;
                found = true;
                break;
            }
            if (current.state.t >= static_cast<int>(options_.max_time_steps)) {
                continue;
            }
            for (const auto& next_point : neighbors(
                     {current.state.x, current.state.y},
                     options_.allow_diagonal, options_.allow_wait)) {
                const State next{next_point.x, next_point.y,
                                 current.state.t + 1};
                if (!map.isFree(next.x, next.y)) {
                    continue;
                }
                if (reservations.vertexReserved(next.x, next.y, next.t)) {
                    ++result.vertex_conflicts;
                    continue;
                }
                if (reservations.edgeReserved(next.x, next.y,
                                              current.state.x, current.state.y,
                                              next.t)) {
                    ++result.edge_conflicts;
                    continue;
                }
                const double next_cost = current.cost +
                    ((next.x == current.state.x &&
                      next.y == current.state.y) ? 1.25 : 1.0);
                const auto found_cost = costs.find(next);
                if (found_cost != costs.end() && found_cost->second <= next_cost) {
                    continue;
                }
                costs[next] = next_cost;
                parents[next] = current.state;
                open.push({next, next_cost,
                           next_cost + heuristic(next, goal)});
            }
        }
        if (!found) {
            single.message = "no collision-free cooperative path within time horizon";
            result.plans[agent] = std::move(single);
            result.message = "agent " + std::to_string(agent) + " has no path";
            return result;
        }

        std::vector<State> states;
        for (State cursor = final_state;; cursor = parents.at(cursor)) {
            states.push_back(cursor);
            if (cursor == initial) break;
        }
        std::reverse(states.begin(), states.end());
        for (const auto& state : states) {
            single.path.emplace_back(static_cast<double>(state.x),
                                     static_cast<double>(state.y));
        }
        single.path_length = autoplanner::computePathLength(single.path);
        single.success = true;
        single.message = "cooperative path planned";
        reservations.reserve(states);
        result.plans[agent] = std::move(single);
        ++result.planned_agents;
    }
    result.success = result.planned_agents == starts.size();
    result.message = result.success ? "all agents planned" : "planning failed";
    return result;
}

}  // namespace robotnav
