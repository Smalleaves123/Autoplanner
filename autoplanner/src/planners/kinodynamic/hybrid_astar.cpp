#include "autoplanner/planners/kinodynamic/hybrid_astar.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

#include "autoplanner/heuristics/euclidean.h"

namespace autoplanner {

namespace {

// 4-D state: (x, y, theta, gear) → 1-D index. Keeping the incoming gear in
// the key prevents a lower-cost forward arrival from discarding the reverse
// maneuver that is needed to leave a tight space.
struct State {
    double x, y, theta;
};
using StateIdx = std::tuple<int, int, int, int>;

struct Hash {
    size_t operator()(const StateIdx& s) const {
        return std::get<0>(s) * 73856093 ^
               std::get<1>(s) * 19349663 ^
               std::get<2>(s) * 83492791 ^
               std::get<3>(s) * 2654435761U;
    }
};

struct QNode {
    State state;
    double f = 0.0;
    StateIdx parent;
    bool operator>(const QNode& o) const { return f > o.f; }
};

StateIdx discretize(const State& s, double res, int ab, int direction) {
    const double normalized = std::atan2(std::sin(s.theta),
                                         std::cos(s.theta));
    const double wrapped = normalized + M_PI;
    const int angle_bin = std::clamp(
        static_cast<int>(std::floor(wrapped * ab / (2.0 * M_PI))),
        0, ab - 1);
    return {static_cast<int>(std::floor(s.x / res)),
            static_cast<int>(std::floor(s.y / res)),
            angle_bin, direction};
}

State integrate(const State& state, double distance, double curvature) {
    State next = state;
    next.theta = state.theta + curvature * distance;
    if (std::abs(curvature) <= 1e-9) {
        next.x += distance * std::cos(state.theta);
        next.y += distance * std::sin(state.theta);
    } else {
        next.x += (std::sin(next.theta) - std::sin(state.theta)) /
                  curvature;
        next.y += (-std::cos(next.theta) + std::cos(state.theta)) /
                  curvature;
    }
    next.theta = std::atan2(std::sin(next.theta), std::cos(next.theta));
    return next;
}

}  // namespace

HybridAStarPlanner::HybridAStarPlanner(double turning_radius,
                                       double step_size, int angle_bins,
                                       bool allow_reverse,
                                       double reverse_penalty,
                                       double collision_check_resolution)
    : turning_radius_(turning_radius), step_size_(step_size),
      angle_bins_(angle_bins), allow_reverse_(allow_reverse),
      reverse_penalty_(reverse_penalty),
      collision_check_resolution_(collision_check_resolution)
    , heuristic_(std::make_unique<EuclideanHeuristic>()) {}

void HybridAStarPlanner::setHeuristic(std::unique_ptr<Heuristic> h) {
    heuristic_ = std::move(h);
}

std::string HybridAStarPlanner::name() const { return "hybrid_astar"; }

PlannerResult HybridAStarPlanner::plan(const GridMap& map,
                                        const Point2i& start,
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
    if (!std::isfinite(turning_radius_) || turning_radius_ <= 0.0 ||
        !std::isfinite(step_size_) || step_size_ <= 0.0 || angle_bins_ <= 0 ||
        !std::isfinite(reverse_penalty_) || reverse_penalty_ <= 0.0 ||
        !std::isfinite(collision_check_resolution_) ||
        collision_check_resolution_ <= 0.0) {
        result.message = "Invalid Hybrid A* kinematic options.";
        return result;
    }

    const double INF = std::numeric_limits<double>::infinity();
    const double res = step_size_;
    const int ab = angle_bins_;

    State start_s{static_cast<double>(start.x), static_cast<double>(start.y), 0.0};
    State goal_s{static_cast<double>(goal.x), static_cast<double>(goal.y), 0.0};

    // Normalized curvature controls. turning_radius_ is the actual minimum
    // radius, so every primitive is bounded by |kappa| <= 1/R.
    const std::vector<double> curvature_scales = {-1.0, -0.5, 0.0, 0.5, 1.0};

    std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> open;
    std::unordered_map<StateIdx, double, Hash> g_score;
    std::unordered_map<StateIdx, StateIdx, Hash> parent;
    std::unordered_map<StateIdx, State, Hash> states;
    std::unordered_map<StateIdx, int, Hash> motion_directions;

    auto sid = discretize(start_s, res, ab, 0);
    g_score[sid] = 0.0;
    states[sid] = start_s;
    motion_directions[sid] = 0;
    open.push({start_s, heuristic_->compute(Point2i(start.x, start.y),
                                             Point2i(goal.x, goal.y)), sid});

    StateIdx goal_id;
    bool found = false;

    while (!open.empty()) {
        auto cur = open.top(); open.pop();
        const auto cid = cur.parent;

        double gs = g_score.count(cid) ? g_score[cid] : INF;
        if (cur.f > gs + heuristic_->compute(
                Point2i(static_cast<int>(cur.state.x),
                        static_cast<int>(cur.state.y)),
                Point2i(goal.x, goal.y)) + 1e-6) continue;

        result.expanded_nodes++;

        // Check goal
        double dg = std::sqrt((cur.state.x - goal_s.x)*(cur.state.x - goal_s.x) +
                              (cur.state.y - goal_s.y)*(cur.state.y - goal_s.y));
        if (dg <= std::max(0.5, step_size_ * 0.5)) {
            goal_id = cid;
            found = true;
            break;
        }

        const std::vector<int> directions = allow_reverse_
            ? std::vector<int>{1, -1} : std::vector<int>{1};
        for (const double scale : curvature_scales) {
            const double curvature = scale / turning_radius_;
            for (const int direction : directions) {
                const double ds = step_size_ * direction;
                const int samples = std::max(
                    1, static_cast<int>(std::ceil(
                        std::abs(ds) / collision_check_resolution_)));
                bool collision_free = true;
                State ns = cur.state;
                for (int sample = 1; sample <= samples; ++sample) {
                    const double fraction = static_cast<double>(sample) /
                                            static_cast<double>(samples);
                    ns = integrate(cur.state, ds * fraction, curvature);
                    if (!map.isFree(static_cast<int>(std::floor(ns.x)),
                                    static_cast<int>(std::floor(ns.y)))) {
                        collision_free = false;
                        break;
                    }
                }
                if (!collision_free) continue;

                const int cx = static_cast<int>(std::floor(ns.x));
                const int cy = static_cast<int>(std::floor(ns.y));
                const auto nid = discretize(ns, res, ab, direction);
                const double new_g = gs + std::abs(ds) *
                    (direction < 0 ? reverse_penalty_ : 1.0);

                if (!g_score.count(nid) || new_g < g_score[nid]) {
                    g_score[nid] = new_g;
                    parent[nid] = cid;
                    states[nid] = ns;
                    motion_directions[nid] = direction;
                    double h = heuristic_->compute(
                        Point2i(cx, cy), Point2i(goal.x, goal.y));
                    open.push({ns, new_g + h, nid});
                }
            }
        }
    }

    if (found) {
        Path2d path;
        std::vector<int> directions;
        auto cid = goal_id;
        while (states.count(cid)) {
            auto& s = states[cid];
            path.push_back({s.x, s.y});
            directions.push_back(motion_directions[cid]);
            if (!parent.count(cid)) break;
            cid = parent[cid];
        }
        std::reverse(path.begin(), path.end());
        std::reverse(directions.begin(), directions.end());
        result.success = true;
        result.path = std::move(path);
        result.motion_directions = std::move(directions);
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
