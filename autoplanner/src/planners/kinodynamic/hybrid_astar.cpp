#include "autoplanner/planners/kinodynamic/hybrid_astar.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

#include "autoplanner/heuristics/euclidean.h"

namespace autoplanner {

namespace {

// 3-D state: (x, y, theta) → 1-D index.
struct State {
    double x, y, theta;
};
using StateIdx = std::tuple<int,int,int>;

struct Hash {
    size_t operator()(const StateIdx& s) const {
        return std::get<0>(s) * 73856093 ^
               std::get<1>(s) * 19349663 ^
               std::get<2>(s) * 83492791;
    }
};

struct QNode {
    State state;
    double f = 0.0;
    StateIdx parent;
    bool operator>(const QNode& o) const { return f > o.f; }
};

StateIdx discretize(const State& s, double res, int ab) {
    return {static_cast<int>(std::floor(s.x / res)),
            static_cast<int>(std::floor(s.y / res)),
            static_cast<int>(std::floor(s.theta * ab / (2.0 * M_PI))) % ab};
}

}  // namespace

HybridAStarPlanner::HybridAStarPlanner(double turning_radius,
                                        double step_size, int angle_bins)
    : turning_radius_(turning_radius), step_size_(step_size),
      angle_bins_(angle_bins)
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

    const double INF = std::numeric_limits<double>::infinity();
    const double res = step_size_;
    const int ab = angle_bins_;

    State start_s{static_cast<double>(start.x), static_cast<double>(start.y), 0.0};
    State goal_s{static_cast<double>(goal.x), static_cast<double>(goal.y), 0.0};

    // Action set: steer angles for bicycle model.
    std::vector<double> steer_angles;
    int n_steer = 5;
    for (int i = 0; i < n_steer; ++i)
        steer_angles.push_back(-M_PI_4 + (2.0 * M_PI_4 * i) / (n_steer - 1));
    steer_angles.push_back(0.0); // always include straight

    // Remove duplicates and sort
    std::sort(steer_angles.begin(), steer_angles.end());
    steer_angles.erase(std::unique(steer_angles.begin(), steer_angles.end()),
                       steer_angles.end());

    std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> open;
    std::unordered_map<StateIdx, double, Hash> g_score;
    std::unordered_map<StateIdx, StateIdx, Hash> parent;
    std::unordered_map<StateIdx, State, Hash> states;

    auto sid = discretize(start_s, res, ab);
    g_score[sid] = 0.0;
    states[sid] = start_s;
    open.push({start_s, heuristic_->compute(Point2i(start.x, start.y),
                                             Point2i(goal.x, goal.y)), sid});

    StateIdx goal_id;
    bool found = false;

    while (!open.empty()) {
        auto cur = open.top(); open.pop();
        auto cid = discretize(cur.state, res, ab);

        double gs = g_score.count(cid) ? g_score[cid] : INF;
        if (cur.f > gs + heuristic_->compute(
                Point2i(static_cast<int>(cur.state.x),
                        static_cast<int>(cur.state.y)),
                Point2i(goal.x, goal.y)) + 1e-6) continue;

        result.expanded_nodes++;

        // Check goal
        double dg = std::sqrt((cur.state.x - goal_s.x)*(cur.state.x - goal_s.x) +
                              (cur.state.y - goal_s.y)*(cur.state.y - goal_s.y));
        if (dg < step_size_ * 2.0) {
            goal_id = cid;
            found = true;
            break;
        }

        // Expand using bicycle model
        for (double phi : steer_angles) {
            double L = turning_radius_;
            double wb = L; // wheelbase ≈ turning_radius for simplicity
            double beta = std::atan(std::tan(phi) / 2.0);

            for (int dir : {1, -1}) {
                double ds = step_size_ * dir;
                double nx = cur.state.x + ds * std::cos(cur.state.theta + beta);
                double ny = cur.state.y + ds * std::sin(cur.state.theta + beta);
                double nt = cur.state.theta + ds * std::sin(beta) / wb;

                // Normalize angle
                while (nt > M_PI) nt -= 2.0 * M_PI;
                while (nt < -M_PI) nt += 2.0 * M_PI;

                int cx = static_cast<int>(std::floor(nx));
                int cy = static_cast<int>(std::floor(ny));
                if (!map.isFree(cx, cy)) continue;

                State ns{nx, ny, nt};
                auto nid = discretize(ns, res, ab);
                double new_g = gs + std::abs(ds);

                if (!g_score.count(nid) || new_g < g_score[nid]) {
                    g_score[nid] = new_g;
                    parent[nid] = cid;
                    states[nid] = ns;
                    double h = heuristic_->compute(
                        Point2i(cx, cy), Point2i(goal.x, goal.y));
                    open.push({ns, new_g + h, cid});
                }
            }
        }
    }

    if (found) {
        Path2d path;
        auto cid = goal_id;
        while (states.count(cid)) {
            auto& s = states[cid];
            path.push_back({s.x, s.y});
            if (!parent.count(cid)) break;
            cid = parent[cid];
        }
        std::reverse(path.begin(), path.end());
        result.success = true;
        result.path = std::move(path);
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
