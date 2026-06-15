#include "autoplanner/planners/sampling/rrt_star.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace autoplanner {

RRTStarPlanner::RRTStarPlanner(double step_size,
                               int max_iter,
                               double goal_sample_rate,
                               double goal_tolerance,
                               double rewire_radius)
    : step_size_(step_size)
    , max_iter_(max_iter)
    , goal_sample_rate_(goal_sample_rate)
    , goal_tolerance_(goal_tolerance)
    , rewire_radius_(rewire_radius)
    , rng_(std::random_device{}()) {}

std::string RRTStarPlanner::name() const {
    return "rrt_star";
}

const std::vector<std::pair<Point2d, Point2d>>& RRTStarPlanner::treeEdges() const {
    return tree_edges_;
}

PlannerResult RRTStarPlanner::plan(
    const GridMap& map,
    const Point2i& start,
    const Point2i& goal
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

    Point2d start_d(static_cast<double>(start.x), static_cast<double>(start.y));
    Point2d goal_d(static_cast<double>(goal.x), static_cast<double>(goal.y));

    std::vector<Point2d> nodes;
    std::vector<int> parent;
    std::vector<double> cost;
    nodes.push_back(start_d);
    parent.push_back(-1);
    cost.push_back(0.0);
    tree_edges_.clear();

    std::uniform_real_distribution<double> dist01(0.0, 1.0);
    std::uniform_real_distribution<double> dist_x(0.0, static_cast<double>(map.width() - 1));
    std::uniform_real_distribution<double> dist_y(0.0, static_cast<double>(map.height() - 1));

    int best_goal_parent = -1;
    double best_goal_cost = std::numeric_limits<double>::max();

    for (int iter = 0; iter < max_iter_; ++iter) {
        Point2d q_rand;
        if (dist01(rng_) < goal_sample_rate_) {
            q_rand = goal_d;
        } else {
            q_rand.x = dist_x(rng_);
            q_rand.y = dist_y(rng_);
        }

        int nearest_idx = 0;
        double nearest_dist = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            double d = distance(q_rand, nodes[i]);
            if (d < nearest_dist) {
                nearest_dist = d;
                nearest_idx = static_cast<int>(i);
            }
        }

        Point2d q_near = nodes[static_cast<std::size_t>(nearest_idx)];
        double dir_x = q_rand.x - q_near.x;
        double dir_y = q_rand.y - q_near.y;
        double dir_len = std::sqrt(dir_x * dir_x + dir_y * dir_y);
        if (dir_len < 1e-9) continue;

        double extend = std::min(step_size_, dir_len);
        Point2d q_new;
        q_new.x = q_near.x + (dir_x / dir_len) * extend;
        q_new.y = q_near.y + (dir_y / dir_len) * extend;

        if (!map.isFree(static_cast<int>(std::floor(q_new.x)),
                        static_cast<int>(std::floor(q_new.y)))) continue;

        // Check segment from nearest to new.
        {
            double seg_len = distance(q_near, q_new);
            int samples = std::max(2, static_cast<int>(seg_len / 0.5) + 1);
            bool collision = false;
            for (int s = 1; s <= samples; ++s) {
                double t = static_cast<double>(s) / static_cast<double>(samples);
                double cx = q_near.x + t * (q_new.x - q_near.x);
                double cy = q_near.y + t * (q_new.y - q_near.y);
                if (!map.isFree(static_cast<int>(std::floor(cx)),
                                static_cast<int>(std::floor(cy)))) {
                    collision = true;
                    break;
                }
            }
            if (collision) continue;
        }

        // Choose parent from nearby nodes.
        int best_parent = nearest_idx;
        double best_cost = cost[static_cast<std::size_t>(nearest_idx)] +
                           distance(q_new, q_near);

        for (std::size_t i = 0; i < nodes.size(); ++i) {
            if (static_cast<int>(i) == nearest_idx) continue;
            double d = distance(q_new, nodes[i]);
            if (d > rewire_radius_) continue;

            {
                int samples = std::max(2, static_cast<int>(d / 0.5) + 1);
                bool collision = false;
                for (int s = 1; s <= samples; ++s) {
                    double t = static_cast<double>(s) / static_cast<double>(samples);
                    double cx = nodes[i].x + t * (q_new.x - nodes[i].x);
                    double cy = nodes[i].y + t * (q_new.y - nodes[i].y);
                    if (!map.isFree(static_cast<int>(std::floor(cx)),
                                    static_cast<int>(std::floor(cy)))) {
                        collision = true;
                        break;
                    }
                }
                if (collision) continue;
            }

            double new_cost = cost[i] + d;
            if (new_cost < best_cost) {
                best_cost = new_cost;
                best_parent = static_cast<int>(i);
            }
        }

        int new_idx = static_cast<int>(nodes.size());
        nodes.push_back(q_new);
        parent.push_back(best_parent);
        cost.push_back(best_cost);
        tree_edges_.emplace_back(nodes[static_cast<std::size_t>(best_parent)], q_new);

        // Rewire nearby nodes.
        for (std::size_t i = 0; i < nodes.size() - 1; ++i) {
            double d = distance(q_new, nodes[i]);
            if (d > rewire_radius_) continue;
            if (static_cast<int>(i) == best_parent) continue;

            double new_cost = best_cost + d;
            if (new_cost < cost[i]) {
                int samples = std::max(2, static_cast<int>(d / 0.5) + 1);
                bool collision = false;
                for (int s = 1; s <= samples; ++s) {
                    double t = static_cast<double>(s) / static_cast<double>(samples);
                    double cx = q_new.x + t * (nodes[i].x - q_new.x);
                    double cy = q_new.y + t * (nodes[i].y - q_new.y);
                    if (!map.isFree(static_cast<int>(std::floor(cx)),
                                    static_cast<int>(std::floor(cy)))) {
                        collision = true;
                        break;
                    }
                }
                if (!collision) {
                    parent[i] = new_idx;
                    cost[i] = new_cost;
                }
            }
        }

        double d_to_goal = distance(q_new, goal_d);
        if (d_to_goal < goal_tolerance_) {
            double goal_cost = best_cost + d_to_goal;
            if (goal_cost < best_goal_cost) {
                best_goal_cost = goal_cost;
                best_goal_parent = new_idx;
            }
        }
    }

    if (best_goal_parent >= 0) {
        Path2d path;
        path.push_back(goal_d);
        int current = best_goal_parent;
        while (current != -1) {
            path.push_back(nodes[static_cast<std::size_t>(current)]);
            current = parent[static_cast<std::size_t>(current)];
        }
        std::reverse(path.begin(), path.end());

        result.success = true;
        result.path = std::move(path);
        result.path_length = computePathLength(result.path);
        result.message = "Path found.";
    } else {
        result.message = "Failed to find path within max iterations.";
    }

    result.expanded_nodes = static_cast<int>(nodes.size());
    result.iterations = max_iter_;
    const auto time_end = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(time_end - time_begin).count();
    return result;
}

}  // namespace autoplanner
