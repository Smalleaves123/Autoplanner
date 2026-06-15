#include "autoplanner/planners/sampling/rrt.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace autoplanner {

RRTPlanner::RRTPlanner(double step_size,
                       int max_iter,
                       double goal_sample_rate,
                       double goal_tolerance)
    : step_size_(step_size)
    , max_iter_(max_iter)
    , goal_sample_rate_(goal_sample_rate)
    , goal_tolerance_(goal_tolerance)
    , rng_(std::random_device{}()) {}

std::string RRTPlanner::name() const {
    return "rrt";
}

const std::vector<std::pair<Point2d, Point2d>>& RRTPlanner::treeEdges() const {
    return tree_edges_;
}

PlannerResult RRTPlanner::plan(
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
    nodes.push_back(start_d);
    parent.push_back(-1);
    tree_edges_.clear();

    std::uniform_real_distribution<double> dist01(0.0, 1.0);
    std::uniform_real_distribution<double> dist_x(0.0, static_cast<double>(map.width() - 1));
    std::uniform_real_distribution<double> dist_y(0.0, static_cast<double>(map.height() - 1));

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

        // Check intermediate points along the segment.
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

        int new_idx = static_cast<int>(nodes.size());
        nodes.push_back(q_new);
        parent.push_back(nearest_idx);
        tree_edges_.emplace_back(q_near, q_new);

        if (distance(q_new, goal_d) < goal_tolerance_) {
            nodes.push_back(goal_d);
            parent.push_back(new_idx);
            tree_edges_.emplace_back(q_new, goal_d);

            Path2d path;
            int current = static_cast<int>(nodes.size()) - 1;
            while (current != -1) {
                path.push_back(nodes[static_cast<std::size_t>(current)]);
                current = parent[static_cast<std::size_t>(current)];
            }
            std::reverse(path.begin(), path.end());

            result.success = true;
            result.path = std::move(path);
            result.path_length = computePathLength(result.path);
            result.expanded_nodes = static_cast<int>(nodes.size());
            result.iterations = iter + 1;
            result.message = "Path found.";
            break;
        }
    }

    if (!result.success) {
        result.expanded_nodes = static_cast<int>(nodes.size());
        result.iterations = max_iter_;
        result.message = "Failed to find path within max iterations.";
    }

    const auto time_end = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(time_end - time_begin).count();
    return result;
}

}  // namespace autoplanner
