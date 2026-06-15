#include "autoplanner/planners/sampling/informed_rrt_star.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace autoplanner {

InformedRRTStarPlanner::InformedRRTStarPlanner(double step_size, int max_iter,
                                               double goal_sample_rate,
                                               double goal_tolerance,
                                               double rewire_radius)
    : step_size_(step_size), max_iter_(max_iter),
      goal_sample_rate_(goal_sample_rate), goal_tolerance_(goal_tolerance),
      rewire_radius_(rewire_radius), rng_(std::random_device{}()) {}

std::string InformedRRTStarPlanner::name() const { return "informed_rrt_star"; }

const std::vector<std::pair<Point2d, Point2d>>&
InformedRRTStarPlanner::treeEdges() const { return tree_edges_; }

// Sample uniformly within an ellipse with foci at (cx1,cy1) and (cx2,cy2),
// and transverse diameter cBest.
static Point2d sampleEllipse(double cx1, double cy1, double cx2, double cy2,
                              double cBest, double w, double h,
                              std::mt19937& rng) {
    double cMin = std::sqrt((cx2-cx1)*(cx2-cx1) + (cy2-cy1)*(cy2-cy1));
    if (cBest <= cMin + 1e-6) cBest = cMin + 1e-6;
    double a = cBest / 2.0;
    double b = std::sqrt(a*a - (cMin/2.0)*(cMin/2.0));
    std::uniform_real_distribution<double> du(0, 1);

    double theta = std::atan2(cy2 - cy1, cx2 - cx1);
    double r = std::sqrt(du(rng));
    double angle = du(rng) * 2.0 * M_PI;
    double px = a * r * std::cos(angle);
    double py = b * r * std::sin(angle);

    double cx = (cx1 + cx2) / 2.0, cy = (cy1 + cy2) / 2.0;
    double rx = cx + px * std::cos(theta) - py * std::sin(theta);
    double ry = cy + px * std::sin(theta) + py * std::cos(theta);
    return {std::max(0.0, std::min(w - 1.0, rx)),
            std::max(0.0, std::min(h - 1.0, ry))};
}

PlannerResult InformedRRTStarPlanner::plan(const GridMap& map,
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

    double sd = static_cast<double>(start.x), sg = static_cast<double>(start.y);
    double gd_x = static_cast<double>(goal.x), gd_y = static_cast<double>(goal.y);
    Point2d start_d(sd, sg), goal_d(gd_x, gd_y);

    std::vector<Point2d> nodes;
    std::vector<int> parent;
    std::vector<double> cost;
    nodes.push_back(start_d); parent.push_back(-1); cost.push_back(0.0);
    tree_edges_.clear();

    std::uniform_real_distribution<double> d01(0, 1);
    std::uniform_real_distribution<double> dx(0, map.width() - 1.0);
    std::uniform_real_distribution<double> dy(0, map.height() - 1.0);

    int best_goal_parent = -1;
    double best_goal_cost = std::numeric_limits<double>::max();
    bool has_path = false;

    for (int iter = 0; iter < max_iter_; ++iter) {
        Point2d q_rand;
        if (has_path) {
            q_rand = sampleEllipse(sd, sg, gd_x, gd_y, best_goal_cost,
                                    map.width() - 1.0, map.height() - 1.0, rng_);
        } else if (d01(rng_) < goal_sample_rate_) {
            q_rand = goal_d;
        } else {
            q_rand = {dx(rng_), dy(rng_)};
        }

        // Find nearest
        int ni = 0; double nd = std::numeric_limits<double>::max();
        for (size_t i = 0; i < nodes.size(); ++i) {
            double d = distance(q_rand, nodes[i]);
            if (d < nd) { nd = d; ni = static_cast<int>(i); }
        }

        Point2d q_near = nodes[static_cast<size_t>(ni)];
        double dir_x = q_rand.x - q_near.x, dir_y = q_rand.y - q_near.y;
        double dl = std::sqrt(dir_x*dir_x + dir_y*dir_y);
        if (dl < 1e-9) continue;
        double ext = std::min(step_size_, dl);
        Point2d q_new{q_near.x + dir_x/dl*ext, q_near.y + dir_y/dl*ext};

        if (!map.isFree(static_cast<int>(std::floor(q_new.x)),
                        static_cast<int>(std::floor(q_new.y)))) continue;

        // Collision check segment
        { bool ok = true;
          int ns = std::max(2, static_cast<int>(distance(q_near,q_new)/0.5)+1);
          for (int s = 1; s <= ns; ++s) {
              double t = static_cast<double>(s)/ns;
              double cx = q_near.x + t*(q_new.x - q_near.x);
              double cy = q_near.y + t*(q_new.y - q_near.y);
              if (!map.isFree(static_cast<int>(std::floor(cx)),
                              static_cast<int>(std::floor(cy)))) { ok = false; break; }
          }
          if (!ok) continue; }

        // Choose parent
        int bp = ni;
        double bc = cost[static_cast<size_t>(ni)] + distance(q_new, q_near);
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (static_cast<int>(i) == ni) continue;
            double d = distance(q_new, nodes[i]);
            if (d > rewire_radius_) continue;
            { bool ok = true;
              int ns = std::max(2, static_cast<int>(d/0.5)+1);
              for (int s = 1; s <= ns; ++s) {
                  double t = static_cast<double>(s)/ns;
                  double cx = nodes[i].x + t*(q_new.x - nodes[i].x);
                  double cy = nodes[i].y + t*(q_new.y - nodes[i].y);
                  if (!map.isFree(static_cast<int>(std::floor(cx)),
                                  static_cast<int>(std::floor(cy)))) { ok = false; break; }
              }
              if (!ok) continue; }
            double nc = cost[i] + d;
            if (nc < bc) { bc = nc; bp = static_cast<int>(i); }
        }

        int ni2 = static_cast<int>(nodes.size());
        nodes.push_back(q_new); parent.push_back(bp); cost.push_back(bc);
        tree_edges_.emplace_back(nodes[static_cast<size_t>(bp)], q_new);

        // Rewire
        for (size_t i = 0; i < nodes.size()-1; ++i) {
            double d = distance(q_new, nodes[i]);
            if (d > rewire_radius_ || static_cast<int>(i) == bp) continue;
            double nc = bc + d;
            if (nc < cost[i]) {
                int ns = std::max(2, static_cast<int>(d/0.5)+1);
                bool ok = true;
                for (int s = 1; s <= ns; ++s) {
                    double t = static_cast<double>(s)/ns;
                    double cx = q_new.x + t*(nodes[i].x - q_new.x);
                    double cy = q_new.y + t*(nodes[i].y - q_new.y);
                    if (!map.isFree(static_cast<int>(std::floor(cx)),
                                    static_cast<int>(std::floor(cy)))) { ok = false; break; }
                }
                if (ok) { parent[i] = ni2; cost[i] = nc; }
            }
        }

        double dg = distance(q_new, goal_d);
        if (dg < goal_tolerance_ && bc + dg < best_goal_cost) {
            best_goal_cost = bc + dg;
            best_goal_parent = ni2;
            has_path = true;
        }
    }

    if (best_goal_parent >= 0) {
        Path2d path; path.push_back(goal_d);
        int cur = best_goal_parent;
        while (cur != -1) {
            path.push_back(nodes[static_cast<size_t>(cur)]);
            cur = parent[static_cast<size_t>(cur)];
        }
        std::reverse(path.begin(), path.end());
        result.success = true;
        result.path = std::move(path);
        result.path_length = computePathLength(result.path);
        result.message = "Path found.";
    } else {
        result.message = "No path found.";
    }
    result.expanded_nodes = static_cast<int>(nodes.size());
    result.iterations = max_iter_;
    auto t1 = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

}  // namespace autoplanner
