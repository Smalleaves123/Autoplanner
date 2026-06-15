#include "autoplanner/planners/sampling/bi_rrt.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace autoplanner {

BiRRTPlanner::BiRRTPlanner(double step_size, int max_iter, double goal_tolerance)
    : step_size_(step_size), max_iter_(max_iter),
      goal_tolerance_(goal_tolerance), rng_(std::random_device{}()) {}

std::string BiRRTPlanner::name() const { return "bi_rrt"; }

PlannerResult BiRRTPlanner::plan(const GridMap& map, const Point2i& start,
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

    Point2d sd(static_cast<double>(start.x), static_cast<double>(start.y));
    Point2d gd(static_cast<double>(goal.x), static_cast<double>(goal.y));

    // Tree A (from start)
    std::vector<Point2d> na; na.push_back(sd);
    std::vector<int> pa; pa.push_back(-1);
    // Tree B (from goal)
    std::vector<Point2d> nb; nb.push_back(gd);
    std::vector<int> pb; pb.push_back(-1);

    std::uniform_real_distribution<double> dx(0, map.width()-1.0);
    std::uniform_real_distribution<double> dy(0, map.height()-1.0);

    for (int iter = 0; iter < max_iter_; ++iter) {
        // Sample random point and extend tree A toward it.
        Point2d q_rand{dx(rng_), dy(rng_)};
        auto extend = [&](std::vector<Point2d>& nodes, std::vector<int>& parent,
                          const Point2d& target) -> int {
            int ni = 0; double nd = std::numeric_limits<double>::max();
            for (size_t i = 0; i < nodes.size(); ++i) {
                double d = distance(target, nodes[i]);
                if (d < nd) { nd = d; ni = static_cast<int>(i); }
            }
            Point2d qn = nodes[static_cast<size_t>(ni)];
            double dx2 = target.x - qn.x, dy2 = target.y - qn.y;
            double dl = std::sqrt(dx2*dx2 + dy2*dy2);
            if (dl < 1e-9) return -1;
            double ext = std::min(step_size_, dl);
            Point2d qn2{qn.x + dx2/dl*ext, qn.y + dy2/dl*ext};
            if (!map.isFree(static_cast<int>(std::floor(qn2.x)),
                            static_cast<int>(std::floor(qn2.y)))) return -1;
            { bool ok = true;
              int ns = std::max(2, static_cast<int>(distance(qn,qn2)/0.5)+1);
              for (int s = 1; s <= ns; ++s) {
                  double t = static_cast<double>(s)/ns;
                  double cx = qn.x + t*(qn2.x - qn.x);
                  double cy = qn.y + t*(qn2.y - qn.y);
                  if (!map.isFree(static_cast<int>(std::floor(cx)),
                                  static_cast<int>(std::floor(cy)))) { ok=false; break; }
              }
              if (!ok) return -1; }
            nodes.push_back(qn2);
            parent.push_back(ni);
            return static_cast<int>(nodes.size()) - 1;
        };

        int new_a = extend(na, pa, q_rand);
        if (new_a < 0) { result.iterations = iter+1; continue; }

        // Extend tree B toward the new node in A.
        int new_b = extend(nb, pb, na[static_cast<size_t>(new_a)]);
        if (new_b < 0) { result.iterations = iter+1; continue; }

        // Check if trees are close enough.
        if (distance(na[static_cast<size_t>(new_a)],
                     nb[static_cast<size_t>(new_b)]) < goal_tolerance_) {
            // Reconstruct path: start->...->new_a  +  new_b->...->goal (reversed)
            Path2d path;
            int cur = new_a;
            while (cur != -1) {
                path.push_back(na[static_cast<size_t>(cur)]);
                cur = pa[static_cast<size_t>(cur)];
            }
            std::reverse(path.begin(), path.end());
            cur = pb[static_cast<size_t>(new_b)];
            while (cur != -1) {
                path.push_back(nb[static_cast<size_t>(cur)]);
                cur = pb[static_cast<size_t>(cur)];
            }
            result.success = true;
            result.path = std::move(path);
            result.path_length = computePathLength(result.path);
            result.expanded_nodes = static_cast<int>(na.size() + nb.size());
            result.iterations = iter + 1;
            result.message = "Path found.";
            break;
        }
        result.iterations = iter + 1;
    }

    if (!result.success && result.message.empty())
        result.message = "No path found.";

    auto t1 = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

}  // namespace autoplanner
