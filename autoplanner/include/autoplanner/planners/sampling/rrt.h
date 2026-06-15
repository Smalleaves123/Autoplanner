#pragma once

#include <memory>
#include <random>
#include <vector>

#include "autoplanner/collision/collision_checker.h"
#include "autoplanner/core/planner_base.h"

namespace autoplanner {

// Rapidly-exploring Random Tree (RRT) planner.
//
// Samples random configurations in continuous space and grows a tree from
// the start toward those samples.  Uses goal biasing to accelerate search.
// Produces a feasible but not necessarily optimal path.
class RRTPlanner final : public PlannerBase {
public:
    // step_size:     maximum extension distance per iteration
    // max_iter:      maximum number of iterations before giving up
    // goal_sample_rate: probability of sampling the goal directly [0, 1]
    // goal_tolerance:    distance to goal considered "reached"
    RRTPlanner(double step_size = 2.0,
               int max_iter = 5000,
               double goal_sample_rate = 0.1,
               double goal_tolerance = 2.0);

    PlannerResult plan(
        const GridMap& map,
        const Point2i& start,
        const Point2i& goal
    ) override;

    std::string name() const override;

    // Access the tree nodes for visualization (each pair is an edge).
    const std::vector<std::pair<Point2d, Point2d>>& treeEdges() const;

private:
    double step_size_;
    int max_iter_;
    double goal_sample_rate_;
    double goal_tolerance_;

    std::mt19937 rng_;
    std::vector<std::pair<Point2d, Point2d>> tree_edges_;
};

}  // namespace autoplanner
