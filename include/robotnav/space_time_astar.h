#pragma once

#include <cstddef>
#include <vector>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/planner_result.h"
#include "autoplanner/core/point.h"
#include "robotnav/dynamic_obstacle_prediction.h"

namespace robotnav {

struct SpaceTimeAStarOptions {
    bool allow_diagonal = true;
    bool allow_wait = true;
    std::size_t max_time_steps = 120;
    double obstacle_margin = 0.0;
    double risk_weight = 0.0;
    double risk_clearance = 0.0;
};

class SpaceTimeAStarPlanner {
public:
    explicit SpaceTimeAStarPlanner(SpaceTimeAStarOptions options = {});

    autoplanner::PlannerResult plan(
        const autoplanner::GridMap& map,
        const autoplanner::Point2i& start,
        const autoplanner::Point2i& goal,
        const std::vector<MovingObstacle>& moving_obstacles,
        std::size_t start_frame = 0) const;

private:
    SpaceTimeAStarOptions options_;
};

}  // namespace robotnav
