#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "autoplanner/core/planner_factory.h"

using namespace autoplanner;

TEST(PlannerFactory, CreatesAllSupportedPlanners) {
    const std::vector<std::string> names = {
        "astar", "dijkstra", "weighted_astar", "improved_astar", "jps",
        "dstar_lite", "rrt", "rrt_star", "informed_rrt_star", "bi_rrt",
        "hybrid_astar",
    };

    for (const auto& name : names) {
        EXPECT_NE(createPlanner(name), nullptr) << name;
    }
}

TEST(PlannerFactory, UnknownPlannerReturnsNull) {
    EXPECT_EQ(createPlanner("does_not_exist"), nullptr);
}

TEST(PlannerFactory, PassesSamplingOptions) {
    PlannerFactoryOptions options;
    options.step_size = 1.0;
    options.max_iterations = 10;
    options.goal_sample_rate = 0.5;
    options.goal_tolerance = 1.0;

    EXPECT_NE(createPlanner("rrt", options), nullptr);
    EXPECT_NE(createPlanner("bi_rrt", options), nullptr);
}
