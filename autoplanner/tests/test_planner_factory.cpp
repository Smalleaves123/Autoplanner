#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "autoplanner/core/planner_factory.h"

using namespace autoplanner;

namespace {

class RegisteredTestPlanner final : public PlannerBase {
public:
    PlannerResult plan(const GridMap&, const Point2i&, const Point2i&) override {
        PlannerResult result;
        result.planner_name = name();
        return result;
    }

    std::string name() const override { return "registered_test"; }
};

}  // namespace

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

TEST(PlannerFactory, ListsBuiltInPlannersInDeterministicOrder) {
    const auto names = availablePlanners();
    EXPECT_TRUE(std::is_sorted(names.begin(), names.end()));
    EXPECT_NE(std::find(names.begin(), names.end(), "astar"), names.end());
    EXPECT_NE(std::find(names.begin(), names.end(), "hybrid_astar"), names.end());
}

TEST(PlannerFactory, RegistersAndRemovesApplicationPlanner) {
    auto& registry = PlannerRegistry::instance();
    const std::string name = "registered_test";
    registry.unregisterPlanner(name);

    EXPECT_TRUE(registry.registerPlanner(
        name, [](const PlannerFactoryOptions&, const Costmap2D*) {
            return std::make_unique<RegisteredTestPlanner>();
        }));
    EXPECT_TRUE(registry.contains(name));
    EXPECT_FALSE(registry.registerPlanner(
        name, [](const PlannerFactoryOptions&, const Costmap2D*) {
            return std::make_unique<RegisteredTestPlanner>();
        }));

    auto planner = createPlanner(name);
    ASSERT_NE(planner, nullptr);
    EXPECT_EQ(planner->name(), name);

    EXPECT_TRUE(registry.unregisterPlanner(name));
    EXPECT_FALSE(registry.contains(name));
    EXPECT_EQ(createPlanner(name), nullptr);
}
