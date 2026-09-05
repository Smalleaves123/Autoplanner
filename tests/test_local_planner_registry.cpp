#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>

#include "autoplanner/collision/grid_collision_checker.h"
#include "autoplanner/core/grid_map.h"
#include "robotnav/local_planner_registry.h"

using namespace robotnav;

namespace {

class RegisteredTestLocalPlanner final : public LocalPlanner {
public:
    LocalPlannerDecision computeCommand(
        const autompc::State&,
        double,
        const autompc::Trajectory&,
        const autompc::Control& nominal_command,
        const DynamicObstacleContext&) const override {
        LocalPlannerDecision decision;
        decision.feasible = true;
        decision.command = nominal_command;
        return decision;
    }
};

}  // namespace

TEST(LocalPlannerRegistry, CreatesBuiltInLocalPlanners) {
    autoplanner::GridMap map;
    ASSERT_TRUE(map.loadFromTxt(
        "autoplanner/data/maps/simple_50x50.txt"));
    autoplanner::GridCollisionChecker checker(map);

    EXPECT_NE(createLocalPlanner("dwa", checker, {}), nullptr);
    EXPECT_NE(createLocalPlanner("mppi", checker, {}), nullptr);
}

TEST(LocalPlannerRegistry, RegistersApplicationLocalPlanner) {
    auto& registry = LocalPlannerRegistry::instance();
    const std::string name = "registered_test";
    registry.unregisterLocalPlanner(name);

    EXPECT_TRUE(registry.registerLocalPlanner(
        name,
        [](const autoplanner::CollisionChecker&,
           const autompc::SimulationOptions&,
           const DwaOptions&,
           const MppiOptions&) {
            return std::make_unique<RegisteredTestLocalPlanner>();
        }));
    EXPECT_FALSE(registry.registerLocalPlanner("none", {}));
    EXPECT_TRUE(registry.contains(name));

    autoplanner::GridMap map;
    ASSERT_TRUE(map.loadFromTxt(
        "autoplanner/data/maps/simple_50x50.txt"));
    autoplanner::GridCollisionChecker checker(map);
    auto planner = createLocalPlanner(name, checker, {});
    ASSERT_NE(planner, nullptr);
    const autompc::Control nominal{0.5, 0.1};
    const auto decision = planner->computeCommand({}, 0.0, {}, nominal);
    EXPECT_TRUE(decision.feasible);
    EXPECT_DOUBLE_EQ(decision.command.velocity, nominal.velocity);

    EXPECT_TRUE(registry.unregisterLocalPlanner(name));
    EXPECT_FALSE(registry.contains(name));
}

TEST(LocalPlannerRegistry, ListsNamesInDeterministicOrder) {
    const auto names = availableLocalPlanners();
    EXPECT_TRUE(std::is_sorted(names.begin(), names.end()));
    EXPECT_NE(std::find(names.begin(), names.end(), "dwa"), names.end());
    EXPECT_NE(std::find(names.begin(), names.end(), "mppi"), names.end());
}
