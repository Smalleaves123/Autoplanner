#include <gtest/gtest.h>

#include <cmath>

#include "autoplanner/core/grid_map.h"
#include "robotnav/cooperative_astar.h"

namespace {

autoplanner::GridMap map() {
    autoplanner::GridMap result;
    EXPECT_TRUE(result.loadFromTxt(
        "autoplanner/data/maps/simple_50x50.txt"));
    return result;
}

}  // namespace

TEST(CooperativeAStarTest, PlansAgentsWithoutVertexConflicts) {
    robotnav::CooperativeAStarPlanner planner({false, true, 80, false, false, {}, 10000});
    const auto result = planner.plan(
        map(), {{2, 2}, {8, 2}}, {{8, 2}, {2, 2}});
    ASSERT_TRUE(result.success) << result.message;
    ASSERT_EQ(result.plans.size(), 2u);
    ASSERT_EQ(result.planned_agents, 2u);
    const auto& first = result.plans[0].path;
    const auto& second = result.plans[1].path;
    const std::size_t horizon = std::max(first.size(), second.size());
    for (std::size_t time = 0; time < horizon; ++time) {
        const auto& a = first[std::min(time, first.size() - 1)];
        const auto& b = second[std::min(time, second.size() - 1)];
        EXPECT_FALSE(a.x == b.x && a.y == b.y) << "time=" << time;
    }
}

TEST(CooperativeAStarTest, RejectsInvalidAgentInput) {
    robotnav::CooperativeAStarPlanner planner;
    const auto result = planner.plan(map(), {{2, 2}}, {{8, 8}, {9, 9}});
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.planned_agents, 0u);
}

TEST(CooperativeAStarTest, ReportsNoPathWithinHorizon) {
    robotnav::CooperativeAStarPlanner planner({false, false, 1, false, false, {}, 10000});
    const auto result = planner.plan(map(), {{2, 2}}, {{8, 8}});
    EXPECT_FALSE(result.success);
    ASSERT_EQ(result.plans.size(), 1u);
    EXPECT_FALSE(result.plans.front().success);
}

TEST(CooperativeAStarTest, CBSResolvesSharedDestination) {
    robotnav::CooperativeAStarPlanner planner({false, true, 80, true, false, {}, 10000});
    const auto result = planner.plan(
        map(), {{2, 2}, {8, 2}, {2, 8}}, {{5, 5}, {5, 5}, {5, 5}});
    ASSERT_TRUE(result.success) << result.message;
    EXPECT_EQ(result.planned_agents, 3u);
    ASSERT_EQ(result.plans.size(), 3u);
    for (const auto& plan : result.plans) EXPECT_TRUE(plan.success);
}

TEST(CooperativeAStarTest, PrioritizedOrderDoesNotReorderResults) {
    robotnav::CooperativeAStarPlanner planner(
        {false, true, 80, false, false, {1, 0}, 10000});
    const auto result = planner.plan(
        map(), {{2, 2}, {8, 2}}, {{8, 2}, {2, 2}});
    ASSERT_TRUE(result.success) << result.message;
    ASSERT_EQ(result.plans.size(), 2u);
    EXPECT_EQ(result.plans[0].path.front().x, 2.0);
    EXPECT_EQ(result.plans[1].path.front().x, 8.0);
}
