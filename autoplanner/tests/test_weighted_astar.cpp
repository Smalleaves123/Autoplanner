#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/planners/graph_search/weighted_astar.h"
#include "autoplanner/planners/graph_search/astar.h"

using namespace autoplanner;

namespace {

std::unique_ptr<GridMap> loadMap(const std::string& path) {
    auto map = std::make_unique<GridMap>();
    map->loadFromTxt(path);
    return map;
}

}  // namespace

TEST(WeightedAStar, FindsPathOnSimpleMap) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    WeightedAStarPlanner planner(1.5, true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
    EXPECT_GT(result.path_length, 0.0);
}

TEST(WeightedAStar, StartIsGoal) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    WeightedAStarPlanner planner;

    PlannerResult result = planner.plan(*map, Point2i{5, 5}, Point2i{5, 5});

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.path.size(), 1u);
}

TEST(WeightedAStar, InvalidStartFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    WeightedAStarPlanner planner;

    PlannerResult result = planner.plan(*map, Point2i{0, 0}, Point2i{48, 48});
    EXPECT_FALSE(result.success);
}

TEST(WeightedAStar, InvalidGoalFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    WeightedAStarPlanner planner;

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{0, 0});
    EXPECT_FALSE(result.success);
}

TEST(WeightedAStar, PathIsCollisionFree) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    WeightedAStarPlanner planner(1.5, true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    ASSERT_TRUE(result.success);
    for (const auto& p : result.path) {
        int x = static_cast<int>(p.x);
        int y = static_cast<int>(p.y);
        EXPECT_TRUE(map->isFree(x, y))
            << "Path point (" << x << ", " << y << ") is occupied";
    }
}

TEST(WeightedAStar, HigherWeightExpandsFewerNodes) {
    auto map = loadMap("data/maps/simple_50x50.txt");

    WeightedAStarPlanner w1(1.0, true);   // = standard A*
    WeightedAStarPlanner w5(5.0, true);   // greedy

    PlannerResult r1 = w1.plan(*map, Point2i{1, 1}, Point2i{48, 48});
    PlannerResult r5 = w5.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    ASSERT_TRUE(r1.success);
    ASSERT_TRUE(r5.success);
    // Higher weight should expand fewer nodes.
    EXPECT_LE(r5.expanded_nodes, r1.expanded_nodes);
}

TEST(WeightedAStar, FourConnectedWorks) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    WeightedAStarPlanner planner(1.5, false);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
}

TEST(WeightedAStar, SetWeightChangesBehaviour) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    WeightedAStarPlanner planner(1.0, true);

    // Plan with weight 1.0 (standard A*).
    PlannerResult r1 = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});
    ASSERT_TRUE(r1.success);

    // Change weight and plan again.
    planner.setWeight(3.0);
    PlannerResult r2 = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});
    ASSERT_TRUE(r2.success);

    // Higher weight should give different (likely shorter/fewer expanded) result.
    EXPECT_LE(r2.expanded_nodes, r1.expanded_nodes);
}
