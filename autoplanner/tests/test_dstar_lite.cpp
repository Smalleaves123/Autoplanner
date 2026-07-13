#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/planners/graph_search/dstar_lite.h"
#include "autoplanner/planners/graph_search/astar.h"

using namespace autoplanner;

namespace {

std::unique_ptr<GridMap> loadMap(const std::string& path) {
    auto map = std::make_unique<GridMap>();
    map->loadFromTxt(path);
    return map;
}

}  // namespace

TEST(DStarLite, FindsPathOnSimpleMap) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    DStarLitePlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
    EXPECT_GT(result.path_length, 0.0);
}

TEST(DStarLite, StartIsGoal) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    DStarLitePlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{5, 5}, Point2i{5, 5});

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.path.size(), 1u);
}

TEST(DStarLite, InvalidStartFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    DStarLitePlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{0, 0}, Point2i{48, 48});
    EXPECT_FALSE(result.success);
}

TEST(DStarLite, InvalidGoalFails) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    DStarLitePlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{0, 0});
    EXPECT_FALSE(result.success);
}

TEST(DStarLite, PathIsCollisionFree) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    DStarLitePlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    ASSERT_TRUE(result.success);
    for (const auto& p : result.path) {
        int x = static_cast<int>(p.x);
        int y = static_cast<int>(p.y);
        EXPECT_TRUE(map->isFree(x, y))
            << "Path point (" << x << ", " << y << ") is occupied";
    }
}

TEST(DStarLite, ConsistentWithAStar) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    DStarLitePlanner dstar(true);
    AStarPlanner astar(true);

    PlannerResult r1 = dstar.plan(*map, Point2i{5, 5}, Point2i{45, 45});
    PlannerResult r2 = astar.plan(*map, Point2i{5, 5}, Point2i{45, 45});

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    // Path lengths should be close (both find shortest paths).
    EXPECT_NEAR(r1.path_length, r2.path_length, 1.0);
}

TEST(DStarLite, FourConnectedWorks) {
    auto map = loadMap("data/maps/simple_50x50.txt");
    DStarLitePlanner planner(false);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
}
