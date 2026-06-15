#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/planners/graph_search/dijkstra.h"

using namespace autoplanner;

namespace {

std::unique_ptr<GridMap> loadSimpleMap() {
    auto map = std::make_unique<GridMap>();
    map->loadFromTxt("data/maps/simple_50x50.txt");
    return map;
}

}  // namespace

TEST(Dijkstra, FindsPathOnSimpleMap) {
    auto map = loadSimpleMap();
    DijkstraPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
    EXPECT_GT(result.path_length, 0.0);
    EXPECT_GT(result.expanded_nodes, 0);
}

TEST(Dijkstra, StartIsGoal) {
    auto map = loadSimpleMap();
    DijkstraPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{10, 10}, Point2i{10, 10});

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.path.size(), 1u);
    EXPECT_DOUBLE_EQ(result.path_length, 0.0);
}

TEST(Dijkstra, InvalidStart) {
    auto map = loadSimpleMap();
    DijkstraPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{0, 0}, Point2i{48, 48});
    EXPECT_FALSE(result.success);
}

TEST(Dijkstra, InvalidGoal) {
    auto map = loadSimpleMap();
    DijkstraPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{0, 0});
    EXPECT_FALSE(result.success);
}

TEST(Dijkstra, PathIsCollisionFree) {
    auto map = loadSimpleMap();
    DijkstraPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    ASSERT_TRUE(result.success);
    for (const auto& p : result.path) {
        int x = static_cast<int>(p.x);
        int y = static_cast<int>(p.y);
        EXPECT_TRUE(map->isFree(x, y))
            << "Path point (" << x << ", " << y << ") is occupied";
    }
}

TEST(Dijkstra, PlanningTimeIsReasonable) {
    auto map = loadSimpleMap();
    DijkstraPlanner planner(true);

    PlannerResult result = planner.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_LT(result.planning_time_ms, 5000.0);  // should be well under 5s
}
