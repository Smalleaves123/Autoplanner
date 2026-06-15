#include <gtest/gtest.h>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/dijkstra.h"

using namespace autoplanner;

namespace {

std::unique_ptr<GridMap> loadSimpleMap() {
    auto map = std::make_unique<GridMap>();
    map->loadFromTxt("data/maps/simple_50x50.txt");
    return map;
}

}  // namespace

// --- A* Tests ---

TEST(AStar, FindsPathOnSimpleMap) {
    auto map = loadSimpleMap();
    AStarPlanner astar(true);

    PlannerResult result = astar.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
    EXPECT_GT(result.path_length, 0.0);
    EXPECT_GT(result.expanded_nodes, 0);
}

TEST(AStar, StartIsGoal) {
    auto map = loadSimpleMap();
    AStarPlanner astar(true);

    PlannerResult result = astar.plan(*map, Point2i{5, 5}, Point2i{5, 5});

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.path.size(), 1u);
}

TEST(AStar, InvalidStartFails) {
    auto map = loadSimpleMap();
    AStarPlanner astar(true);

    PlannerResult result = astar.plan(*map, Point2i{0, 0}, Point2i{48, 48});
    EXPECT_FALSE(result.success);
}

TEST(AStar, InvalidGoalFails) {
    auto map = loadSimpleMap();
    AStarPlanner astar(true);

    PlannerResult result = astar.plan(*map, Point2i{1, 1}, Point2i{0, 0});
    EXPECT_FALSE(result.success);
}

TEST(AStar, FourConnectedWorks) {
    auto map = loadSimpleMap();
    AStarPlanner astar(false);

    PlannerResult result = astar.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0u);
}

TEST(AStar, PathIsCollisionFree) {
    auto map = loadSimpleMap();
    AStarPlanner astar(true);

    PlannerResult result = astar.plan(*map, Point2i{1, 1}, Point2i{48, 48});

    ASSERT_TRUE(result.success);
    for (const auto& p : result.path) {
        int x = static_cast<int>(p.x);
        int y = static_cast<int>(p.y);
        EXPECT_TRUE(map->isFree(x, y)) << "Path point (" << x << ", " << y << ") is occupied";
    }
}

TEST(AStar, ConsistentWithDijkstra) {
    auto map = loadSimpleMap();
    AStarPlanner astar(false);
    DijkstraPlanner dijkstra(false);

    // On a uniform grid without obstacles in the way, both should find
    // paths with similar lengths when using 4-connectivity.
    PlannerResult r1 = astar.plan(*map, Point2i{5, 5}, Point2i{45, 45});
    PlannerResult r2 = dijkstra.plan(*map, Point2i{5, 5}, Point2i{45, 45});

    EXPECT_TRUE(r1.success);
    EXPECT_TRUE(r2.success);
    // Path lengths should be close (A* is optimal with admissible heuristic).
    EXPECT_NEAR(r1.path_length, r2.path_length, 1.0);
}
