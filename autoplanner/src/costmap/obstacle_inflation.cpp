#include "autoplanner/costmap/obstacle_inflation.h"

#include "autoplanner/core/grid_map.h"
#include "autoplanner/costmap/costmap_2d.h"

namespace autoplanner {
namespace ObstacleInflation {

Costmap2D createInflatedCostmap(const GridMap& map, double robot_radius) {
    Costmap2D costmap;
    costmap.buildFromGridMap(map);
    costmap.inflateObstacles(robot_radius);
    return costmap;
}

}  // namespace ObstacleInflation
}  // namespace autoplanner
