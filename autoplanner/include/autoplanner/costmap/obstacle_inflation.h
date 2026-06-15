#pragma once

namespace autoplanner {

class GridMap;
class Costmap2D;

// Utility: inflate obstacles in a binary GridMap to produce a safety buffer.
//
// The inflation radius (in metres) is converted to cells using the map
// resolution.  Every free cell within that radius of an obstacle receives
// a cost proportional to its proximity.
namespace ObstacleInflation {

// Create a Costmap2D from a GridMap and inflate obstacles.
Costmap2D createInflatedCostmap(const GridMap& map, double robot_radius);

}  // namespace ObstacleInflation

}  // namespace autoplanner
