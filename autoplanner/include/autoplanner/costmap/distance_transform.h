#pragma once

#include <vector>

namespace autoplanner {

class GridMap;

// Compute a 2-D Euclidean distance transform from a binary occupancy grid.
//
// Each cell of the output contains the Euclidean distance (in cell units)
// to the nearest obstacle.  Free cells far from obstacles get large values;
// obstacle cells get 0.0.
//
// Uses the 8SSEDT algorithm (two-pass linear-time approximate Euclidean
// distance transform).
std::vector<double> computeDistanceTransform(const GridMap& map);

}  // namespace autoplanner
