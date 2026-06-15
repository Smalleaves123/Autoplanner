#pragma once

#include <string>
#include <vector>

#include "autoplanner/core/point.h"

namespace autoplanner {

// A path is an ordered sequence of continuous-world waypoints.
using Path2d = std::vector<Point2d>;

// Compute the total Euclidean length of the path.
double computePathLength(const Path2d& path);

// Write path waypoints to a CSV file with a "x,y" header.
bool savePathCsv(const Path2d& path, const std::string& file_path);

}  // namespace autoplanner
