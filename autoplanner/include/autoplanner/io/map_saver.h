#pragma once

#include <string>
#include "autoplanner/core/grid_map.h"

namespace autoplanner {
namespace io {

// Save a GridMap to a text file.
// Returns true on success.
bool saveMap(const GridMap& map, const std::string& path);

}  // namespace io
}  // namespace autoplanner
