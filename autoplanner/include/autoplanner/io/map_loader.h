#pragma once

#include <string>
#include "autoplanner/core/grid_map.h"

namespace autoplanner {
namespace io {

// Load a GridMap from a text file.  Delegates to GridMap::loadFromTxt.
// Returns true on success.
inline bool loadMap(const std::string& path, GridMap& map) {
    return map.loadFromTxt(path);
}

}  // namespace io
}  // namespace autoplanner
