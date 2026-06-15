#pragma once

#include <string>
#include "autoplanner/core/path.h"

namespace autoplanner {
namespace io {

// Save a path to a CSV file.  Delegates to savePathCsv.
inline bool savePath(const Path2d& path, const std::string& file_path) {
    return savePathCsv(path, file_path);
}

}  // namespace io
}  // namespace autoplanner
