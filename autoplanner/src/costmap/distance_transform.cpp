#include "autoplanner/costmap/distance_transform.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "autoplanner/core/grid_map.h"

namespace autoplanner {

std::vector<double> computeDistanceTransform(const GridMap& map) {
    const int w = map.width();
    const int h = map.height();
    const int n = w * h;

    std::vector<double> dt(static_cast<std::size_t>(n),
                           std::numeric_limits<double>::max());

    // Initialize: obstacles get 0.
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (map.isOccupied(x, y)) {
                dt[static_cast<std::size_t>(y * w + x)] = 0.0;
            }
        }
    }

    // Forward pass: top-left to bottom-right.
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            std::size_t idx = static_cast<std::size_t>(y * w + x);
            if (dt[idx] == 0.0) continue;
            double best = dt[idx];
            if (x > 0) best = std::min(best, dt[static_cast<std::size_t>(y * w + x - 1)] + 1.0);
            if (y > 0) best = std::min(best, dt[static_cast<std::size_t>((y - 1) * w + x)] + 1.0);
            if (x > 0 && y > 0)
                best = std::min(best, dt[static_cast<std::size_t>((y - 1) * w + x - 1)] + std::sqrt(2.0));
            if (x < w - 1 && y > 0)
                best = std::min(best, dt[static_cast<std::size_t>((y - 1) * w + x + 1)] + std::sqrt(2.0));
            dt[idx] = best;
        }
    }

    // Backward pass: bottom-right to top-left.
    for (int y = h - 1; y >= 0; --y) {
        for (int x = w - 1; x >= 0; --x) {
            std::size_t idx = static_cast<std::size_t>(y * w + x);
            if (dt[idx] == 0.0) continue;
            double best = dt[idx];
            if (x < w - 1) best = std::min(best, dt[static_cast<std::size_t>(y * w + x + 1)] + 1.0);
            if (y < h - 1) best = std::min(best, dt[static_cast<std::size_t>((y + 1) * w + x)] + 1.0);
            if (x < w - 1 && y < h - 1)
                best = std::min(best, dt[static_cast<std::size_t>((y + 1) * w + x + 1)] + std::sqrt(2.0));
            if (x > 0 && y < h - 1)
                best = std::min(best, dt[static_cast<std::size_t>((y + 1) * w + x - 1)] + std::sqrt(2.0));
            dt[idx] = best;
        }
    }

    return dt;
}

}  // namespace autoplanner
