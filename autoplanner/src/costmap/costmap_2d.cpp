#include "autoplanner/costmap/costmap_2d.h"

#include <algorithm>
#include <cmath>
#include <queue>

namespace autoplanner {

void Costmap2D::buildFromGridMap(const GridMap& map) {
    width_ = map.width();
    height_ = map.height();
    resolution_ = map.resolution();

    cost_.assign(static_cast<std::size_t>(width_ * height_), 0.0);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (map.isOccupied(x, y)) {
                cost_[static_cast<std::size_t>(y * width_ + x)] = 1.0;
            }
        }
    }
}

void Costmap2D::inflateObstacles(double robot_radius) {
    if (robot_radius <= 0.0) {
        return;
    }

    // Convert radius from metres to cells.
    const int radius_cells =
        static_cast<int>(std::ceil(robot_radius / resolution_));

    if (radius_cells < 1) {
        return;
    }

    // Work on a copy to avoid inflating already-inflated cells.
    std::vector<double> inflated = cost_;

    // For every obstacle cell, mark a circular neighbourhood with a
    // cost that decays linearly with distance from the obstacle.
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            const std::size_t idx = static_cast<std::size_t>(y * width_ + x);
            if (cost_[idx] < 1.0) {
                continue;  // not an obstacle
            }

            // Scan a square window around the obstacle, then filter by radius.
            const int x_min = std::max(0, x - radius_cells);
            const int x_max = std::min(width_ - 1, x + radius_cells);
            const int y_min = std::max(0, y - radius_cells);
            const int y_max = std::min(height_ - 1, y + radius_cells);

            for (int ny = y_min; ny <= y_max; ++ny) {
                for (int nx = x_min; nx <= x_max; ++nx) {
                    const double dx = static_cast<double>(nx - x);
                    const double dy = static_cast<double>(ny - y);
                    const double dist = std::sqrt(dx * dx + dy * dy);

                    if (dist > static_cast<double>(radius_cells)) {
                        continue;
                    }

                    // Linear decay: cost = 1.0 at the obstacle, 0.0 at radius.
                    const double dist_cost =
                        1.0 - dist / static_cast<double>(radius_cells);

                    const std::size_t n_idx =
                        static_cast<std::size_t>(ny * width_ + nx);
                    inflated[n_idx] = std::max(inflated[n_idx], dist_cost);
                }
            }
        }
    }

    cost_ = std::move(inflated);
}

double Costmap2D::getCost(int x, int y) const {
    if (x < 0 || y < 0 || x >= width_ || y >= height_) {
        return 1.0;  // out-of-bounds is lethal
    }
    return cost_[static_cast<std::size_t>(y * width_ + x)];
}

bool Costmap2D::isFree(int x, int y) const {
    return getCost(x, y) < 1.0;
}

int Costmap2D::width() const {
    return width_;
}

int Costmap2D::height() const {
    return height_;
}

double Costmap2D::resolution() const {
    return resolution_;
}

}  // namespace autoplanner
