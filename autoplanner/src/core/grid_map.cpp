#include "autoplanner/core/grid_map.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace autoplanner {

bool GridMap::loadFromTxt(const std::string& file_path) {
    std::ifstream fin(file_path);
    if (!fin.is_open()) {
        return false;
    }

    // Read every non-empty line, stripping whitespace and validating chars.
    std::vector<std::string> rows;
    std::string line;

    while (std::getline(fin, line)) {
        std::string row;
        for (char c : line) {
            if (std::isspace(static_cast<unsigned char>(c))) {
                continue;
            }

            // '0' / '.' = free,  '1' / '#' / '@' = obstacle.
            if (c == '0' || c == '.' || c == '1' || c == '#' || c == '@') {
                row.push_back(c);
            } else {
                return false;  // invalid character in map file
            }
        }

        if (!row.empty()) {
            rows.push_back(row);
        }
    }

    if (rows.empty()) {
        return false;
    }

    // All rows must have the same width (rectangular grid).
    const int expected_width = static_cast<int>(rows.front().size());
    for (const auto& row : rows) {
        if (static_cast<int>(row.size()) != expected_width) {
            return false;
        }
    }

    width_ = expected_width;
    height_ = static_cast<int>(rows.size());
    data_.assign(static_cast<std::size_t>(width_ * height_), 0);

    // Convert character grid to packed integer array.
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            const char c = rows[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)];
            const bool occupied = (c == '1' || c == '#' || c == '@');
            data_[static_cast<std::size_t>(index(x, y))] = occupied ? 1 : 0;
        }
    }

    return true;
}

bool GridMap::isInside(int x, int y) const {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
}

bool GridMap::isFree(int x, int y) const {
    if (!isInside(x, y)) {
        return false;  // out-of-bounds is treated as blocked
    }
    return data_[static_cast<std::size_t>(index(x, y))] == 0;
}

bool GridMap::isOccupied(int x, int y) const {
    if (!isInside(x, y)) {
        return true;  // out-of-bounds is treated as occupied
    }
    return data_[static_cast<std::size_t>(index(x, y))] != 0;
}

int GridMap::width() const {
    return width_;
}

int GridMap::height() const {
    return height_;
}

void GridMap::setResolution(double resolution) {
    resolution_ = std::max(0.0, resolution);
}

double GridMap::resolution() const {
    return resolution_;
}

void GridMap::inflateObstacles(double radius) {
    if (isEmpty() || radius <= 0.0) return;

    const double cell_size = resolution_ > 0.0 ? resolution_ : 1.0;
    const int radius_cells = static_cast<int>(std::ceil(radius / cell_size));
    const std::vector<int> original = data_;

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            bool occupied = false;
            for (int dy = -radius_cells; dy <= radius_cells && !occupied; ++dy) {
                for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                    if (dx * dx + dy * dy > radius_cells * radius_cells) continue;
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (isInside(nx, ny) &&
                        original[static_cast<std::size_t>(index(nx, ny))] != 0) {
                        occupied = true;
                        break;
                    }
                }
            }
            if (occupied) data_[static_cast<std::size_t>(index(x, y))] = 1;
        }
    }
}

bool GridMap::isEmpty() const {
    return width_ == 0 || height_ == 0;
}

// Row-major linearisation: index = y * width + x.
// Returns -1 for out-of-bounds coordinates (caller must check).
int GridMap::index(int x, int y) const {
    if (!isInside(x, y)) return -1;
    return y * width_ + x;
}

}  // namespace autoplanner
