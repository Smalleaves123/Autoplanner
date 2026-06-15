#pragma once

#include <string>
#include <vector>

namespace autoplanner {

// 2-D occupancy grid map.
//
// Each cell is either free (0) or occupied (1).  Maps are loaded from a
// text file where '0' / '.' denote free space and '1' / '#' / '@' denote
// obstacles.  All rows must have the same width.
class GridMap {
public:
    GridMap() = default;

    // Load an occupancy grid from a plain-text file.
    // Returns false on format errors or if the file cannot be opened.
    bool loadFromTxt(const std::string& file_path);

    // Bounds and occupancy queries (all operate on integer cell indices).
    bool isInside(int x, int y) const;
    bool isFree(int x, int y) const;
    bool isOccupied(int x, int y) const;

    int width() const;
    int height() const;

    // World resolution in metres / cell.
    void setResolution(double resolution);
    double resolution() const;

    // Flatten 2-D coordinates into a 1-D row-major index.
    // Returns -1 for out-of-bounds coordinates.
    int index(int x, int y) const;

    // True if the map has no data (width or height is zero).
    bool isEmpty() const;

private:
    int width_ = 0;
    int height_ = 0;
    double resolution_ = 1.0;
    std::vector<int> data_;
};

}  // namespace autoplanner
