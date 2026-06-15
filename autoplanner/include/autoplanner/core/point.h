#pragma once

#include <cmath>

namespace autoplanner {

// Integer grid coordinates (cell indices).
struct Point2i {
    int x = 0;
    int y = 0;

    Point2i() = default;
    Point2i(int x_in, int y_in) : x(x_in), y(y_in) {}

    bool operator==(const Point2i& other) const {
        return x == other.x && y == other.y;
    }
};

// Continuous-world coordinates (metres).
struct Point2d {
    double x = 0.0;
    double y = 0.0;

    Point2d() = default;
    Point2d(double x_in, double y_in) : x(x_in), y(y_in) {}
};

// Euclidean distance between two continuous points.
inline double distance(const Point2d& a, const Point2d& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Euclidean distance between two grid cells.
inline double distance(const Point2i& a, const Point2i& b) {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace autoplanner
