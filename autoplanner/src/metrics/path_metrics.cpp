#include "autoplanner/metrics/path_metrics.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoplanner {

int computeTurningCount(const std::vector<Point2d>& path) {
    if (path.size() < 3) return 0;
    int turns = 0;
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        double dx1 = path[i].x - path[i - 1].x;
        double dy1 = path[i].y - path[i - 1].y;
        double dx2 = path[i + 1].x - path[i].x;
        double dy2 = path[i + 1].y - path[i].y;
        if (std::abs(dx1 * dy2 - dy1 * dx2) > 1e-9) turns++;
    }
    return turns;
}

double computeTotalTurning(const std::vector<Point2d>& path) {
    if (path.size() < 3) return 0.0;
    double total = 0.0;
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        double dx1 = path[i].x - path[i - 1].x;
        double dy1 = path[i].y - path[i - 1].y;
        double dx2 = path[i + 1].x - path[i].x;
        double dy2 = path[i + 1].y - path[i].y;
        double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        if (len1 < 1e-9 || len2 < 1e-9) continue;
        double cos_angle = (dx1 * dx2 + dy1 * dy2) / (len1 * len2);
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        total += std::acos(cos_angle);
    }
    return total;
}

double computeAverageCurvature(const std::vector<Point2d>& path) {
    if (path.size() < 3) return 0.0;
    double total = 0.0;
    int count = 0;
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        double dx1 = path[i].x - path[i - 1].x;
        double dy1 = path[i].y - path[i - 1].y;
        double dx2 = path[i + 1].x - path[i].x;
        double dy2 = path[i + 1].y - path[i].y;
        double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        if (len1 < 1e-9 || len2 < 1e-9) continue;
        double cross = dx1 * dy2 - dy1 * dx2;
        double dot = dx1 * dx2 + dy1 * dy2;
        double angle = std::atan2(cross, dot);
        total += std::abs(angle) / std::min(len1, len2);
        count++;
    }
    return (count > 0) ? total / static_cast<double>(count) : 0.0;
}

double computeSmoothnessScore(const std::vector<Point2d>& path) {
    if (path.size() < 2) return 1.0;
    double actual_length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        actual_length += distance(path[i - 1], path[i]);
    }
    double straight_line = distance(path.front(), path.back());
    if (actual_length < 1e-9) return 1.0;
    return straight_line / actual_length;
}

double computeMinObstacleDistance(
    const std::vector<Point2d>& path,
    const std::vector<Point2i>& obstacles
) {
    if (path.empty() || obstacles.empty())
        return std::numeric_limits<double>::infinity();
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& pt : path) {
        for (const auto& obs : obstacles) {
            double d = distance(pt, Point2d(static_cast<double>(obs.x),
                                            static_cast<double>(obs.y)));
            if (d < min_dist) min_dist = d;
        }
    }
    return min_dist;
}

}  // namespace autoplanner
