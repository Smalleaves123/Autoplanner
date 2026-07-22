#include "autoplanner/collision/footprint_collision_checker.h"

#include <algorithm>
#include <cmath>

namespace autoplanner {

RobotFootprint RobotFootprint::circle(double radius) {
    RobotFootprint result;
    result.type = FootprintType::Circle;
    result.radius = std::max(0.0, radius);
    return result;
}

RobotFootprint RobotFootprint::rectangle(double length, double width) {
    RobotFootprint result;
    result.type = FootprintType::Rectangle;
    result.length = std::max(0.0, length);
    result.width = std::max(0.0, width);
    return result;
}

FootprintCollisionChecker::FootprintCollisionChecker(
    const GridMap& map, RobotFootprint footprint)
    : map_(map), footprint_(footprint) {}

bool FootprintCollisionChecker::isStateValid(const Point2d& p) const {
    return isPoseValid({p.x, p.y, 0.0});
}

bool FootprintCollisionChecker::isPoseValid(const Pose2d& pose) const {
    const double resolution = map_.resolution() > 0.0 ? map_.resolution() : 1.0;
    const double cell_half = 0.5 * resolution;
    double extent = footprint_.radius;
    if (footprint_.type == FootprintType::Rectangle) {
        extent = std::hypot(0.5 * footprint_.length,
                            0.5 * footprint_.width);
    }

    const int min_x = static_cast<int>(std::floor(
        (pose.x - extent) / resolution)) - 1;
    const int max_x = static_cast<int>(std::ceil(
        (pose.x + extent) / resolution)) + 1;
    const int min_y = static_cast<int>(std::floor(
        (pose.y - extent) / resolution)) - 1;
    const int max_y = static_cast<int>(std::ceil(
        (pose.y + extent) / resolution)) + 1;

    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    const double half_length = 0.5 * footprint_.length;
    const double half_width = 0.5 * footprint_.width;

    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            if (map_.isFree(x, y)) continue;
            // Planner coordinates use integer cell centres; keep the same
            // convention as GridMap and the physics scene.
            const double cell_x = static_cast<double>(x) * resolution;
            const double cell_y = static_cast<double>(y) * resolution;
            const double dx = cell_x - pose.x;
            const double dy = cell_y - pose.y;

            if (footprint_.type == FootprintType::Circle) {
                const double closest_x = std::max(
                    std::abs(dx) - cell_half, 0.0);
                const double closest_y = std::max(
                    std::abs(dy) - cell_half, 0.0);
                if (closest_x * closest_x + closest_y * closest_y <=
                    footprint_.radius * footprint_.radius) {
                    return false;
                }
                continue;
            }

            // Separating-axis test between the oriented vehicle rectangle and
            // the occupied axis-aligned grid cell.
            const double axes[4][2] = {{c, s}, {-s, c}, {1.0, 0.0},
                                       {0.0, 1.0}};
            bool separated = false;
            for (const auto& axis : axes) {
                const double center_distance = std::abs(
                    dx * axis[0] + dy * axis[1]);
                const double vehicle_projection =
                    half_length * std::abs(c * axis[0] + s * axis[1]) +
                    half_width * std::abs(-s * axis[0] + c * axis[1]);
                const double cell_projection = cell_half *
                    (std::abs(axis[0]) + std::abs(axis[1]));
                if (center_distance > vehicle_projection + cell_projection) {
                    separated = true;
                    break;
                }
            }
            if (!separated) return false;
        }
    }
    return true;
}

bool FootprintCollisionChecker::isSegmentValid(const Point2d& p1,
                                                const Point2d& p2) const {
    return isPoseSegmentValid({p1.x, p1.y, 0.0},
                              {p2.x, p2.y, 0.0});
}

bool FootprintCollisionChecker::isPathValid(
    const std::vector<Point2d>& path) const {
    if (path.empty()) return true;
    if (!isStateValid(path.front())) return false;
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (!isSegmentValid(path[i - 1], path[i])) return false;
    }
    return true;
}

bool FootprintCollisionChecker::isPoseSegmentValid(const Pose2d& p1,
                                                   const Pose2d& p2) const {
    const double distance = std::hypot(p2.x - p1.x, p2.y - p1.y);
    const int samples = std::max(2, static_cast<int>(std::ceil(distance / 0.5)));
    double heading_delta = p2.theta - p1.theta;
    heading_delta = std::atan2(std::sin(heading_delta),
                               std::cos(heading_delta));

    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        Pose2d pose;
        pose.x = p1.x + t * (p2.x - p1.x);
        pose.y = p1.y + t * (p2.y - p1.y);
        pose.theta = p1.theta + t * heading_delta;
        if (!isPoseValid(pose)) return false;
    }
    return true;
}

bool FootprintCollisionChecker::isPosePathValid(
    const std::vector<Pose2d>& path) const {
    if (path.empty()) return true;
    if (!isPoseValid(path.front())) return false;
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (!isPoseSegmentValid(path[i - 1], path[i])) return false;
    }
    return true;
}

}  // namespace autoplanner
