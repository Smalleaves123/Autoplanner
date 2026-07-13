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
    const double half_cell = 0.5;

    double extent_x = footprint_.radius / resolution;
    double extent_y = extent_x;
    if (footprint_.type == FootprintType::Rectangle) {
        const double half_length = 0.5 * footprint_.length / resolution;
        const double half_width = 0.5 * footprint_.width / resolution;
        extent_x = std::hypot(half_length, half_width);
        extent_y = extent_x;
    }

    const int min_x = static_cast<int>(std::floor(pose.x - extent_x)) - 1;
    const int max_x = static_cast<int>(std::ceil(pose.x + extent_x)) + 1;
    const int min_y = static_cast<int>(std::floor(pose.y - extent_y)) - 1;
    const int max_y = static_cast<int>(std::ceil(pose.y + extent_y)) + 1;

    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    const double half_length = 0.5 * footprint_.length / resolution + half_cell;
    const double half_width = 0.5 * footprint_.width / resolution + half_cell;
    const double radius = footprint_.radius / resolution + half_cell;

    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            const double dx = static_cast<double>(x) - pose.x;
            const double dy = static_cast<double>(y) - pose.y;

            bool covered = false;
            if (footprint_.type == FootprintType::Circle) {
                covered = dx * dx + dy * dy <= radius * radius;
            } else {
                const double local_x = c * dx + s * dy;
                const double local_y = -s * dx + c * dy;
                covered = std::abs(local_x) <= half_length &&
                          std::abs(local_y) <= half_width;
            }

            if (covered && !map_.isFree(x, y)) return false;
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
