#include "autoplanner/collision/collision_checker.h"

#include <algorithm>
#include <cmath>

namespace autoplanner {

bool CollisionChecker::isPoseValid(const Pose2d& pose) const {
    return isStateValid({pose.x, pose.y});
}

bool CollisionChecker::isPoseSegmentValid(const Pose2d& p1,
                                           const Pose2d& p2) const {
    const double dx = p2.x - p1.x;
    const double dy = p2.y - p1.y;
    const double distance = std::hypot(dx, dy);
    const int samples = std::max(2, static_cast<int>(std::ceil(distance / 0.5)));

    double heading_delta = p2.theta - p1.theta;
    heading_delta = std::atan2(std::sin(heading_delta),
                               std::cos(heading_delta));
    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        Pose2d pose;
        pose.x = p1.x + t * dx;
        pose.y = p1.y + t * dy;
        pose.theta = p1.theta + t * heading_delta;
        if (!isPoseValid(pose)) return false;
    }
    return true;
}

bool CollisionChecker::isPosePathValid(const std::vector<Pose2d>& path) const {
    if (path.empty()) return true;
    if (!isPoseValid(path.front())) return false;
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (!isPoseSegmentValid(path[i - 1], path[i])) return false;
    }
    return true;
}

}  // namespace autoplanner
