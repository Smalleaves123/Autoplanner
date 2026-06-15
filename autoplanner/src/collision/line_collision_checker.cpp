#include "autoplanner/collision/line_collision_checker.h"

#include <cmath>

namespace autoplanner {

LineCollisionChecker::LineCollisionChecker(const GridMap& map)
    : map_(map) {}

void LineCollisionChecker::setCheckResolution(double resolution) {
    check_resolution_ = resolution;
}

bool LineCollisionChecker::isStateValid(const Point2d& p) const {
    int x = static_cast<int>(std::floor(p.x));
    int y = static_cast<int>(std::floor(p.y));
    return map_.isFree(x, y);
}

bool LineCollisionChecker::isSegmentValid(const Point2d& p1, const Point2d& p2) const {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    int samples = std::max(2, static_cast<int>(dist / check_resolution_) + 1);

    for (int i = 0; i <= samples; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(samples);
        Point2d p;
        p.x = p1.x + t * (p2.x - p1.x);
        p.y = p1.y + t * (p2.y - p1.y);
        if (!isStateValid(p)) return false;
    }
    return true;
}

bool LineCollisionChecker::isPathValid(const std::vector<Point2d>& path) const {
    if (path.empty()) return true;
    if (!isStateValid(path.front())) return false;
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (!isSegmentValid(path[i - 1], path[i])) return false;
    }
    return true;
}

}  // namespace autoplanner
