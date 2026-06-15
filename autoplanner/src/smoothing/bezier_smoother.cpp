#include "autoplanner/smoothing/bezier_smoother.h"

namespace autoplanner {

BezierSmoother::BezierSmoother(int samples_per_segment)
    : samples_per_segment_(samples_per_segment) {}

std::string BezierSmoother::name() const {
    return "bezier";
}

static Point2d cubicBezier(const Point2d& p0, const Point2d& p1,
                           const Point2d& p2, const Point2d& p3, double t) {
    double u = 1.0 - t;
    double uu = u * u;
    double tt = t * t;
    Point2d r;
    r.x = uu * u * p0.x + 3.0 * uu * t * p1.x + 3.0 * u * tt * p2.x + tt * t * p3.x;
    r.y = uu * u * p0.y + 3.0 * uu * t * p1.y + 3.0 * u * tt * p2.y + tt * t * p3.y;
    return r;
}

std::vector<Point2d> BezierSmoother::smooth(const std::vector<Point2d>& path) {
    if (path.size() < 4) return path;

    std::vector<Point2d> smoothed;

    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const Point2d& p0 = path[i];
        const Point2d& p3 = path[i + 1];
        Point2d prev = (i > 0) ? path[i - 1] : p0;
        Point2d next = (i + 2 < path.size()) ? path[i + 2] : p3;

        Point2d p1, p2;
        p1.x = p0.x + (p3.x - prev.x) / 3.0;
        p1.y = p0.y + (p3.y - prev.y) / 3.0;
        p2.x = p3.x - (next.x - p0.x) / 3.0;
        p2.y = p3.y - (next.y - p0.y) / 3.0;

        for (int s = 0; s < samples_per_segment_; ++s) {
            double t = static_cast<double>(s) / static_cast<double>(samples_per_segment_);
            smoothed.push_back(cubicBezier(p0, p1, p2, p3, t));
        }
    }
    smoothed.push_back(path.back());
    return smoothed;
}

}  // namespace autoplanner
