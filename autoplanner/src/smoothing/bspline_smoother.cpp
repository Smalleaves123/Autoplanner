#include "autoplanner/smoothing/bspline_smoother.h"

#include <cmath>

namespace autoplanner {

BSplineSmoother::BSplineSmoother(int samples_per_segment)
    : samples_per_segment_(samples_per_segment) {}

std::string BSplineSmoother::name() const {
    return "bspline";
}

static double basis(int i, double t) {
    double u = t + static_cast<double>(i);
    auto N = [](double x) -> double {
        double ax = std::abs(x);
        if (ax < 1.0) {
            double xx = x * x;
            return 0.5 * xx * x - xx + 2.0 / 3.0;
        }
        if (ax < 2.0) {
            double v = 2.0 - ax;
            return v * v * v / 6.0;
        }
        return 0.0;
    };
    return N(u);
}

std::vector<Point2d> BSplineSmoother::smooth(const std::vector<Point2d>& path) {
    if (path.size() < 4) return path;

    std::vector<Point2d> smoothed;
    const int n = static_cast<int>(path.size());

    for (int seg = 1; seg < n - 2; ++seg) {
        for (int s = 0; s < samples_per_segment_; ++s) {
            double t = static_cast<double>(s) / static_cast<double>(samples_per_segment_);
            Point2d pt{0.0, 0.0};
            double total_weight = 0.0;

            for (int k = -1; k <= 2; ++k) {
                int idx = seg + k;
                if (idx < 0) idx = 0;
                if (idx >= n) idx = n - 1;
                double w = basis(k, t);
                pt.x += w * path[static_cast<std::size_t>(idx)].x;
                pt.y += w * path[static_cast<std::size_t>(idx)].y;
                total_weight += w;
            }

            if (total_weight > 1e-9) {
                pt.x /= total_weight;
                pt.y /= total_weight;
            }
            smoothed.push_back(pt);
        }
    }

    if (!path.empty()) smoothed.push_back(path.back());
    return smoothed;
}

}  // namespace autoplanner
