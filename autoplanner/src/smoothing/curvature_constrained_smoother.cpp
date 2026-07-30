#include "autoplanner/smoothing/curvature_constrained_smoother.h"

#include <algorithm>
#include <cmath>

namespace autoplanner {
namespace {

constexpr double kEpsilon = 1e-9;

Point2d interpolate(const Point2d& from, const Point2d& to, double t) {
    return {
        from.x + t * (to.x - from.x),
        from.y + t * (to.y - from.y)};
}

bool localUpdateIsValid(const CollisionChecker& checker,
                        const Point2d& previous,
                        const Point2d& candidate,
                        const Point2d& next) {
    return checker.isSegmentValid(previous, candidate) &&
           checker.isSegmentValid(candidate, next);
}

}  // namespace

CurvatureConstrainedSmoother::CurvatureConstrainedSmoother(
    const CollisionChecker& checker,
    double max_curvature,
    int max_iterations,
    double correction_gain)
    : checker_(checker),
      max_curvature_(std::max(max_curvature, kEpsilon)),
      max_iterations_(std::max(0, max_iterations)),
      correction_gain_(std::clamp(correction_gain, 0.0, 1.0)) {}

std::string CurvatureConstrainedSmoother::name() const {
    return "curvature";
}

double discreteCurvature(const Point2d& previous,
                         const Point2d& current,
                         const Point2d& next) {
    const double a = distance(previous, current);
    const double b = distance(current, next);
    const double c = distance(previous, next);
    if (a <= kEpsilon || b <= kEpsilon || c <= kEpsilon) return 0.0;

    const double cross =
        (current.x - previous.x) * (next.y - previous.y) -
        (current.y - previous.y) * (next.x - previous.x);
    return 2.0 * std::abs(cross) / (a * b * c);
}

std::vector<Point2d> CurvatureConstrainedSmoother::smooth(
    const std::vector<Point2d>& path) {
    if (path.size() < 3 || max_iterations_ == 0 ||
        correction_gain_ <= kEpsilon) {
        return path;
    }

    std::vector<Point2d> result = path;
    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
        bool changed = false;
        for (std::size_t index = 1; index + 1 < result.size(); ++index) {
            const auto& previous = result[index - 1];
            const auto& current = result[index];
            const auto& next = result[index + 1];
            if (discreteCurvature(previous, current, next) <=
                max_curvature_) {
                continue;
            }

            const Point2d midpoint{
                0.5 * (previous.x + next.x),
                0.5 * (previous.y + next.y)};
            double gain = correction_gain_;
            for (int attempt = 0; attempt < 6; ++attempt) {
                const Point2d candidate = interpolate(current, midpoint, gain);
                if (localUpdateIsValid(checker_, previous, candidate, next)) {
                    result[index] = candidate;
                    changed = true;
                    break;
                }
                gain *= 0.5;
            }
        }
        if (!changed) break;
    }

    if (!checker_.isPathValid(result)) return path;
    return result;
}

}  // namespace autoplanner
