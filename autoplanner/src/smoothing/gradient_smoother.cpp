#include "autoplanner/smoothing/gradient_smoother.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "autoplanner/costmap/distance_transform.h"

namespace autoplanner {

GradientSmoother::GradientSmoother(const GridMap& map,
                                   const CollisionChecker& checker,
                                   double alpha, double beta, double gamma,
                                   int max_iterations, double learning_rate)
    : map_(map), checker_(checker)
    , alpha_(alpha), beta_(beta), gamma_(gamma)
    , max_iterations_(max_iterations), learning_rate_(learning_rate) {

    // Pre-compute distance transform for fast obstacle-distance lookups.
    dt_ = computeDistanceTransform(map_);
    dt_width_ = map_.width();
    dt_height_ = map_.height();
}

std::string GradientSmoother::name() const {
    return "gradient_smoother";
}

double GradientSmoother::distanceToObstacle(const Point2d& p) const {
    const int x = static_cast<int>(std::round(p.x));
    const int y = static_cast<int>(std::round(p.y));
    if (x < 0 || y < 0 || x >= dt_width_ || y >= dt_height_) {
        return 0.0;
    }
    return dt_[static_cast<std::size_t>(y * dt_width_ + x)];
}

Point2d GradientSmoother::distanceGradient(const Point2d& p, double eps) const {
    // Central finite differences.
    double d_x = (distanceToObstacle({p.x + eps, p.y}) -
                  distanceToObstacle({p.x - eps, p.y})) / (2.0 * eps);
    double d_y = (distanceToObstacle({p.x, p.y + eps}) -
                  distanceToObstacle({p.x, p.y - eps})) / (2.0 * eps);
    return {d_x, d_y};
}

std::vector<Point2d> GradientSmoother::smooth(const std::vector<Point2d>& path) {
    if (path.size() <= 2) {
        return path;  // nothing to smooth
    }

    // Work on a mutable copy; keep endpoints fixed.
    std::vector<Point2d> result = path;
    const std::size_t n = result.size();

    // Guard: safety distance in cells below which obstacle cost activates.
    const double safety_dist = 2.0;

    for (int iter = 0; iter < max_iterations_; ++iter) {
        std::vector<Point2d> grad(n, {0.0, 0.0});

        for (std::size_t i = 1; i < n - 1; ++i) {
            // ── Obstacle gradient ──────────────────────────────────────────
            // If the point is too close to an obstacle, push it away along
            // the negative distance gradient (i.e. toward higher distance).
            double dist = distanceToObstacle(result[i]);
            if (dist < safety_dist) {
                Point2d dg = distanceGradient(result[i]);
                double dg_norm = std::sqrt(dg.x * dg.x + dg.y * dg.y);
                if (dg_norm > 1e-9) {
                    // Push away from obstacle: negative gradient direction.
                    double strength = (safety_dist - dist) / safety_dist;
                    grad[i].x += alpha_ * strength * (-dg.x / dg_norm);
                    grad[i].y += alpha_ * strength * (-dg.y / dg_norm);
                }
            }

            // ── Smoothness (curvature) gradient ────────────────────────────
            // Finite-difference approximation of the second derivative:
            //   laplacian ≈ p[i-1] - 2*p[i] + p[i+1]
            // Moving in the negative direction reduces curvature.
            {
                double sx = result[i - 1].x - 2.0 * result[i].x + result[i + 1].x;
                double sy = result[i - 1].y - 2.0 * result[i].y + result[i + 1].y;
                grad[i].x += beta_ * sx;
                grad[i].y += beta_ * sy;
            }

            // ── Length gradient (toward neighbours) ────────────────────────
            // Pull the point toward the midpoint of its neighbours to shorten
            // the path.
            {
                double mx = (result[i - 1].x + result[i + 1].x) * 0.5 - result[i].x;
                double my = (result[i - 1].y + result[i + 1].y) * 0.5 - result[i].y;
                grad[i].x += gamma_ * mx;
                grad[i].y += gamma_ * my;
            }
        }

        // ── Gradient descent step ──────────────────────────────────────────
        double max_move = 0.0;
        for (std::size_t i = 1; i < n - 1; ++i) {
            result[i].x += learning_rate_ * grad[i].x;
            result[i].y += learning_rate_ * grad[i].y;

            // Clamp to map bounds.
            result[i].x = std::max(0.0, std::min(static_cast<double>(dt_width_ - 1),
                                                  result[i].x));
            result[i].y = std::max(0.0, std::min(static_cast<double>(dt_height_ - 1),
                                                  result[i].y));

            double dx = learning_rate_ * grad[i].x;
            double dy = learning_rate_ * grad[i].y;
            max_move = std::max(max_move, std::sqrt(dx * dx + dy * dy));
        }

        // Early exit if converged (all movements below 0.01 cells).
        if (max_move < 0.01) {
            break;
        }
    }

    // Ensure the smoothed path is still valid.
    // If any segment became obstructed, fall back to the original path.
    if (!checker_.isPathValid(result)) {
        return path;
    }

    return result;
}

}  // namespace autoplanner
