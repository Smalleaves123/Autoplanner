#pragma once

#include "autoplanner/heuristics/heuristic.h"

namespace autoplanner {

// Reeds-Shepp heuristic — computes the length of the optimal Reeds-Shepp
// curve between two (x, y, theta) poses for a given minimum turning radius.
//
// Reeds-Shepp paths allow forward and backward motion and consist of
// straight segments (S) and maximum-curvature arcs (L = left, R = right).
//
// This heuristic is admissible for Hybrid A* and other kinodynamic planners
// that respect a minimum turning radius.
//
// Reference: Reeds & Shepp, "Optimal paths for a car that goes both
// forwards and backwards", Pacific J. Math, 1990.
class ReedsSheppHeuristic final : public Heuristic {
public:
    // rho: minimum turning radius of the vehicle.
    explicit ReedsSheppHeuristic(double rho = 1.0);

    // Returns the Reeds-Shepp path length in radians (unit circle).
    // Multiply by rho to get metres.
    double compute(const Point2i& current, const Point2i& goal) const override;

    // Compute the full Reeds-Shepp distance between two full poses.
    double distance(double x0, double y0, double t0,
                    double x1, double y1, double t1) const;

    std::string name() const override { return "reeds_shepp"; }

private:
    double rho_;  // minimum turning radius
};

}  // namespace autoplanner
