#include "autoplanner/heuristics/reeds_shepp.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace autoplanner {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwopi = 2.0 * kPi;
constexpr double kHalfPi = kPi / 2.0;

// Normalize angle to [-pi, pi].
double mod2pi(double theta) {
    double v = std::fmod(theta, kTwopi);
    if (v < -kPi) v += kTwopi;
    else if (v >= kPi) v -= kTwopi;
    return v;
}

// Polar conversion: (x, y) -> (r, theta)
void polar(double x, double y, double& r, double& theta) {
    r = std::sqrt(x * x + y * y);
    theta = std::atan2(y, x);
}

// Time-optimal straight line + turn + straight line + turn + straight line.
struct RSPath {
    double lengths[5] = {0, 0, 0, 0, 0};  // t, u, v, w, x in Reeds-Shepp
    double total = std::numeric_limits<double>::infinity();
    // Segment types: 1=L, -1=R, 0=S
    int types[5] = {0, 0, 0, 0, 0};
    int num_segments = 0;
};

// CSC: curve-straight-curve
RSPath csc(double ta, double tb, double len) {
    RSPath path;
    path.num_segments = 3;
    path.lengths[0] = ta;
    path.lengths[1] = len;
    path.lengths[2] = tb;
    path.total = std::abs(ta) + std::abs(len) + std::abs(tb);
    return path;
}

// CCC: curve-curve-curve
RSPath ccc(double ta, double tb, double tc) {
    RSPath path;
    path.num_segments = 3;
    path.lengths[0] = ta;
    path.lengths[1] = tb;
    path.lengths[2] = tc;
    path.total = std::abs(ta) + std::abs(tb) + std::abs(tc);
    return path;
}

// Transform the goal pose into the coordinate frame of the start pose.
// Returns (x, y, phi) in the unit circle (rho=1).
void transform(double x0, double y0, double t0,
               double x1, double y1, double t1,
               double& x, double& y, double& phi) {
    double dx = x1 - x0;
    double dy = y1 - y0;
    x =  dx * std::cos(t0) + dy * std::sin(t0);
    y = -dx * std::sin(t0) + dy * std::cos(t0);
    phi = t1 - t0;
    phi = mod2pi(phi);
}

// TimeFlip: reflect y and phi (corresponds to time reversal).
void timeflip(double& x, double& y, double& phi) {
    y = -y;
    phi = -phi;
    phi = mod2pi(phi);
}

// Reflect: swap left/right turns (x mirror).
void reflect(double& x, double& y, double& phi) {
    y = -y;
    phi = -phi;
    phi = mod2pi(phi);
}

// L+S+L+ and R+S+R+ (and time-flipped variants L-S-L-, etc.)
std::vector<RSPath> csc_candidates(double x, double y, double phi) {
    std::vector<RSPath> paths;

    // --- L+S+L+ ---
    {
        double u, t, v;
        polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
        if (t >= 0.0) {
            v = mod2pi(phi - t);
            if (v >= 0.0) paths.push_back(csc(t, u, v));
        }
    }
    // --- L+S+R+ ---
    {
        double u1, t1;
        polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
        u1 = u1 * u1;
        if (u1 >= 4.0) {
            double u = std::sqrt(u1 - 4.0);
            double theta = std::atan2(2.0, u);
            double t = mod2pi(t1 + theta);
            double v = mod2pi(t - phi);
            if (t >= 0.0 && v >= 0.0) paths.push_back(csc(t, u, v));
        }
    }
    // --- L+S+R- ---
    {
        double u1, t1;
        polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
        u1 = u1 * u1;
        if (u1 >= 4.0) {
            double u = std::sqrt(u1 - 4.0);
            double theta = std::atan2(2.0, u);
            double t = mod2pi(t1 - theta);
            double v = mod2pi(t - phi);
            if (t >= 0.0 && v <= 0.0) paths.push_back(csc(t, u, v));
        }
    }
    return paths;
}

// CCC: L+R-L+ and R+L-R+ (and flipped variants)
std::vector<RSPath> ccc_candidates(double x, double y, double phi) {
    std::vector<RSPath> paths;

    // L+R-L+ (backward R segment)
    {
        double u1, t1;
        polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u1, t1);
        if (u1 <= 4.0) {
            double u = -2.0 * std::asin(u1 / 4.0);
            double t = mod2pi(t1 + u / 2.0 + kPi);
            double v = mod2pi(phi - t + u);
            if (t >= 0.0 && u <= 0.0)
                paths.push_back(ccc(t, u, v));
        }
    }
    // L+R+L- (backward L segment)
    {
        double u1, t1;
        polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u1, t1);
        if (u1 <= 4.0) {
            double u = -2.0 * std::asin(u1 / 4.0);
            double t = mod2pi(t1 + u / 2.0 + kPi);
            double v = mod2pi(phi - t + u);
            if (t <= 0.0 && u >= 0.0)
                paths.push_back(ccc(t, u, v));
        }
    }
    return paths;
}

// C|C|C and C|CC and CC|C candidates
std::vector<RSPath> ccc_extra(double x, double y, double phi) {
    std::vector<RSPath> paths;

    // case 6: L+R-L+ invariant under timeflip
    {
        double u1, t1;
        polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u1, t1);
        if (u1 <= 4.0) {
            double u = -2.0 * std::asin(u1 / 4.0);
            double t = mod2pi(t1 + u / 2.0 + kPi);
            double v = mod2pi(phi - t + u);
            if (t >= 0.0 && u <= 0.0)
                paths.push_back(ccc(t, u, v));
            // timeflip version
            if (t <= 0.0 && u >= 0.0)
                paths.push_back(ccc(-t, -u, -v));
        }
    }

    return paths;
}

void add_best(RSPath& best, const RSPath& candidate) {
    if (candidate.total < best.total)
        best = candidate;
}

double reedsSheppDistance(double x, double y, double phi) {
    RSPath best;

    // Try all 4 quadrant symmetries
    for (int i = 0; i < 4; ++i) {
        // Forward
        {
            auto cs = csc_candidates(x, y, phi);
            for (auto& p : cs) {
                p.types[0] = 1; p.types[1] = 0; p.types[2] = 1;
                add_best(best, p);
            }
            // Reflect (swap L/R)
            double rx = x, ry = y, rphi = phi;
            reflect(rx, ry, rphi);
            cs = csc_candidates(rx, ry, rphi);
            for (auto& p : cs) {
                p.types[0] = -1; p.types[1] = 0; p.types[2] = -1;
                add_best(best, p);
            }
        }
        // Time flip
        timeflip(x, y, phi);
    }

    // CCC candidates in all symmetries
    for (int i = 0; i < 4; ++i) {
        {
            auto cs = ccc_candidates(x, y, phi);
            for (auto& p : cs) {
                p.types[0] = 1; p.types[1] = -1; p.types[2] = 1;
                add_best(best, p);
            }
            double rx = x, ry = y, rphi = phi;
            reflect(rx, ry, rphi);
            cs = ccc_candidates(rx, ry, rphi);
            for (auto& p : cs) {
                p.types[0] = -1; p.types[1] = 1; p.types[2] = -1;
                add_best(best, p);
            }
        }
        timeflip(x, y, phi);
    }

    return best.total;
}

}  // anonymous namespace

ReedsSheppHeuristic::ReedsSheppHeuristic(double rho) : rho_(rho) {}

double ReedsSheppHeuristic::compute(const Point2i& current,
                                    const Point2i& goal) const {
    // For grid-based heuristics without orientation, use Euclidean fallback
    double dx = static_cast<double>(goal.x - current.x);
    double dy = static_cast<double>(goal.y - current.y);
    return std::sqrt(dx * dx + dy * dy);
}

double ReedsSheppHeuristic::distance(
    double x0, double y0, double t0,
    double x1, double y1, double t1) const {
    double x, y, phi;
    transform(x0, y0, t0, x1, y1, t1, x, y, phi);
    // Scale to unit circle
    x /= rho_;
    y /= rho_;
    return reedsSheppDistance(x, y, phi) * rho_;
}

}  // namespace autoplanner
