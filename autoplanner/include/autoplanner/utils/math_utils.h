#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoplanner {

// Clamp a value to [lo, hi].
template <typename T>
inline T clamp(T value, T lo, T hi) {
    return std::max(lo, std::min(hi, value));
}

// Linearly interpolate between a and b where t is in [0, 1].
inline double lerp(double a, double b, double t) {
    return a + t * (b - a);
}

// Normalize an angle to [-pi, pi].
inline double normalizeAngle(double rad) {
    rad = std::fmod(rad, 2.0 * M_PI);
    if (rad > M_PI) rad -= 2.0 * M_PI;
    else if (rad < -M_PI) rad += 2.0 * M_PI;
    return rad;
}

// Convert degrees to radians.
inline double degToRad(double deg) { return deg * M_PI / 180.0; }

// Convert radians to degrees.
inline double radToDeg(double rad) { return rad * 180.0 / M_PI; }

// Check if two doubles are nearly equal.
inline bool nearEqual(double a, double b, double epsilon = 1e-9) {
    return std::abs(a - b) < epsilon;
}

}  // namespace autoplanner
