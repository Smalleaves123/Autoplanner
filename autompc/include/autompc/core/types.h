#pragma once

#include <cmath>

namespace autompc {

// 2D robot state: position (x, y) and heading (theta).
struct State {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v = 0.0;  // linear velocity
};

// Control command: linear velocity + steering angle.
struct Control {
    double velocity = 0.0;
    double steering = 0.0;  // radians
};

// A point on a reference trajectory with desired velocity.
struct TrajectoryPoint {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v = 0.0;
};

// Unicycle kinematics step.
inline State step(const State& s, const Control& u, double dt) {
    State next = s;
    next.x += s.v * std::cos(s.theta) * dt;
    next.y += s.v * std::sin(s.theta) * dt;
    next.theta += (s.v * std::tan(u.steering)) * dt;
    next.v = u.velocity;
    return next;
}

}  // namespace autompc
