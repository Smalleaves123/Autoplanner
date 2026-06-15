#include "autompc/core/trajectory.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autompc {

double closestPointDistance(const Trajectory& traj, const State& state) {
    double best = std::numeric_limits<double>::max();
    for (auto& p : traj) {
        double dx = p.x - state.x;
        double dy = p.y - state.y;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < best) best = d;
    }
    return best;
}

TrajectoryPoint interpolate(const Trajectory& traj, double s) {
    if (traj.empty()) return {};
    if (s <= 0.0) return traj.front();
    double accumulated = 0.0;
    for (size_t i = 1; i < traj.size(); ++i) {
        double dx = traj[i].x - traj[i - 1].x;
        double dy = traj[i].y - traj[i - 1].y;
        double seg_len = std::sqrt(dx * dx + dy * dy);
        if (accumulated + seg_len >= s) {
            double t = (s - accumulated) / seg_len;
            TrajectoryPoint p;
            p.x = traj[i - 1].x + t * dx;
            p.y = traj[i - 1].y + t * dy;
            p.theta = std::atan2(dy, dx);
            p.v = traj[i - 1].v + t * (traj[i].v - traj[i - 1].v);
            return p;
        }
        accumulated += seg_len;
    }
    return traj.back();
}

double arcLength(const Trajectory& traj) {
    double len = 0.0;
    for (size_t i = 1; i < traj.size(); ++i) {
        double dx = traj[i].x - traj[i - 1].x;
        double dy = traj[i].y - traj[i - 1].y;
        len += std::sqrt(dx * dx + dy * dy);
    }
    return len;
}

Trajectory makeCircle(double radius, double velocity, int n) {
    Trajectory traj;
    for (int i = 0; i < n; ++i) {
        double angle = 2.0 * M_PI * i / n;
        TrajectoryPoint p;
        p.x = radius * std::cos(angle);
        p.y = radius * std::sin(angle);
        p.theta = angle + M_PI_2;
        p.v = velocity;
        traj.push_back(p);
    }
    return traj;
}

Trajectory makeStraightLine(double x0, double y0, double x1, double y1,
                             double velocity, int n) {
    Trajectory traj;
    double dx = (x1 - x0) / (n - 1);
    double dy = (y1 - y0) / (n - 1);
    double theta = std::atan2(dy, dx);
    for (int i = 0; i < n; ++i) {
        TrajectoryPoint p;
        p.x = x0 + dx * i;
        p.y = y0 + dy * i;
        p.theta = theta;
        p.v = velocity;
        traj.push_back(p);
    }
    return traj;
}

}  // namespace autompc
