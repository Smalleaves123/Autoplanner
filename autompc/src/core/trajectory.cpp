#include "autompc/core/trajectory.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

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

bool loadPathCsv(const std::string& file_path, double velocity,
                 Trajectory& trajectory) {
    trajectory.clear();

    std::ifstream fin(file_path);
    if (!fin.is_open()) return false;

    std::string line;
    std::vector<TrajectoryPoint> raw;
    while (std::getline(fin, line)) {
        if (line.empty() || line == "x,y") continue;

        std::stringstream ss(line);
        std::string x_text;
        std::string y_text;
        if (!std::getline(ss, x_text, ',') || !std::getline(ss, y_text)) {
            trajectory.clear();
            return false;
        }

        try {
            raw.push_back({std::stod(x_text), std::stod(y_text),
                           0.0, velocity});
        } catch (...) {
            trajectory.clear();
            return false;
        }
    }

    if (raw.empty()) return false;

    // Controllers need enough reference samples to identify both straight
    // segments and turns. Resample the planner polyline at a fixed spatial
    // interval instead of exposing sparse shortcut waypoints directly.
    constexpr double kSampleSpacing = 0.5;
    trajectory.push_back(raw.front());
    for (std::size_t i = 1; i < raw.size(); ++i) {
        const double x0 = raw[i - 1].x;
        const double y0 = raw[i - 1].y;
        const double dx = raw[i].x - x0;
        const double dy = raw[i].y - y0;
        const double length = std::sqrt(dx * dx + dy * dy);
        const int samples = std::max(1, static_cast<int>(
            std::ceil(length / kSampleSpacing)));

        for (int sample = 1; sample <= samples; ++sample) {
            const double t = static_cast<double>(sample) /
                             static_cast<double>(samples);
            trajectory.push_back({x0 + t * dx, y0 + t * dy,
                                  0.0, velocity});
        }
    }

    for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
        const double dx = trajectory[i + 1].x - trajectory[i].x;
        const double dy = trajectory[i + 1].y - trajectory[i].y;
        trajectory[i].theta = std::atan2(dy, dx);
    }
    if (trajectory.size() > 1) {
        trajectory.back().theta = trajectory[trajectory.size() - 2].theta;
    }

    return true;
}

}  // namespace autompc
