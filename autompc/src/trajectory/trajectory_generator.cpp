#include "autompc/trajectory/trajectory_generator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace autompc {
namespace {

constexpr double kEpsilon = 1e-9;

double distance(const Waypoint2d& a, const Waypoint2d& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

Waypoints removeDuplicateWaypoints(const Waypoints& input) {
    Waypoints result;
    result.reserve(input.size());
    for (const auto& point : input) {
        if (result.empty() || distance(result.back(), point) > kEpsilon) {
            result.push_back(point);
        }
    }
    return result;
}

Trajectory resample(const Waypoints& waypoints, double spacing) {
    Trajectory result;
    if (waypoints.empty()) return result;

    result.push_back({waypoints.front().x, waypoints.front().y});
    if (waypoints.size() == 1) return result;

    std::vector<double> cumulative(waypoints.size(), 0.0);
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        cumulative[i] = cumulative[i - 1] +
                        distance(waypoints[i - 1], waypoints[i]);
    }

    const double total_length = cumulative.back();
    if (total_length <= kEpsilon) return result;

    double target = spacing;
    std::size_t segment = 1;
    while (target < total_length - kEpsilon) {
        while (segment < cumulative.size() && cumulative[segment] < target) {
            ++segment;
        }
        if (segment >= waypoints.size()) break;

        const double segment_length = cumulative[segment] - cumulative[segment - 1];
        if (segment_length > kEpsilon) {
            const double t = (target - cumulative[segment - 1]) /
                             segment_length;
            result.push_back({
                waypoints[segment - 1].x +
                    t * (waypoints[segment].x - waypoints[segment - 1].x),
                waypoints[segment - 1].y +
                    t * (waypoints[segment].y - waypoints[segment - 1].y)});
        }
        target += spacing;
    }

    result.push_back({waypoints.back().x, waypoints.back().y});
    return result;
}

void computeGeometry(Trajectory& trajectory) {
    if (trajectory.empty()) return;

    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        const std::size_t previous = i == 0 ? i : i - 1;
        const std::size_t next = std::min(i + 1, trajectory.size() - 1);
        const double dx = trajectory[next].x - trajectory[previous].x;
        const double dy = trajectory[next].y - trajectory[previous].y;
        if (std::hypot(dx, dy) > kEpsilon) {
            trajectory[i].theta = std::atan2(dy, dx);
        } else if (i > 0) {
            trajectory[i].theta = trajectory[i - 1].theta;
        }
    }

    if (trajectory.size() < 3) return;
    for (std::size_t i = 1; i + 1 < trajectory.size(); ++i) {
        const auto& previous = trajectory[i - 1];
        const auto& current = trajectory[i];
        const auto& next = trajectory[i + 1];
        const double a = std::hypot(current.x - previous.x,
                                    current.y - previous.y);
        const double b = std::hypot(next.x - current.x,
                                    next.y - current.y);
        const double c = std::hypot(next.x - previous.x,
                                    next.y - previous.y);
        if (a > kEpsilon && b > kEpsilon && c > kEpsilon) {
            const double cross =
                (current.x - previous.x) * (next.y - current.y) -
                (current.y - previous.y) * (next.x - current.x);
            trajectory[i].curvature = 2.0 * cross / (a * b * c);
        }
    }
}

void computeSpeedProfile(Trajectory& trajectory,
                         const TrajectoryOptions& options) {
    if (trajectory.empty()) return;

    const double target_velocity = std::max(0.0, options.target_velocity);
    const double max_velocity = std::max(0.0, options.max_velocity);
    const double max_acceleration = std::max(0.0, options.max_acceleration);
    const double max_deceleration = std::max(0.0, options.max_deceleration);
    const double max_lateral_acceleration =
        std::max(0.0, options.max_lateral_acceleration);

    std::vector<double> speeds(trajectory.size(),
                               std::min(target_velocity, max_velocity));
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        if (max_lateral_acceleration > kEpsilon &&
            std::abs(trajectory[i].curvature) > kEpsilon) {
            const double curvature_limit = std::sqrt(
                max_lateral_acceleration / std::abs(trajectory[i].curvature));
            speeds[i] = std::min(speeds[i], curvature_limit);
        }
    }

    if (speeds.size() > 1) {
        for (std::size_t i = 1; i < speeds.size(); ++i) {
            const double ds = std::hypot(
                trajectory[i].x - trajectory[i - 1].x,
                trajectory[i].y - trajectory[i - 1].y);
            if (max_acceleration > kEpsilon) {
                const double reachable = std::sqrt(
                    speeds[i - 1] * speeds[i - 1] +
                    2.0 * max_acceleration * ds);
                speeds[i] = std::min(speeds[i], reachable);
            }
        }

        speeds.back() = 0.0;
        for (std::size_t i = speeds.size() - 1; i > 0; --i) {
            const double ds = std::hypot(
                trajectory[i].x - trajectory[i - 1].x,
                trajectory[i].y - trajectory[i - 1].y);
            if (max_deceleration > kEpsilon) {
                const double reachable = std::sqrt(
                    speeds[i] * speeds[i] +
                    2.0 * max_deceleration * ds);
                speeds[i - 1] = std::min(speeds[i - 1], reachable);
            }
        }
    }

    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        trajectory[i].v = speeds[i];
        trajectory[i].acceleration = 0.0;
        if (i > 0) {
            const double ds = std::hypot(
                trajectory[i].x - trajectory[i - 1].x,
                trajectory[i].y - trajectory[i - 1].y);
            if (ds > kEpsilon) {
                trajectory[i].acceleration =
                    (speeds[i] * speeds[i] - speeds[i - 1] * speeds[i - 1]) /
                    (2.0 * ds);
            }
        }
    }
}

}  // namespace

Trajectory generateTrajectory(const Waypoints& input,
                               const TrajectoryOptions& input_options) {
    TrajectoryOptions options = input_options;
    options.sample_spacing = std::max(options.sample_spacing, 1e-3);
    const Waypoints waypoints = removeDuplicateWaypoints(input);
    Trajectory trajectory = resample(waypoints, options.sample_spacing);
    computeGeometry(trajectory);
    computeSpeedProfile(trajectory, options);
    return trajectory;
}

bool loadPathCsv(const std::string& file_path, double velocity,
                 Trajectory& trajectory,
                 const TrajectoryOptions& input_options) {
    trajectory.clear();
    std::ifstream input(file_path);
    if (!input.is_open()) return false;

    Waypoints waypoints;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line == "x,y") continue;

        std::stringstream stream(line);
        std::string x_text;
        std::string y_text;
        if (!std::getline(stream, x_text, ',') ||
            !std::getline(stream, y_text)) {
            return false;
        }

        try {
            waypoints.push_back({std::stod(x_text), std::stod(y_text)});
        } catch (...) {
            return false;
        }
    }

    if (waypoints.empty()) return false;
    TrajectoryOptions options = input_options;
    options.target_velocity = std::max(0.0, velocity);
    trajectory = generateTrajectory(waypoints, options);
    return !trajectory.empty();
}

bool saveTrajectoryCsv(const Trajectory& trajectory,
                       const std::string& file_path) {
    std::ofstream output(file_path);
    if (!output.is_open()) return false;

    output << "x,y,theta,velocity,curvature,acceleration\n";
    output << std::fixed << std::setprecision(6);
    for (const auto& point : trajectory) {
        output << point.x << "," << point.y << ","
               << point.theta << "," << point.v << ","
               << point.curvature << "," << point.acceleration << "\n";
    }
    return true;
}

}  // namespace autompc
