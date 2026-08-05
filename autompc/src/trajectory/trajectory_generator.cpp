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

void appendWaypoint(Waypoints& waypoints, const Waypoint2d& point) {
    if (waypoints.empty() || distance(waypoints.back(), point) > kEpsilon) {
        waypoints.push_back(point);
    }
}

// Replace geometrically feasible polyline corners with tangent circular arcs.
// This keeps the path position-continuous and limits each inserted turn to
// max_curvature. Corners without enough lead-in/lead-out distance are left in
// place so the quality gate can reject the path rather than silently violating
// the requested vehicle turning radius.
Waypoints roundCorners(const Waypoints& input, double spacing,
                       double max_curvature) {
    if (input.size() < 3 || max_curvature <= kEpsilon) return input;

    // A small radius margin absorbs chord/resampling error when curvature is
    // re-estimated from the emitted discrete trajectory.
    const double radius = 1.001 / max_curvature;
    Waypoints result;
    result.reserve(input.size());
    appendWaypoint(result, input.front());
    for (std::size_t index = 1; index + 1 < input.size(); ++index) {
        const auto& previous = input[index - 1];
        const auto& current = input[index];
        const auto& next = input[index + 1];
        const double in_x = current.x - previous.x;
        const double in_y = current.y - previous.y;
        const double out_x = next.x - current.x;
        const double out_y = next.y - current.y;
        const double in_length = std::hypot(in_x, in_y);
        const double out_length = std::hypot(out_x, out_y);
        if (in_length <= kEpsilon || out_length <= kEpsilon) {
            appendWaypoint(result, current);
            continue;
        }

        const double in_unit_x = in_x / in_length;
        const double in_unit_y = in_y / in_length;
        const double out_unit_x = out_x / out_length;
        const double out_unit_y = out_y / out_length;
        const double cross = in_unit_x * out_unit_y -
                             in_unit_y * out_unit_x;
        const double dot = std::clamp(
            in_unit_x * out_unit_x + in_unit_y * out_unit_y, -1.0, 1.0);
        const double turn = std::atan2(cross, dot);
        const double absolute_turn = std::abs(turn);
        if (absolute_turn <= 1e-4 ||
            std::abs(M_PI - absolute_turn) <= 1e-4) {
            appendWaypoint(result, current);
            continue;
        }

        const double tangent_length = radius * std::tan(absolute_turn * 0.5);
        if (!std::isfinite(tangent_length) || tangent_length <= kEpsilon ||
            tangent_length > 0.45 * std::min(in_length, out_length)) {
            appendWaypoint(result, current);
            continue;
        }

        const Waypoint2d arc_start{
            current.x - in_unit_x * tangent_length,
            current.y - in_unit_y * tangent_length};
        const double sign = turn > 0.0 ? 1.0 : -1.0;
        const Waypoint2d center{
            arc_start.x - sign * in_unit_y * radius,
            arc_start.y + sign * in_unit_x * radius};
        const double initial_angle = std::atan2(arc_start.y - center.y,
                                                arc_start.x - center.x);
        const int arc_samples = std::max(
            1, static_cast<int>(std::ceil(
                   radius * absolute_turn / std::max(spacing, 1e-3))));

        appendWaypoint(result, arc_start);
        for (int sample = 1; sample <= arc_samples; ++sample) {
            const double fraction = static_cast<double>(sample) /
                                    static_cast<double>(arc_samples);
            const double angle = initial_angle + turn * fraction;
            appendWaypoint(result, {center.x + radius * std::cos(angle),
                                    center.y + radius * std::sin(angle)});
        }
    }
    appendWaypoint(result, input.back());
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

std::vector<int> resampleDirections(const Waypoints& waypoints,
                                     const std::vector<int>& directions,
                                     double spacing) {
    if (waypoints.empty() || directions.size() != waypoints.size()) return {};

    std::vector<double> cumulative(waypoints.size(), 0.0);
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        cumulative[i] = cumulative[i - 1] +
                        distance(waypoints[i - 1], waypoints[i]);
    }

    const double total_length = cumulative.back();
    std::vector<int> result;
    result.reserve(static_cast<std::size_t>(total_length / spacing) + 2);
    auto directionAt = [&directions](std::size_t segment) {
        const int direction = directions[segment];
        return direction < 0 ? -1 : direction > 0 ? 1 : 0;
    };
    result.push_back(directionAt(0));
    for (double target = spacing; target < total_length - kEpsilon;
         target += spacing) {
        const auto upper = std::upper_bound(cumulative.begin(), cumulative.end(),
                                            target);
        const std::size_t segment = upper == cumulative.end()
            ? waypoints.size() - 1
            : static_cast<std::size_t>(
                  std::max<std::ptrdiff_t>(1, upper - cumulative.begin())) - 1;
        result.push_back(directionAt(std::min(segment, waypoints.size() - 1)));
    }
    result.push_back(directionAt(waypoints.size() - 1));
    return result;
}

void computeGeometry(Trajectory& trajectory,
                     const std::vector<int>* motion_directions = nullptr) {
    if (trajectory.empty()) return;

    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        const std::size_t previous = i == 0 ? i : i - 1;
        const std::size_t next = std::min(i + 1, trajectory.size() - 1);
        const double dx = trajectory[next].x - trajectory[previous].x;
        const double dy = trajectory[next].y - trajectory[previous].y;
        if (std::hypot(dx, dy) > kEpsilon) {
            trajectory[i].theta = std::atan2(dy, dx);
            if (motion_directions != nullptr &&
                i < motion_directions->size() &&
                (*motion_directions)[i] < 0) {
                trajectory[i].theta += M_PI;
            }
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
                         const TrajectoryOptions& options,
                         const std::vector<int>* motion_directions = nullptr) {
    if (trajectory.empty()) return;

    const double target_velocity = std::abs(options.target_velocity);
    const double max_velocity = std::max(0.0, options.max_velocity);
    const double max_reverse_velocity = options.allow_reverse
        ? std::max(0.0, options.max_reverse_velocity) : max_velocity;
    const double max_acceleration = std::max(0.0, options.max_acceleration);
    const double max_deceleration = std::max(0.0, options.max_deceleration);
    const double max_lateral_acceleration =
        std::max(0.0, options.max_lateral_acceleration);
    const double max_curvature = std::max(0.0, options.max_curvature);

    std::vector<int> directions(trajectory.size(), 1);
    if (motion_directions != nullptr &&
        motion_directions->size() == trajectory.size() &&
        options.allow_reverse) {
        int previous = 1;
        for (std::size_t i = 0; i < directions.size(); ++i) {
            const int requested = (*motion_directions)[i];
            if (requested < 0) previous = -1;
            else if (requested > 0) previous = 1;
            directions[i] = previous;
        }
    }

    std::vector<double> speeds(trajectory.size(), 0.0);
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        const double speed_limit = directions[i] < 0
            ? max_reverse_velocity : max_velocity;
        speeds[i] = std::min(target_velocity, speed_limit);
        if (max_lateral_acceleration > kEpsilon &&
            std::abs(trajectory[i].curvature) > kEpsilon) {
            const double curvature_limit = std::sqrt(
                max_lateral_acceleration / std::abs(trajectory[i].curvature));
            speeds[i] = std::min(speeds[i], curvature_limit);
        }
        if (max_curvature > kEpsilon &&
            std::abs(trajectory[i].curvature) > max_curvature) {
            // An infeasible geometric corner must not be traversed at speed;
            // the pipeline's quality gate reports it as a hard constraint
            // violation so callers can replan or select a larger radius.
            speeds[i] = 0.0;
        }
        if (i > 0 && directions[i] != directions[i - 1]) {
            speeds[i] = 0.0;
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
        trajectory[i].v = directions[i] < 0 ? -speeds[i] : speeds[i];
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
    return generateTrajectory(input, {}, input_options);
}

Trajectory generateTrajectory(const Waypoints& input,
                               const std::vector<int>& input_directions,
                               const TrajectoryOptions& input_options) {
    TrajectoryOptions options = input_options;
    options.sample_spacing = std::max(options.sample_spacing, 1e-3);
    Waypoints waypoints = removeDuplicateWaypoints(input);
    const bool contains_reverse = std::any_of(
        input_directions.begin(), input_directions.end(),
        [](int direction) { return direction < 0; });
    // Direction labels for a reverse path refer to original Hybrid A*
    // primitives. Do not alter that geometry here because it would make gear
    // transitions ambiguous; the pipeline preserves those primitives instead.
    if (!contains_reverse && options.max_curvature > kEpsilon) {
        waypoints = roundCorners(waypoints, options.sample_spacing,
                                 options.max_curvature);
    }
    Trajectory trajectory = resample(waypoints, options.sample_spacing);
    std::vector<int> directions;
    if (contains_reverse && input_directions.size() == input.size()) {
        directions = resampleDirections(input, input_directions,
                                        options.sample_spacing);
        if (directions.size() != trajectory.size()) directions.clear();
    }
    computeGeometry(trajectory, directions.empty() ? nullptr : &directions);
    computeSpeedProfile(trajectory, options,
                        directions.empty() ? nullptr : &directions);
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
