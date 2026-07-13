#include "autompc/controllers/controllers.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "autompc/core/trajectory.h"

namespace autompc {

// ── PID Controller ──────────────────────────────────────────────────────

PIDController::PIDController(double kp_vel, double ki_vel, double kd_vel,
                             double kp_steer, double ki_steer, double kd_steer,
                             double wheelbase)
    : kp_vel_(kp_vel), ki_vel_(ki_vel), kd_vel_(kd_vel)
    , kp_steer_(kp_steer), ki_steer_(ki_steer), kd_steer_(kd_steer)
    , wheelbase_(wheelbase) {}

Control PIDController::compute(const State& state, const TrajectoryPoint& ref,
                                double dt) {
    // Cross-track error
    double dx = ref.x - state.x;
    double dy = ref.y - state.y;
    double cross_track = -std::sin(state.theta) * dx + std::cos(state.theta) * dy;

    // Heading error
    double heading_err = ref.theta - state.theta;
    heading_err = std::atan2(std::sin(heading_err), std::cos(heading_err));

    // Velocity error (along-track)
    double along_track = std::cos(state.theta) * dx + std::sin(state.theta) * dy;
    double vel_err = ref.v - state.v;
    if (along_track < 0) vel_err = 0.0;  // don't rush if reference is behind

    // Steering error = cross-track + heading
    double steer_err = cross_track + heading_err;

    // PID for velocity
    integral_vel_ += vel_err * dt;
    double vel_deriv = (vel_err - prev_vel_err_) / std::max(dt, 1e-6);
    prev_vel_err_ = vel_err;
    double velocity = kp_vel_ * vel_err + ki_vel_ * integral_vel_ + kd_vel_ * vel_deriv;
    velocity = std::max(0.0, ref.v + velocity);  // feedforward + correction, no reverse

    // PID for steering
    integral_steer_ += steer_err * dt;
    double steer_deriv = (steer_err - prev_steer_err_) / std::max(dt, 1e-6);
    prev_steer_err_ = steer_err;
    double steering = kp_steer_ * steer_err + ki_steer_ * integral_steer_
                      + kd_steer_ * steer_deriv;
    steering = std::max(-0.7, std::min(0.7, steering));

    return {velocity, steering};
}

void PIDController::reset() {
    prev_vel_err_ = prev_steer_err_ = 0.0;
    integral_vel_ = integral_steer_ = 0.0;
}

// ── Pure Pursuit ────────────────────────────────────────────────────────

PurePursuitController::PurePursuitController(double lookahead, double wheelbase)
    : lookahead_(lookahead), wheelbase_(wheelbase) {}

Control PurePursuitController::compute(const State& state,
                                        const Trajectory& traj,
                                        double target_vel) {
    if (traj.empty()) return {};

    // Start searching from the closest point instead of always using the
    // beginning of the trajectory. This matters for long planner paths.
    std::size_t closest_index = 0;
    double closest_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < traj.size(); ++i) {
        double dx = traj[i].x - state.x;
        double dy = traj[i].y - state.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < closest_distance) {
            closest_distance = distance;
            closest_index = i;
        }
    }

    // Find the lookahead point — first point after the closest point that is
    // at least lookahead_ away from the robot.
    TrajectoryPoint lookahead_pt;
    bool found = false;
    for (std::size_t i = closest_index; i < traj.size(); ++i) {
        const auto& p = traj[i];
        double dx = p.x - state.x;
        double dy = p.y - state.y;
        if (std::sqrt(dx * dx + dy * dy) >= lookahead_) {
            lookahead_pt = p;
            found = true;
            break;
        }
    }
    if (!found) lookahead_pt = traj.back();

    // Transform lookahead point to vehicle frame
    double dx = lookahead_pt.x - state.x;
    double dy = lookahead_pt.y - state.y;
    double lx =  std::cos(state.theta) * dx + std::sin(state.theta) * dy;
    double ly = -std::sin(state.theta) * dx + std::cos(state.theta) * dy;

    // Pure pursuit curvature: kappa = 2 * ly / L^2
    double L2 = lx * lx + ly * ly;
    double steering = 0.0;
    if (L2 > 1e-6)
        steering = std::atan2(2.0 * wheelbase_ * ly, L2);

    steering = std::max(-0.7, std::min(0.7, steering));
    return {std::max(0.0, target_vel), steering};
}

// ── Stanley Controller ──────────────────────────────────────────────────

StanleyController::StanleyController(double k, double wheelbase)
    : k_(k), wheelbase_(wheelbase) {}

Control StanleyController::compute(const State& state,
                                    const TrajectoryPoint& ref,
                                    double target_vel) {
    // Heading error
    double heading_err = ref.theta - state.theta;
    heading_err = std::atan2(std::sin(heading_err), std::cos(heading_err));

    // Cross-track error
    double dx = ref.x - state.x;
    double dy = ref.y - state.y;
    double cross_track = -std::sin(state.theta) * dx + std::cos(state.theta) * dy;

    // Stanley formula: delta = heading_err + atan2(k * cross_track, v)
    double steering = heading_err;
    if (std::abs(state.v) > 0.1)
        steering += std::atan2(k_ * cross_track, state.v);

    steering = std::max(-0.7, std::min(0.7, steering));
    return {target_vel, steering};
}

}  // namespace autompc
