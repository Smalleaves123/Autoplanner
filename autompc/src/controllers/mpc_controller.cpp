#include "autompc/controllers/mpc_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace autompc {

namespace {

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

std::size_t closestIndex(const Trajectory& trajectory, const State& state) {
    std::size_t best = 0;
    double best_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        const double dx = trajectory[i].x - state.x;
        const double dy = trajectory[i].y - state.y;
        const double distance = dx * dx + dy * dy;
        if (distance < best_distance) {
            best_distance = distance;
            best = i;
        }
    }
    return best;
}

double referenceSteering(const Trajectory& trajectory,
                         std::size_t index,
                         double wheelbase) {
    if (trajectory.size() < 2) return 0.0;

    const std::size_t next = std::min(index + 1, trajectory.size() - 1);
    const std::size_t previous = index == 0 ? index : index - 1;
    const double ds = std::hypot(
        trajectory[next].x - trajectory[previous].x,
        trajectory[next].y - trajectory[previous].y);
    if (ds < 1e-6) return 0.0;

    const double dtheta = wrapAngle(
        trajectory[next].theta - trajectory[previous].theta);
    return std::atan(wheelbase * dtheta / ds);
}

}  // namespace

MPCController::MPCController(int horizon,
                             double dt,
                             double wheelbase,
                             double max_velocity,
                             double max_steering,
                             const Eigen::Vector4d& state_weights,
                             const Eigen::Vector2d& input_weights)
    : horizon_(std::max(1, horizon))
    , dt_(std::max(1e-4, dt))
    , wheelbase_(std::max(1e-4, wheelbase))
    , max_velocity_(std::max(0.0, max_velocity))
    , max_steering_(std::max(0.0, max_steering))
    , state_weights_(state_weights)
    , input_weights_(input_weights) {}

Control MPCController::compute(const State& state,
                               const Trajectory& reference,
                               double target_velocity) {
    if (reference.empty()) return {};

    const std::size_t nearest = closestIndex(reference, state);
    const double fallback_velocity = std::max(0.0, target_velocity);

    std::vector<Eigen::Matrix4d> a_matrices;
    std::vector<Eigen::Matrix<double, 4, 2>> b_matrices;
    a_matrices.reserve(static_cast<std::size_t>(horizon_));
    b_matrices.reserve(static_cast<std::size_t>(horizon_));

    for (int k = 0; k < horizon_; ++k) {
        const std::size_t index = std::min(
            nearest + static_cast<std::size_t>(k), reference.size() - 1);
        const auto& ref = reference[index];
        const double ref_velocity = std::max(0.0, ref.v > 0.0
            ? ref.v : fallback_velocity);
        const double ref_steering = referenceSteering(
            reference, index, wheelbase_);
        const double cos_theta = std::cos(ref.theta);
        const double sin_theta = std::sin(ref.theta);
        const double cos_steering = std::cos(ref_steering);

        Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
        A(0, 2) = -ref_velocity * sin_theta * dt_;
        A(0, 3) = cos_theta * dt_;
        A(1, 2) = ref_velocity * cos_theta * dt_;
        A(1, 3) = sin_theta * dt_;
        A(2, 3) = std::tan(ref_steering) * dt_ / wheelbase_;
        A(3, 3) = 0.0;

        Eigen::Matrix<double, 4, 2> B =
            Eigen::Matrix<double, 4, 2>::Zero();
        B(2, 1) = ref_velocity * dt_ /
                  (wheelbase_ * std::max(1e-4, cos_steering * cos_steering));
        B(3, 0) = 1.0;

        a_matrices.push_back(A);
        b_matrices.push_back(B);
    }

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q.diagonal() = state_weights_;
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R.diagonal() = input_weights_;
    Eigen::Matrix4d P = 2.0 * Q;

    std::vector<Eigen::Matrix<double, 2, 4>> gains(
        static_cast<std::size_t>(horizon_));
    for (int k = horizon_ - 1; k >= 0; --k) {
        const Eigen::Matrix4d& A = a_matrices[static_cast<std::size_t>(k)];
        const Eigen::Matrix<double, 4, 2>& B =
            b_matrices[static_cast<std::size_t>(k)];
        const Eigen::Matrix2d S = R + B.transpose() * P * B;
        const Eigen::Matrix<double, 2, 4> K =
            S.ldlt().solve(B.transpose() * P * A);
        gains[static_cast<std::size_t>(k)] = K;
        P = Q + A.transpose() * P * (A - B * K);
    }

    const auto& ref = reference[nearest];
    const Eigen::Vector4d error(
        state.x - ref.x,
        state.y - ref.y,
        wrapAngle(state.theta - ref.theta),
        state.v - (ref.v > 0.0 ? ref.v : fallback_velocity));
    const Eigen::Vector2d correction =
        -gains.front() * error;

    const double nominal_velocity = ref.v > 0.0 ? ref.v : fallback_velocity;
    const double nominal_steering = referenceSteering(
        reference, nearest, wheelbase_);
    Control control;
    control.velocity = std::clamp(
        nominal_velocity + correction(0), 0.0, max_velocity_);
    control.steering = std::clamp(
        nominal_steering + correction(1), -max_steering_, max_steering_);
    return control;
}

}  // namespace autompc
