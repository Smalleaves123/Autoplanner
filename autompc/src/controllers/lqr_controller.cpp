#include "autompc/controllers/lqr_controller.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace autompc {

LQRController::LQRController(const Eigen::Vector4d& Q,
                             const Eigen::Vector2d& R,
                             double dt, double wheelbase)
    : dt_(dt), wheelbase_(wheelbase) {
    // Discrete-time LQR: compute feedback gain K via DARE
    // A = I + dt * A_continuous (linearized unicycle)
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    A(0, 3) = dt;  // x += v*cos(theta)*dt  →  ∂x/∂v = cos(theta)*dt ≈ dt
    A(2, 3) = dt * std::tan(0.0) / wheelbase_;  // near zero steer

    // B matrix
    Eigen::Matrix<double, 4, 2> B = Eigen::Matrix<double, 4, 2>::Zero();
    B(3, 0) = dt;                     // ∂v/∂u_vel = dt
    B(2, 1) = dt / wheelbase_;        // ∂theta/∂u_steer = dt/L

    // Solve discrete-time Riccati via eigenvalue decomposition of symplectic matrix
    // For small systems, iterate the Riccati equation to convergence
    Eigen::Matrix4d Qmat = Eigen::Matrix4d::Zero();
    Qmat.diagonal() = Q;
    Eigen::Matrix4d P = Qmat;
    Eigen::Matrix2d Rmat = R.asDiagonal();

    for (int iter = 0; iter < 200; ++iter) {
        // K = (R + B'PB)^{-1} B'PA
        Eigen::Matrix2d S = Rmat + B.transpose() * P * B;
        Eigen::Matrix<double, 2, 4> K_new = S.llt().solve(B.transpose() * P * A);
        // P = Q + A'PA - A'PB K
        Eigen::Matrix4d P_new = Qmat + A.transpose() * P * A
                              - A.transpose() * P * B * K_new;
        if ((P_new - P).norm() < 1e-8) {
            K_ = -K_new;
            break;
        }
        P = P_new;
        if (iter == 199) K_ = -K_new;
    }
}

Control LQRController::compute(const State& s, const TrajectoryPoint& ref) {
    // State error [dx, dy, dtheta, dv]
    double dx = s.x - ref.x;
    double dy = s.y - ref.y;
    double dtheta = std::atan2(std::sin(s.theta - ref.theta),
                               std::cos(s.theta - ref.theta));
    double dv = s.v - ref.v;

    Eigen::Vector4d x_err(dx, dy, dtheta, dv);
    Eigen::Vector2d u = K_ * x_err;

    // Add feedforward: reference velocity for forward term
    Control ctrl;
    ctrl.velocity = ref.v + u(0);
    ctrl.steering = std::max(-0.7, std::min(0.7, u(1)));
    return ctrl;
}

}  // namespace autompc
