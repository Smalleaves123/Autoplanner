#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "autompc/autompc.h"

using namespace autompc;

namespace {

SimulationOptions constrainedOptions() {
    SimulationOptions options;
    options.dt = 0.1;
    options.wheelbase = 2.0;
    options.max_velocity = 3.0;
    options.max_acceleration = 1.0;
    options.max_deceleration = 2.0;
    options.max_steering = 0.5;
    options.max_steering_rate = 1.0;
    return options;
}

DifferentialDriveOptions constrainedDifferentialOptions() {
    DifferentialDriveOptions options;
    options.dt = 0.1;
    options.track_width = 0.6;
    options.max_linear_velocity = 3.0;
    options.max_reverse_velocity = 0.5;
    options.max_linear_acceleration = 1.0;
    options.max_linear_deceleration = 2.0;
    options.max_angular_velocity = 2.0;
    options.max_angular_acceleration = 1.0;
    options.max_wheel_velocity = 3.0;
    return options;
}

}  // namespace

TEST(KinematicBicycle, LimitsAccelerationAndVelocity) {
    KinematicBicycleSimulator simulator({0.0, 0.0, 0.0, 0.0},
                                        constrainedOptions());

    const auto first = simulator.step({10.0, 0.0});
    const auto second = simulator.step({10.0, 0.0});
    EXPECT_NEAR(first.v, 0.1, 1e-9);
    EXPECT_NEAR(second.v, 0.2, 1e-9);
    EXPECT_LE(simulator.step({10.0, 0.0}).v, 3.0 + 1e-9);
}

TEST(KinematicBicycle, LimitsDeceleration) {
    SimulationOptions options = constrainedOptions();
    KinematicBicycleSimulator simulator({0.0, 0.0, 0.0, 2.0}, options);

    const auto next = simulator.step({0.0, 0.0});
    EXPECT_NEAR(next.v, 1.8, 1e-9);
}

TEST(KinematicBicycle, LimitsSteeringRateAndAngle) {
    const auto options = constrainedOptions();
    KinematicBicycleSimulator simulator({0.0, 0.0, 0.0, 1.0}, options);

    simulator.step({1.0, 1.0});
    EXPECT_NEAR(simulator.steering(), 0.1, 1e-9);
    simulator.step({1.0, 1.0});
    EXPECT_NEAR(simulator.steering(), 0.2, 1e-9);
    for (int i = 0; i < 20; ++i) simulator.step({1.0, 1.0});
    EXPECT_NEAR(simulator.steering(), 0.5, 1e-9);
}

TEST(KinematicBicycle, IntegratesFiniteBicycleMotion) {
    auto options = constrainedOptions();
    options.dt = 0.01;
    KinematicBicycleSimulator simulator({0.0, 0.0, 0.0, 1.0}, options);
    for (int i = 0; i < 100; ++i) simulator.step({1.0, 0.25});

    const auto& state = simulator.state();
    EXPECT_GT(state.x, 0.0);
    EXPECT_GT(state.theta, 0.0);
    EXPECT_TRUE(std::isfinite(state.x));
    EXPECT_TRUE(std::isfinite(state.y));
    EXPECT_TRUE(std::isfinite(state.theta));
    EXPECT_TRUE(std::isfinite(state.v));
}

TEST(KinematicBicycle, ResetClearsActuatorState) {
    auto options = constrainedOptions();
    KinematicBicycleSimulator simulator({0.0, 0.0, 0.0, 1.0}, options);
    simulator.step({1.0, 0.4});
    ASSERT_GT(simulator.steering(), 0.0);

    simulator.reset({2.0, 3.0, 1.0, 0.5});
    EXPECT_DOUBLE_EQ(simulator.state().x, 2.0);
    EXPECT_DOUBLE_EQ(simulator.state().y, 3.0);
    EXPECT_DOUBLE_EQ(simulator.state().theta, 1.0);
    EXPECT_DOUBLE_EQ(simulator.state().v, 0.5);
    EXPECT_DOUBLE_EQ(simulator.steering(), 0.0);
}

TEST(KinematicBicycle, SupportsBoundedReverseVelocityWhenEnabled) {
    auto options = constrainedOptions();
    options.allow_reverse = true;
    options.max_reverse_velocity = 0.5;
    KinematicBicycleSimulator simulator({0.0, 0.0, 0.0, 0.0}, options);

    for (int index = 0; index < 10; ++index) {
        simulator.step({-5.0, 0.0});
    }

    EXPECT_NEAR(simulator.state().v, -0.5, 1e-9);
    EXPECT_LT(simulator.state().x, 0.0);
}

TEST(DifferentialDrive, LimitsLinearAndAngularAcceleration) {
    DifferentialDriveSimulator simulator(
        {0.0, 0.0, 0.0, 0.0}, constrainedDifferentialOptions());

    const auto first = simulator.step({10.0, 10.0});
    EXPECT_NEAR(first.v, 0.1, 1e-9);
    EXPECT_NEAR(simulator.angularVelocity(), 0.1, 1e-9);
    EXPECT_GT(first.x, 0.0);
    EXPECT_GT(first.theta, 0.0);
}

TEST(DifferentialDrive, RotatesInPlaceWithOppositeWheelVelocities) {
    auto options = constrainedDifferentialOptions();
    options.max_angular_acceleration = 100.0;
    DifferentialDriveSimulator simulator({1.0, 2.0, 0.0, 0.0}, options);

    const auto next = simulator.step({0.0, 1.0});
    EXPECT_NEAR(next.x, 1.0, 1e-9);
    EXPECT_NEAR(next.y, 2.0, 1e-9);
    EXPECT_GT(next.theta, 0.0);
    EXPECT_NEAR(simulator.leftWheelVelocity(), -0.3, 1e-9);
    EXPECT_NEAR(simulator.rightWheelVelocity(), 0.3, 1e-9);
}

TEST(DifferentialDrive, WheelSaturationPreservesCurvature) {
    auto options = constrainedDifferentialOptions();
    options.max_linear_acceleration = 100.0;
    options.max_angular_acceleration = 100.0;
    options.max_linear_velocity = 10.0;
    options.max_angular_velocity = 10.0;
    options.track_width = 0.5;
    options.max_wheel_velocity = 2.0;
    DifferentialDriveSimulator simulator({0.0, 0.0, 0.0, 0.0}, options);

    simulator.step({2.0, 4.0});
    EXPECT_NEAR(simulator.leftWheelVelocity(), 2.0 / 3.0, 1e-9);
    EXPECT_NEAR(simulator.rightWheelVelocity(), 2.0, 1e-9);
    EXPECT_NEAR(simulator.angularVelocity() / simulator.state().v, 2.0, 1e-9);
}

TEST(DifferentialDrive, AppliesReversePolicyAndReset) {
    auto options = constrainedDifferentialOptions();
    DifferentialDriveSimulator forward_only({0.0, 0.0, 0.0, 0.0}, options);
    EXPECT_DOUBLE_EQ(forward_only.step({-1.0, 0.0}).v, 0.0);

    options.allow_reverse = true;
    options.max_linear_acceleration = 10.0;
    DifferentialDriveSimulator reversible({0.0, 0.0, 0.0, 0.0}, options);
    EXPECT_NEAR(reversible.step({-5.0, 0.0}).v, -0.5, 1e-9);
    EXPECT_LT(reversible.state().x, 0.0);

    reversible.reset({2.0, 3.0, 1.0, 0.25});
    EXPECT_DOUBLE_EQ(reversible.state().x, 2.0);
    EXPECT_DOUBLE_EQ(reversible.state().v, 0.25);
    EXPECT_DOUBLE_EQ(reversible.angularVelocity(), 0.0);
    EXPECT_DOUBLE_EQ(reversible.leftWheelVelocity(), 0.25);
    EXPECT_DOUBLE_EQ(reversible.rightWheelVelocity(), 0.25);
}

TEST(DifferentialDrive, RejectsInvalidConfigurationAndState) {
    auto options = constrainedDifferentialOptions();
    options.track_width = 0.0;
    EXPECT_THROW(
        DifferentialDriveSimulator({0.0, 0.0, 0.0, 0.0}, options),
        std::invalid_argument);

    options = constrainedDifferentialOptions();
    DifferentialDriveSimulator simulator({0.0, 0.0, 0.0, 0.0}, options);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(simulator.reset({nan, 0.0, 0.0, 0.0}),
                 std::invalid_argument);

    const auto stopped = simulator.step({nan, nan});
    EXPECT_DOUBLE_EQ(stopped.v, 0.0);
    EXPECT_DOUBLE_EQ(simulator.angularVelocity(), 0.0);
}
