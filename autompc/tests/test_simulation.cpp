#include <gtest/gtest.h>

#include <cmath>

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
