#include <gtest/gtest.h>

#include "autompc/controllers/mpc_controller.h"
#include "autompc/core/trajectory.h"

using namespace autompc;

TEST(MPCController, ComputesBoundedControl) {
    MPCController mpc(12, 0.05, 1.0, 1.5, 0.6);
    const auto reference = makeStraightLine(0.0, 0.0, 20.0, 0.0, 1.0, 41);
    const State state{0.0, 1.0, 0.0, 0.0};

    const Control control = mpc.compute(state, reference, 1.0);
    EXPECT_GE(control.velocity, 0.0);
    EXPECT_LE(control.velocity, 1.5);
    EXPECT_GE(control.steering, -0.6);
    EXPECT_LE(control.steering, 0.6);
    EXPECT_LT(control.steering, 0.0);
}

TEST(MPCController, RecedingHorizonReducesLineError) {
    MPCController mpc(15, 0.05, 1.0, 2.0, 0.7);
    const auto reference = makeStraightLine(0.0, 0.0, 30.0, 0.0, 1.0, 61);
    State state{0.0, 1.0, 0.0, 0.0};
    const double initial_error = std::abs(state.y);

    for (int i = 0; i < 40; ++i) {
        const auto control = mpc.compute(state, reference, 1.0);
        state = step(state, control, 0.05);
    }

    EXPECT_LT(std::abs(state.y), initial_error);
}
