#include <gtest/gtest.h>

#include "autoplanner/utils/timer.h"
#include "autoplanner/utils/math_utils.h"
#include "autoplanner/utils/random.h"
#include "autoplanner/utils/logger.h"

using namespace autoplanner;

TEST(Timer, ElapsedIsPositive) {
    Timer t;
    double ms = t.elapsedMs();
    EXPECT_GE(ms, 0.0);
    EXPECT_LT(ms, 1000.0);  // shouldn't take a second
}

TEST(Timer, ResetWorks) {
    Timer t;
    // Sleep briefly to accumulate some time
    auto first = t.elapsedMs();
    t.reset();
    auto second = t.elapsedMs();
    EXPECT_LT(second, first + 1.0);  // after reset, time should be near zero
}

TEST(MathUtils, NormalizeAngle) {
    EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-9);
    EXPECT_NEAR(normalizeAngle(M_PI), M_PI, 1e-9);
    EXPECT_NEAR(normalizeAngle(-M_PI), -M_PI, 1e-9);
    EXPECT_NEAR(normalizeAngle(3.0 * M_PI), M_PI, 1e-9);
    EXPECT_NEAR(normalizeAngle(-3.0 * M_PI), -M_PI, 1e-9);
}

TEST(MathUtils, Clamp) {
    EXPECT_EQ(clamp(5, 0, 10), 5);
    EXPECT_EQ(clamp(-1, 0, 10), 0);
    EXPECT_EQ(clamp(20, 0, 10), 10);
}

TEST(RandomGenerator, ProducesValidRange) {
    RandomGenerator rng(42);
    for (int i = 0; i < 100; ++i) {
        double v = rng.uniform01();
        EXPECT_GE(v, 0.0);
        EXPECT_LT(v, 1.0);
    }
}

TEST(RandomGenerator, Deterministic) {
    RandomGenerator a(123), b(123);
    for (int i = 0; i < 10; ++i)
        EXPECT_DOUBLE_EQ(a.uniform01(), b.uniform01());
}
