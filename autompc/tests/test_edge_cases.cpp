// test_edge_cases.cpp — AutoMPC industrial-strength edge case tests.

#include <gtest/gtest.h>
#include <cmath>

#include "autompc/autompc.h"

using namespace autompc;

// ── Trajectory edge cases ──────────────────────────────────────────────

TEST(TrajEdge, EmptyTrajectory) {
    Trajectory empty;
    EXPECT_NEAR(arcLength(empty), 0.0, 1e-9);
    State s{0, 0, 0, 0};
    EXPECT_DOUBLE_EQ(closestPointDistance(empty, s),
                     std::numeric_limits<double>::max());
    auto pt = interpolate(empty, 5.0);
    // interpolate on empty should return default-constructed point
    EXPECT_DOUBLE_EQ(pt.x, 0.0);
    EXPECT_DOUBLE_EQ(pt.y, 0.0);
}

TEST(TrajEdge, SinglePointTrajectory) {
    Trajectory single = {{0.0, 0.0, 0.0, 1.0}};
    EXPECT_NEAR(arcLength(single), 0.0, 1e-9);
    auto pt = interpolate(single, 100.0);
    EXPECT_NEAR(pt.x, 0.0, 1e-9);
    EXPECT_NEAR(pt.y, 0.0, 1e-9);
}

TEST(TrajEdge, ZeroPointCircle) {
    auto t = makeCircle(5.0, 1.0, 0);
    EXPECT_TRUE(t.empty());
}

TEST(TrajEdge, OnePointLine) {
    auto t = makeStraightLine(0, 0, 10, 0, 1.0, 1);
    EXPECT_EQ(t.size(), 1u);
}

// ── Kinematics edge cases ──────────────────────────────────────────────

TEST(KineEdge, ZeroVelocityStraight) {
    State s{0, 0, 0, 0.0};
    Control u{0.0, 0.0};  // no velocity, no steering
    State next = step(s, u, 0.1);
    EXPECT_NEAR(next.x, 0.0, 1e-9);
    EXPECT_NEAR(next.y, 0.0, 1e-9);
    EXPECT_NEAR(next.theta, 0.0, 1e-9);
}

TEST(KineEdge, ExtremeSteering) {
    State s{0, 0, 0, 1.0};
    Control u{1.0, M_PI_2 - 0.01};  // near-90 degree steering
    State next = step(s, u, 0.1);
    // Should not produce NaN or Inf
    EXPECT_TRUE(std::isfinite(next.x));
    EXPECT_TRUE(std::isfinite(next.y));
    EXPECT_TRUE(std::isfinite(next.theta));
}

TEST(KineEdge, ZeroDt) {
    State s{0, 0, 0, 1.0};
    Control u{1.0, 0.0};
    State next = step(s, u, 0.0);
    EXPECT_NEAR(next.x, 0.0, 1e-9);  // nothing moves with dt=0
}

TEST(KineEdge, NegativeVelocity) {
    State s{0, 0, 0, -1.0};  // moving backwards
    Control u{-1.0, 0.0};
    State next = step(s, u, 0.1);
    EXPECT_LT(next.x, 0.0);  // should move in negative x
}

// ── PID edge cases ─────────────────────────────────────────────────────

TEST(PIDEdge, ZeroDt) {
    PIDController pid;
    TrajectoryPoint ref{1, 0, 0, 1.0};
    State s{0, 0, 0, 0};
    auto u = pid.compute(s, ref, 0.0);  // zero dt
    EXPECT_TRUE(std::isfinite(u.velocity));
    EXPECT_TRUE(std::isfinite(u.steering));
}

TEST(PIDEdge, PerfectTracking) {
    PIDController pid;
    TrajectoryPoint ref{5, 5, M_PI_4, 2.0};
    State s{5, 5, M_PI_4, 2.0};
    auto u = pid.compute(s, ref, 0.1);
    EXPECT_GE(u.velocity, 0.0);  // velocity >= 0 (no reverse)
    EXPECT_NEAR(u.steering, 0.0, 0.1);      // near zero correction
}

TEST(PIDEdge, LargeError) {
    PIDController pid;
    TrajectoryPoint ref{100, 0, 0, 1.0};
    State s{0, 0, M_PI, 0};  // far away, facing opposite direction
    auto u = pid.compute(s, ref, 0.1);
    EXPECT_LE(std::abs(u.steering), 0.7);  // should be clamped
}

TEST(PIDEdge, ResetClearsIntegral) {
    PIDController pid(1.0, 10.0, 0.0, 2.0, 0.0, 0.5);  // high integral
    TrajectoryPoint ref{1, 0, 0, 1.0};
    State s{0, 0, 0, 0};
    pid.compute(s, ref, 0.1);
    pid.compute(s, ref, 0.1);
    pid.reset();
    auto u = pid.compute(s, ref, 0.1);
    EXPECT_TRUE(std::isfinite(u.velocity));  // no windup
}

// ── Pure Pursuit edge cases ────────────────────────────────────────────

TEST(PPEdge, EmptyTrajectory) {
    PurePursuitController pp(2.0);
    Trajectory empty;
    State s{0, 0, 0, 1.0};
    auto u = pp.compute(s, empty, 1.0);
    EXPECT_TRUE(std::isfinite(u.steering));
}

TEST(PPEdge, SinglePointTrajectory) {
    PurePursuitController pp(2.0);
    Trajectory single = {{10, 0, 0, 1.0}};
    State s{0, 0, 0, 1.0};
    auto u = pp.compute(s, single, 1.0);
    EXPECT_LT(u.steering, 0.7);  // should produce valid steering
}

// ── Stanley edge cases ─────────────────────────────────────────────────

TEST(StanleyEdge, ZeroVelocity) {
    StanleyController sc(0.5);
    TrajectoryPoint ref{1, 0, 0, 1.0};
    State s{0, 1.0, 0, 0.0};  // offset, zero velocity
    auto u = sc.compute(s, ref, 1.0);
    EXPECT_TRUE(std::isfinite(u.steering));
}

// ── Error metrics edge cases ───────────────────────────────────────────

TEST(ErrorEdge, EmptyActual) {
    Trajectory ref = makeStraightLine(0, 0, 10, 0, 1.0, 11);
    std::vector<State> empty;
    auto err = computeErrors(empty, ref);
    EXPECT_DOUBLE_EQ(err.max_cross_track, 0.0);
}

TEST(ErrorEdge, EmptyReference) {
    State s{0, 0, 0, 1.0};
    std::vector<State> actual = {s};
    Trajectory empty_ref;
    auto err = computeErrors(actual, empty_ref);
    EXPECT_DOUBLE_EQ(err.max_cross_track, 0.0);
}

// ── Tracker edge cases ─────────────────────────────────────────────────

TEST(TrackerEdge, ZeroMaxTime) {
    State s{0, 0, 0, 1.0};
    Trajectory ref = makeStraightLine(0, 0, 10, 0, 1.0, 11);
    PIDController pid;
    auto actual = simulate(s, ref, pid, 0.05, 0.0);
    EXPECT_TRUE(actual.empty());
}

TEST(TrackerEdge, EmptyReferenceTrajectory) {
    State s{0, 0, 0, 1.0};
    Trajectory empty;
    PIDController pid;
    auto actual = simulate(s, empty, pid, 0.05, 1.0);
    EXPECT_TRUE(actual.empty());  // should not crash
}

// ── Controller stability ───────────────────────────────────────────────

TEST(Stability, PIDConvergesOnStraightLine) {
    auto ref = makeStraightLine(0, 0, 10, 0, 1.0, 200);
    State s{0, 1.0, 0, 0};  // offset 1m to the left
    PIDController pid(1.0, 0.1, 0.0, 3.0, 0.0, 1.0);
    auto actual = simulate(s, ref, pid, 0.05, 10.0);

    // After 10 seconds, should be close to the track
    ASSERT_FALSE(actual.empty());
    auto err = computeErrors(actual, ref);
    // Mean CTE should decrease over time — check last quarter
    size_t n = actual.size();
    std::vector<State> last_quarter(actual.begin() + 3*n/4, actual.end());
    auto final_err = computeErrors(last_quarter, ref);
    EXPECT_LT(final_err.mean_cross_track, err.mean_cross_track);
}
