#include <gtest/gtest.h>
#include <cmath>
#include <fstream>

#include "autompc/autompc.h"

using namespace autompc;

TEST(Trajectory, CircleHasCorrectSize) {
    auto t = makeCircle(5.0, 1.0, 100);
    EXPECT_EQ(t.size(), 100u);
}

TEST(Trajectory, StraightLine) {
    auto t = makeStraightLine(0, 0, 10, 0, 2.0, 11);
    EXPECT_EQ(t.size(), 11u);
    EXPECT_NEAR(t[0].x, 0.0, 1e-9);
    EXPECT_NEAR(t.back().x, 10.0, 1e-9);
}

TEST(Trajectory, ArcLength) {
    auto t = makeStraightLine(0, 0, 3, 4, 1.0, 2);
    EXPECT_NEAR(arcLength(t), 5.0, 1e-9);
}

TEST(Trajectory, Interpolate) {
    auto t = makeStraightLine(0, 0, 10, 0, 1.0, 11);
    auto pt = interpolate(t, 5.0);
    EXPECT_NEAR(pt.x, 5.0, 1e-9);
    EXPECT_NEAR(pt.y, 0.0, 1e-9);
}

TEST(Trajectory, LoadPathCsv) {
    const std::string path = "/tmp/autompc_test_path.csv";
    std::ofstream out(path);
    out << "x,y\n0,0\n3,0\n3,4\n";
    out.close();

    Trajectory trajectory;
    ASSERT_TRUE(loadPathCsv(path, 2.0, trajectory));
    ASSERT_GT(trajectory.size(), 3u);
    EXPECT_DOUBLE_EQ(trajectory.front().x, 0.0);
    EXPECT_DOUBLE_EQ(trajectory.back().x, 3.0);
    EXPECT_DOUBLE_EQ(trajectory.back().y, 4.0);
    EXPECT_DOUBLE_EQ(trajectory[0].theta, 0.0);
    EXPECT_NEAR(trajectory[trajectory.size() / 2].theta, M_PI_2, 1e-9);
    EXPECT_DOUBLE_EQ(trajectory[2].v, 2.0);
}

TEST(Kinematics, Step) {
    State s{0, 0, 0, 1.0};
    Control u{1.0, 0.0};
    s = step(s, u, 0.1);
    EXPECT_NEAR(s.x, 0.1, 1e-9);
    EXPECT_NEAR(s.y, 0.0, 1e-9);
}

TEST(PIDController, ZeroError) {
    PIDController pid;
    TrajectoryPoint ref{0, 0, 0, 1.0};
    State s{0, 0, 0, 1.0};
    auto u = pid.compute(s, ref, 0.1);
    EXPECT_NEAR(u.steering, 0.0, 1e-6);
}

TEST(PurePursuit, Basic) {
    PurePursuitController pp(2.0);
    auto ref = makeStraightLine(0, 0, 10, 0, 1.0, 11);
    State s{0, 0.5, 0, 1.0};  // offset to the left
    auto u = pp.compute(s, ref, 1.0);
    // Should steer right to correct
    EXPECT_LT(u.steering, 0.0);
}

TEST(StanleyController, CrossTrackCorrection) {
    StanleyController sc(0.5);
    TrajectoryPoint ref{1, 0, 0, 1.0};
    State s{0, 1.0, 0, 1.0};  // left of track, parallel heading
    auto u = sc.compute(s, ref, 1.0);
    EXPECT_LT(u.steering, 0.0);  // should steer right
}

TEST(ErrorMetrics, ZeroError) {
    auto ref = makeStraightLine(0, 0, 10, 0, 1.0, 11);
    std::vector<State> actual;
    for (auto& p : ref)
        actual.push_back({p.x, p.y, p.theta, p.v});
    auto err = computeErrors(actual, ref);
    EXPECT_NEAR(err.max_cross_track, 0.0, 1e-9);
}
