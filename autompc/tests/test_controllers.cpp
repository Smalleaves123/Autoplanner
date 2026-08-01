#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <filesystem>

#include "autompc/autompc.h"

namespace {

std::filesystem::path artifactPath(const std::string& filename) {
    const auto directory = std::filesystem::path("test_artifacts") /
                           "autompc";
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    return directory / filename;
}

struct ArtifactCleanup {
    ~ArtifactCleanup() {
        std::error_code error;
        std::filesystem::remove_all(
            std::filesystem::path("test_artifacts") / "autompc", error);
    }
};

const ArtifactCleanup artifact_cleanup{};

}  // namespace

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
    const auto path = artifactPath("autompc_test_path.csv");
    std::ofstream out(path);
    out << "x,y\n0,0\n3,0\n3,4\n";
    out.close();

    Trajectory trajectory;
    ASSERT_TRUE(loadPathCsv(path.string(), 2.0, trajectory));
    ASSERT_GT(trajectory.size(), 3u);
    EXPECT_DOUBLE_EQ(trajectory.front().x, 0.0);
    EXPECT_DOUBLE_EQ(trajectory.back().x, 3.0);
    EXPECT_DOUBLE_EQ(trajectory.back().y, 4.0);
    EXPECT_DOUBLE_EQ(trajectory[0].theta, 0.0);
    EXPECT_NEAR(trajectory[trajectory.size() / 2].theta, M_PI_2, 1e-9);
    EXPECT_DOUBLE_EQ(trajectory[2].v, 2.0);
}

TEST(Trajectory, GeneratesCurvatureAndLimitedSpeedProfile) {
    const Waypoints waypoints = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
    TrajectoryOptions options;
    options.sample_spacing = 0.5;
    options.target_velocity = 2.0;
    options.max_velocity = 2.0;
    options.max_acceleration = 1.0;
    options.max_deceleration = 1.0;
    options.max_lateral_acceleration = 1.0;

    const auto trajectory = generateTrajectory(waypoints, options);
    ASSERT_GT(trajectory.size(), 10u);
    EXPECT_NEAR(trajectory.front().x, 0.0, 1e-9);
    EXPECT_NEAR(trajectory.back().y, 4.0, 1e-9);
    EXPECT_NEAR(trajectory.back().v, 0.0, 1e-9);

    bool found_curvature = false;
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        EXPECT_LE(trajectory[i].v, 2.0 + 1e-9);
        if (std::abs(trajectory[i].curvature) > 1e-6) {
            found_curvature = true;
            EXPECT_LE(trajectory[i].v * trajectory[i].v *
                          std::abs(trajectory[i].curvature),
                      1.0 + 1e-6);
        }
        if (i > 0) {
            const double ds = std::hypot(
                trajectory[i].x - trajectory[i - 1].x,
                trajectory[i].y - trajectory[i - 1].y);
            const double acceleration = trajectory[i].acceleration;
            EXPECT_LE(acceleration, 1.0 + 1e-6);
            EXPECT_GE(acceleration, -1.0 - 1e-6);
            EXPECT_GT(ds, 0.0);
        }
    }
    EXPECT_TRUE(found_curvature);
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
