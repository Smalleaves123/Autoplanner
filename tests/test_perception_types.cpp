#include <limits>

#include <gtest/gtest.h>

#include "robotnav/perception/types.h"

namespace perception = robotnav::perception;

namespace {

perception::SensorFrame validSensorFrame() {
    perception::SensorFrame frame;
    frame.sequence = 7;
    frame.timestamp_ns = 1'000'000'000;
    frame.frame_id = "map";
    frame.source_id = "lidar.front";
    frame.sensor_pose = {1.0, 2.0, 0.25};
    frame.points.push_back({
        {3.0, 4.0}, true,
        perception::ObservationKind::DynamicObstacle,
        {0.04, 0.01, 0.09}});
    return frame;
}

perception::ObstacleTrack validTrack() {
    perception::ObstacleTrack track;
    track.track_id = 42;
    track.timestamp_ns = 2'000'000'000;
    track.last_observation_timestamp_ns = 1'900'000'000;
    track.frame_id = "map";
    track.position = {3.0, 4.0};
    track.velocity = {0.5, -0.25};
    track.acceleration = {0.0, 0.0};
    track.radius = 0.6;
    track.position_covariance = {0.2, 0.05, 0.3};
    track.confidence = 0.9;
    track.age = 4;
    track.hits = 3;
    track.missed_observations = 1;
    track.state = perception::TrackState::Coasting;
    return track;
}

}  // namespace

TEST(PerceptionTypesTest, AcceptsVersionedSensorFrame) {
    const auto frame = validSensorFrame();
    const auto validation = perception::validate(frame);

    EXPECT_TRUE(validation.valid());
    EXPECT_EQ(perception::toString(frame.points.front().kind), "dynamic");
    EXPECT_EQ(perception::toString(validation.error), "none");
}

TEST(PerceptionTypesTest, RejectsUnsupportedVersionAndInvalidPoint) {
    auto frame = validSensorFrame();
    frame.schema_version = perception::kSchemaVersion + 1;
    EXPECT_EQ(perception::validate(frame).error,
              perception::ValidationError::UnsupportedSchemaVersion);

    frame = validSensorFrame();
    frame.points.front().position.x =
        std::numeric_limits<double>::quiet_NaN();
    const auto invalid_point = perception::validate(frame);
    EXPECT_EQ(invalid_point.error,
              perception::ValidationError::NonFiniteValue);
    EXPECT_EQ(invalid_point.element_index, 0u);

    frame = validSensorFrame();
    frame.points.front().kind =
        static_cast<perception::ObservationKind>(255);
    EXPECT_EQ(perception::validate(frame).error,
              perception::ValidationError::InvalidObservationKind);
}

TEST(PerceptionTypesTest, ValidatesPositiveSemidefiniteCovariance) {
    EXPECT_TRUE(perception::isValidCovariance({1.0, 0.5, 1.0}));
    EXPECT_FALSE(perception::isValidCovariance({1.0, 2.0, 1.0}));
    EXPECT_FALSE(perception::isValidCovariance({-1.0, 0.0, 1.0}));
}

TEST(PerceptionTypesTest, AcceptsCoastingTimestampedTrack) {
    const auto track = validTrack();
    const auto validation = perception::validate(track);

    EXPECT_TRUE(validation);
    EXPECT_EQ(perception::toString(track.state), "coasting");
}

TEST(PerceptionTypesTest, RejectsInvalidTrackHistoryAndConfidence) {
    auto track = validTrack();
    track.last_observation_timestamp_ns = track.timestamp_ns + 1;
    EXPECT_EQ(perception::validate(track).error,
              perception::ValidationError::InvalidTimestamp);

    track = validTrack();
    track.confidence = 1.1;
    EXPECT_EQ(perception::validate(track).error,
              perception::ValidationError::InvalidConfidence);

    track = validTrack();
    track.missed_observations = 2;
    EXPECT_EQ(perception::validate(track).error,
              perception::ValidationError::InvalidTrackCounters);

    track = validTrack();
    track.state = static_cast<perception::TrackState>(255);
    EXPECT_EQ(perception::validate(track).error,
              perception::ValidationError::InvalidTrackState);
}

TEST(PerceptionTypesTest, ValidatesCoherentTrackFrames) {
    perception::ObstacleTrackFrame frame;
    frame.sequence = 8;
    frame.timestamp_ns = 2'000'000'000;
    frame.frame_id = "map";
    frame.source_id = "tracker.primary";
    frame.tracks = {validTrack()};
    EXPECT_TRUE(perception::validate(frame));

    frame.tracks.push_back(validTrack());
    const auto duplicate = perception::validate(frame);
    EXPECT_EQ(duplicate.error,
              perception::ValidationError::DuplicateTrackId);
    EXPECT_EQ(duplicate.element_index, 1u);

    frame.tracks.back().track_id = 43;
    frame.tracks.back().timestamp_ns += 1;
    const auto inconsistent = perception::validate(frame);
    EXPECT_EQ(inconsistent.error,
              perception::ValidationError::InconsistentTrackFrame);
    EXPECT_EQ(inconsistent.element_index, 1u);
}
