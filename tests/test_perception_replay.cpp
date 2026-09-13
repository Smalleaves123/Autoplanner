#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "robotnav/perception/frame_stream.h"
#include "robotnav/perception/replay.h"

namespace perception = robotnav::perception;

namespace {

perception::SensorFrame sensorFrame() {
    perception::SensorFrame frame;
    frame.sequence = 7;
    frame.timestamp_ns = 123456789;
    frame.frame_id = "map,world";
    frame.source_id = "lidar, \"front\"\nprimary";
    frame.sensor_pose = {1.25, 2.5, -0.3};
    frame.points = {{{3.0, 4.0}, true,
                     perception::ObservationKind::DynamicObstacle,
                     {0.1, 0.02, 0.2}},
                    {{5.0, 6.0}, false,
                     perception::ObservationKind::Unknown, {}}};
    return frame;
}

perception::ObstacleTrackFrame trackFrame() {
    perception::ObstacleTrack track;
    track.track_id = 9;
    track.timestamp_ns = 200;
    track.last_observation_timestamp_ns = 190;
    track.frame_id = "map";
    track.position = {2.0, 3.0};
    track.velocity = {0.5, -0.25};
    track.acceleration = {0.1, 0.0};
    track.radius = 0.6;
    track.position_covariance = {0.2, 0.01, 0.3};
    track.confidence = 0.8;
    track.age = 3;
    track.hits = 2;
    track.missed_observations = 1;
    track.state = perception::TrackState::Coasting;

    perception::ObstacleTrackFrame frame;
    frame.sequence = 8;
    frame.timestamp_ns = 200;
    frame.frame_id = "map";
    frame.source_id = "tracker.primary";
    frame.tracks.push_back(track);
    return frame;
}

void expectSensorFrameEqual(const perception::SensorFrame& actual,
                            const perception::SensorFrame& expected) {
    EXPECT_EQ(actual.schema_version, expected.schema_version);
    EXPECT_EQ(actual.sequence, expected.sequence);
    EXPECT_EQ(actual.timestamp_ns, expected.timestamp_ns);
    EXPECT_EQ(actual.frame_id, expected.frame_id);
    EXPECT_EQ(actual.source_id, expected.source_id);
    EXPECT_DOUBLE_EQ(actual.sensor_pose.x, expected.sensor_pose.x);
    EXPECT_DOUBLE_EQ(actual.sensor_pose.y, expected.sensor_pose.y);
    EXPECT_DOUBLE_EQ(actual.sensor_pose.theta, expected.sensor_pose.theta);
    ASSERT_EQ(actual.points.size(), expected.points.size());
    for (std::size_t index = 0; index < actual.points.size(); ++index) {
        EXPECT_DOUBLE_EQ(actual.points[index].position.x,
                         expected.points[index].position.x);
        EXPECT_DOUBLE_EQ(actual.points[index].position.y,
                         expected.points[index].position.y);
        EXPECT_EQ(actual.points[index].hit, expected.points[index].hit);
        EXPECT_EQ(actual.points[index].kind, expected.points[index].kind);
        EXPECT_DOUBLE_EQ(actual.points[index].covariance.xx,
                         expected.points[index].covariance.xx);
        EXPECT_DOUBLE_EQ(actual.points[index].covariance.xy,
                         expected.points[index].covariance.xy);
        EXPECT_DOUBLE_EQ(actual.points[index].covariance.yy,
                         expected.points[index].covariance.yy);
    }
}

}  // namespace

TEST(PerceptionReplayTest, RoundTripsSensorFramesInCsvAndJsonLines) {
    for (const auto format : {perception::ReplayFormat::Csv,
                              perception::ReplayFormat::JsonLines}) {
        std::stringstream stream;
        const auto expected = sensorFrame();
        const auto written = perception::writeSensorFrames(
            stream, {expected}, format);
        ASSERT_TRUE(written) << written.message;
        EXPECT_EQ(written.frame_count, 1u);

        std::vector<perception::SensorFrame> restored;
        const auto replayed = perception::replaySensorFrames(
            stream, format, [&](const auto& frame) {
                restored.push_back(frame);
            });
        ASSERT_TRUE(replayed) << replayed.message;
        ASSERT_EQ(restored.size(), 1u);
        expectSensorFrameEqual(restored.front(), expected);
    }
}

TEST(PerceptionReplayTest, RoundTripsTrackFramesInCsvAndJsonLines) {
    for (const auto format : {perception::ReplayFormat::Csv,
                              perception::ReplayFormat::JsonLines}) {
        std::stringstream stream;
        const auto expected = trackFrame();
        ASSERT_TRUE(perception::writeObstacleTrackFrames(
            stream, {expected}, format));
        std::vector<perception::ObstacleTrackFrame> restored;
        const auto replayed = perception::replayObstacleTrackFrames(
            stream, format, [&](const auto& frame) {
                restored.push_back(frame);
            });
        ASSERT_TRUE(replayed) << replayed.message;
        ASSERT_EQ(restored.size(), 1u);
        ASSERT_EQ(restored.front().tracks.size(), 1u);
        const auto& actual = restored.front().tracks.front();
        const auto& track = expected.tracks.front();
        EXPECT_EQ(actual.track_id, track.track_id);
        EXPECT_EQ(actual.timestamp_ns, track.timestamp_ns);
        EXPECT_DOUBLE_EQ(actual.position.x, track.position.x);
        EXPECT_DOUBLE_EQ(actual.velocity.y, track.velocity.y);
        EXPECT_DOUBLE_EQ(actual.position_covariance.xy,
                         track.position_covariance.xy);
        EXPECT_EQ(actual.state, track.state);
    }
}

TEST(PerceptionReplayTest, StreamsReplayedFramesIntoValidatedBuffer) {
    std::stringstream stream;
    auto first = sensorFrame();
    auto second = sensorFrame();
    second.sequence = 8;
    second.timestamp_ns += 1;
    ASSERT_TRUE(perception::writeSensorFrames(
        stream, {first, second}, perception::ReplayFormat::Csv));

    perception::BufferedSensorFrameStream buffer(2);
    const auto replayed = perception::replaySensorFrames(
        stream, perception::ReplayFormat::Csv,
        [&](const auto& frame) { ASSERT_TRUE(buffer.push(frame)); });
    ASSERT_TRUE(replayed) << replayed.message;
    EXPECT_EQ(replayed.frame_count, 2u);
    EXPECT_EQ(buffer.size(), 2u);
    EXPECT_EQ(buffer.pop()->sequence, 7u);
    EXPECT_EQ(buffer.pop()->sequence, 8u);
}

TEST(PerceptionReplayTest, PreservesEmptyFrames) {
    auto empty_sensor = sensorFrame();
    empty_sensor.points.clear();
    std::stringstream sensor_stream;
    ASSERT_TRUE(perception::writeSensorFrames(
        sensor_stream, {empty_sensor}, perception::ReplayFormat::JsonLines));
    std::size_t sensor_point_count = 1;
    ASSERT_TRUE(perception::replaySensorFrames(
        sensor_stream, perception::ReplayFormat::JsonLines,
        [&](const auto& frame) { sensor_point_count = frame.points.size(); }));
    EXPECT_EQ(sensor_point_count, 0u);

    auto empty_tracks = trackFrame();
    empty_tracks.tracks.clear();
    std::stringstream track_stream;
    ASSERT_TRUE(perception::writeObstacleTrackFrames(
        track_stream, {empty_tracks}, perception::ReplayFormat::Csv));
    std::size_t track_count = 1;
    ASSERT_TRUE(perception::replayObstacleTrackFrames(
        track_stream, perception::ReplayFormat::Csv,
        [&](const auto& frame) { track_count = frame.tracks.size(); }));
    EXPECT_EQ(track_count, 0u);
}

TEST(PerceptionReplayTest, RejectsMalformedAndTruncatedInputWithLine) {
    std::stringstream malformed;
    malformed << "[\"robotnav_perception_replay\",\"1\",\"sensor_frames\"]\n"
              << "[\"sensor_frame\",\"1\",\"7\"]\n";
    const auto malformed_result = perception::replaySensorFrames(
        malformed, perception::ReplayFormat::JsonLines,
        [](const auto&) {});
    EXPECT_FALSE(malformed_result);
    EXPECT_EQ(malformed_result.line, 2u);

    std::stringstream truncated;
    truncated << "robotnav_perception_replay,1,sensor_frames\n"
              << "sensor_frame,1,7,10,map,lidar,0,0,0,1\n";
    const auto truncated_result = perception::replaySensorFrames(
        truncated, perception::ReplayFormat::Csv, [](const auto&) {});
    EXPECT_FALSE(truncated_result);
    EXPECT_EQ(truncated_result.line, 3u);
}

TEST(PerceptionReplayTest, ValidatesBeforeWritingAnyBytes) {
    auto invalid = sensorFrame();
    invalid.source_id.clear();
    std::stringstream stream;
    const auto result = perception::writeSensorFrames(
        stream, {sensorFrame(), invalid}, perception::ReplayFormat::Csv);
    EXPECT_FALSE(result);
    EXPECT_TRUE(stream.str().empty());
}
