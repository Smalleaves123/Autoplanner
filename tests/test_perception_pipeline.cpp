#include <gtest/gtest.h>

#include "robotnav/perception/frame_stream.h"
#include "robotnav/perception/obstacle_tracker.h"
#include "robotnav/perception/occupancy_grid.h"

namespace perception = robotnav::perception;

namespace {

perception::SensorFrame scanFrame(bool hit = true) {
    perception::SensorFrame frame;
    frame.timestamp_ns = 1'000'000'000;
    frame.frame_id = "map";
    frame.source_id = "lidar.front";
    frame.sensor_pose = {1.5, 1.5, 0.0};
    frame.points.push_back({
        {4.5, 1.5}, hit, perception::ObservationKind::Unknown, {}});
    return frame;
}

perception::DetectionFrame detectionFrame(
    perception::TimestampNs timestamp_ns, double x) {
    perception::DetectionFrame frame;
    frame.sequence = static_cast<std::uint64_t>(timestamp_ns);
    frame.timestamp_ns = timestamp_ns;
    frame.frame_id = "map";
    frame.source_id = "detector";
    frame.detections.push_back({{x, 2.0}, 0.5, {0.1, 0.0, 0.1}, 0.9, 4});
    return frame;
}

}  // namespace

TEST(OccupancyGridTest, IntegratesHitRayAsFreeAndOccupiedCells) {
    perception::OccupancyGrid grid({6, 4});
    const auto result = grid.integrate(scanFrame());

    ASSERT_TRUE(result);
    EXPECT_FALSE(result.changed_cells.empty());
    EXPECT_EQ(grid.state(1, 1), perception::OccupancyState::Free);
    EXPECT_EQ(grid.state(2, 1), perception::OccupancyState::Free);
    EXPECT_EQ(grid.state(3, 1), perception::OccupancyState::Free);
    EXPECT_EQ(grid.state(4, 1), perception::OccupancyState::Occupied);
}

TEST(OccupancyGridTest, IntegratesMissEndpointAsFree) {
    perception::OccupancyGrid grid({6, 4});
    ASSERT_TRUE(grid.integrate(scanFrame(false)));
    EXPECT_EQ(grid.state(4, 1), perception::OccupancyState::Free);
}

TEST(OccupancyGridTest, RejectsFrameMismatchWithoutMutation) {
    perception::OccupancyGrid grid({6, 4});
    auto frame = scanFrame();
    frame.frame_id = "sensor";
    const auto result = grid.integrate(frame);

    EXPECT_FALSE(result);
    EXPECT_EQ(result.validation.error,
              perception::ValidationError::InconsistentFrameId);
    EXPECT_EQ(grid.state(1, 1), perception::OccupancyState::Unknown);
}

TEST(ObstacleTrackerTest, AssociatesDetectionsAndEstimatesVelocity) {
    perception::ObstacleTracker tracker;
    const auto first = tracker.update(detectionFrame(1'000'000'000, 1.0));
    const auto second = tracker.update(detectionFrame(2'000'000'000, 2.0));

    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    ASSERT_EQ(first.frame.tracks.size(), 1u);
    ASSERT_EQ(second.frame.tracks.size(), 1u);
    EXPECT_EQ(second.frame.tracks.front().track_id,
              first.frame.tracks.front().track_id);
    EXPECT_EQ(second.frame.tracks.front().state,
              perception::TrackState::Confirmed);
    EXPECT_NEAR(second.frame.tracks.front().velocity.x, 0.5, 1e-9);
}

TEST(ObstacleTrackerTest, CoastsAndExpiresMissedTrack) {
    perception::ObstacleTrackerOptions options;
    options.maximum_missed_observations = 1;
    perception::ObstacleTracker tracker(options);
    ASSERT_TRUE(tracker.update(detectionFrame(1'000'000'000, 1.0)));

    auto empty = detectionFrame(2'000'000'000, 0.0);
    empty.detections.clear();
    const auto coasting = tracker.update(empty);
    ASSERT_TRUE(coasting);
    ASSERT_EQ(coasting.frame.tracks.size(), 1u);
    EXPECT_EQ(coasting.frame.tracks.front().state,
              perception::TrackState::Coasting);

    empty.timestamp_ns = 3'000'000'000;
    empty.sequence = 3;
    const auto expired = tracker.update(empty);
    ASSERT_TRUE(expired);
    EXPECT_TRUE(expired.frame.tracks.empty());
}

TEST(ObstacleTrackerTest, RejectsNonMonotonicFrames) {
    perception::ObstacleTracker tracker;
    ASSERT_TRUE(tracker.update(detectionFrame(2'000'000'000, 1.0)));
    const auto stale = tracker.update(detectionFrame(1'000'000'000, 1.0));
    EXPECT_FALSE(stale);
    EXPECT_EQ(stale.validation.error,
              perception::ValidationError::InvalidTimestamp);
}

TEST(FrameDispatcherTest, ValidatesAndFansOutWithoutHoldingLock) {
    perception::SensorFrameDispatcher dispatcher;
    std::size_t first_count = 0;
    std::size_t second_count = 0;
    perception::SensorFrameDispatcher::Subscription second_subscription = 0;
    const auto first_subscription = dispatcher.subscribe(
        [&](const perception::SensorFrame&) {
            ++first_count;
            dispatcher.unsubscribe(second_subscription);
        });
    second_subscription = dispatcher.subscribe(
        [&](const perception::SensorFrame&) { ++second_count; });

    ASSERT_NE(first_subscription, 0u);
    ASSERT_NE(second_subscription, 0u);
    const auto first_delivery = dispatcher.publish(scanFrame());
    const auto second_delivery = dispatcher.publish(scanFrame());
    EXPECT_TRUE(first_delivery);
    EXPECT_EQ(first_delivery.consumer_count, 2u);
    EXPECT_EQ(second_delivery.consumer_count, 1u);
    EXPECT_EQ(first_count, 2u);
    EXPECT_EQ(second_count, 1u);

    auto invalid = scanFrame();
    invalid.source_id.clear();
    EXPECT_FALSE(dispatcher.publish(invalid));
    EXPECT_EQ(first_count, 2u);
}

TEST(BufferedFrameStreamTest, AppliesExplicitOverflowPolicy) {
    perception::BufferedSensorFrameStream drop_oldest(2);
    auto first = scanFrame();
    auto second = scanFrame();
    auto third = scanFrame();
    first.sequence = 1;
    second.sequence = 2;
    third.sequence = 3;
    EXPECT_TRUE(drop_oldest.push(first));
    EXPECT_TRUE(drop_oldest.push(second));
    const auto overflow = drop_oldest.push(third);
    EXPECT_TRUE(overflow);
    EXPECT_TRUE(overflow.dropped_oldest);
    EXPECT_EQ(drop_oldest.droppedCount(), 1u);
    ASSERT_TRUE(drop_oldest.pop());
    EXPECT_EQ(drop_oldest.pop()->sequence, 3u);

    perception::BufferedSensorFrameStream reject_newest(
        1, perception::BufferOverflowPolicy::RejectNewest);
    EXPECT_TRUE(reject_newest.push(first));
    EXPECT_FALSE(reject_newest.push(second));
    EXPECT_EQ(reject_newest.rejectedCount(), 1u);
    EXPECT_EQ(reject_newest.pop()->sequence, 1u);
}
