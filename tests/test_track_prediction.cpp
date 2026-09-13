#include <cmath>

#include <gtest/gtest.h>

#include "robotnav/perception/track_prediction.h"

namespace perception = robotnav::perception;

namespace {

perception::ObstacleTrack observation(
    perception::TimestampNs timestamp_ns, double x, double velocity_x = 1.0) {
    perception::ObstacleTrack track;
    track.track_id = 1;
    track.timestamp_ns = timestamp_ns;
    track.last_observation_timestamp_ns = timestamp_ns;
    track.frame_id = "map";
    track.position = {x, 0.0};
    track.velocity = {velocity_x, 0.0};
    track.radius = 0.5;
    track.position_covariance = {0.1, 0.0, 0.1};
    track.confidence = 0.9;
    track.state = perception::TrackState::Confirmed;
    return track;
}

}  // namespace

TEST(KalmanTrackPredictorTest, PredictsConstantVelocityAndCovariance) {
    perception::KalmanTrackPredictor predictor;
    EXPECT_EQ(predictor.update(observation(1'000'000'000, 0.0)),
              perception::PredictionStatus::Success);
    const auto prediction = predictor.predict(2'000'000'000);

    ASSERT_TRUE(prediction);
    EXPECT_NEAR(prediction.state.position.x, 1.0, 1e-12);
    EXPECT_NEAR(prediction.state.velocity.x, 1.0, 1e-12);
    EXPECT_GT(prediction.state.position_covariance.xx, 0.1);
    EXPECT_TRUE(perception::isValidCovariance(
        prediction.state.position_covariance));
}

TEST(KalmanTrackPredictorTest, CorrectsPredictionAndRejectsStaleInput) {
    perception::KalmanTrackPredictor predictor;
    ASSERT_EQ(predictor.update(observation(1'000'000'000, 0.0)),
              perception::PredictionStatus::Success);
    ASSERT_EQ(predictor.update(observation(2'000'000'000, 1.2)),
              perception::PredictionStatus::Success);
    const auto prediction = predictor.predict(3'000'000'000);
    ASSERT_TRUE(prediction);
    EXPECT_GT(prediction.state.position.x, 2.0);
    EXPECT_LT(prediction.state.position.x, 2.5);
    EXPECT_EQ(predictor.update(observation(2'000'000'000, 1.2)),
              perception::PredictionStatus::NonMonotonicTimestamp);
}

TEST(KalmanTrackPredictorTest, ReportsLifecycleErrors) {
    perception::KalmanTrackPredictor predictor;
    EXPECT_EQ(predictor.predict(1).status,
              perception::PredictionStatus::NotInitialized);
    auto invalid = observation(1, 0.0);
    invalid.confidence = 2.0;
    EXPECT_EQ(predictor.update(invalid),
              perception::PredictionStatus::InvalidTrack);
    ASSERT_EQ(predictor.update(observation(10, 0.0)),
              perception::PredictionStatus::Success);
    EXPECT_EQ(predictor.predict(9).status,
              perception::PredictionStatus::NonMonotonicTimestamp);
    predictor.reset();
    EXPECT_FALSE(predictor.initialized());
}

TEST(ImmTrackPredictorTest, MaintainsNormalizedModelProbabilities) {
    perception::ImmTrackPredictor predictor;
    ASSERT_EQ(predictor.update(observation(1'000'000'000, 0.0)),
              perception::PredictionStatus::Success);
    ASSERT_EQ(predictor.update(observation(2'000'000'000, 1.0)),
              perception::PredictionStatus::Success);
    const auto prediction = predictor.predict(3'000'000'000);

    ASSERT_TRUE(prediction);
    EXPECT_NEAR(prediction.state.smooth_model_probability +
                    prediction.state.maneuver_model_probability,
                1.0, 1e-12);
    EXPECT_GE(prediction.state.smooth_model_probability, 0.0);
    EXPECT_GE(prediction.state.maneuver_model_probability, 0.0);
    EXPECT_TRUE(perception::isValidCovariance(
        prediction.state.position_covariance));
}

TEST(ImmTrackPredictorTest, RaisesManeuverProbabilityForAbruptMotion) {
    perception::ImmTrackPredictor predictor;
    ASSERT_EQ(predictor.update(observation(1'000'000'000, 0.0, 0.0)),
              perception::PredictionStatus::Success);
    ASSERT_EQ(predictor.update(observation(2'000'000'000, 0.0, 0.0)),
              perception::PredictionStatus::Success);
    const auto before = predictor.predict(2'000'000'000);
    ASSERT_TRUE(before);

    ASSERT_EQ(predictor.update(observation(3'000'000'000, 4.0, 4.0)),
              perception::PredictionStatus::Success);
    const auto after = predictor.predict(3'000'000'000);
    ASSERT_TRUE(after);
    EXPECT_GT(after.state.maneuver_model_probability,
              before.state.maneuver_model_probability);
}
