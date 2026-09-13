#pragma once

#include <cstdint>
#include <memory>
#include <string_view>

#include "robotnav/perception/types.h"

namespace robotnav::perception {

enum class PredictionStatus : std::uint8_t {
    Success,
    InvalidConfiguration,
    InvalidTrack,
    NotInitialized,
    NonMonotonicTimestamp,
};

constexpr std::string_view toString(PredictionStatus status) noexcept {
    switch (status) {
        case PredictionStatus::Success: return "success";
        case PredictionStatus::InvalidConfiguration:
            return "invalid_configuration";
        case PredictionStatus::InvalidTrack: return "invalid_track";
        case PredictionStatus::NotInitialized: return "not_initialized";
        case PredictionStatus::NonMonotonicTimestamp:
            return "non_monotonic_timestamp";
    }
    return "invalid_configuration";
}

struct PredictedTrackState {
    TimestampNs timestamp_ns = 0;
    autoplanner::Point2d position;
    autoplanner::Point2d velocity;
    Covariance2d position_covariance;
    double smooth_model_probability = 1.0;
    double maneuver_model_probability = 0.0;
};

struct TrackPredictionResult {
    PredictionStatus status = PredictionStatus::Success;
    PredictedTrackState state;

    explicit operator bool() const noexcept {
        return status == PredictionStatus::Success;
    }
};

struct KalmanPredictorOptions {
    double process_acceleration_variance = 0.5;
    double default_measurement_variance = 0.25;
    double initial_velocity_variance = 1.0;
};

class KalmanTrackPredictor {
public:
    explicit KalmanTrackPredictor(KalmanPredictorOptions options = {});
    ~KalmanTrackPredictor();
    KalmanTrackPredictor(KalmanTrackPredictor&&) noexcept;
    KalmanTrackPredictor& operator=(KalmanTrackPredictor&&) noexcept;
    KalmanTrackPredictor(const KalmanTrackPredictor&) = delete;
    KalmanTrackPredictor& operator=(const KalmanTrackPredictor&) = delete;

    PredictionStatus update(const ObstacleTrack& observation);
    TrackPredictionResult predict(TimestampNs timestamp_ns) const;
    void reset() noexcept;
    bool initialized() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct ImmPredictorOptions {
    double smooth_process_acceleration_variance = 0.05;
    double maneuver_process_acceleration_variance = 4.0;
    double default_measurement_variance = 0.25;
    double initial_velocity_variance = 1.0;
    double initial_maneuver_probability = 0.2;
    double smooth_stay_probability = 0.97;
    double maneuver_stay_probability = 0.90;
};

class ImmTrackPredictor {
public:
    explicit ImmTrackPredictor(ImmPredictorOptions options = {});
    ~ImmTrackPredictor();
    ImmTrackPredictor(ImmTrackPredictor&&) noexcept;
    ImmTrackPredictor& operator=(ImmTrackPredictor&&) noexcept;
    ImmTrackPredictor(const ImmTrackPredictor&) = delete;
    ImmTrackPredictor& operator=(const ImmTrackPredictor&) = delete;

    PredictionStatus update(const ObstacleTrack& observation);
    TrackPredictionResult predict(TimestampNs timestamp_ns) const;
    void reset() noexcept;
    bool initialized() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace robotnav::perception
