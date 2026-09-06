#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "robotnav/perception/types.h"

namespace robotnav::perception {

struct ObstacleDetection {
    autoplanner::Point2d position;
    double radius = 0.0;
    Covariance2d position_covariance;
    double confidence = 1.0;
    std::size_t point_count = 1;
};

struct DetectionFrame {
    std::uint32_t schema_version = kSchemaVersion;
    std::uint64_t sequence = 0;
    TimestampNs timestamp_ns = 0;
    std::string frame_id = "map";
    std::string source_id;
    std::vector<ObstacleDetection> detections;
};

struct ObstacleTrackerOptions {
    double association_distance = 3.0;
    std::uint32_t maximum_missed_observations = 3;
    std::uint32_t confirmation_hits = 2;
    double position_blend = 0.65;
    double velocity_blend = 0.5;
    std::string output_source_id = "tracker";
};

struct TrackerUpdateResult {
    ValidationResult validation;
    ObstacleTrackFrame frame;

    explicit operator bool() const noexcept { return validation.valid(); }
};

ValidationResult validate(const DetectionFrame& frame) noexcept;

class ObstacleTracker {
public:
    explicit ObstacleTracker(ObstacleTrackerOptions options = {});

    TrackerUpdateResult update(const DetectionFrame& detections);
    void reset() noexcept;

private:
    ObstacleTrackerOptions options_;
    std::vector<ObstacleTrack> tracks_;
    std::uint64_t next_track_id_ = 1;
    bool has_timestamp_ = false;
    TimestampNs last_timestamp_ns_ = 0;
};

}  // namespace robotnav::perception
