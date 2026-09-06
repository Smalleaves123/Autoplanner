#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "autoplanner/core/point.h"
#include "autoplanner/core/pose.h"

namespace robotnav::perception {

// Version 1 is the first stable in-memory and replay contract. Adapters must
// reject unknown versions instead of silently interpreting newer fields.
constexpr std::uint32_t kSchemaVersion = 1;
using TimestampNs = std::int64_t;

struct Covariance2d {
    double xx = 0.0;
    double xy = 0.0;
    double yy = 0.0;
};

enum class ObservationKind : std::uint8_t {
    Unknown,
    StaticObstacle,
    DynamicObstacle,
};

constexpr std::string_view toString(ObservationKind kind) noexcept {
    switch (kind) {
        case ObservationKind::Unknown: return "unknown";
        case ObservationKind::StaticObstacle: return "static";
        case ObservationKind::DynamicObstacle: return "dynamic";
    }
    return "unknown";
}

// A 2-D beam endpoint expressed in SensorFrame::frame_id. A miss is the
// observed free-space endpoint at maximum range rather than an obstacle hit.
struct SensorPoint {
    autoplanner::Point2d position;
    bool hit = true;
    ObservationKind kind = ObservationKind::Unknown;
    Covariance2d covariance;
};

struct SensorFrame {
    std::uint32_t schema_version = kSchemaVersion;
    std::uint64_t sequence = 0;
    TimestampNs timestamp_ns = 0;
    std::string frame_id = "map";
    std::string source_id;
    autoplanner::Pose2d sensor_pose;
    std::vector<SensorPoint> points;
};

enum class TrackState : std::uint8_t {
    Tentative,
    Confirmed,
    Coasting,
};

constexpr std::string_view toString(TrackState state) noexcept {
    switch (state) {
        case TrackState::Tentative: return "tentative";
        case TrackState::Confirmed: return "confirmed";
        case TrackState::Coasting: return "coasting";
    }
    return "tentative";
}

// The kinematic estimate is valid at timestamp_ns. The last observation may
// be older when a tracker is coasting through one or more missing frames.
struct ObstacleTrack {
    std::uint64_t track_id = 0;
    TimestampNs timestamp_ns = 0;
    TimestampNs last_observation_timestamp_ns = 0;
    std::string frame_id = "map";
    autoplanner::Point2d position;
    autoplanner::Point2d velocity;
    autoplanner::Point2d acceleration;
    double radius = 0.0;
    Covariance2d position_covariance;
    double confidence = 1.0;
    std::uint32_t age = 1;
    std::uint32_t hits = 1;
    std::uint32_t missed_observations = 0;
    TrackState state = TrackState::Tentative;
};

// A coherent snapshot: every track estimate is projected to timestamp_ns and
// expressed in frame_id. Individual last-observation times remain available.
struct ObstacleTrackFrame {
    std::uint32_t schema_version = kSchemaVersion;
    std::uint64_t sequence = 0;
    TimestampNs timestamp_ns = 0;
    std::string frame_id = "map";
    std::string source_id;
    std::vector<ObstacleTrack> tracks;
};

enum class ValidationError : std::uint8_t {
    None,
    UnsupportedSchemaVersion,
    InvalidTimestamp,
    EmptyFrameId,
    EmptySourceId,
    NonFiniteValue,
    InvalidCovariance,
    InvalidObservationKind,
    InvalidTrackId,
    InvalidTrackState,
    InvalidRadius,
    InvalidConfidence,
    InvalidTrackCounters,
    DuplicateTrackId,
    InconsistentTrackFrame,
};

constexpr std::string_view toString(ValidationError error) noexcept {
    switch (error) {
        case ValidationError::None: return "none";
        case ValidationError::UnsupportedSchemaVersion:
            return "unsupported_schema_version";
        case ValidationError::InvalidTimestamp: return "invalid_timestamp";
        case ValidationError::EmptyFrameId: return "empty_frame_id";
        case ValidationError::EmptySourceId: return "empty_source_id";
        case ValidationError::NonFiniteValue: return "non_finite_value";
        case ValidationError::InvalidCovariance: return "invalid_covariance";
        case ValidationError::InvalidObservationKind:
            return "invalid_observation_kind";
        case ValidationError::InvalidTrackId: return "invalid_track_id";
        case ValidationError::InvalidTrackState: return "invalid_track_state";
        case ValidationError::InvalidRadius: return "invalid_radius";
        case ValidationError::InvalidConfidence: return "invalid_confidence";
        case ValidationError::InvalidTrackCounters:
            return "invalid_track_counters";
        case ValidationError::DuplicateTrackId: return "duplicate_track_id";
        case ValidationError::InconsistentTrackFrame:
            return "inconsistent_track_frame";
    }
    return "non_finite_value";
}

struct ValidationResult {
    ValidationError error = ValidationError::None;
    std::size_t element_index = std::numeric_limits<std::size_t>::max();

    constexpr bool valid() const noexcept {
        return error == ValidationError::None;
    }

    constexpr explicit operator bool() const noexcept { return valid(); }
};

bool isValidCovariance(const Covariance2d& covariance) noexcept;
ValidationResult validate(const SensorFrame& frame) noexcept;
ValidationResult validate(const ObstacleTrack& track) noexcept;
ValidationResult validate(const ObstacleTrackFrame& frame) noexcept;

}  // namespace robotnav::perception
