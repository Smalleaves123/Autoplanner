#include "robotnav/perception/types.h"

#include <algorithm>
#include <cmath>

namespace robotnav::perception {
namespace {

bool finitePoint(const autoplanner::Point2d& point) noexcept {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

bool finitePose(const autoplanner::Pose2d& pose) noexcept {
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
        std::isfinite(pose.theta);
}

bool validObservationKind(ObservationKind kind) noexcept {
    switch (kind) {
        case ObservationKind::Unknown:
        case ObservationKind::StaticObstacle:
        case ObservationKind::DynamicObstacle:
            return true;
    }
    return false;
}

bool validTrackState(TrackState state) noexcept {
    switch (state) {
        case TrackState::Tentative:
        case TrackState::Confirmed:
        case TrackState::Coasting:
            return true;
    }
    return false;
}

}  // namespace

bool isValidCovariance(const Covariance2d& covariance) noexcept {
    if (!std::isfinite(covariance.xx) ||
        !std::isfinite(covariance.xy) ||
        !std::isfinite(covariance.yy) ||
        covariance.xx < 0.0 || covariance.yy < 0.0) {
        return false;
    }
    const double determinant =
        covariance.xx * covariance.yy - covariance.xy * covariance.xy;
    const double scale = std::max(
        1.0, covariance.xx * covariance.yy);
    return determinant >= -1e-12 * scale;
}

ValidationResult validate(const SensorFrame& frame) noexcept {
    if (frame.schema_version != kSchemaVersion) {
        return {ValidationError::UnsupportedSchemaVersion};
    }
    if (frame.timestamp_ns < 0) {
        return {ValidationError::InvalidTimestamp};
    }
    if (frame.frame_id.empty()) {
        return {ValidationError::EmptyFrameId};
    }
    if (frame.source_id.empty()) {
        return {ValidationError::EmptySourceId};
    }
    if (!finitePose(frame.sensor_pose)) {
        return {ValidationError::NonFiniteValue};
    }
    for (std::size_t index = 0; index < frame.points.size(); ++index) {
        const auto& point = frame.points[index];
        if (!finitePoint(point.position)) {
            return {ValidationError::NonFiniteValue, index};
        }
        if (!validObservationKind(point.kind)) {
            return {ValidationError::InvalidObservationKind, index};
        }
        if (!isValidCovariance(point.covariance)) {
            return {ValidationError::InvalidCovariance, index};
        }
    }
    return {};
}

ValidationResult validate(const ObstacleTrack& track) noexcept {
    if (track.track_id == 0) {
        return {ValidationError::InvalidTrackId};
    }
    if (track.timestamp_ns < 0 ||
        track.last_observation_timestamp_ns < 0 ||
        track.last_observation_timestamp_ns > track.timestamp_ns) {
        return {ValidationError::InvalidTimestamp};
    }
    if (track.frame_id.empty()) {
        return {ValidationError::EmptyFrameId};
    }
    if (!finitePoint(track.position) || !finitePoint(track.velocity) ||
        !finitePoint(track.acceleration)) {
        return {ValidationError::NonFiniteValue};
    }
    if (!std::isfinite(track.radius) || track.radius < 0.0) {
        return {ValidationError::InvalidRadius};
    }
    if (!isValidCovariance(track.position_covariance)) {
        return {ValidationError::InvalidCovariance};
    }
    if (!std::isfinite(track.confidence) || track.confidence < 0.0 ||
        track.confidence > 1.0) {
        return {ValidationError::InvalidConfidence};
    }
    if (track.age == 0 || track.hits == 0 || track.hits > track.age ||
        track.missed_observations > track.age ||
        track.missed_observations > track.age - track.hits) {
        return {ValidationError::InvalidTrackCounters};
    }
    if (!validTrackState(track.state)) {
        return {ValidationError::InvalidTrackState};
    }
    return {};
}

ValidationResult validate(const ObstacleTrackFrame& frame) noexcept {
    if (frame.schema_version != kSchemaVersion) {
        return {ValidationError::UnsupportedSchemaVersion};
    }
    if (frame.timestamp_ns < 0) {
        return {ValidationError::InvalidTimestamp};
    }
    if (frame.frame_id.empty()) {
        return {ValidationError::EmptyFrameId};
    }
    if (frame.source_id.empty()) {
        return {ValidationError::EmptySourceId};
    }

    for (std::size_t index = 0; index < frame.tracks.size(); ++index) {
        const auto& track = frame.tracks[index];
        const auto track_validation = validate(track);
        if (!track_validation) {
            return {track_validation.error, index};
        }
        if (track.timestamp_ns != frame.timestamp_ns ||
            track.frame_id != frame.frame_id) {
            return {ValidationError::InconsistentTrackFrame, index};
        }
        for (std::size_t previous = 0; previous < index; ++previous) {
            if (frame.tracks[previous].track_id == track.track_id) {
                return {ValidationError::DuplicateTrackId, index};
            }
        }
    }
    return {};
}

}  // namespace robotnav::perception
