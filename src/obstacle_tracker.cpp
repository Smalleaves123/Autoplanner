#include "robotnav/perception/obstacle_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace robotnav::perception {
namespace {

constexpr double kNanosecondsPerSecond = 1e9;

bool finitePoint(const autoplanner::Point2d& point) noexcept {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

autoplanner::Point2d predict(const ObstacleTrack& track,
                             TimestampNs timestamp_ns) noexcept {
    const double dt = static_cast<double>(timestamp_ns - track.timestamp_ns) /
        kNanosecondsPerSecond;
    return {
        track.position.x + track.velocity.x * dt +
            0.5 * track.acceleration.x * dt * dt,
        track.position.y + track.velocity.y * dt +
            0.5 * track.acceleration.y * dt * dt};
}

}  // namespace

ValidationResult validate(const DetectionFrame& frame) noexcept {
    if (frame.schema_version != kSchemaVersion) {
        return {ValidationError::UnsupportedSchemaVersion};
    }
    if (frame.timestamp_ns < 0) {
        return {ValidationError::InvalidTimestamp};
    }
    if (frame.frame_id.empty()) return {ValidationError::EmptyFrameId};
    if (frame.source_id.empty()) return {ValidationError::EmptySourceId};
    for (std::size_t index = 0; index < frame.detections.size(); ++index) {
        const auto& detection = frame.detections[index];
        if (!finitePoint(detection.position)) {
            return {ValidationError::NonFiniteValue, index};
        }
        if (!std::isfinite(detection.radius) || detection.radius < 0.0) {
            return {ValidationError::InvalidRadius, index};
        }
        if (!isValidCovariance(detection.position_covariance)) {
            return {ValidationError::InvalidCovariance, index};
        }
        if (!std::isfinite(detection.confidence) ||
            detection.confidence < 0.0 || detection.confidence > 1.0) {
            return {ValidationError::InvalidConfidence, index};
        }
        if (detection.point_count == 0) {
            return {ValidationError::InvalidTrackCounters, index};
        }
    }
    return {};
}

ObstacleTracker::ObstacleTracker(ObstacleTrackerOptions options)
    : options_(std::move(options)) {
    if (!std::isfinite(options_.association_distance) ||
        options_.association_distance <= 0.0 ||
        options_.confirmation_hits == 0 ||
        !std::isfinite(options_.position_blend) ||
        options_.position_blend < 0.0 || options_.position_blend > 1.0 ||
        !std::isfinite(options_.velocity_blend) ||
        options_.velocity_blend < 0.0 || options_.velocity_blend > 1.0 ||
        options_.output_source_id.empty()) {
        throw std::invalid_argument("invalid obstacle tracker options");
    }
}

TrackerUpdateResult ObstacleTracker::update(
    const DetectionFrame& detections) {
    TrackerUpdateResult result;
    result.frame.schema_version = kSchemaVersion;
    result.frame.sequence = detections.sequence;
    result.frame.timestamp_ns = detections.timestamp_ns;
    result.frame.frame_id = detections.frame_id;
    result.frame.source_id = options_.output_source_id;
    result.validation = validate(detections);
    if (!result.validation) return result;
    if (has_timestamp_ && detections.timestamp_ns <= last_timestamp_ns_) {
        result.validation = {ValidationError::InvalidTimestamp};
        return result;
    }
    if (!tracks_.empty() &&
        detections.frame_id != tracks_.front().frame_id) {
        result.validation = {ValidationError::InconsistentFrameId};
        return result;
    }

    std::vector<std::uint8_t> track_matched(tracks_.size(), 0);
    for (const auto& detection : detections.detections) {
        std::size_t best_index = tracks_.size();
        double best_distance = options_.association_distance;
        for (std::size_t index = 0; index < tracks_.size(); ++index) {
            if (track_matched[index] != 0) continue;
            const auto predicted = predict(tracks_[index],
                                           detections.timestamp_ns);
            const double distance = autoplanner::distance(
                predicted, detection.position);
            if (distance < best_distance) {
                best_distance = distance;
                best_index = index;
            }
        }

        if (best_index == tracks_.size()) {
            ObstacleTrack track;
            track.track_id = next_track_id_++;
            track.timestamp_ns = detections.timestamp_ns;
            track.last_observation_timestamp_ns = detections.timestamp_ns;
            track.frame_id = detections.frame_id;
            track.position = detection.position;
            track.radius = detection.radius;
            track.position_covariance = detection.position_covariance;
            track.confidence = detection.confidence;
            track.state = options_.confirmation_hits == 1
                ? TrackState::Confirmed : TrackState::Tentative;
            tracks_.push_back(track);
            track_matched.push_back(1);
            continue;
        }

        auto& track = tracks_[best_index];
        const auto predicted = predict(track, detections.timestamp_ns);
        const double dt = static_cast<double>(
            detections.timestamp_ns - track.timestamp_ns) /
            kNanosecondsPerSecond;
        const autoplanner::Point2d measured_velocity{
            track.velocity.x + (detection.position.x - predicted.x) / dt,
            track.velocity.y + (detection.position.y - predicted.y) / dt};
        track.position = {
            options_.position_blend * detection.position.x +
                (1.0 - options_.position_blend) * predicted.x,
            options_.position_blend * detection.position.y +
                (1.0 - options_.position_blend) * predicted.y};
        track.velocity = {
            options_.velocity_blend * measured_velocity.x +
                (1.0 - options_.velocity_blend) * track.velocity.x,
            options_.velocity_blend * measured_velocity.y +
                (1.0 - options_.velocity_blend) * track.velocity.y};
        track.timestamp_ns = detections.timestamp_ns;
        track.last_observation_timestamp_ns = detections.timestamp_ns;
        track.radius = std::max(track.radius, detection.radius);
        track.position_covariance = detection.position_covariance;
        track.confidence = detection.confidence;
        ++track.age;
        ++track.hits;
        track.missed_observations = 0;
        track.state = track.hits >= options_.confirmation_hits
            ? TrackState::Confirmed : TrackState::Tentative;
        track_matched[best_index] = 1;
    }

    for (std::size_t index = 0; index < tracks_.size(); ++index) {
        if (track_matched[index] != 0) continue;
        auto& track = tracks_[index];
        track.position = predict(track, detections.timestamp_ns);
        track.timestamp_ns = detections.timestamp_ns;
        ++track.age;
        ++track.missed_observations;
        track.state = TrackState::Coasting;
    }
    tracks_.erase(std::remove_if(
        tracks_.begin(), tracks_.end(), [this](const auto& track) {
            return track.missed_observations >
                options_.maximum_missed_observations;
        }), tracks_.end());

    std::sort(tracks_.begin(), tracks_.end(),
              [](const auto& left, const auto& right) {
                  return left.track_id < right.track_id;
              });
    result.frame.tracks = tracks_;
    has_timestamp_ = true;
    last_timestamp_ns_ = detections.timestamp_ns;
    return result;
}

void ObstacleTracker::reset() noexcept {
    tracks_.clear();
    next_track_id_ = 1;
    has_timestamp_ = false;
    last_timestamp_ns_ = 0;
}

}  // namespace robotnav::perception
