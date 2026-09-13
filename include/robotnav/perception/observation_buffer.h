#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "robotnav/perception/types.h"

namespace robotnav::perception {

enum class ObservationStatus : std::uint8_t {
    Accepted,
    InvalidFrame,
    InvalidArrivalTimestamp,
    Duplicate,
    StaleSequence,
    TooLate,
    BufferFull,
};

constexpr std::string_view toString(ObservationStatus status) noexcept {
    switch (status) {
        case ObservationStatus::Accepted: return "accepted";
        case ObservationStatus::InvalidFrame: return "invalid_frame";
        case ObservationStatus::InvalidArrivalTimestamp:
            return "invalid_arrival_timestamp";
        case ObservationStatus::Duplicate: return "duplicate";
        case ObservationStatus::StaleSequence: return "stale_sequence";
        case ObservationStatus::TooLate: return "too_late";
        case ObservationStatus::BufferFull: return "buffer_full";
    }
    return "invalid_frame";
}

struct ObservationBufferOptions {
    TimestampNs reorder_window_ns = 100'000'000;
    TimestampNs delayed_threshold_ns = 50'000'000;
    std::size_t capacity = 256;
};

struct ObservationBufferMetrics {
    std::size_t accepted_count = 0;
    std::size_t emitted_count = 0;
    std::size_t delayed_count = 0;
    std::size_t out_of_order_count = 0;
    std::size_t duplicate_count = 0;
    std::size_t stale_sequence_count = 0;
    std::size_t too_late_count = 0;
    std::size_t buffer_full_count = 0;
    std::uint64_t missing_sequence_count = 0;
    TimestampNs maximum_delay_ns = 0;
};

// Reorders validated frames by event timestamp. arrival_timestamp_ns must use
// the same clock domain as Frame::timestamp_ns. Frames become ready after the
// maximum observed timestamp advances beyond the configured watermark.
template <typename Frame>
class ObservationBuffer {
public:
    explicit ObservationBuffer(ObservationBufferOptions options = {})
        : options_(options) {
        if (options_.reorder_window_ns < 0 ||
            options_.delayed_threshold_ns < 0 || options_.capacity == 0) {
            throw std::invalid_argument("invalid observation buffer options");
        }
    }

    ObservationStatus push(const Frame& frame,
                           TimestampNs arrival_timestamp_ns) {
        if (!validate(frame)) return ObservationStatus::InvalidFrame;
        if (arrival_timestamp_ns < frame.timestamp_ns) {
            return ObservationStatus::InvalidArrivalTimestamp;
        }
        const auto emitted = last_emitted_sequence_.find(frame.source_id);
        if (emitted != last_emitted_sequence_.end()) {
            if (frame.sequence == emitted->second) {
                ++metrics_.duplicate_count;
                return ObservationStatus::Duplicate;
            }
            if (frame.sequence < emitted->second) {
                ++metrics_.stale_sequence_count;
                return ObservationStatus::StaleSequence;
            }
        }
        if (has_emitted_timestamp_ &&
            frame.timestamp_ns < last_emitted_timestamp_ns_) {
            ++metrics_.too_late_count;
            return ObservationStatus::TooLate;
        }
        const auto duplicate = std::find_if(
            pending_.begin(), pending_.end(), [&](const auto& queued) {
                return queued.frame.source_id == frame.source_id &&
                    queued.frame.sequence == frame.sequence;
            });
        if (duplicate != pending_.end()) {
            ++metrics_.duplicate_count;
            return ObservationStatus::Duplicate;
        }
        if (pending_.size() == options_.capacity) {
            ++metrics_.buffer_full_count;
            return ObservationStatus::BufferFull;
        }

        if (has_observed_timestamp_ &&
            frame.timestamp_ns < maximum_observed_timestamp_ns_) {
            ++metrics_.out_of_order_count;
        }
        maximum_observed_timestamp_ns_ = has_observed_timestamp_
            ? std::max(maximum_observed_timestamp_ns_, frame.timestamp_ns)
            : frame.timestamp_ns;
        has_observed_timestamp_ = true;
        const TimestampNs delay = arrival_timestamp_ns - frame.timestamp_ns;
        metrics_.maximum_delay_ns = std::max(metrics_.maximum_delay_ns, delay);
        if (delay > options_.delayed_threshold_ns) {
            ++metrics_.delayed_count;
        }

        QueuedFrame queued{frame, arrival_timestamp_ns};
        const auto position = std::upper_bound(
            pending_.begin(), pending_.end(), queued,
            [](const auto& left, const auto& right) {
                if (left.frame.timestamp_ns != right.frame.timestamp_ns) {
                    return left.frame.timestamp_ns < right.frame.timestamp_ns;
                }
                if (left.frame.source_id != right.frame.source_id) {
                    return left.frame.source_id < right.frame.source_id;
                }
                return left.frame.sequence < right.frame.sequence;
            });
        pending_.insert(position, std::move(queued));
        ++metrics_.accepted_count;
        return ObservationStatus::Accepted;
    }

    std::optional<Frame> popReady() {
        if (pending_.empty() || !has_observed_timestamp_) return std::nullopt;
        if (maximum_observed_timestamp_ns_ < options_.reorder_window_ns) {
            return std::nullopt;
        }
        const TimestampNs watermark =
            maximum_observed_timestamp_ns_ - options_.reorder_window_ns;
        if (pending_.front().frame.timestamp_ns > watermark) {
            return std::nullopt;
        }
        return emitFront();
    }

    // Flush is intended for end-of-stream and deterministic replay shutdown.
    std::optional<Frame> flushNext() {
        if (pending_.empty()) return std::nullopt;
        return emitFront();
    }

    std::size_t size() const noexcept { return pending_.size(); }
    const ObservationBufferMetrics& metrics() const noexcept {
        return metrics_;
    }

    void reset() {
        pending_.clear();
        last_emitted_sequence_.clear();
        metrics_ = {};
        has_observed_timestamp_ = false;
        has_emitted_timestamp_ = false;
        maximum_observed_timestamp_ns_ = 0;
        last_emitted_timestamp_ns_ = 0;
    }

private:
    struct QueuedFrame {
        Frame frame;
        TimestampNs arrival_timestamp_ns = 0;
    };

    std::optional<Frame> emitFront() {
        Frame frame = std::move(pending_.front().frame);
        pending_.erase(pending_.begin());
        const auto previous = last_emitted_sequence_.find(frame.source_id);
        if (previous != last_emitted_sequence_.end() &&
            frame.sequence > previous->second &&
            frame.sequence - previous->second > 1) {
            const auto missing = frame.sequence - previous->second - 1;
            metrics_.missing_sequence_count =
                missing > std::numeric_limits<std::uint64_t>::max() -
                              metrics_.missing_sequence_count
                ? std::numeric_limits<std::uint64_t>::max()
                : metrics_.missing_sequence_count + missing;
        }
        last_emitted_sequence_[frame.source_id] = frame.sequence;
        last_emitted_timestamp_ns_ = frame.timestamp_ns;
        has_emitted_timestamp_ = true;
        ++metrics_.emitted_count;
        return frame;
    }

    ObservationBufferOptions options_;
    std::vector<QueuedFrame> pending_;
    std::map<std::string, std::uint64_t> last_emitted_sequence_;
    ObservationBufferMetrics metrics_;
    bool has_observed_timestamp_ = false;
    bool has_emitted_timestamp_ = false;
    TimestampNs maximum_observed_timestamp_ns_ = 0;
    TimestampNs last_emitted_timestamp_ns_ = 0;
};

using SensorObservationBuffer = ObservationBuffer<SensorFrame>;
using TrackObservationBuffer = ObservationBuffer<ObstacleTrackFrame>;

}  // namespace robotnav::perception
