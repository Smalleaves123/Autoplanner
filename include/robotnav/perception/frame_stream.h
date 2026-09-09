#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "robotnav/perception/types.h"

namespace robotnav::perception {

enum class BufferOverflowPolicy : std::uint8_t {
    RejectNewest,
    DropOldest,
};

struct FrameDeliveryResult {
    ValidationResult validation;
    std::size_t consumer_count = 0;

    explicit operator bool() const noexcept { return validation.valid(); }
};

struct FrameBufferResult {
    ValidationResult validation;
    bool accepted = false;
    bool dropped_oldest = false;

    explicit operator bool() const noexcept {
        return validation.valid() && accepted;
    }
};

// Synchronous, transport-independent fan-out. Validation occurs before any
// callback runs. The callback snapshot is invoked without holding the lock,
// so consumers may subscribe or unsubscribe from inside a callback.
template <typename Frame>
class FrameDispatcher {
public:
    using Callback = std::function<void(const Frame&)>;
    using Subscription = std::uint64_t;

    Subscription subscribe(Callback callback) {
        if (!callback) return 0;
        std::lock_guard<std::mutex> lock(mutex_);
        const auto subscription = next_subscription_++;
        callbacks_.emplace(subscription, std::move(callback));
        return subscription;
    }

    bool unsubscribe(Subscription subscription) {
        std::lock_guard<std::mutex> lock(mutex_);
        return callbacks_.erase(subscription) != 0;
    }

    FrameDeliveryResult publish(const Frame& frame) const {
        FrameDeliveryResult result;
        result.validation = validate(frame);
        if (!result.validation) return result;

        std::vector<Callback> callbacks;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            callbacks.reserve(callbacks_.size());
            for (const auto& entry : callbacks_) {
                callbacks.push_back(entry.second);
            }
        }
        result.consumer_count = callbacks.size();
        for (const auto& callback : callbacks) callback(frame);
        return result;
    }

    std::size_t subscriberCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return callbacks_.size();
    }

private:
    mutable std::mutex mutex_;
    std::map<Subscription, Callback> callbacks_;
    Subscription next_subscription_ = 1;
};

// A small pull adapter for bridging producer callbacks to polling consumers.
// It is safe for one or more producer and consumer threads. Frames are copied
// deliberately so producers retain ownership of their input objects.
template <typename Frame>
class BufferedFrameStream {
public:
    explicit BufferedFrameStream(
        std::size_t capacity,
        BufferOverflowPolicy overflow_policy =
            BufferOverflowPolicy::DropOldest)
        : capacity_(capacity), overflow_policy_(overflow_policy) {
        if (capacity_ == 0) {
            throw std::invalid_argument("frame stream capacity must be positive");
        }
    }

    FrameBufferResult push(const Frame& frame) {
        FrameBufferResult result;
        result.validation = validate(frame);
        if (!result.validation) return result;

        std::lock_guard<std::mutex> lock(mutex_);
        if (frames_.size() == capacity_) {
            if (overflow_policy_ == BufferOverflowPolicy::RejectNewest) {
                ++rejected_count_;
                return result;
            }
            // Append first so a copy/allocation failure leaves the existing
            // buffer intact.
            frames_.push_back(frame);
            frames_.pop_front();
            ++dropped_count_;
            result.dropped_oldest = true;
        } else {
            frames_.push_back(frame);
        }
        result.accepted = true;
        return result;
    }

    std::optional<Frame> pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (frames_.empty()) return std::nullopt;
        Frame frame = std::move(frames_.front());
        frames_.pop_front();
        return frame;
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return frames_.size();
    }

    std::size_t droppedCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_count_;
    }

    std::size_t rejectedCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return rejected_count_;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.clear();
    }

private:
    const std::size_t capacity_;
    const BufferOverflowPolicy overflow_policy_;
    mutable std::mutex mutex_;
    std::deque<Frame> frames_;
    std::size_t dropped_count_ = 0;
    std::size_t rejected_count_ = 0;
};

using SensorFrameDispatcher = FrameDispatcher<SensorFrame>;
using ObstacleTrackFrameDispatcher = FrameDispatcher<ObstacleTrackFrame>;
using BufferedSensorFrameStream = BufferedFrameStream<SensorFrame>;
using BufferedObstacleTrackFrameStream =
    BufferedFrameStream<ObstacleTrackFrame>;

}  // namespace robotnav::perception
