#pragma once

#include <cstddef>
#include <functional>
#include <iosfwd>
#include <string>
#include <vector>

#include "robotnav/perception/types.h"

namespace robotnav::perception {

enum class ReplayFormat {
    Csv,
    JsonLines,
};

struct ReplayResult {
    bool success = true;
    std::size_t frame_count = 0;
    std::size_t line = 0;
    std::string message;

    explicit operator bool() const noexcept { return success; }
};

using SensorFrameCallback = std::function<void(const SensorFrame&)>;
using ObstacleTrackFrameCallback =
    std::function<void(const ObstacleTrackFrame&)>;

// Writers validate the complete input before emitting any bytes.
ReplayResult writeSensorFrames(
    std::ostream& output,
    const std::vector<SensorFrame>& frames,
    ReplayFormat format);
ReplayResult writeObstacleTrackFrames(
    std::ostream& output,
    const std::vector<ObstacleTrackFrame>& frames,
    ReplayFormat format);

// Readers consume one complete frame at a time and invoke callbacks only
// after schema and semantic validation succeeds.
ReplayResult replaySensorFrames(
    std::istream& input,
    ReplayFormat format,
    const SensorFrameCallback& callback);
ReplayResult replayObstacleTrackFrames(
    std::istream& input,
    ReplayFormat format,
    const ObstacleTrackFrameCallback& callback);

}  // namespace robotnav::perception
