#include "robotnav/perception/occupancy_grid.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace robotnav::perception {

OccupancyGrid::OccupancyGrid(OccupancyGridOptions options)
    : options_(std::move(options)) {
    if (options_.width == 0 || options_.height == 0 ||
        options_.width > std::numeric_limits<std::size_t>::max() /
            options_.height ||
        options_.frame_id.empty() ||
        !std::isfinite(options_.known_threshold) ||
        options_.known_threshold <= 0.0 ||
        !std::isfinite(options_.free_update) ||
        options_.free_update >= 0.0 ||
        !std::isfinite(options_.occupied_update) ||
        options_.occupied_update <= 0.0 ||
        !std::isfinite(options_.minimum_log_odds) ||
        !std::isfinite(options_.maximum_log_odds) ||
        options_.minimum_log_odds >= options_.maximum_log_odds ||
        !std::isfinite(options_.ray_samples_per_cell) ||
        options_.ray_samples_per_cell <= 0.0) {
        throw std::invalid_argument("invalid occupancy grid options");
    }
    const auto cell_count = options_.width * options_.height;
    log_odds_.assign(cell_count, 0.0);
    observed_.assign(cell_count, 0);
}

bool OccupancyGrid::isInside(int x, int y) const noexcept {
    return x >= 0 && y >= 0 &&
        static_cast<std::size_t>(x) < options_.width &&
        static_cast<std::size_t>(y) < options_.height;
}

std::size_t OccupancyGrid::index(int x, int y) const noexcept {
    return static_cast<std::size_t>(y) * options_.width +
        static_cast<std::size_t>(x);
}

bool OccupancyGrid::isObserved(int x, int y) const noexcept {
    return isInside(x, y) && observed_[index(x, y)] != 0;
}

double OccupancyGrid::logOdds(int x, int y) const noexcept {
    return isInside(x, y)
        ? log_odds_[index(x, y)]
        : std::numeric_limits<double>::quiet_NaN();
}

OccupancyState OccupancyGrid::state(int x, int y) const noexcept {
    if (!isInside(x, y) || !isObserved(x, y)) {
        return OccupancyState::Unknown;
    }
    return log_odds_[index(x, y)] >= options_.known_threshold
        ? OccupancyState::Occupied : OccupancyState::Free;
}

void OccupancyGrid::setPrior(int x, int y, OccupancyState cell_state,
                             double strength) {
    if (!isInside(x, y) || !std::isfinite(strength) || strength <= 0.0 ||
        cell_state == OccupancyState::Unknown) {
        throw std::invalid_argument("invalid occupancy prior");
    }
    const auto cell_index = index(x, y);
    observed_[cell_index] = 1;
    log_odds_[cell_index] = cell_state == OccupancyState::Occupied
        ? strength : -strength;
    log_odds_[cell_index] = std::clamp(
        log_odds_[cell_index], options_.minimum_log_odds,
        options_.maximum_log_odds);
}

int OccupancyGrid::binaryState(int x, int y,
                               UnknownCellPolicy policy) const noexcept {
    const auto cell_state = state(x, y);
    if (cell_state == OccupancyState::Occupied) return 1;
    if (cell_state == OccupancyState::Unknown &&
        policy == UnknownCellPolicy::Occupied) {
        return 1;
    }
    return 0;
}

void OccupancyGrid::updateCell(int x, int y, bool occupied) {
    if (!isInside(x, y)) return;
    const auto cell_index = index(x, y);
    observed_[cell_index] = 1;
    log_odds_[cell_index] = std::clamp(
        log_odds_[cell_index] +
            (occupied ? options_.occupied_update : options_.free_update),
        options_.minimum_log_odds, options_.maximum_log_odds);
}

OccupancyIntegrationResult OccupancyGrid::integrate(
    const SensorFrame& frame, UnknownCellPolicy unknown_policy) {
    const auto frame_validation = validate(frame);
    if (!frame_validation) return {frame_validation, {}};
    if (frame.frame_id != options_.frame_id) {
        return {{ValidationError::InconsistentFrameId}, {}};
    }

    const auto before = binaryGrid(unknown_policy);
    const auto observed_before = observed_;
    for (const auto& point : frame.points) {
        const double dx = point.position.x - frame.sensor_pose.x;
        const double dy = point.position.y - frame.sensor_pose.y;
        const auto samples = std::max<std::size_t>(
            1, static_cast<std::size_t>(std::ceil(
                std::max(std::abs(dx), std::abs(dy)) *
                options_.ray_samples_per_cell)));
        int previous_x = std::numeric_limits<int>::min();
        int previous_y = std::numeric_limits<int>::min();
        for (std::size_t sample = 0; sample <= samples; ++sample) {
            const double ratio = static_cast<double>(sample) /
                static_cast<double>(samples);
            const int x = static_cast<int>(std::floor(
                frame.sensor_pose.x + ratio * dx));
            const int y = static_cast<int>(std::floor(
                frame.sensor_pose.y + ratio * dy));
            const bool endpoint = sample == samples;
            if (x == previous_x && y == previous_y) {
                if (endpoint && point.hit) updateCell(x, y, true);
                continue;
            }
            previous_x = x;
            previous_y = y;
            updateCell(x, y, endpoint && point.hit);
        }
    }

    const auto after = binaryGrid(unknown_policy);
    OccupancyIntegrationResult result;
    for (std::size_t y = 0; y < options_.height; ++y) {
        for (std::size_t x = 0; x < options_.width; ++x) {
            const auto cell_index = y * options_.width + x;
            if (before[cell_index] != after[cell_index] ||
                observed_before[cell_index] != observed_[cell_index]) {
                result.changed_cells.push_back({
                    static_cast<int>(x), static_cast<int>(y)});
            }
        }
    }
    return result;
}

std::vector<int> OccupancyGrid::binaryGrid(
    UnknownCellPolicy unknown_policy) const {
    std::vector<int> result(options_.width * options_.height);
    for (std::size_t y = 0; y < options_.height; ++y) {
        for (std::size_t x = 0; x < options_.width; ++x) {
            result[y * options_.width + x] = binaryState(
                static_cast<int>(x), static_cast<int>(y), unknown_policy);
        }
    }
    return result;
}

}  // namespace robotnav::perception
