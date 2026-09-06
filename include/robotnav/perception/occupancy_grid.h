#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "autoplanner/core/point.h"
#include "robotnav/perception/types.h"

namespace robotnav::perception {

enum class OccupancyState : std::uint8_t {
    Unknown,
    Free,
    Occupied,
};

enum class UnknownCellPolicy : std::uint8_t {
    Free,
    Occupied,
};

struct OccupancyGridOptions {
    std::size_t width = 0;
    std::size_t height = 0;
    std::string frame_id = "map";
    double known_threshold = 0.5;
    double free_update = -0.7;
    double occupied_update = 1.2;
    double minimum_log_odds = -6.0;
    double maximum_log_odds = 6.0;
    double ray_samples_per_cell = 4.0;
};

struct OccupancyIntegrationResult {
    ValidationResult validation;
    std::vector<autoplanner::Point2i> changed_cells;

    explicit operator bool() const noexcept { return validation.valid(); }
};

class OccupancyGrid {
public:
    explicit OccupancyGrid(OccupancyGridOptions options);

    std::size_t width() const noexcept { return options_.width; }
    std::size_t height() const noexcept { return options_.height; }
    const std::string& frameId() const noexcept { return options_.frame_id; }

    bool isInside(int x, int y) const noexcept;
    bool isObserved(int x, int y) const noexcept;
    double logOdds(int x, int y) const noexcept;
    OccupancyState state(int x, int y) const noexcept;

    void setPrior(int x, int y, OccupancyState state, double strength = 1.0);
    OccupancyIntegrationResult integrate(
        const SensorFrame& frame,
        UnknownCellPolicy unknown_policy = UnknownCellPolicy::Occupied);

    std::vector<int> binaryGrid(UnknownCellPolicy unknown_policy) const;

private:
    std::size_t index(int x, int y) const noexcept;
    int binaryState(int x, int y, UnknownCellPolicy policy) const noexcept;
    void updateCell(int x, int y, bool occupied);

    OccupancyGridOptions options_;
    std::vector<double> log_odds_;
    std::vector<std::uint8_t> observed_;
};

}  // namespace robotnav::perception
