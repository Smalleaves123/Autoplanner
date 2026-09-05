#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autompc/core/trajectory.h"
#include "autompc/core/types.h"
#include "autompc/simulation/kinematic_bicycle.h"

namespace robotnav {

// Common adapter used by the navigation pipelines. It hides the different
// compute signatures of AutoMPC controllers and gives application-defined
// controllers one stable extension point.
class TrajectoryController {
public:
    virtual ~TrajectoryController() = default;

    virtual autompc::Control compute(
        const autompc::State& state,
        const autompc::Trajectory& trajectory,
        const autompc::TrajectoryPoint& reference) = 0;

    virtual void reset() {}
    virtual void onTrajectoryChanged() {}
};

class ControllerRegistry {
public:
    using Factory = std::function<std::unique_ptr<TrajectoryController>(
        const autompc::SimulationOptions&)>;

    static ControllerRegistry& instance();

    bool registerController(const std::string& name,
                            Factory factory,
                            bool replace = false);
    bool unregisterController(const std::string& name);
    bool contains(const std::string& name) const;
    std::vector<std::string> availableControllers() const;

    std::unique_ptr<TrajectoryController> create(
        const std::string& name,
        const autompc::SimulationOptions& simulation_options = {}) const;

private:
    ControllerRegistry();

    mutable std::mutex mutex_;
    std::map<std::string, Factory> factories_;
};

std::unique_ptr<TrajectoryController> createController(
    const std::string& name,
    const autompc::SimulationOptions& simulation_options = {});
std::vector<std::string> availableControllers();

}  // namespace robotnav
