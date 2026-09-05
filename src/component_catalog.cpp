#include "robotnav/component_catalog.h"

#include <algorithm>

#include "autoplanner/core/planner_factory.h"
#include "autoplanner/smoothing/smoother_factory.h"
#include "robotnav/local_planner_registry.h"
#include "robotnav/trajectory_controller.h"

namespace robotnav {
namespace {

void addName(std::vector<std::string>& names, const std::string& name) {
    names.push_back(name);
    std::sort(names.begin(), names.end());
    names.erase(std::unique(names.begin(), names.end()), names.end());
}

bool contains(const std::vector<std::string>& names,
              const std::string& name) {
    return std::binary_search(names.begin(), names.end(), name);
}

}  // namespace

ComponentCatalog availableComponents() {
    ComponentCatalog catalog;
    catalog.planners = autoplanner::availablePlanners();
    addName(catalog.planners, "space_time_astar");
    catalog.controllers = availableControllers();
    catalog.local_planners = availableLocalPlanners();
    addName(catalog.local_planners, "none");
    catalog.smoothers = autoplanner::availableSmoothers();
    addName(catalog.smoothers, "none");
    return catalog;
}

ComponentSelectionResult validateComponentSelection(
    const PipelineConfig& config,
    bool dynamic_mode) {
    const auto catalog = availableComponents();
    const bool planner_valid =
        autoplanner::PlannerRegistry::instance().contains(config.planner) ||
        (dynamic_mode && config.planner == "space_time_astar");
    if (!planner_valid) {
        return {false, "unknown planner: " + config.planner};
    }
    if (!contains(catalog.controllers, config.controller)) {
#ifndef AUTOMPC_HAS_EIGEN
        if (config.controller == "mpc") {
            return {false, "MPC controller requires Eigen3"};
        }
#endif
        return {false, "unknown controller: " + config.controller};
    }
    if (!contains(catalog.local_planners, config.local_planner)) {
        return {false, "unsupported local planner: " + config.local_planner};
    }
    if (!contains(catalog.smoothers, config.smoother)) {
        return {false, "unsupported path smoother: " + config.smoother};
    }
    return {true, {}};
}

}  // namespace robotnav
