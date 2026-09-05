#include "robotnav/local_planner_registry.h"

#include <utility>

namespace robotnav {
namespace {

class DwaLocalPlannerAdapter final : public LocalPlanner {
public:
    DwaLocalPlannerAdapter(
        const autoplanner::CollisionChecker& collision_checker,
        const autompc::SimulationOptions& simulation_options,
        const DwaOptions& options)
        : planner_(collision_checker, simulation_options, options) {}

    LocalPlannerDecision computeCommand(
        const autompc::State& state,
        double current_steering,
        const autompc::Trajectory& trajectory,
        const autompc::Control& nominal_command,
        const DynamicObstacleContext& dynamic_context) const override {
        const auto decision = planner_.computeCommand(
            state, current_steering, trajectory, nominal_command,
            dynamic_context);
        return {decision.feasible,
                decision.command,
                decision.score,
                0,
                decision.dynamic_collision_rejections,
                decision.minimum_dynamic_clearance,
                false,
                0.0,
                0.0,
                1.0};
    }

private:
    DwaLocalPlanner planner_;
};

class MppiLocalPlannerAdapter final : public LocalPlanner {
public:
    MppiLocalPlannerAdapter(
        const autoplanner::CollisionChecker& collision_checker,
        const autompc::SimulationOptions& simulation_options,
        const MppiOptions& options)
        : planner_(collision_checker, simulation_options, options) {}

    void onTrajectoryChanged() override {
        planner_.resetWarmStart();
    }

    LocalPlannerDecision computeCommand(
        const autompc::State& state,
        double current_steering,
        const autompc::Trajectory& trajectory,
        const autompc::Control& nominal_command,
        const DynamicObstacleContext& dynamic_context) const override {
        const auto decision = planner_.computeCommand(
            state, current_steering, trajectory, nominal_command,
            dynamic_context);
        return {decision.feasible,
                decision.command,
                decision.score,
                decision.feasible_rollouts,
                decision.dynamic_collision_rejections,
                decision.minimum_dynamic_clearance,
                decision.warm_started,
                decision.effective_sample_size,
                decision.effective_sample_ratio,
                decision.sampling_noise_scale};
    }

private:
    MppiLocalPlanner planner_;
};

}  // namespace

LocalPlannerRegistry::LocalPlannerRegistry() {
    factories_.emplace(
        "dwa",
        [](const autoplanner::CollisionChecker& checker,
           const autompc::SimulationOptions& simulation,
           const DwaOptions& dwa,
           const MppiOptions&) {
            return std::make_unique<DwaLocalPlannerAdapter>(
                checker, simulation, dwa);
        });
    factories_.emplace(
        "mppi",
        [](const autoplanner::CollisionChecker& checker,
           const autompc::SimulationOptions& simulation,
           const DwaOptions&,
           const MppiOptions& mppi) {
            return std::make_unique<MppiLocalPlannerAdapter>(
                checker, simulation, mppi);
        });
}

LocalPlannerRegistry& LocalPlannerRegistry::instance() {
    static LocalPlannerRegistry registry;
    return registry;
}

bool LocalPlannerRegistry::registerLocalPlanner(const std::string& name,
                                                Factory factory,
                                                bool replace) {
    if (name.empty() || name == "none" || !factory) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto existing = factories_.find(name);
    if (existing != factories_.end() && !replace) return false;
    factories_[name] = std::move(factory);
    return true;
}

bool LocalPlannerRegistry::unregisterLocalPlanner(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.erase(name) != 0;
}

bool LocalPlannerRegistry::contains(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.find(name) != factories_.end();
}

std::vector<std::string> LocalPlannerRegistry::availableLocalPlanners() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> names;
    names.reserve(factories_.size());
    for (const auto& entry : factories_) names.push_back(entry.first);
    return names;
}

std::unique_ptr<LocalPlanner> LocalPlannerRegistry::create(
    const std::string& name,
    const autoplanner::CollisionChecker& collision_checker,
    const autompc::SimulationOptions& simulation_options,
    const DwaOptions& dwa_options,
    const MppiOptions& mppi_options) const {
    Factory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto entry = factories_.find(name);
        if (entry == factories_.end()) return nullptr;
        factory = entry->second;
    }
    return factory(collision_checker, simulation_options,
                   dwa_options, mppi_options);
}

std::unique_ptr<LocalPlanner> createLocalPlanner(
    const std::string& name,
    const autoplanner::CollisionChecker& collision_checker,
    const autompc::SimulationOptions& simulation_options,
    const DwaOptions& dwa_options,
    const MppiOptions& mppi_options) {
    return LocalPlannerRegistry::instance().create(
        name, collision_checker, simulation_options,
        dwa_options, mppi_options);
}

std::vector<std::string> availableLocalPlanners() {
    return LocalPlannerRegistry::instance().availableLocalPlanners();
}

}  // namespace robotnav
