#include "robotnav/trajectory_controller.h"

#include <utility>

#include "autompc/controllers/controllers.h"
#ifdef AUTOMPC_HAS_EIGEN
#include "autompc/controllers/mpc_controller.h"
#endif

namespace robotnav {
namespace {

class PidControllerAdapter final : public TrajectoryController {
public:
    explicit PidControllerAdapter(const autompc::SimulationOptions& options)
        : controller_(1.0, 0.0, 0.0, 2.0, 0.0, 0.5,
                      options.wheelbase),
          dt_(options.dt) {}

    autompc::Control compute(
        const autompc::State& state,
        const autompc::Trajectory&,
        const autompc::TrajectoryPoint& reference) override {
        return controller_.compute(state, reference, dt_);
    }

    void reset() override { controller_.reset(); }
    void onTrajectoryChanged() override { controller_.reset(); }

private:
    autompc::PIDController controller_;
    double dt_;
};

class PurePursuitControllerAdapter final : public TrajectoryController {
public:
    explicit PurePursuitControllerAdapter(
        const autompc::SimulationOptions& options)
        : controller_(2.0, options.wheelbase) {}

    autompc::Control compute(
        const autompc::State& state,
        const autompc::Trajectory& trajectory,
        const autompc::TrajectoryPoint& reference) override {
        return controller_.compute(state, trajectory, reference.v);
    }

private:
    autompc::PurePursuitController controller_;
};

class StanleyControllerAdapter final : public TrajectoryController {
public:
    explicit StanleyControllerAdapter(const autompc::SimulationOptions& options)
        : controller_(0.5, options.wheelbase) {}

    autompc::Control compute(
        const autompc::State& state,
        const autompc::Trajectory&,
        const autompc::TrajectoryPoint& reference) override {
        return controller_.compute(state, reference, reference.v);
    }

private:
    autompc::StanleyController controller_;
};

#ifdef AUTOMPC_HAS_EIGEN
class MpcControllerAdapter final : public TrajectoryController {
public:
    explicit MpcControllerAdapter(const autompc::SimulationOptions& options)
        : controller_(15, options.dt, options.wheelbase,
                      options.max_velocity, options.max_steering,
                      options.max_acceleration, options.max_deceleration,
                      options.max_steering_rate) {}

    autompc::Control compute(
        const autompc::State& state,
        const autompc::Trajectory& trajectory,
        const autompc::TrajectoryPoint& reference) override {
        return controller_.compute(state, trajectory, reference.v);
    }

    void reset() override { controller_.reset(); }
    void onTrajectoryChanged() override {
        controller_.resetReferenceProgress();
    }

private:
    autompc::MPCController controller_;
};
#endif

}  // namespace

ControllerRegistry::ControllerRegistry() {
    factories_.emplace(
        "pid", [](const autompc::SimulationOptions& options) {
            return std::make_unique<PidControllerAdapter>(options);
        });
    factories_.emplace(
        "pure_pursuit", [](const autompc::SimulationOptions& options) {
            return std::make_unique<PurePursuitControllerAdapter>(options);
        });
    factories_.emplace(
        "stanley", [](const autompc::SimulationOptions& options) {
            return std::make_unique<StanleyControllerAdapter>(options);
        });
#ifdef AUTOMPC_HAS_EIGEN
    factories_.emplace(
        "mpc", [](const autompc::SimulationOptions& options) {
            return std::make_unique<MpcControllerAdapter>(options);
        });
#endif
}

ControllerRegistry& ControllerRegistry::instance() {
    static ControllerRegistry registry;
    return registry;
}

bool ControllerRegistry::registerController(const std::string& name,
                                            Factory factory,
                                            bool replace) {
    if (name.empty() || !factory) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto existing = factories_.find(name);
    if (existing != factories_.end() && !replace) return false;
    factories_[name] = std::move(factory);
    return true;
}

bool ControllerRegistry::unregisterController(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.erase(name) != 0;
}

bool ControllerRegistry::contains(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.find(name) != factories_.end();
}

std::vector<std::string> ControllerRegistry::availableControllers() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> names;
    names.reserve(factories_.size());
    for (const auto& entry : factories_) names.push_back(entry.first);
    return names;
}

std::unique_ptr<TrajectoryController> ControllerRegistry::create(
    const std::string& name,
    const autompc::SimulationOptions& simulation_options) const {
    Factory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto entry = factories_.find(name);
        if (entry == factories_.end()) return nullptr;
        factory = entry->second;
    }
    return factory(simulation_options);
}

std::unique_ptr<TrajectoryController> createController(
    const std::string& name,
    const autompc::SimulationOptions& simulation_options) {
    return ControllerRegistry::instance().create(name, simulation_options);
}

std::vector<std::string> availableControllers() {
    return ControllerRegistry::instance().availableControllers();
}

}  // namespace robotnav
