#include "autoplanner/smoothing/smoother_factory.h"

#include <utility>

#include "autoplanner/smoothing/curvature_constrained_smoother.h"
#include "autoplanner/smoothing/shortcut_smoother.h"

namespace autoplanner {

SmootherRegistry::SmootherRegistry() {
    factories_.emplace(
        "shortcut",
        [](const CollisionChecker& checker,
           const SmootherFactoryOptions& options) {
            return std::make_unique<ShortcutSmoother>(
                checker, options.max_iterations);
        });
    factories_.emplace(
        "curvature",
        [](const CollisionChecker& checker,
           const SmootherFactoryOptions& options) {
            return std::make_unique<CurvatureConstrainedSmoother>(
                checker, options.max_curvature, options.max_iterations);
        });
}

SmootherRegistry& SmootherRegistry::instance() {
    static SmootherRegistry registry;
    return registry;
}

bool SmootherRegistry::registerSmoother(const std::string& name,
                                        Factory factory,
                                        bool replace) {
    if (name.empty() || name == "none" || !factory) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto existing = factories_.find(name);
    if (existing != factories_.end() && !replace) return false;
    factories_[name] = std::move(factory);
    return true;
}

bool SmootherRegistry::unregisterSmoother(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.erase(name) != 0;
}

bool SmootherRegistry::contains(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.find(name) != factories_.end();
}

std::vector<std::string> SmootherRegistry::availableSmoothers() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> names;
    names.reserve(factories_.size());
    for (const auto& entry : factories_) names.push_back(entry.first);
    return names;
}

std::unique_ptr<PathSmoother> SmootherRegistry::create(
    const std::string& name,
    const CollisionChecker& collision_checker,
    const SmootherFactoryOptions& options) const {
    Factory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto entry = factories_.find(name);
        if (entry == factories_.end()) return nullptr;
        factory = entry->second;
    }
    return factory(collision_checker, options);
}

std::unique_ptr<PathSmoother> createSmoother(
    const std::string& name,
    const CollisionChecker& collision_checker,
    const SmootherFactoryOptions& options) {
    return SmootherRegistry::instance().create(
        name, collision_checker, options);
}

std::vector<std::string> availableSmoothers() {
    return SmootherRegistry::instance().availableSmoothers();
}

}  // namespace autoplanner
