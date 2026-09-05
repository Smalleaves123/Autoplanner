#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autoplanner/collision/collision_checker.h"
#include "autoplanner/smoothing/path_smoother.h"

namespace autoplanner {

struct SmootherFactoryOptions {
    int max_iterations = 100;
    double max_curvature = 0.5;
};

class SmootherRegistry {
public:
    using Factory = std::function<std::unique_ptr<PathSmoother>(
        const CollisionChecker&, const SmootherFactoryOptions&)>;

    static SmootherRegistry& instance();

    bool registerSmoother(const std::string& name,
                          Factory factory,
                          bool replace = false);
    bool unregisterSmoother(const std::string& name);
    bool contains(const std::string& name) const;
    std::vector<std::string> availableSmoothers() const;

    std::unique_ptr<PathSmoother> create(
        const std::string& name,
        const CollisionChecker& collision_checker,
        const SmootherFactoryOptions& options = {}) const;

private:
    SmootherRegistry();

    mutable std::mutex mutex_;
    std::map<std::string, Factory> factories_;
};

std::unique_ptr<PathSmoother> createSmoother(
    const std::string& name,
    const CollisionChecker& collision_checker,
    const SmootherFactoryOptions& options = {});
std::vector<std::string> availableSmoothers();

}  // namespace autoplanner
