#include "autoplanner/smoothing/shortcut_smoother.h"

namespace autoplanner {

ShortcutSmoother::ShortcutSmoother(const CollisionChecker& checker, int max_iterations)
    : checker_(checker), max_iterations_(max_iterations) {}

std::string ShortcutSmoother::name() const {
    return "shortcut";
}

std::vector<Point2d> ShortcutSmoother::smooth(const std::vector<Point2d>& path) {
    if (path.size() < 3) return path;

    std::vector<Point2d> result = path;

    for (int iter = 0; iter < max_iterations_; ++iter) {
        if (result.size() < 3) break;

        bool changed = false;
        // Deterministic farthest-visible shortcut. This keeps benchmark
        // results reproducible while removing redundant grid turns.
        for (std::size_t i = 0; i + 2 < result.size(); ++i) {
            std::size_t farthest = i + 1;
            for (std::size_t j = result.size() - 1; j > i + 1; --j) {
                if (checker_.isSegmentValid(result[i], result[j])) {
                    farthest = j;
                    break;
                }
            }

            if (farthest > i + 1) {
                result.erase(result.begin() + static_cast<long>(i) + 1,
                             result.begin() + static_cast<long>(farthest));
                changed = true;
                break;
            }
        }

        if (!changed) break;
    }
    return result;
}

}  // namespace autoplanner
