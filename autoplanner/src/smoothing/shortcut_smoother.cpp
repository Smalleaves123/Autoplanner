#include "autoplanner/smoothing/shortcut_smoother.h"

#include <random>

namespace autoplanner {

ShortcutSmoother::ShortcutSmoother(const CollisionChecker& checker, int max_iterations)
    : checker_(checker), max_iterations_(max_iterations) {}

std::string ShortcutSmoother::name() const {
    return "shortcut";
}

std::vector<Point2d> ShortcutSmoother::smooth(const std::vector<Point2d>& path) {
    if (path.size() < 3) return path;

    std::vector<Point2d> result = path;
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<std::size_t> dist_idx(0, result.size() - 1);

    for (int iter = 0; iter < max_iterations_; ++iter) {
        if (result.size() < 3) break;

        std::size_t i = dist_idx(rng) % (result.size() - 1);
        std::size_t j = dist_idx(rng) % result.size();
        if (i > j) std::swap(i, j);
        if (j - i < 2) continue;

        if (checker_.isSegmentValid(result[i], result[j])) {
            result.erase(result.begin() + static_cast<long>(i) + 1,
                         result.begin() + static_cast<long>(j));
        }
    }
    return result;
}

}  // namespace autoplanner
