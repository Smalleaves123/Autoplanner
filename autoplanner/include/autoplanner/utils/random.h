#pragma once

#include <random>

namespace autoplanner {

// Thread-safe random number generator wrapper.
// Used by sampling-based planners (RRT, RRT*) for reproducibility.
class RandomGenerator {
public:
    using Engine = std::mt19937;

    // Seed with a fixed value for deterministic runs, or use random_device.
    explicit RandomGenerator(unsigned int seed = 0) {
        if (seed == 0) {
            std::random_device rd;
            rng_.seed(rd());
        } else {
            rng_.seed(seed);
        }
    }

    void seed(unsigned int s) { rng_.seed(s); }

    // Uniform real in [0, 1).
    double uniform01() {
        return std::uniform_real_distribution<double>(0.0, 1.0)(rng_);
    }

    // Uniform real in [lo, hi).
    double uniform(double lo, double hi) {
        return std::uniform_real_distribution<double>(lo, hi)(rng_);
    }

    // Uniform integer in [lo, hi].
    int uniformInt(int lo, int hi) {
        return std::uniform_int_distribution<int>(lo, hi)(rng_);
    }

    // Normal distribution.
    double normal(double mean = 0.0, double stddev = 1.0) {
        return std::normal_distribution<double>(mean, stddev)(rng_);
    }

    // Access the underlying engine.
    Engine& engine() { return rng_; }

private:
    Engine rng_;
};

}  // namespace autoplanner
