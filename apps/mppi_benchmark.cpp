#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "autoplanner/collision/collision_checker.h"
#include "autompc/core/trajectory.h"
#include "robotnav/mppi_local_planner.h"

namespace {

class FreeSpaceCollisionChecker final
    : public autoplanner::CollisionChecker {
public:
    bool isStateValid(const autoplanner::Point2d& point) const override {
        return std::isfinite(point.x) && std::isfinite(point.y);
    }

    bool isSegmentValid(const autoplanner::Point2d& start,
                        const autoplanner::Point2d& end) const override {
        return isStateValid(start) && isStateValid(end);
    }

    bool isPathValid(
        const std::vector<autoplanner::Point2d>& path) const override {
        return std::all_of(path.begin(), path.end(),
                           [this](const auto& point) {
                               return isStateValid(point);
                           });
    }
};

struct BenchmarkOptions {
    std::size_t iterations = 100;
    int rollouts = 64;
    int horizon = 20;
};

std::size_t parsePositiveSize(const std::string& value,
                              const std::string& option) {
    std::size_t parsed_characters = 0;
    const auto parsed = std::stoull(value, &parsed_characters);
    if (parsed == 0 || parsed_characters != value.size()) {
        throw std::invalid_argument(option + " must be a positive integer");
    }
    return static_cast<std::size_t>(parsed);
}

BenchmarkOptions parseArguments(int argc, char** argv) {
    BenchmarkOptions options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help") {
            std::cout
                << "Usage: mppi_benchmark [--iterations N] [--rollouts N] "
                   "[--horizon N]\n";
            std::exit(EXIT_SUCCESS);
        }
        if (index + 1 >= argc) {
            throw std::invalid_argument("missing value for " + argument);
        }
        const auto value = parsePositiveSize(argv[++index], argument);
        if (argument == "--iterations") {
            options.iterations = value;
        } else if (argument == "--rollouts") {
            if (value > static_cast<std::size_t>(
                            std::numeric_limits<int>::max())) {
                throw std::invalid_argument(
                    argument + " exceeds the supported range");
            }
            options.rollouts = static_cast<int>(value);
        } else if (argument == "--horizon") {
            if (value > static_cast<std::size_t>(
                            std::numeric_limits<int>::max())) {
                throw std::invalid_argument(
                    argument + " exceeds the supported range");
            }
            options.horizon = static_cast<int>(value);
        } else {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }
    return options;
}

double percentile(const std::vector<double>& sorted_values,
                  double fraction) {
    const double position = fraction *
        static_cast<double>(sorted_values.size() - 1);
    const auto lower = static_cast<std::size_t>(std::floor(position));
    const auto upper = static_cast<std::size_t>(std::ceil(position));
    const double blend = position - static_cast<double>(lower);
    return sorted_values[lower] * (1.0 - blend) +
        sorted_values[upper] * blend;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const auto benchmark_options = parseArguments(argc, argv);
        FreeSpaceCollisionChecker collision_checker;
        autompc::SimulationOptions simulation_options;
        simulation_options.dt = 0.05;

        robotnav::MppiOptions mppi_options;
        mppi_options.prediction_time =
            simulation_options.dt * benchmark_options.horizon;
        mppi_options.rollouts = benchmark_options.rollouts;
        mppi_options.horizon = benchmark_options.horizon;
        robotnav::MppiLocalPlanner planner(
            collision_checker, simulation_options, mppi_options);
        const auto trajectory = autompc::makeStraightLine(
            0.0, 0.0, 20.0, 0.0, 1.0, 200);
        const autompc::State state{0.0, 0.0, 0.0, 1.0};
        const autompc::Control nominal{1.0, 0.0};

        const auto warmup = planner.computeCommand(
            state, 0.0, trajectory, nominal);
        if (!warmup.feasible) {
            std::cerr << "MPPI warm-up did not produce a feasible command\n";
            return EXIT_FAILURE;
        }

        std::vector<double> latency_ms;
        latency_ms.reserve(benchmark_options.iterations);
        std::size_t workspace_allocations = 0;
        std::size_t workspace_reuses = 0;
        double effective_sample_size_sum = 0.0;
        double effective_sample_ratio_sum = 0.0;
        double final_noise_scale = warmup.sampling_noise_scale;

        for (std::size_t iteration = 0;
             iteration < benchmark_options.iterations; ++iteration) {
            const auto start = std::chrono::steady_clock::now();
            const auto decision = planner.computeCommand(
                state, 0.0, trajectory, nominal);
            const auto end = std::chrono::steady_clock::now();
            if (!decision.feasible) {
                std::cerr << "MPPI iteration " << iteration
                          << " did not produce a feasible command\n";
                return EXIT_FAILURE;
            }
            latency_ms.push_back(
                std::chrono::duration<double, std::milli>(end - start).count());
            workspace_allocations += decision.workspace_allocation_count;
            workspace_reuses += decision.workspace_reused ? 1U : 0U;
            effective_sample_size_sum += decision.effective_sample_size;
            effective_sample_ratio_sum += decision.effective_sample_ratio;
            final_noise_scale = decision.sampling_noise_scale;
        }

        std::sort(latency_ms.begin(), latency_ms.end());
        const double mean_latency = std::accumulate(
            latency_ms.begin(), latency_ms.end(), 0.0) /
            static_cast<double>(latency_ms.size());
        const double iteration_count =
            static_cast<double>(benchmark_options.iterations);

        std::cout << std::fixed << std::setprecision(3)
                  << "iterations: " << benchmark_options.iterations << '\n'
                  << "rollouts: " << benchmark_options.rollouts << '\n'
                  << "horizon: " << benchmark_options.horizon << '\n'
                  << "mean_latency_ms: " << mean_latency << '\n'
                  << "p50_latency_ms: " << percentile(latency_ms, 0.50) << '\n'
                  << "p95_latency_ms: " << percentile(latency_ms, 0.95) << '\n'
                  << "p99_latency_ms: " << percentile(latency_ms, 0.99) << '\n'
                  << "workspace_allocation_count: "
                  << workspace_allocations << '\n'
                  << "workspace_reuse_count: " << workspace_reuses << '\n'
                  << "mean_effective_sample_size: "
                  << effective_sample_size_sum / iteration_count << '\n'
                  << "mean_effective_sample_ratio: "
                  << effective_sample_ratio_sum / iteration_count << '\n'
                  << "final_sampling_noise_scale: " << final_noise_scale
                  << '\n';

        if (workspace_allocations != 0 ||
            workspace_reuses != benchmark_options.iterations) {
            std::cerr
                << "MPPI-owned rollout workspace expanded after warm-up\n";
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "mppi_benchmark: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
