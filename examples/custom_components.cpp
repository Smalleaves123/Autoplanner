// Application-defined planner and controller registry example.
//
// This executable deliberately keeps both components small. Its purpose is
// to show the complete extension lifecycle without modifying RobotNav.

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "autoplanner/collision/grid_collision_checker.h"
#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/core/planner_factory.h"
#include "autoplanner/utils/math_utils.h"
#include "autompc/core/trajectory.h"
#include "robotnav/trajectory_controller.h"

namespace {

class DirectLinePlanner final : public autoplanner::PlannerBase {
public:
    autoplanner::PlannerResult plan(
        const autoplanner::GridMap& map,
        const autoplanner::Point2i& start,
        const autoplanner::Point2i& goal) override {
        autoplanner::PlannerResult result;
        result.planner_name = name();
        if (!map.isInside(start.x, start.y) ||
            !map.isFree(start.x, start.y)) {
            result.message = "Start is outside the map or occupied";
            return result;
        }
        if (!map.isInside(goal.x, goal.y) || !map.isFree(goal.x, goal.y)) {
            result.message = "Goal is outside the map or occupied";
            return result;
        }

        const autoplanner::Point2d start_world{
            static_cast<double>(start.x), static_cast<double>(start.y)};
        const autoplanner::Point2d goal_world{
            static_cast<double>(goal.x), static_cast<double>(goal.y)};
        const autoplanner::GridCollisionChecker collision_checker(map);
        if (!collision_checker.isSegmentValid(start_world, goal_world)) {
            result.message = "Direct segment is in collision";
            return result;
        }

        result.success = true;
        result.collision_free = true;
        result.path = {start_world, goal_world};
        result.path_length = autoplanner::computePathLength(result.path);
        result.message = "Direct path found";
        return result;
    }

    std::string name() const override { return "example_direct_line"; }
};

class HeadingController final : public robotnav::TrajectoryController {
public:
    explicit HeadingController(const autompc::SimulationOptions& options)
        : maximum_velocity_(options.max_velocity),
          maximum_steering_(options.max_steering) {}

    autompc::Control compute(
        const autompc::State& state,
        const autompc::Trajectory&,
        const autompc::TrajectoryPoint& reference) override {
        return {
            std::clamp(reference.v, 0.0, maximum_velocity_),
            std::clamp(
                autoplanner::normalizeAngle(reference.theta - state.theta),
                -maximum_steering_, maximum_steering_)};
    }

private:
    double maximum_velocity_;
    double maximum_steering_;
};

}  // namespace

int main(int argc, char** argv) {
    const std::string map_path = argc > 1
        ? argv[1] : "autoplanner/data/maps/simple_50x50.txt";

    auto& planner_registry = autoplanner::PlannerRegistry::instance();
    const bool planner_registered = planner_registry.registerPlanner(
        "example_direct_line",
        [](const autoplanner::PlannerFactoryOptions&,
           const autoplanner::Costmap2D*) {
            return std::make_unique<DirectLinePlanner>();
        });

    auto& controller_registry = robotnav::ControllerRegistry::instance();
    const bool controller_registered = controller_registry.registerController(
        "example_heading",
        [](const autompc::SimulationOptions& options) {
            return std::make_unique<HeadingController>(options);
        });

    if (!planner_registered || !controller_registered) {
        std::cerr << "Failed to register application components\n";
        return 1;
    }

    autoplanner::GridMap map;
    if (!map.loadFromTxt(map_path)) {
        std::cerr << "Failed to load map: " << map_path << '\n';
        return 1;
    }

    auto planner = autoplanner::createPlanner("example_direct_line");
    auto controller = robotnav::createController("example_heading");
    if (!planner || !controller) {
        std::cerr << "Failed to create registered components\n";
        return 1;
    }

    const auto plan = planner->plan(map, {1, 1}, {8, 1});
    const auto trajectory = autompc::makeStraightLine(
        1.0, 1.0, 8.0, 1.0, 1.0, 20);
    const auto command = controller->compute(
        {1.0, 1.0, 0.0, 0.0}, trajectory, trajectory.front());
    if (!plan.success || plan.path.size() != 2 || command.velocity <= 0.0 ||
        !std::isfinite(command.steering)) {
        std::cerr << "Application component smoke test failed\n";
        return 1;
    }

    std::cout << "planner: " << planner->name() << '\n'
              << "path_points: " << plan.path.size() << '\n'
              << "controller: example_heading\n"
              << "command_velocity: " << command.velocity << '\n'
              << "command_steering: " << command.steering << '\n';

    planner_registry.unregisterPlanner("example_direct_line");
    controller_registry.unregisterController("example_heading");
    return 0;
}
