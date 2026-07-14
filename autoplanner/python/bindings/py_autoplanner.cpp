#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "autoplanner/collision/footprint_collision_checker.h"
#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/core/planner_factory.h"
#include "autoplanner/core/planner_result.h"
#include "autoplanner/core/pose.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/metrics/path_metrics.h"
#include "autoplanner/planners/graph_search/dstar_lite.h"

namespace py = pybind11;
using namespace autoplanner;

namespace {

std::shared_ptr<PlannerBase> makePlanner(
    const std::string& name, const PlannerFactoryOptions& options) {
    auto planner = createPlanner(name, options);
    if (!planner) {
        throw std::invalid_argument("unknown planner: " + name);
    }
    return std::shared_ptr<PlannerBase>(std::move(planner));
}

PlannerResult planWithGILReleased(
    PlannerBase& planner, const GridMap& map,
    const Point2i& start, const Point2i& goal) {
    py::gil_scoped_release release;
    return planner.plan(map, start, goal);
}

PlannerResult planNamed(
    const std::string& name, const GridMap& map,
    const Point2i& start, const Point2i& goal,
    const PlannerFactoryOptions& options) {
    Costmap2D costmap;
    const Costmap2D* costmap_ptr = nullptr;
    if (name == "improved_astar") {
        costmap.buildFromGridMap(map);
        if (options.robot_radius > 0.0) {
            costmap.inflateObstacles(options.robot_radius);
        }
        costmap_ptr = &costmap;
    }
    auto planner = createPlanner(name, options, costmap_ptr);
    if (!planner) {
        throw std::invalid_argument("unknown planner: " + name);
    }
    return planWithGILReleased(*planner, map, start, goal);
}

PlannerResult replanWithGILReleased(
    DStarLitePlanner& planner, const GridMap& map, const Point2i& start) {
    py::gil_scoped_release release;
    return planner.replan(map, start);
}

}  // namespace

PYBIND11_MODULE(_autoplanner, m) {
    m.doc() = "Optimized C++ path planning backend for RobotNav";

    py::class_<Point2i>(m, "Point2i")
        .def(py::init<int, int>())
        .def_readwrite("x", &Point2i::x)
        .def_readwrite("y", &Point2i::y)
        .def("as_tuple", [](const Point2i& p) { return std::make_pair(p.x, p.y); });

    py::class_<Point2d>(m, "Point2d")
        .def(py::init<double, double>())
        .def_readwrite("x", &Point2d::x)
        .def_readwrite("y", &Point2d::y);

    py::class_<Pose2d>(m, "Pose2d")
        .def(py::init<>())
        .def(py::init<double, double, double>())
        .def_readwrite("x", &Pose2d::x)
        .def_readwrite("y", &Pose2d::y)
        .def_readwrite("theta", &Pose2d::theta);

    py::class_<GridMap>(m, "GridMap")
        .def(py::init<>())
        .def("load_from_txt", &GridMap::loadFromTxt)
        .def("is_inside", &GridMap::isInside)
        .def("is_free", &GridMap::isFree)
        .def("is_occupied", &GridMap::isOccupied)
        .def("set_occupied", &GridMap::setOccupied)
        .def("inflate_obstacles", &GridMap::inflateObstacles)
        .def("set_resolution", &GridMap::setResolution)
        .def("resolution", &GridMap::resolution)
        .def("width", &GridMap::width)
        .def("height", &GridMap::height);

    py::class_<PlannerResult>(m, "PlannerResult")
        .def_readonly("success", &PlannerResult::success)
        .def_readonly("planner_name", &PlannerResult::planner_name)
        .def_readonly("path", &PlannerResult::path)
        .def_readonly("path_length", &PlannerResult::path_length)
        .def_readonly("planning_time_ms", &PlannerResult::planning_time_ms)
        .def_readonly("expanded_nodes", &PlannerResult::expanded_nodes)
        .def_readonly("iterations", &PlannerResult::iterations)
        .def_readonly("message", &PlannerResult::message);

    py::class_<PlannerFactoryOptions>(m, "PlannerOptions")
        .def(py::init<>())
        .def_readwrite("allow_diagonal", &PlannerFactoryOptions::allow_diagonal)
        .def_readwrite("robot_radius", &PlannerFactoryOptions::robot_radius)
        .def_readwrite("heuristic_weight", &PlannerFactoryOptions::heuristic_weight)
        .def_readwrite("weighted_astar_weight", &PlannerFactoryOptions::weighted_astar_weight)
        .def_readwrite("obstacle_weight", &PlannerFactoryOptions::obstacle_weight)
        .def_readwrite("turning_weight", &PlannerFactoryOptions::turning_weight)
        .def_readwrite("step_size", &PlannerFactoryOptions::step_size)
        .def_readwrite("max_iterations", &PlannerFactoryOptions::max_iterations)
        .def_readwrite("goal_sample_rate", &PlannerFactoryOptions::goal_sample_rate)
        .def_readwrite("goal_tolerance", &PlannerFactoryOptions::goal_tolerance)
        .def_readwrite("rewire_radius", &PlannerFactoryOptions::rewire_radius)
        .def_readwrite("turning_radius", &PlannerFactoryOptions::turning_radius)
        .def_readwrite("angle_bins", &PlannerFactoryOptions::angle_bins);

    py::class_<PlannerBase, std::shared_ptr<PlannerBase>>(m, "Planner")
        .def("name", &PlannerBase::name)
        .def("plan", &planWithGILReleased);

    py::class_<DStarLitePlanner, PlannerBase,
               std::shared_ptr<DStarLitePlanner>>(m, "DStarLitePlanner")
        .def(py::init<bool>(), py::arg("allow_diagonal") = true)
        .def("plan", &planWithGILReleased)
        .def("replan", &replanWithGILReleased)
        .def("name", &DStarLitePlanner::name);

    py::enum_<FootprintType>(m, "FootprintType")
        .value("circle", FootprintType::Circle)
        .value("rectangle", FootprintType::Rectangle);

    py::class_<RobotFootprint>(m, "RobotFootprint")
        .def_static("circle", &RobotFootprint::circle)
        .def_static("rectangle", &RobotFootprint::rectangle)
        .def_readwrite("type", &RobotFootprint::type)
        .def_readwrite("radius", &RobotFootprint::radius)
        .def_readwrite("length", &RobotFootprint::length)
        .def_readwrite("width", &RobotFootprint::width);

    py::class_<FootprintCollisionChecker>(m, "FootprintCollisionChecker")
        .def(py::init<const GridMap&, RobotFootprint>(), py::keep_alive<1, 2>())
        .def("is_state_valid", &FootprintCollisionChecker::isStateValid)
        .def("is_pose_valid", &FootprintCollisionChecker::isPoseValid)
        .def("is_segment_valid", &FootprintCollisionChecker::isSegmentValid)
        .def("is_pose_segment_valid", &FootprintCollisionChecker::isPoseSegmentValid)
        .def("is_path_valid", &FootprintCollisionChecker::isPathValid)
        .def("is_pose_path_valid", &FootprintCollisionChecker::isPosePathValid);

    py::class_<Costmap2D>(m, "Costmap2D")
        .def(py::init<>())
        .def("build_from_grid_map", &Costmap2D::buildFromGridMap)
        .def("inflate_obstacles", &Costmap2D::inflateObstacles)
        .def("get_cost", &Costmap2D::getCost)
        .def("is_free", &Costmap2D::isFree)
        .def("width", &Costmap2D::width)
        .def("height", &Costmap2D::height);

    m.def("create_planner", &makePlanner,
          py::arg("name"), py::arg("options") = PlannerFactoryOptions{});
    m.def("plan", &planNamed,
          py::arg("name"), py::arg("map"), py::arg("start"), py::arg("goal"),
          py::arg("options") = PlannerFactoryOptions{});
    m.def("compute_path_length", &computePathLength);
    m.def("compute_turning_count", &computeTurningCount);
    m.def("compute_smoothness_score", &computeSmoothnessScore);
    m.def("save_path_csv", &savePathCsv);
    m.def("save_metrics_json", &saveMetricsJson);
}
