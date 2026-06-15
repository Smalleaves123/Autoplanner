// py_autoplanner.cpp — Python bindings for AutoPlanner via pybind11
//
// Build:
//   mkdir -p build-bindings && cd build-bindings
//   cmake ../python/bindings -DCMAKE_BUILD_TYPE=Release
//   cmake --build . -j
//
// Usage:
//   import sys; sys.path.append('build-bindings')
//   import _autoplanner as ap
//   map = ap.GridMap()
//   map.load_from_txt('data/maps/simple_50x50.txt')
//   planner = ap.AStarPlanner()
//   result = planner.plan(map, (1,1), (48,48))

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/core/point.h"
#include "autoplanner/core/planner_result.h"
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/heuristics/euclidean.h"
#include "autoplanner/heuristics/manhattan.h"
#include "autoplanner/heuristics/diagonal.h"
#include "autoplanner/metrics/path_metrics.h"
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/dijkstra.h"
#include "autoplanner/planners/graph_search/improved_astar.h"
#include "autoplanner/planners/graph_search/jps.h"
#include "autoplanner/planners/graph_search/weighted_astar.h"
#include "autoplanner/planners/sampling/rrt.h"
#include "autoplanner/planners/sampling/rrt_star.h"

namespace py = pybind11;
using namespace autoplanner;

PYBIND11_MODULE(_autoplanner, m) {
    m.doc() = "AutoPlanner — C++ path planning library for mobile robots";

    // --- Core types ---
    py::class_<Point2i>(m, "Point2i")
        .def(py::init<int, int>())
        .def_readwrite("x", &Point2i::x)
        .def_readwrite("y", &Point2i::y)
        .def("__repr__", [](const Point2i& p) {
            return "Point2i(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
        });

    py::class_<Point2d>(m, "Point2d")
        .def(py::init<double, double>())
        .def_readwrite("x", &Point2d::x)
        .def_readwrite("y", &Point2d::y)
        .def("__repr__", [](const Point2d& p) {
            return "Point2d(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
        });

    // --- GridMap ---
    py::class_<GridMap>(m, "GridMap")
        .def(py::init<>())
        .def("load_from_txt", &GridMap::loadFromTxt)
        .def("is_inside", &GridMap::isInside)
        .def("is_free", &GridMap::isFree)
        .def("is_occupied", &GridMap::isOccupied)
        .def("width", &GridMap::width)
        .def("height", &GridMap::height);

    // --- PlannerResult ---
    py::class_<PlannerResult>(m, "PlannerResult")
        .def(py::init<>())
        .def_readonly("success", &PlannerResult::success)
        .def_readonly("path", &PlannerResult::path)
        .def_readonly("path_length", &PlannerResult::path_length)
        .def_readonly("planning_time_ms", &PlannerResult::planning_time_ms)
        .def_readonly("expanded_nodes", &PlannerResult::expanded_nodes)
        .def_readonly("message", &PlannerResult::message);

    // --- PlannerBase ---
    py::class_<PlannerBase>(m, "PlannerBase");

    // --- A* ---
    py::class_<AStarPlanner, PlannerBase>(m, "AStarPlanner")
        .def(py::init<bool>(), py::arg("allow_diagonal") = true)
        .def("plan", &AStarPlanner::plan)
        .def("name", &AStarPlanner::name);

    // --- Dijkstra ---
    py::class_<DijkstraPlanner, PlannerBase>(m, "DijkstraPlanner")
        .def(py::init<bool>(), py::arg("allow_diagonal") = true)
        .def("plan", &DijkstraPlanner::plan)
        .def("name", &DijkstraPlanner::name);

    // --- Weighted A* ---
    py::class_<WeightedAStarPlanner, PlannerBase>(m, "WeightedAStarPlanner")
        .def(py::init<double, bool>(), py::arg("weight") = 1.5, py::arg("allow_diagonal") = true)
        .def("plan", &WeightedAStarPlanner::plan)
        .def("name", &WeightedAStarPlanner::name);

    // --- Improved A* ---
    py::class_<ImprovedAStarPlanner, PlannerBase>(m, "ImprovedAStarPlanner")
        .def(py::init<double, double, double, bool>(),
             py::arg("heuristic_weight") = 1.0,
             py::arg("obstacle_weight") = 2.0,
             py::arg("turning_weight") = 0.5,
             py::arg("allow_diagonal") = true)
        .def("plan", &ImprovedAStarPlanner::plan)
        .def("name", &ImprovedAStarPlanner::name)
        .def("set_costmap", &ImprovedAStarPlanner::setCostmap);

    // --- JPS ---
    py::class_<JPSPlanner, PlannerBase>(m, "JPSPlanner")
        .def(py::init<bool>(), py::arg("allow_diagonal") = true)
        .def("plan", &JPSPlanner::plan)
        .def("name", &JPSPlanner::name);

    // --- RRT ---
    py::class_<RRTPlanner, PlannerBase>(m, "RRTPlanner")
        .def(py::init<double, int, double, double>(),
             py::arg("step_size") = 2.0,
             py::arg("max_iter") = 5000,
             py::arg("goal_sample_rate") = 0.1,
             py::arg("goal_tolerance") = 2.0)
        .def("plan", &RRTPlanner::plan)
        .def("name", &RRTPlanner::name);

    // --- RRT* ---
    py::class_<RRTStarPlanner, PlannerBase>(m, "RRTStarPlanner")
        .def(py::init<double, int, double, double, double>(),
             py::arg("step_size") = 2.0,
             py::arg("max_iter") = 8000,
             py::arg("goal_sample_rate") = 0.1,
             py::arg("goal_tolerance") = 2.0,
             py::arg("rewire_radius") = 5.0)
        .def("plan", &RRTStarPlanner::plan)
        .def("name", &RRTStarPlanner::name);

    // --- Costmap2D ---
    py::class_<Costmap2D>(m, "Costmap2D")
        .def(py::init<>())
        .def("build_from_grid_map", &Costmap2D::buildFromGridMap)
        .def("inflate_obstacles", &Costmap2D::inflateObstacles)
        .def("get_cost", &Costmap2D::getCost)
        .def("is_free", &Costmap2D::isFree);

    // --- Metrics ---
    m.def("compute_path_length", &computePathLength);
    m.def("compute_turning_count", &computeTurningCount);
    m.def("compute_smoothness_score", &computeSmoothnessScore);
    m.def("save_path_csv", &savePathCsv);
    m.def("save_metrics_json", &saveMetricsJson);
}
