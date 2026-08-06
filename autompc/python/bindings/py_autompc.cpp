#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "autompc/autompc.h"

namespace py = pybind11;
using namespace autompc;

namespace {

Trajectory loadTrajectory(const std::string& path, double velocity) {
    Trajectory result;
    if (!loadPathCsv(path, velocity, result)) {
        throw std::runtime_error("failed to load trajectory CSV: " + path);
    }
    return result;
}

Trajectory loadTrajectoryWithOptions(const std::string& path, double velocity,
                                     const TrajectoryOptions& options) {
    Trajectory result;
    if (!loadPathCsv(path, velocity, result, options)) {
        throw std::runtime_error("failed to load trajectory CSV: " + path);
    }
    return result;
}

}  // namespace

PYBIND11_MODULE(_autompc, m) {
    m.doc() = "Optimized C++ trajectory tracking backend for RobotNav";

    py::class_<State>(m, "State")
        .def(py::init<>())
        .def(py::init<double, double, double, double>())
        .def_readwrite("x", &State::x)
        .def_readwrite("y", &State::y)
        .def_readwrite("theta", &State::theta)
        .def_readwrite("v", &State::v);

    py::class_<Control>(m, "Control")
        .def(py::init<>())
        .def(py::init<double, double>())
        .def_readwrite("velocity", &Control::velocity)
        .def_readwrite("steering", &Control::steering);

    py::class_<SimulationOptions>(m, "SimulationOptions")
        .def(py::init<>())
        .def_readwrite("dt", &SimulationOptions::dt)
        .def_readwrite("wheelbase", &SimulationOptions::wheelbase)
        .def_readwrite("max_velocity", &SimulationOptions::max_velocity)
        .def_readwrite("max_acceleration", &SimulationOptions::max_acceleration)
        .def_readwrite("max_deceleration", &SimulationOptions::max_deceleration)
        .def_readwrite("max_steering", &SimulationOptions::max_steering)
        .def_readwrite("max_steering_rate", &SimulationOptions::max_steering_rate)
        .def_readwrite("allow_reverse", &SimulationOptions::allow_reverse)
        .def_readwrite("max_reverse_velocity",
                       &SimulationOptions::max_reverse_velocity);

    py::class_<KinematicBicycleSimulator>(m, "KinematicBicycleSimulator")
        .def(py::init<const State&, SimulationOptions>(),
             py::arg("initial"), py::arg("options") = SimulationOptions{})
        .def("step", &KinematicBicycleSimulator::step)
        .def("reset", &KinematicBicycleSimulator::reset)
        .def_property_readonly("state",
                               &KinematicBicycleSimulator::state,
                               py::return_value_policy::reference_internal)
        .def_property_readonly("steering",
                               &KinematicBicycleSimulator::steering)
        .def_property_readonly("options",
                               &KinematicBicycleSimulator::options,
                               py::return_value_policy::reference_internal);

    py::class_<TrajectoryPoint>(m, "TrajectoryPoint")
        .def(py::init<>())
        .def(py::init<double, double, double, double>())
        .def_readwrite("x", &TrajectoryPoint::x)
        .def_readwrite("y", &TrajectoryPoint::y)
        .def_readwrite("theta", &TrajectoryPoint::theta)
        .def_readwrite("v", &TrajectoryPoint::v)
        .def_readwrite("curvature", &TrajectoryPoint::curvature)
        .def_readwrite("acceleration", &TrajectoryPoint::acceleration);

    py::class_<Waypoint2d>(m, "Waypoint2d")
        .def(py::init<>())
        .def(py::init<double, double>())
        .def_readwrite("x", &Waypoint2d::x)
        .def_readwrite("y", &Waypoint2d::y);

    py::class_<TrajectoryOptions>(m, "TrajectoryOptions")
        .def(py::init<>())
        .def_readwrite("sample_spacing", &TrajectoryOptions::sample_spacing)
        .def_readwrite("target_velocity", &TrajectoryOptions::target_velocity)
        .def_readwrite("max_velocity", &TrajectoryOptions::max_velocity)
        .def_readwrite("max_acceleration", &TrajectoryOptions::max_acceleration)
        .def_readwrite("max_deceleration", &TrajectoryOptions::max_deceleration)
        .def_readwrite("max_lateral_acceleration",
                       &TrajectoryOptions::max_lateral_acceleration)
        .def_readwrite("allow_reverse", &TrajectoryOptions::allow_reverse)
        .def_readwrite("max_reverse_velocity",
                       &TrajectoryOptions::max_reverse_velocity)
        .def_readwrite("max_curvature", &TrajectoryOptions::max_curvature);

    py::class_<TrackingErrors>(m, "TrackingErrors")
        .def_readonly("max_cross_track", &TrackingErrors::max_cross_track)
        .def_readonly("mean_cross_track", &TrackingErrors::mean_cross_track)
        .def_readonly("max_heading_err", &TrackingErrors::max_heading_err)
        .def_readonly("mean_heading_err", &TrackingErrors::mean_heading_err);

    py::class_<PIDController>(m, "PIDController")
        .def(py::init<>())
        .def("compute", &PIDController::compute)
        .def("reset", &PIDController::reset);

    py::class_<PurePursuitController>(m, "PurePursuitController")
        .def(py::init<double, double>(), py::arg("lookahead") = 2.0,
             py::arg("wheelbase") = 1.0)
        .def("compute", &PurePursuitController::compute);

    py::class_<StanleyController>(m, "StanleyController")
        .def(py::init<double, double>(), py::arg("k") = 0.5,
             py::arg("wheelbase") = 1.0)
        .def("compute", &StanleyController::compute);

    py::class_<MPCController>(m, "MPCController")
        .def(py::init<int, double, double, double, double,
                      double, double, double,
                      const Eigen::Vector4d&, const Eigen::Vector2d&,
                      const Eigen::Vector4d&, double, double, double>(),
             py::arg("horizon") = 15,
             py::arg("dt") = 0.05,
             py::arg("wheelbase") = 1.0,
             py::arg("max_velocity") = 2.0,
             py::arg("max_steering") = 0.7,
             py::arg("max_acceleration") = 1.5,
             py::arg("max_deceleration") = 2.0,
             py::arg("max_steering_rate") = 1.5,
             py::arg("state_weights") = Eigen::Vector4d(10, 10, 5, 1),
             py::arg("input_weights") = Eigen::Vector2d(0.1, 0.1),
             py::arg("terminal_state_weights") = Eigen::Vector4d(20, 20, 10, 2),
             py::arg("steering_rate_weight") = 0.2,
             py::arg("max_position_error") = 10.0,
             py::arg("max_heading_error") = 3.141592653589793)
        .def("compute", &MPCController::compute)
        .def("reset", &MPCController::reset)
        .def("reset_reference_progress", &MPCController::resetReferenceProgress)
        .def("reference_index", &MPCController::referenceIndex)
        .def("horizon", &MPCController::horizon);

    m.def("step", &step);
    m.def("make_circle", &makeCircle);
    m.def("make_straight_line", &makeStraightLine);
    m.def("load_path_csv", &loadTrajectory);
    m.def("load_path_csv_with_options", &loadTrajectoryWithOptions);
    m.def("generate_trajectory",
          [](const Waypoints& waypoints, const TrajectoryOptions& options) {
              return generateTrajectory(waypoints, options);
          },
          py::arg("waypoints"), py::arg("options") = TrajectoryOptions{});
    m.def("generate_trajectory_with_directions",
          [](const Waypoints& waypoints,
             const std::vector<int>& motion_directions,
             const TrajectoryOptions& options) {
              return generateTrajectory(waypoints, motion_directions, options);
          },
          py::arg("waypoints"), py::arg("motion_directions"),
          py::arg("options") = TrajectoryOptions{});
    m.def("save_trajectory_csv", &saveTrajectoryCsv);
    m.def("arc_length", &arcLength);
    m.def("closest_point_distance", &closestPointDistance);
    m.def("compute_errors", &computeErrors);
}
