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
        .def_readwrite("velocity", &Control::velocity)
        .def_readwrite("steering", &Control::steering);

    py::class_<TrajectoryPoint>(m, "TrajectoryPoint")
        .def(py::init<>())
        .def(py::init<double, double, double, double>())
        .def_readwrite("x", &TrajectoryPoint::x)
        .def_readwrite("y", &TrajectoryPoint::y)
        .def_readwrite("theta", &TrajectoryPoint::theta)
        .def_readwrite("v", &TrajectoryPoint::v);

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
                      const Eigen::Vector4d&, const Eigen::Vector2d&>(),
             py::arg("horizon") = 15,
             py::arg("dt") = 0.05,
             py::arg("wheelbase") = 1.0,
             py::arg("max_velocity") = 2.0,
             py::arg("max_steering") = 0.7,
             py::arg("state_weights") = Eigen::Vector4d(10, 10, 5, 1),
             py::arg("input_weights") = Eigen::Vector2d(0.1, 0.1))
        .def("compute", &MPCController::compute)
        .def("horizon", &MPCController::horizon);

    m.def("step", &step);
    m.def("make_circle", &makeCircle);
    m.def("make_straight_line", &makeStraightLine);
    m.def("load_path_csv", &loadTrajectory);
    m.def("arc_length", &arcLength);
    m.def("closest_point_distance", &closestPointDistance);
    m.def("compute_errors", &computeErrors);
}
