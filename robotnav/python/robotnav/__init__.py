"""User-facing Python API for the RobotNav navigation library.

The planning and tracking kernels remain implemented in C++.  This package
provides small, typed convenience wrappers around the optional native
``autoplanner`` and ``autompc`` modules so applications do not need to know
about pybind11 implementation details.
"""

from .planning import (
    BackendUnavailableError,
    PlanResult,
    Planner,
    PlannerConfig,
    available_planners,
    plan,
)
from .control import (
    ControlCommand,
    Controller,
    ControllerConfig,
    RobotState,
    SimulationConfig,
    SimulationResult,
    TrackingMetrics,
    TrajectoryConfig,
    TrajectoryPoint,
    TrajectoryResult,
    generate_trajectory,
    simulate,
)

__all__ = [
    "BackendUnavailableError",
    "PlanResult",
    "Planner",
    "PlannerConfig",
    "available_planners",
    "plan",
    "ControlCommand",
    "Controller",
    "ControllerConfig",
    "RobotState",
    "SimulationConfig",
    "SimulationResult",
    "TrackingMetrics",
    "TrajectoryConfig",
    "TrajectoryPoint",
    "TrajectoryResult",
    "generate_trajectory",
    "simulate",
]

__version__ = "0.1.0"
