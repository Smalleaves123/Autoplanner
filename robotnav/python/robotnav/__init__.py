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

__all__ = [
    "BackendUnavailableError",
    "PlanResult",
    "Planner",
    "PlannerConfig",
    "available_planners",
    "plan",
]

__version__ = "0.1.0"
