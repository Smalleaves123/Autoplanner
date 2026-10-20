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
    TwistCommand,
    TrajectoryConfig,
    TrajectoryPoint,
    TrajectoryResult,
    generate_trajectory,
    simulate,
    steering_to_twist,
)
from .dynamic import (
    DynamicConfig,
    DynamicResult,
    DynamicScenario,
    MovingObstacle,
    ObstacleUpdate,
    run_dynamic,
    run_dynamic_scenario,
)
from .multi_agent import (
    CooperativeAgentPlan,
    CooperativePlanResult,
    plan_cooperative,
)
from .simulation import (
    EntityObservation,
    SimulationClock,
    SimulationWorld,
    WorldConfig,
    WorldEntity,
    WorldObservation,
)
from .sensors import (
    DepthCameraConfig,
    DepthImage,
    GridSensorMap,
    ImuConfig,
    ImuMeasurement,
    Lidar2DConfig,
    LidarScan,
    OdometryConfig,
    OdometryMeasurement,
    SensorBatch,
    SensorSuite,
    SensorTiming,
)
from .estimation import EkfConfig, PlanarEkf, PoseMeasurement, StateEstimate
from .localization import GridScanMatcher, ScanMatcherConfig, ScanMatchResult
from .localization_health import (
    LocalizationHealth, LocalizationHealthConfig, LocalizationHealthMonitor, MatchAssessment,
)
from .relocalization import (
    GridRelocalizer, RelocalizationConfig, RelocalizationHypothesis, RelocalizationResult,
)
from .relocalization_tracking import (
    RelocalizationDecision, RelocalizationEpoch, RelocalizationEpochTransition,
    RelocalizationTrack, RelocalizationTracker, RelocalizationTrackingConfig,
)

__all__ = [
    "RelocalizationDecision",
    "RelocalizationEpoch",
    "RelocalizationEpochTransition",
    "RelocalizationTrack",
    "RelocalizationTracker",
    "RelocalizationTrackingConfig",
    "GridRelocalizer",
    "RelocalizationConfig",
    "RelocalizationHypothesis",
    "RelocalizationResult",
    "LocalizationHealth",
    "LocalizationHealthConfig",
    "LocalizationHealthMonitor",
    "MatchAssessment",
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
    "TwistCommand",
    "TrajectoryConfig",
    "TrajectoryPoint",
    "TrajectoryResult",
    "generate_trajectory",
    "simulate",
    "steering_to_twist",
    "DynamicConfig",
    "DynamicResult",
    "DynamicScenario",
    "MovingObstacle",
    "ObstacleUpdate",
    "run_dynamic",
    "run_dynamic_scenario",
    "CooperativeAgentPlan",
    "CooperativePlanResult",
    "plan_cooperative",
    "SimulationClock",
    "WorldConfig",
    "WorldEntity",
    "EntityObservation",
    "WorldObservation",
    "SimulationWorld",
    "SensorTiming",
    "Lidar2DConfig",
    "LidarScan",
    "OdometryConfig",
    "OdometryMeasurement",
    "ImuConfig",
    "ImuMeasurement",
    "DepthCameraConfig",
    "DepthImage",
    "GridSensorMap",
    "SensorBatch",
    "SensorSuite",
    "EkfConfig",
    "PlanarEkf",
    "StateEstimate",
    "PoseMeasurement",
    "GridScanMatcher",
    "ScanMatcherConfig",
    "ScanMatchResult",
]

__version__ = "0.6.0"
