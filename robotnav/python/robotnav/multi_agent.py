"""Python facade for deterministic multi-robot Cooperative A*."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from .planning import BackendUnavailableError


def _backend() -> Any:
    try:
        from . import _robotnav
    except ImportError as error:  # pragma: no cover
        raise BackendUnavailableError(
            "The compiled RobotNav backend is unavailable. Build with "
            "BUILD_PYTHON_BINDINGS=ON first.") from error
    return _robotnav


@dataclass(frozen=True)
class CooperativeAgentPlan:
    success: bool
    message: str
    path: tuple[tuple[float, float], ...]
    path_length: float


@dataclass(frozen=True)
class CooperativePlanResult:
    success: bool
    message: str
    planned_agents: int
    vertex_conflicts: int
    edge_conflicts: int
    plans: tuple[CooperativeAgentPlan, ...]


def plan_cooperative(
    map_path: str | Path,
    starts: Sequence[Sequence[int]],
    goals: Sequence[Sequence[int]],
    *,
    allow_diagonal: bool = True,
    allow_wait: bool = True,
    max_time_steps: int = 200,
    algorithm: str = "prioritized",
    allow_shared_goals: bool = False,
    priority_order: Sequence[int] | None = None,
    max_cbs_nodes: int = 10000,
) -> CooperativePlanResult:
    if len(starts) == 0 or len(starts) != len(goals):
        raise ValueError("starts and goals must have the same non-zero length")
    if max_time_steps <= 0:
        raise ValueError("max_time_steps must be positive")
    if max_cbs_nodes <= 0:
        raise ValueError("max_cbs_nodes must be positive")
    if algorithm not in {"prioritized", "cbs"}:
        raise ValueError("algorithm must be 'prioritized' or 'cbs'")
    order = [] if priority_order is None else [int(value) for value in priority_order]
    if order and (len(order) != len(starts) or sorted(order) != list(range(len(starts)))):
        raise ValueError("priority_order must contain every agent exactly once")

    def point(value: Sequence[int]) -> tuple[int, int]:
        if len(value) != 2:
            raise ValueError("agent points must contain exactly two integers")
        return int(value[0]), int(value[1])

    native = _backend().plan_cooperative(
        str(Path(map_path).expanduser()),
        [point(value) for value in starts],
        [point(value) for value in goals],
        allow_diagonal, allow_wait, max_time_steps, algorithm,
        allow_shared_goals, order, max_cbs_nodes)
    plans = tuple(CooperativeAgentPlan(
        bool(item["success"]), str(item["message"]),
        tuple((float(x), float(y)) for x, y in item["path"]),
        float(item["path_length"])) for item in native["plans"])
    return CooperativePlanResult(
        bool(native["success"]), str(native["message"]),
        int(native["planned_agents"]), int(native["vertex_conflicts"]),
        int(native["edge_conflicts"]), plans)
