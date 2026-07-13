#pragma once

// AutoPlanner — a C++ path planning library for mobile robots.
//
// Include this single header to access all planners and utilities.

// ── Core data structures ───────────────────────────────────────────
#include "autoplanner/core/grid_map.h"
#include "autoplanner/core/path.h"
#include "autoplanner/core/planner_base.h"
#include "autoplanner/core/planner_result.h"
#include "autoplanner/core/point.h"
#include "autoplanner/core/pose.h"
#include "autoplanner/core/node.h"

// ── Collision checking ─────────────────────────────────────────────
#include "autoplanner/collision/collision_checker.h"
#include "autoplanner/collision/grid_collision_checker.h"
#include "autoplanner/collision/line_collision_checker.h"

// ── Costmap ────────────────────────────────────────────────────────
#include "autoplanner/costmap/costmap_2d.h"
#include "autoplanner/costmap/distance_transform.h"
#include "autoplanner/costmap/obstacle_inflation.h"

// ── Heuristics ─────────────────────────────────────────────────────
#include "autoplanner/heuristics/heuristic.h"
#include "autoplanner/heuristics/euclidean.h"
#include "autoplanner/heuristics/manhattan.h"
#include "autoplanner/heuristics/diagonal.h"
#include "autoplanner/heuristics/reeds_shepp.h"

// ── Graph-search planners ──────────────────────────────────────────
#include "autoplanner/planners/graph_search/astar.h"
#include "autoplanner/planners/graph_search/dijkstra.h"
#include "autoplanner/planners/graph_search/weighted_astar.h"
#include "autoplanner/planners/graph_search/improved_astar.h"
#include "autoplanner/planners/graph_search/jps.h"
#include "autoplanner/planners/graph_search/dstar_lite.h"

// ── Sampling-based planners ────────────────────────────────────────
#include "autoplanner/planners/sampling/rrt.h"
#include "autoplanner/planners/sampling/rrt_star.h"
#include "autoplanner/planners/sampling/informed_rrt_star.h"
#include "autoplanner/planners/sampling/bi_rrt.h"

// ── Kinodynamic planners ───────────────────────────────────────────
#include "autoplanner/planners/kinodynamic/hybrid_astar.h"

// ── Path smoothing ─────────────────────────────────────────────────
#include "autoplanner/smoothing/path_smoother.h"
#include "autoplanner/smoothing/shortcut_smoother.h"
#include "autoplanner/smoothing/bezier_smoother.h"
#include "autoplanner/smoothing/bspline_smoother.h"
#include "autoplanner/smoothing/gradient_smoother.h"

// ── Metrics ────────────────────────────────────────────────────────
#include "autoplanner/metrics/path_metrics.h"
#include "autoplanner/metrics/benchmark_metrics.h"
#include "autoplanner/metrics/planner_profiler.h"

// ── I/O ────────────────────────────────────────────────────────────
#include "autoplanner/io/config_loader.h"
#include "autoplanner/io/map_loader.h"
#include "autoplanner/io/map_saver.h"
#include "autoplanner/io/path_writer.h"
#include "autoplanner/io/json_writer.h"

// ── Utilities ──────────────────────────────────────────────────────
#include "autoplanner/utils/logger.h"
#include "autoplanner/utils/math_utils.h"
#include "autoplanner/utils/random.h"
#include "autoplanner/utils/timer.h"
