#pragma once

// AutoMPC — Model Predictive Control for mobile robot trajectory tracking.

#include "autompc/core/types.h"
#include "autompc/core/trajectory.h"
#include "autompc/controllers/controllers.h"
#include "autompc/trajectory/trajectory_tracker.h"
#include "autompc/trajectory/error_metrics.h"

#ifdef AUTOMPC_HAS_EIGEN
#include "autompc/controllers/lqr_controller.h"
#endif
