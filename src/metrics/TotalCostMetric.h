#pragma once

#include <cmath>
#include <vector>

#include "TrajectoryMetric.h"
#include "utils/PlannerUtils.hpp"

class TotalCostMetric : public TMetric<TotalCostMetric> {
 public:
  static double evaluateMetric(const ompl::geometric::PathGeometric &trajectory,
                               double) {
    return PlannerUtils::totalCost(trajectory);
  }

  static double evaluateMetric(const ompl::control::PathControl &trajectory,
                               double) {
    return PlannerUtils::totalCost(trajectory);
  }
  static const bool MoreIsBetter = false;
};