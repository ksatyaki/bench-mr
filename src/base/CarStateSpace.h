/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of ompl_planners_ros.
 *
 *   ompl_planners_ros is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ompl_planners_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with ompl_planners_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <ompl/base/spaces/DubinsStateSpace.h>

namespace ompl::base {
class CarStateSpace : public ompl::base::DubinsStateSpace {
 public:
  explicit CarStateSpace(double turning_radius, bool is_symmetric = true);
  [[nodiscard]] bool isMetricSpace() const override {return true;}
  unsigned int validSegmentCount(const State *state1,
                                 const State *state2) const override;
};
}  // namespace ompl
