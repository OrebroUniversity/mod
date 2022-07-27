/*
 *   Copyright (c) 2019 Chittaranjan Srinivas Swaminathan
 *   This file is part of Maps of Dynamics library (libmod).
 *
 *   ompl_mod_objectives is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ompl_mod_objectives is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with ompl_planners_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <ompl/mod/objectives/IntensityMapOptimizationObjective.h>

#include <Eigen/Dense>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <cmath>

namespace ompl {
namespace MoD {

IntensityMapOptimizationObjective::IntensityMapOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                                                     const std::string &file_name, double wd, double wq,
                                                                     double wc, std::string sampler_type,
                                                                     double sampler_bias, bool sampler_debug)
    : ompl::MoD::MoDOptimizationObjective(si, wd, wq, wc, MapType::IntensityMap, sampler_type, file_name, sampler_bias,
                                          sampler_debug) {
  this->intensity_map_ = ::MoD::IntensityMap(file_name);
  description_ = "Intensity Cost";
  // Setup a default cost-to-go heuristic:
  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

ompl::base::Cost IntensityMapOptimizationObjective::stateCost(const ompl::base::State *s) const {
  return ompl::base::Cost(0.0);
}

ompl::base::Cost IntensityMapOptimizationObjective::motionCostHeuristic(const ompl::base::State *s1,
                                                                        const ompl::base::State *s2) const {
  return motionCost(s1, s2);
}

ompl::base::Cost IntensityMapOptimizationObjective::motionCost(const ompl::base::State *s1,
                                                               const ompl::base::State *s2) const {
  auto space = si_->getStateSpace();
  // 1. Declare the intermediate states.
  std::vector<ompl::base::State *> intermediate_states;

  // 2. How many segments do we want. Each segment should be approximately the
  // size of resolution.
  unsigned int numSegments = space->validSegmentCount(s1, s2);

  // 3. Get intermediate states.
  si_->getMotionStates(s1, s2, intermediate_states, numSegments - 1, true, true);

  double total_cost = 0.0;
  this->last_cost_.cost_d_ = 0.0;
  this->last_cost_.cost_q_ = 0.0;
  this->last_cost_.cost_c_ = 0.0;

  for (unsigned int i = 0; i < intermediate_states.size() - 1; i++) {
    std::array<double, 3> state_a{*space->getValueAddressAtIndex(intermediate_states[i], 0),
                                  *space->getValueAddressAtIndex(intermediate_states[i], 1),
                                  *space->getValueAddressAtIndex(intermediate_states[i], 2)};
    std::array<double, 3> state_b{*space->getValueAddressAtIndex(intermediate_states[i + 1], 0),
                                  *space->getValueAddressAtIndex(intermediate_states[i + 1], 1),
                                  *space->getValueAddressAtIndex(intermediate_states[i + 1], 2)};

    double dot = cos(state_b[2] / 2.0) * cos(state_a[2] / 2.0) + sin(state_b[2] / 2.0) * sin(state_a[2] / 2.0);

    double cost_d = si_->distance(intermediate_states[i], intermediate_states[i + 1]);
    double cost_q = (1.0 - dot * dot);
    double cost_c = intensity_map_(state_b[0], state_b[1]);

    total_cost += (weight_d_ * cost_d) + (weight_q_ * cost_q) + (weight_c_ * cost_c);
    this->last_cost_.cost_c_ += cost_c;
    this->last_cost_.cost_d_ += cost_d;
    this->last_cost_.cost_q_ += cost_q;
    si_->freeState(intermediate_states[i]);
  }
  si_->freeState(intermediate_states[intermediate_states.size() - 1]);
  return ompl::base::Cost(total_cost);
}
}  // namespace MoD
}  // namespace ompl