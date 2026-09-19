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

#include <stdexcept>

namespace ompl::MoD {

IntensityMapOptimizationObjective::IntensityMapOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                                                     const ::MoD::OptObjParameters &params,
                                                                     const ::MoD::SamplerParameters &sampler_params,
                                                                     ::MoD::IntensityMapConstPtr intensity_map)
    : MoDOptimizationObjective(si, params, sampler_params, MapType::IntensityMap, std::move(intensity_map)) {
  if (!intensity_map_) throw std::invalid_argument("IntensityMapOptimizationObjective: no intensity map given");
  description_ = "Intensity Cost";
}

double IntensityMapOptimizationObjective::modCost(double x, double y, double /*alpha*/) const {
  return (*intensity_map_)(x, y);
}

}  // namespace ompl::MoD
