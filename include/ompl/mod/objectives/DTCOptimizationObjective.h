/*
 *   Copyright (c) 2019 Chittaranjan Srinivas Swaminathan
 *   This file is part of ompl_mod_objectives.
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

#pragma once

#include <ompl/mod/objectives/MoDOptimizationObjective.h>

#include <mod/cliffmap.hpp>

namespace ompl::MoD {

/**
 * Down-The-CLiFF cost: Mahalanobis distance of the motion (direction, max speed) to every CLiFF-map component at
 * the point, clamped at `mahalanobis_threshold`, weighted by the mixing factor (optional) and by the intensity q
 * (if an intensity map is given).
 */
class DTCOptimizationObjective : public MoDOptimizationObjective {
  ::MoD::CLiFFMapConstPtr cliffmap_;

 protected:
  double modCost(double x, double y, double alpha) const override;

 public:
  /// Maps are loaded from `params.cliff_map_file` / `params.intensity_map_file` unless given preloaded.
  DTCOptimizationObjective(const ompl::base::SpaceInformationPtr &si, const ::MoD::OptObjParameters &params,
                           const ::MoD::SamplerParameters &sampler_params, ::MoD::CLiFFMapConstPtr cliffmap = nullptr,
                           ::MoD::IntensityMapConstPtr intensity_map = nullptr);

  ~DTCOptimizationObjective() override = default;

  inline const ::MoD::CLiFFMapConstPtr &getCLiFFMap() const { return cliffmap_; }
};

typedef std::shared_ptr<DTCOptimizationObjective> DTCOptimizationObjectivePtr;

}  // namespace ompl::MoD
