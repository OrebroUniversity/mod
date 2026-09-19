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
 *   along with ompl_mod_objectives. If not, see <https://www.gnu.org/licenses/>
 */

#pragma once

#include <ompl/mod/objectives/MoDOptimizationObjective.h>

#include <mod/cliffmap.hpp>
#include <mod/gmmtmap.hpp>

namespace ompl::MoD {

/**
 * Upstream criterion: `sum_k pi_k (1 - cos(alpha - theta_k))` over the flow components at the point, from a
 * CLiFF-map (scaled by intensity q if an intensity map is given) or a GMMT-map (`params.type == gmmt`).
 * STeF-map support is not implemented (remnants kept as in the original).
 */
class UpstreamCriterionOptimizationObjective : public MoDOptimizationObjective {
  ::MoD::GMMTMapConstPtr gmmtmap_;
  ::MoD::CLiFFMapConstPtr cliffmap_;

 protected:
  double modCost(double x, double y, double alpha) const override;

 public:
  /// `params.type` selects the map: `gmmt` -> `gmmt_map_file`, anything else -> `cliff_map_file`.
  /// Maps are loaded from the files unless given preloaded.
  UpstreamCriterionOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                         const ::MoD::OptObjParameters &params,
                                         const ::MoD::SamplerParameters &sampler_params,
                                         ::MoD::CLiFFMapConstPtr cliffmap = nullptr,
                                         ::MoD::GMMTMapConstPtr gmmtmap = nullptr,
                                         ::MoD::IntensityMapConstPtr intensity_map = nullptr);

  ~UpstreamCriterionOptimizationObjective() override = default;

  double getGMMTMapCost(double x, double y, double alpha) const;
  double getCLiFFMapCost(double x, double y, double alpha) const;

  inline const ::MoD::CLiFFMapConstPtr &getCLiFFMap() const { return cliffmap_; }
  inline const ::MoD::GMMTMapConstPtr &getGMMTMap() const { return gmmtmap_; }
};

typedef std::shared_ptr<UpstreamCriterionOptimizationObjective> UpstreamCriterionOptimizationObjectivePtr;

}  // namespace ompl::MoD
