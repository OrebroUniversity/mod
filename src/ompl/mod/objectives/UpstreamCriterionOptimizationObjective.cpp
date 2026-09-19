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

#include <ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h>

#include <boost/geometry.hpp>
#include <cmath>
#include <mod/log.hpp>

namespace ompl::MoD {

UpstreamCriterionOptimizationObjective::UpstreamCriterionOptimizationObjective(
    const ompl::base::SpaceInformationPtr &si, const ::MoD::OptObjParameters &params,
    const ::MoD::SamplerParameters &sampler_params, ::MoD::CLiFFMapConstPtr cliffmap, ::MoD::GMMTMapConstPtr gmmtmap,
    ::MoD::IntensityMapConstPtr intensity_map)
    : MoDOptimizationObjective(si, params, sampler_params,
                               params.type == ::MoD::ObjectiveType::gmmt ? MapType::GMMTMap : MapType::CLiFFMap,
                               std::move(intensity_map)),
      gmmtmap_(std::move(gmmtmap)),
      cliffmap_(std::move(cliffmap)) {
  if (map_type_ == MapType::GMMTMap) {
    if (!gmmtmap_) {
      if (params_.gmmt_map_file.empty())
        throw std::invalid_argument("UpstreamCriterionOptimizationObjective: no GMMT-map given");
      gmmtmap_ = std::make_shared<const ::MoD::GMMTMap>(params_.gmmt_map_file);
    }
    description_ = "Upstream Cost over GMMT-map";
    if (!intensity_map_) {
      cost_step_ = 1.0;
      MOD_LOG("Upstream/GMMT: no grid map to infer the cost step from, default %.2f m", cost_step_);
    }
  } else {
    if (!cliffmap_) {
      if (params_.cliff_map_file.empty())
        throw std::invalid_argument("UpstreamCriterionOptimizationObjective: no CLiFF-map given");
      cliffmap_ = std::make_shared<const ::MoD::CLiFFMap>(params_.cliff_map_file, true);
    }
    if (!cliffmap_->isOrganized())
      throw std::invalid_argument("UpstreamCriterionOptimizationObjective: CLiFF-map must be a grid");
    description_ = intensity_map_ ? "Upstream+q Cost over CLiFF-map" : "Upstream Cost over CLiFF-map";
    cost_step_ = cliffmap_->getResolution();
  }
}

double UpstreamCriterionOptimizationObjective::modCost(double x, double y, double alpha) const {
  switch (map_type_) {
    case MapType::GMMTMap:
      return getGMMTMapCost(x, y, alpha);
    case MapType::CLiFFMap:
      return getCLiFFMapCost(x, y, alpha);
    default:
      return identityCost().value();
  }
}

double UpstreamCriterionOptimizationObjective::getGMMTMapCost(double x, double y, double alpha) const {
  double mod_cost = 0.0;
  const auto dists = (*gmmtmap_)(x, y);
  for (const auto &dist : dists) {
    const double mixing_factor = gmmtmap_->getMixingFactorByClusterID(dist.second[0]);
    const double dist_heading = gmmtmap_->getHeadingAtDist(dist.second[0], dist.second[1]);
    const double distance_to_mean = boost::geometry::distance(dist.first, ::MoD::Point2D(x, y));
    mod_cost += mixing_factor * (1 - distance_to_mean / gmmtmap_->getStdDev()) * (1 - std::cos(alpha - dist_heading));
  }
  return mod_cost;
}

double UpstreamCriterionOptimizationObjective::getCLiFFMapCost(double x, double y, double alpha) const {
  double mod_cost = 0.0;
  const ::MoD::CLiFFMapLocation &cl = (*cliffmap_)(x, y);
  for (const auto &dist : cl.distributions) {
    mod_cost += dist.getMixingFactor() * (1 - std::cos(dist.getMeanHeading() - alpha));
  }
  if (intensity_map_) mod_cost *= (*intensity_map_)(x, y);
  return mod_cost;
}

}  // namespace ompl::MoD
