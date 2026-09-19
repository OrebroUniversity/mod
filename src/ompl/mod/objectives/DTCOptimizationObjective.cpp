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

#include <ompl/mod/objectives/DTCOptimizationObjective.h>

#include <Eigen/Dense>
#include <cmath>
#include <mod/log.hpp>

namespace ompl::MoD {

DTCOptimizationObjective::DTCOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                                   const ::MoD::OptObjParameters &params,
                                                   const ::MoD::SamplerParameters &sampler_params,
                                                   ::MoD::CLiFFMapConstPtr cliffmap,
                                                   ::MoD::IntensityMapConstPtr intensity_map)
    : MoDOptimizationObjective(si, params, sampler_params, MapType::CLiFFMap, std::move(intensity_map)),
      cliffmap_(std::move(cliffmap)) {
  if (!cliffmap_) {
    if (params_.cliff_map_file.empty()) throw std::invalid_argument("DTCOptimizationObjective: no CLiFF-map given");
    cliffmap_ = std::make_shared<const ::MoD::CLiFFMap>(params_.cliff_map_file, true);
  }
  if (!cliffmap_->isOrganized()) throw std::invalid_argument("DTCOptimizationObjective: CLiFF-map must be a grid");
  description_ = intensity_map_ ? "DownTheCLiFF-q Cost" : "DownTheCLiFF Cost";
  cost_step_ = cliffmap_->getResolution();
}

double DTCOptimizationObjective::modCost(double x, double y, double alpha) const {
  Eigen::Vector2d V;
  V[0] = alpha;
  V[1] = params_.max_vehicle_speed;

  const ::MoD::CLiFFMapLocation &cl = (*cliffmap_)(x, y);
  double cost_c = 0.0;
  for (const auto &dist : cl.distributions) {
    Eigen::Matrix2d Sigma;
    const std::array<double, 4> sigma_array = dist.getCovariance();
    Sigma(0, 0) = sigma_array[0];
    Sigma(0, 1) = sigma_array[1];
    Sigma(1, 0) = sigma_array[2];
    Sigma(1, 1) = sigma_array[3];
    Eigen::Vector2d myu;
    myu[0] = std::atan2(std::sin(dist.getMeanHeading()), std::cos(dist.getMeanHeading()));
    myu[1] = dist.getMeanSpeed();

    double inc_cost;
    const double det = Sigma.determinant();
    if (det < 1e-8 && det > -1e-8) {
      inc_cost = params_.mahalanobis_threshold;
    } else {
      double mahalanobis = std::sqrt((V - myu).transpose() * Sigma.inverse() * (V - myu));
      if (mahalanobis > params_.mahalanobis_threshold) mahalanobis = params_.mahalanobis_threshold;
      inc_cost = mahalanobis;
    }
    if (std::isnan(inc_cost)) inc_cost = params_.mahalanobis_threshold;
    if (params_.use_mixing_factor) inc_cost *= dist.getMixingFactor();
    cost_c += inc_cost;
  }
  if (intensity_map_) cost_c *= (*intensity_map_)(x, y);
  return cost_c;
}

}  // namespace ompl::MoD
