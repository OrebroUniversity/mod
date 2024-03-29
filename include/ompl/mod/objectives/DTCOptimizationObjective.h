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

#include <Eigen/Dense>
#include <array>
#include <functional>
#include <mod/cliffmap.hpp>

namespace ompl::MoD {
/**
 * The optimization objective class for DownTheCLiFF cost.
 * This is a multi-optimization objective but doens't derive from the
 * corresponding OMPL class.
 */
class DTCOptimizationObjective : public MoDOptimizationObjective {
  /// Maximum vehicle speed used in the computation of Down-The-CLiFF cost.
  double max_vehicle_speed;

  /// Mahalanobis distance threshold.
  double mahalanobis_distance_threshold;

  /// Will this cost objective use the mixing factor?
  bool use_mixing_factor;

  /// A std smart pointer to the CLiFFMap.
  ::MoD::CLiFFMap cliffmap;

  ::MoD::IntensityMap intensitymap;

  bool use_intensity{false};

 public:
  /**
   * Constructor
   * @param si SpaceInformationPtr that we get from the problem setup.
   */
  DTCOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                           const ::MoD::CLiFFMap &cliffmap,
                           double wd,
                           double wq,
                           double wc,
                           double maxvs,
                           double mahalanobis_distance_threshold = 10.0,
                           bool use_mixing_factor = true,
                           const std::string &sampler_type = "",
                           const std::string &intensity_map_file_name = "",
                           double bias = 0.05,
                           bool uniform_valid = false,
                           bool debug = false);

  DTCOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                           const std::string &cliffmap_file_name,
                           const std::string &intensity_map_file_name,
                           double wd,
                           double wq,
                           double wc,
                           double maxvs,
                           double mahalanobis_distance_threshold = 10.0,
                           bool use_mixing_factor = true,
                           const std::string &sampler_type = "",
                           double bias = 0.05,
                           bool uniform_valid = false,
                           bool debug = false);

  ~DTCOptimizationObjective() override = default;

  bool isSymmetric() const override { return false; }

  ompl::base::Cost stateCost(const ompl::base::State *s) const override;

  ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override;

  inline void setMahalanobisDistanceThreshold(double mahalanobis_distance_threshold) {
    this->mahalanobis_distance_threshold = mahalanobis_distance_threshold;
  }

  ompl::base::Cost motionCostHeuristic(const ompl::base::State *s1, const ompl::base::State *s2) const override;
};

typedef std::shared_ptr<DTCOptimizationObjective> DTCOptimizationObjectivePtr;

}  // namespace ompl
