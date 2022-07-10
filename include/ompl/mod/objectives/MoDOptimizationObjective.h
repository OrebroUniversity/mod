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

#include <boost/log/trivial.hpp>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/samplers/informed/RejectionInfSampler.h>

#include "ompl/mod/samplers/IntensityMapSampler.h"
#include "ompl/mod/samplers/DijkstraSampler.h"

namespace ompl {
namespace MoD {

enum class MapType {
  CLiFFMap = 0,
  STeFMap = 1,
  GMMTMap = 2,
  IntensityMap = 4,
  NOTSET = 101
};

struct Cost {
  /// The last computed distance cost
  double cost_d_{0.0};

  /// The last computed quaternion distance cost
  double cost_q_{0.0};

  /// The last computed MoD cost
  double cost_c_{0.0};

  inline Cost operator+(Cost b) const {
    Cost result;
    result.cost_c_ = this->cost_c_ + b.cost_c_;
    result.cost_d_ = this->cost_d_ + b.cost_d_;
    result.cost_q_ = this->cost_q_ + b.cost_q_;
    return result;
  }
};

class MoDOptimizationObjective : public ompl::base::OptimizationObjective {
 protected:
  /// The weight associated with Euclidean distance cost.
  double weight_d_;

  /// The weight associated with quaternion distance cost.
  double weight_q_;

  /// The weight associated with Down-The-CLiFF cost.
  double weight_c_;

  mutable Cost last_cost_;

  MapType map_type_{MapType::NOTSET};

  std::string informed_sampler_type_;

  std::string intensity_map_file_name_;

  double sampler_bias_;

  bool sampler_debug_{false};

  inline MoDOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                  double weight_d,
                                  double weight_q,
                                  double weight_c,
                                  MapType map_type,
                                  const std::string &sampler_type = "",
                                  const std::string &intensity_map_file_name = "",
                                  double sampler_bias = 0.05,
                                  bool sampler_debug = false)
      : ompl::base::OptimizationObjective(si),
        weight_d_(weight_d),
        weight_q_(weight_q),
        weight_c_(weight_c),
        map_type_(map_type),
        informed_sampler_type_(sampler_type),
        intensity_map_file_name_(intensity_map_file_name),
        sampler_bias_(sampler_bias),
        sampler_debug_(sampler_debug) {}

 public:
  inline double getLastCostD() const { return last_cost_.cost_d_; }
  inline double getLastCostQ() const { return last_cost_.cost_q_; }
  inline double getLastCostC() const { return last_cost_.cost_c_; }
  inline Cost getLastCost() const { return last_cost_; }

  ompl::base::Cost motionCost(const ompl::base::State *s1,
                              const ompl::base::State *s2) const override = 0;

  virtual ompl::base::InformedSamplerPtr allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDefn,
                                                                   unsigned int maxNumberCalls) const override {
    if (this->informed_sampler_type_ == "dijkstra") {
      OMPL_INFORM("MoDOptimization Objective will use Dijkstra Sampling...");
      return ompl::MoD::DijkstraSampler::allocate(probDefn,
                                                  maxNumberCalls,
                                                  0.5,
                                                  sampler_bias_,
                                                  sampler_debug_);
    } else if (this->informed_sampler_type_ == "intensity") {
      OMPL_INFORM("MoDOptimization Objective will use intensity-map Sampling...");
      return ompl::MoD::IntensityMapSampler::allocate(probDefn,
                                                      maxNumberCalls,
                                                      intensity_map_file_name_,
                                                      sampler_bias_,
                                                      sampler_debug_);
    } else {
      OMPL_INFORM(
          "informed_sampler_type = %s is not available for MoDOptimizationObjective, defaulting to rejection sampling.",
          (informed_sampler_type_.empty() or informed_sampler_type_ == "iid") ? "<empty> or iid"
                                                                              : informed_sampler_type_.c_str());
      return ompl::MoD::IntensityMapSampler::allocate(probDefn,
                                                      maxNumberCalls,
                                                      intensity_map_file_name_,
                                                      0.0,
                                                      sampler_debug_);
    }
  }

  inline std::string getMapTypeStr() const {
    if (this->weight_c_ == 0.0)
      return "RRTStar";
    else
      switch (map_type_) {
        case MapType::STeFMap:return "STeF-map";
        case MapType::GMMTMap:return "GMMT-map";
        case MapType::CLiFFMap:return "CLiFF-map";
        case MapType::IntensityMap:return "intensity-map";
        default:return "Not set.";
      }
  }
  inline MapType getMapType() const { return map_type_; }
};

typedef std::shared_ptr<MoDOptimizationObjective> MoDOptimizationObjectivePtr;

} // namespace MoD
} // namespace ompl