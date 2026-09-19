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

#include <ompl/base/OptimizationObjective.h>

#include <memory>
#include <mod/cliffmap.hpp>
#include <mod/parameters.hpp>
#include <mod/sample_sink.hpp>
#include <string>

namespace ompl {
namespace MoD {

enum class MapType { CLiFFMap = 0, STeFMap = 1, GMMTMap = 2, IntensityMap = 4, NOTSET = 101 };

/// The three unweighted components of a motion cost: Euclidean/steering distance, quaternion distance, MoD cost.
struct CostComponents {
  double d{0.0};
  double q{0.0};
  double c{0.0};

  inline CostComponents &operator+=(const CostComponents &o) {
    d += o.d;
    q += o.q;
    c += o.c;
    return *this;
  }
  inline CostComponents operator+(const CostComponents &o) const {
    CostComponents r = *this;
    r += o;
    return r;
  }
};

/**
 * Base class of the MoD objectives. Thread-safe: it holds no mutable per-call state; maps are shared as
 * shared_ptr<const>. The cost of an edge is the sum over `n = max(1, ceil(distance / cost_step))` sub-segments,
 * obtained by `space->interpolate`, of `w_d * d_i + w_q * q_i + w_c * c_i`, where `c_i` is the MoD cost at the end
 * point of the sub-segment for the motion direction of that sub-segment (Paper IV, per-point cost).
 */
class MoDOptimizationObjective : public ompl::base::OptimizationObjective {
 protected:
  ::MoD::OptObjParameters params_;
  ::MoD::SamplerParameters sampler_params_;
  MapType map_type_{MapType::NOTSET};

  double weight_d_{1.0};
  double weight_q_{1.0};
  double weight_c_{1.0};

  /// Interpolation step for the cost integral [m]. Set by the playground to min(MoD cell, pixel); the library
  /// default is the cell size of the map given to the objective.
  double cost_step_{1.0};

  /// The intensity (q) map of the objective (empty if the objective has none).
  ::MoD::IntensityMapConstPtr intensity_map_;

  /// The intensity map handed to the samplers: `sampler_params_.intensity_map_file` if set, else `intensity_map_`.
  ::MoD::IntensityMapConstPtr sampler_intensity_map_;

  /// Optional sample sink handed to every sampler this objective allocates (not owned).
  ::MoD::SampleSink *sample_sink_{nullptr};

  MoDOptimizationObjective(const ompl::base::SpaceInformationPtr &si, const ::MoD::OptObjParameters &params,
                           const ::MoD::SamplerParameters &sampler_params, MapType map_type,
                           ::MoD::IntensityMapConstPtr intensity_map);

  /// MoD cost at (x, y) for a motion in direction `alpha` (the velocity direction, not the robot heading).
  virtual double modCost(double x, double y, double alpha) const = 0;

  /// Cost components of one sub-segment a -> b.
  CostComponents pointCost(const ompl::base::State *a, const ompl::base::State *b) const;

 public:
  ~MoDOptimizationObjective() override = default;

  inline void setCostStep(double metres) { cost_step_ = metres; }
  inline double getCostStep() const { return cost_step_; }

  /// Replaces the intensity map handed to the samplers (to share a preloaded map).
  inline void setSamplerIntensityMap(::MoD::IntensityMapConstPtr map) { sampler_intensity_map_ = std::move(map); }

  /// Samplers allocated after this call record their draws into `sink` (may be null). Not owned.
  inline void setSampleSink(::MoD::SampleSink *sink) { sample_sink_ = sink; }
  inline ::MoD::SampleSink *getSampleSink() const { return sample_sink_; }

  inline const ::MoD::OptObjParameters &getParameters() const { return params_; }
  inline const ::MoD::SamplerParameters &getSamplerParameters() const { return sampler_params_; }
  inline const ::MoD::IntensityMapConstPtr &getIntensityMap() const { return intensity_map_; }
  inline const ::MoD::IntensityMapConstPtr &getSamplerIntensityMap() const { return sampler_intensity_map_; }

  /// Unweighted components of `motionCost(s1, s2)`, computed with the same interpolation.
  virtual CostComponents motionCostComponents(const ompl::base::State *s1, const ompl::base::State *s2) const;

  ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override;
  ompl::base::Cost stateCost(const ompl::base::State *s) const override;
  ompl::base::Cost motionCostHeuristic(const ompl::base::State *s1, const ompl::base::State *s2) const override;
  bool isSymmetric() const override { return false; }

  ompl::base::InformedSamplerPtr allocInformedStateSampler(const ompl::base::ProblemDefinitionPtr &probDefn,
                                                           unsigned int maxNumberCalls) const override;

  std::string getMapTypeStr() const;
  inline MapType getMapType() const { return map_type_; }
};

typedef std::shared_ptr<MoDOptimizationObjective> MoDOptimizationObjectivePtr;

}  // namespace MoD
}  // namespace ompl
