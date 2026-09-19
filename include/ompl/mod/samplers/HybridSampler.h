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

#pragma once

#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <ompl/mod/samplers/DijkstraSampler.h>
#include <ompl/mod/samplers/IntensityMapSampler.h>

#include <functional>
#include <mod/parameters.hpp>
#include <mod/sample_sink.hpp>

namespace ompl::MoD {

/**
 * Hybrid sampler of Paper IV: with probability `bias` (alpha) the Dijkstra sampler, with probability
 * `hybrid_intensity_bias` the intensity sampler, otherwise the ellipsoidal (informed) sampler, which is uniform
 * until a solution exists. The sub-samplers are built with bias 1 so their informed branch is always taken.
 */
class HybridSampler : public ompl::base::InformedSampler {
 public:
  enum class Branch { dijkstra, intensity, ellipse };
  typedef std::function<void(Branch)> BranchHook;

 protected:
  std::shared_ptr<IntensityMapSampler> intensity_map_sampler_;
  std::shared_ptr<DijkstraSampler> dijkstra_sampler_;
  std::shared_ptr<ompl::base::InformedSampler> ellipse_sampler_;

  double intensity_bias_{0.0};
  double dijkstra_bias_{0.0};
  bool uniform_valid_{false};

  ompl::RNG rng_;

  /// Optional receiver of every draw (not owned).
  ::MoD::SampleSink *sink_{nullptr};

  /// Test hook: called with the sub-sampler chosen for each draw.
  BranchHook branch_hook_;

 public:
  HybridSampler(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                const ::MoD::SamplerParameters &params, ::MoD::IntensityMapConstPtr intensity_map);

  static ompl::base::InformedSamplerPtr allocate(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                                                 const ::MoD::SamplerParameters &params,
                                                 ::MoD::IntensityMapConstPtr intensity_map) {
    return std::make_shared<HybridSampler>(pdef, maxCalls, params, std::move(intensity_map));
  }

  ~HybridSampler() override = default;

  inline void setUniformValid(bool uniform_valid) { uniform_valid_ = uniform_valid; }
  inline void setBranchHook(BranchHook hook) { branch_hook_ = std::move(hook); }

  bool sampleUniform(ompl::base::State *state, const ompl::base::Cost &maxCost) override;

  inline bool sampleUniform(ompl::base::State *state, const ompl::base::Cost & /*minCost*/,
                            const ompl::base::Cost &maxCost) override {
    return sampleUniform(state, maxCost);
  }

  /// The sink receives dijkstra / intensity draws from the sub-samplers and the ellipse branch from here.
  inline void setSampleSink(::MoD::SampleSink *sink) {
    sink_ = sink;
    dijkstra_sampler_->setSampleSink(sink);
    intensity_map_sampler_->setSampleSink(sink);
  }

  inline bool hasInformedMeasure() const override { return false; }

  inline double getInformedMeasure(const ompl::base::Cost & /*currentCost*/) const override {
    return this->space_->getMeasure();
  }
};

}  // namespace ompl::MoD
