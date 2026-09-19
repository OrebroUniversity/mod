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

#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/util/RandomNumbers.h>

#include <memory>
#include <mod/cliffmap.hpp>
#include <mod/parameters.hpp>
#include <mod/sample_sink.hpp>
#include <vector>

namespace ompl {
namespace MoD {

/**
 * Informed sampler over the valid cells of an intensity map. With probability `bias` a cell is drawn with
 * probability proportional to (1 - q) ("q branch", prefix sums + upper_bound); otherwise uniformly over the
 * valid cells ("uniform branch", direct index). The position is uniform within the chosen cell and clamped to
 * the state bounds; the heading is uniform.
 */
class IntensityMapSampler : public ompl::base::InformedSampler {
 private:
  bool checkValidity(double xi, double yi);

 protected:
  /// Valid cells, struct of arrays, sorted ascending by weight (1 - q).
  std::vector<double> xs_;
  std::vector<double> ys_;
  std::vector<double> weights_;
  /// Cumulative weights: prefix_q_[i] = sum of weights_[0..i].
  std::vector<double> prefix_q_;

  double half_cell_size{0.0};

  double bias_{0.5};

  double x_min_{0.0};
  double x_max_{0.0};
  double y_min_{0.0};
  double y_max_{0.0};

  ompl::RNG rng_;

  /// Optional receiver of every draw (not owned).
  ::MoD::SampleSink *sink_{nullptr};

 public:
  IntensityMapSampler(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                      const ::MoD::SamplerParameters &params, ::MoD::IntensityMapConstPtr intensity_map);

  static ompl::base::InformedSamplerPtr allocate(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                                                 const ::MoD::SamplerParameters &params,
                                                 ::MoD::IntensityMapConstPtr intensity_map) {
    return std::make_shared<IntensityMapSampler>(pdef, maxCalls, params, std::move(intensity_map));
  }

  ~IntensityMapSampler() override = default;

  void setBias(double bias) { bias_ = bias; }
  double getBias() const { return bias_; }

  /// Number of valid cells.
  size_t size() const { return xs_.size(); }

  void setup(const ::MoD::IntensityMap &intensity_map);

  bool sampleUniform(ompl::base::State *state, const ompl::base::Cost &maxCost) override;

  inline bool sampleUniform(ompl::base::State *state, const ompl::base::Cost & /*minCost*/,
                            const ompl::base::Cost &maxCost) override {
    return sampleUniform(state, maxCost);
  }

  inline void setSampleSink(::MoD::SampleSink *sink) { sink_ = sink; }

  inline bool hasInformedMeasure() const override { return false; }

  inline double getInformedMeasure(const ompl::base::Cost & /*currentCost*/) const override {
    return this->space_->getMeasure();
  }

  void sampleNecessarilyValid(ompl::base::State *state);
};

}  // namespace MoD
}  // namespace ompl
