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

#include <ompl/mod/samplers/HybridSampler.h>

namespace ompl::MoD {

HybridSampler::HybridSampler(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                             const ::MoD::SamplerParameters &params, ::MoD::IntensityMapConstPtr intensity_map)
    : ompl::base::InformedSampler(pdef, maxCalls),
      intensity_bias_(params.hybrid_intensity_bias),
      dijkstra_bias_(params.bias) {
  ::MoD::SamplerParameters sub = params;
  sub.bias = 1.0;
  dijkstra_sampler_ = std::make_shared<DijkstraSampler>(pdef, maxCalls, sub, intensity_map);
  intensity_map_sampler_ = std::make_shared<IntensityMapSampler>(pdef, maxCalls, sub, intensity_map);
  ellipse_sampler_ = std::make_shared<ompl::base::PathLengthDirectInfSampler>(pdef, maxCalls);
}

bool HybridSampler::sampleUniform(ompl::base::State *state, const ompl::base::Cost &maxCost) {
  const double rndm = rng_.uniformReal(0.0, 1.0);

  if (rndm < dijkstra_bias_) {
    if (branch_hook_) branch_hook_(Branch::dijkstra);
    return dijkstra_sampler_->sampleUniform(state, maxCost);
  } else if (rndm < (intensity_bias_ + dijkstra_bias_)) {
    if (branch_hook_) branch_hook_(Branch::intensity);
    return intensity_map_sampler_->sampleUniform(state, maxCost);
  } else {
    if (branch_hook_) branch_hook_(Branch::ellipse);
    bool result;
    if (uniform_valid_) {
      intensity_map_sampler_->setBias(0.0);
      result = ellipse_sampler_->sampleUniform(state, maxCost);
      intensity_map_sampler_->setBias(1.0);
    } else {
      result = ellipse_sampler_->sampleUniform(state, maxCost);
    }
    if (sink_ && result) {
      const auto *se2 = state->as<ompl::base::SE2StateSpace::StateType>();
      sink_->record(se2->getX(), se2->getY(), se2->getYaw(), ::MoD::SampleSource::ellipse);
    }
    return result;
  }
}

}  // namespace ompl::MoD
