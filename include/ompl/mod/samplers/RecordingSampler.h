/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of mod.
 *
 *   mod is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   mod is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with mod.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <memory>
#include <mod/sample_sink.hpp>

namespace ompl::MoD {

/// Forwards to another informed sampler and records every successful draw into a sink under a fixed source.
class RecordingSampler : public ompl::base::InformedSampler {
  ompl::base::InformedSamplerPtr inner_;
  ::MoD::SampleSource source_;
  ::MoD::SampleSink *sink_;

  void record(const ompl::base::State *state) const {
    const auto *se2 = state->as<ompl::base::SE2StateSpace::StateType>();
    sink_->record(se2->getX(), se2->getY(), se2->getYaw(), source_);
  }

 public:
  RecordingSampler(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                   ompl::base::InformedSamplerPtr inner, ::MoD::SampleSource source, ::MoD::SampleSink *sink)
      : ompl::base::InformedSampler(pdef, maxCalls), inner_(std::move(inner)), source_(source), sink_(sink) {}

  bool sampleUniform(ompl::base::State *state, const ompl::base::Cost &maxCost) override {
    const bool ok = inner_->sampleUniform(state, maxCost);
    if (ok && sink_) record(state);
    return ok;
  }

  bool sampleUniform(ompl::base::State *state, const ompl::base::Cost &minCost,
                     const ompl::base::Cost &maxCost) override {
    const bool ok = inner_->sampleUniform(state, minCost, maxCost);
    if (ok && sink_) record(state);
    return ok;
  }

  bool hasInformedMeasure() const override { return inner_->hasInformedMeasure(); }

  double getInformedMeasure(const ompl::base::Cost &currentCost) const override {
    return inner_->getInformedMeasure(currentCost);
  }

  ompl::base::Cost heuristicSolnCost(const ompl::base::State *statePtr) const override {
    return inner_->heuristicSolnCost(statePtr);
  }
};

}  // namespace ompl::MoD
