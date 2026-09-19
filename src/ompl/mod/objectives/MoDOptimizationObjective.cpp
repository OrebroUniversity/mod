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

#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <ompl/base/samplers/informed/RejectionInfSampler.h>
#include <ompl/mod/objectives/MoDOptimizationObjective.h>
#include <ompl/mod/samplers/DijkstraSampler.h>
#include <ompl/mod/samplers/HybridSampler.h>
#include <ompl/mod/samplers/IntensityMapSampler.h>
#include <ompl/mod/samplers/RecordingSampler.h>

#include <algorithm>
#include <cmath>
#include <mod/log.hpp>

namespace ompl::MoD {

MoDOptimizationObjective::MoDOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                                   const ::MoD::OptObjParameters &params,
                                                   const ::MoD::SamplerParameters &sampler_params, MapType map_type,
                                                   ::MoD::IntensityMapConstPtr intensity_map)
    : ompl::base::OptimizationObjective(si),
      params_(params),
      sampler_params_(sampler_params),
      map_type_(map_type),
      weight_d_(params.w_d),
      weight_q_(params.w_q),
      weight_c_(params.w_c),
      intensity_map_(std::move(intensity_map)) {
  if (!intensity_map_ && !params_.intensity_map_file.empty()) {
    intensity_map_ = std::make_shared<const ::MoD::IntensityMap>(params_.intensity_map_file);
  }
  if (!sampler_params_.intensity_map_file.empty() && sampler_params_.intensity_map_file != params_.intensity_map_file) {
    sampler_intensity_map_ = std::make_shared<const ::MoD::IntensityMap>(sampler_params_.intensity_map_file);
  } else {
    sampler_intensity_map_ = intensity_map_;
  }
  if (intensity_map_) cost_step_ = intensity_map_->getCellSize();
  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

CostComponents MoDOptimizationObjective::pointCost(const ompl::base::State *a, const ompl::base::State *b) const {
  const auto space = si_->getStateSpace();
  const double xa = *space->getValueAddressAtIndex(a, 0);
  const double ya = *space->getValueAddressAtIndex(a, 1);
  const double ta = *space->getValueAddressAtIndex(a, 2);
  const double xb = *space->getValueAddressAtIndex(b, 0);
  const double yb = *space->getValueAddressAtIndex(b, 1);
  const double tb = *space->getValueAddressAtIndex(b, 2);

  CostComponents cc;
  cc.d = si_->distance(a, b);
  const double dot = std::cos((tb - ta) / 2.0);
  cc.q = 1.0 - dot * dot;
  const double alpha = std::atan2(yb - ya, xb - xa);
  cc.c = modCost(xb, yb, alpha);
  return cc;
}

CostComponents MoDOptimizationObjective::motionCostComponents(const ompl::base::State *s1,
                                                              const ompl::base::State *s2) const {
  const auto space = si_->getStateSpace();
  const double distance = si_->distance(s1, s2);
  const double step = cost_step_ > 0.0 ? cost_step_ : distance;
  const unsigned int n = std::max(1u, static_cast<unsigned int>(std::ceil(distance / step)));

  ompl::base::State *a = si_->allocState();
  ompl::base::State *b = si_->allocState();
  si_->copyState(a, s1);

  CostComponents total;
  for (unsigned int i = 1; i <= n; ++i) {
    if (i == n)
      si_->copyState(b, s2);
    else
      space->interpolate(s1, s2, static_cast<double>(i) / static_cast<double>(n), b);
    total += pointCost(a, b);
    std::swap(a, b);
  }
  si_->freeState(a);
  si_->freeState(b);
  return total;
}

ompl::base::Cost MoDOptimizationObjective::motionCost(const ompl::base::State *s1,
                                                      const ompl::base::State *s2) const {
  const CostComponents cc = motionCostComponents(s1, s2);
  return ompl::base::Cost(weight_d_ * cc.d + weight_q_ * cc.q + weight_c_ * cc.c);
}

ompl::base::Cost MoDOptimizationObjective::stateCost(const ompl::base::State * /*s*/) const {
  return ompl::base::Cost(0.0);
}

ompl::base::Cost MoDOptimizationObjective::motionCostHeuristic(const ompl::base::State *s1,
                                                               const ompl::base::State *s2) const {
  return motionCost(s1, s2);
}

std::string MoDOptimizationObjective::getMapTypeStr() const {
  if (weight_c_ == 0.0) return "RRTStar";
  switch (map_type_) {
    case MapType::STeFMap:
      return "STeF-map";
    case MapType::GMMTMap:
      return "GMMT-map";
    case MapType::CLiFFMap:
      return "CLiFF-map";
    case MapType::IntensityMap:
      return "intensity-map";
    default:
      return "Not set.";
  }
}

ompl::base::InformedSamplerPtr MoDOptimizationObjective::allocInformedStateSampler(
    const ompl::base::ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls) const {
  const std::string type = ::MoD::to_string(sampler_params_.type);
  switch (sampler_params_.type) {
    case ::MoD::SamplerType::dijkstra: {
      MOD_LOG("Informed sampler: dijkstra (bias %.3f, cell %.3f m)", sampler_params_.bias,
              sampler_params_.dijkstra_cell_size);
      auto s = std::make_shared<DijkstraSampler>(probDefn, maxNumberCalls, sampler_params_, sampler_intensity_map_);
      s->setSampleSink(sample_sink_);
      return s;
    }
    case ::MoD::SamplerType::ellipse: {
      MOD_LOG("Informed sampler: ellipse (PathLengthDirectInfSampler)");
      auto s = std::make_shared<ompl::base::PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
      if (!sample_sink_) return s;
      return std::make_shared<RecordingSampler>(probDefn, maxNumberCalls, s, ::MoD::SampleSource::ellipse,
                                                sample_sink_);
    }
    case ::MoD::SamplerType::intensity: {
      if (!sampler_intensity_map_) break;
      MOD_LOG("Informed sampler: intensity (bias %.3f)", sampler_params_.bias);
      auto s =
          std::make_shared<IntensityMapSampler>(probDefn, maxNumberCalls, sampler_params_, sampler_intensity_map_);
      s->setSampleSink(sample_sink_);
      return s;
    }
    case ::MoD::SamplerType::hybrid: {
      if (!sampler_intensity_map_) break;
      MOD_LOG("Informed sampler: hybrid (dijkstra bias %.3f, intensity bias %.3f, cell %.3f m)",
              sampler_params_.bias, sampler_params_.hybrid_intensity_bias, sampler_params_.dijkstra_cell_size);
      auto s = std::make_shared<HybridSampler>(probDefn, maxNumberCalls, sampler_params_, sampler_intensity_map_);
      s->setSampleSink(sample_sink_);
      return s;
    }
    case ::MoD::SamplerType::iid:
      if (sampler_intensity_map_) {
        // As in Paper IV: uniform over the valid cells of the intensity grid (intensity sampler with bias 0).
        MOD_LOG("Informed sampler: iid (uniform over valid intensity-map cells)");
        ::MoD::SamplerParameters p = sampler_params_;
        p.bias = 0.0;
        auto s = std::make_shared<IntensityMapSampler>(probDefn, maxNumberCalls, p, sampler_intensity_map_);
        s->setSampleSink(sample_sink_);
        return s;
      }
      break;
  }
  MOD_LOG("Informed sampler: %s needs an intensity map and none is configured; using rejection sampling",
          type.c_str());
  auto s = std::make_shared<ompl::base::RejectionInfSampler>(probDefn, maxNumberCalls);
  if (!sample_sink_) return s;
  return std::make_shared<RecordingSampler>(probDefn, maxNumberCalls, s, ::MoD::SampleSource::uniform, sample_sink_);
}

}  // namespace ompl::MoD
