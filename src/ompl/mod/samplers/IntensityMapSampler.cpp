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

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/mod/samplers/IntensityMapSampler.h>

#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <mod/log.hpp>
#include <numeric>
#include <stdexcept>

namespace ompl {
namespace MoD {

IntensityMapSampler::IntensityMapSampler(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                                         const ::MoD::SamplerParameters &params,
                                         ::MoD::IntensityMapConstPtr intensity_map)
    : ompl::base::InformedSampler(pdef, maxCalls), bias_(params.bias) {
  if (!intensity_map) throw std::invalid_argument("IntensityMapSampler: no intensity map given");
  setup(*intensity_map);
}

bool IntensityMapSampler::checkValidity(double xi, double yi) {
  const auto si = probDefn_->getSpaceInformation();
  const auto checker = si->getStateValidityChecker();
  if (!checker) return true;
  ompl::base::State *first = si->allocState();
  auto *s = first->as<ompl::base::SE2StateSpace::StateType>();
  s->setX(xi);
  s->setY(yi);
  s->setYaw(0.0);
  const bool valid = checker->isValid(first);
  si->freeState(first);
  return valid;
}

void IntensityMapSampler::setup(const ::MoD::IntensityMap &intensity_map) {
  struct Cell {
    double x, y, w;
  };
  std::vector<Cell> cells;
  const size_t total = intensity_map.getRows() * intensity_map.getColumns();
  if (!probDefn_->getSpaceInformation()->getStateValidityChecker()) {
    MOD_LOG("IntensityMapSampler: no state validity checker set, every cell is treated as valid");
  }
  for (size_t i = 0; i < total; ++i) {
    const auto xy = intensity_map.getXYatIndex(i);
    if (this->checkValidity(xy[0], xy[1])) cells.push_back({xy[0], xy[1], 1.0 - intensity_map.valueAt(i)});
  }
  MOD_LOG("IntensityMapSampler: %zu of %zu cells are valid, bias %.3f", cells.size(), total, bias_);
  if (cells.empty()) throw std::runtime_error("IntensityMapSampler: no valid cell in the intensity map");

  // Sorted ascending by weight, as before.
  std::stable_sort(cells.begin(), cells.end(), [](const Cell &a, const Cell &b) { return a.w < b.w; });

  xs_.resize(cells.size());
  ys_.resize(cells.size());
  weights_.resize(cells.size());
  prefix_q_.resize(cells.size());
  double accum = 0.0;
  for (size_t i = 0; i < cells.size(); ++i) {
    xs_[i] = cells[i].x;
    ys_[i] = cells[i].y;
    weights_[i] = cells[i].w;
    accum += cells[i].w;
    prefix_q_[i] = accum;
  }

  this->half_cell_size = intensity_map.getCellSize() / 2.0;

  const ompl::base::RealVectorBounds bounds =
      probDefn_->getSpaceInformation()->getStateSpace()->as<ompl::base::SE2StateSpace>()->getBounds();
  x_min_ = bounds.low[0];
  x_max_ = bounds.high[0];
  y_min_ = bounds.low[1];
  y_max_ = bounds.high[1];
}

bool IntensityMapSampler::sampleUniform(ompl::base::State *state, const ompl::base::Cost & /*maxCost*/) {
  sampleNecessarilyValid(state);
  return true;
}

void IntensityMapSampler::sampleNecessarilyValid(ompl::base::State *state) {
  // Sample theta first. This is the easiest part.
  const double theta = rng_.uniformReal(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());

  // Choose the branch: with probability bias_ the q branch (cells weighted by 1 - q), else uniform over the
  // valid cells. Each branch draws its position value from its own weight sum.
  const double random_num = rng_.uniformReal(0.0, 1.0);
  const bool q_branch = random_num < bias_;
  const size_t n = xs_.size();
  size_t index;
  if (q_branch) {
    const double sampled_value = rng_.uniformReal(0.0, prefix_q_.back());
    index = static_cast<size_t>(std::upper_bound(prefix_q_.begin(), prefix_q_.end(), sampled_value) -
                                prefix_q_.begin());
  } else {
    const double sampled_value = rng_.uniformReal(0.0, static_cast<double>(n));
    index = static_cast<size_t>(std::floor(sampled_value));
  }
  if (index >= n) index = n - 1;

  double sampled_x = rng_.uniformReal(xs_[index] - half_cell_size, xs_[index] + half_cell_size);
  double sampled_y = rng_.uniformReal(ys_[index] - half_cell_size, ys_[index] + half_cell_size);
  sampled_x = std::clamp(sampled_x, x_min_, x_max_);
  sampled_y = std::clamp(sampled_y, y_min_, y_max_);

  auto *se2 = state->as<ompl::base::SE2StateSpace::StateType>();
  se2->setX(sampled_x);
  se2->setY(sampled_y);
  se2->setYaw(theta);
  if (sink_)
    sink_->record(sampled_x, sampled_y, theta,
                  q_branch ? ::MoD::SampleSource::intensity : ::MoD::SampleSource::uniform);
}

}  // namespace MoD
}  // namespace ompl
