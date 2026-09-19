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
#include <ompl/base/goals/GoalState.h>
#include <ompl/mod/samplers/DijkstraSampler.h>

#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <chrono>
#include <cmath>
#include <mod/log.hpp>

namespace ompl::MoD {

DijkstraSampler::DijkstraSampler(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                                 const ::MoD::SamplerParameters &params, ::MoD::IntensityMapConstPtr /*intensity_map*/)
    : ompl::base::InformedSampler(pdef, maxCalls), bias_(params.bias), cell_size_(params.dijkstra_cell_size) {
  const auto *s = probDefn_->getStartState(0)->as<ompl::base::SE2StateSpace::StateType>();
  start_ = {s->getX(), s->getY(), s->getYaw()};

  const auto *g =
      probDefn_->getGoal()->as<ompl::base::GoalState>()->getState()->as<ompl::base::SE2StateSpace::StateType>();
  goal_ = {g->getX(), g->getY(), g->getYaw()};

  setup();
}

double DijkstraSampler::edgeCost(size_t from, size_t to, ompl::base::State *a, ompl::base::State *b) const {
  const double xi = grid_->x(from), yi = grid_->y(from);
  const double xf = grid_->x(to), yf = grid_->y(to);
  const double heading = std::atan2(yf - yi, xf - xi);
  auto *sa = a->as<ompl::base::SE2StateSpace::StateType>();
  auto *sb = b->as<ompl::base::SE2StateSpace::StateType>();
  sa->setX(xi);
  sa->setY(yi);
  sa->setYaw(heading);
  sb->setX(xf);
  sb->setY(yf);
  sb->setYaw(heading);
  return opt_->motionCost(a, b).value();
}

void DijkstraSampler::setup() {
  const auto t0 = std::chrono::steady_clock::now();
  const auto si = probDefn_->getSpaceInformation();
  const ompl::base::RealVectorBounds bounds = si->getStateSpace()->as<ompl::base::SE2StateSpace>()->getBounds();
  x_min_ = bounds.low[0];
  x_max_ = bounds.high[0];
  y_min_ = bounds.low[1];
  y_max_ = bounds.high[1];

  grid_ = std::make_unique<::MoD::GridDijkstra>(x_min_, x_max_, y_min_, y_max_, cell_size_);
  MOD_LOG("DijkstraSampler: bias %.3f, cell %.3f m, grid %zu x %zu", bias_, cell_size_, grid_->rows(),
          grid_->cols());

  // Validity: one check per node at yaw 0 (the footprint is a circle, so yaw is irrelevant).
  ompl::base::State *probe = si->allocState();
  const auto checker = si->getStateValidityChecker();
  if (checker) {
    grid_->computeValidity([&](double x, double y) {
      auto *p = probe->as<ompl::base::SE2StateSpace::StateType>();
      p->setX(x);
      p->setY(y);
      p->setYaw(0.0);
      return checker->isValid(probe);
    });
  } else {
    MOD_LOG("DijkstraSampler: no state validity checker set, every cell is treated as valid");
  }
  si->freeState(probe);

  ompl::base::State *a = si->allocState();
  ompl::base::State *b = si->allocState();
  grid_->setWeight([this, a, b](size_t from, size_t to) { return edgeCost(from, to, a, b); });

  const size_t start_node = grid_->nodeAt(start_[0], start_[1]);
  const size_t goal_node = grid_->nodeAt(goal_[0], goal_[1]);
  MOD_LOG("DijkstraSampler: start cell (%zu, %zu) = (%.2f, %.2f), goal cell (%zu, %zu) = (%.2f, %.2f)",
          grid_->row(start_node), grid_->col(start_node), start_[0], start_[1], grid_->row(goal_node),
          grid_->col(goal_node), goal_[0], goal_[1]);

  grid_->setRoot(start_node, ::MoD::GridDijkstra::Mode::forward);
  const double cost = grid_->costTo(goal_node);
  path_ = grid_->pathTo(goal_node);
  si->freeState(a);
  si->freeState(b);

  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (path_.empty()) {
    MOD_LOG("DijkstraSampler: no path from start to goal; only the uniform branch will be used");
  } else {
    MOD_LOG("DijkstraSampler: path of %zu nodes, cost %.3f", path_.size(), cost);
  }
  MOD_LOG("DijkstraSampler: setup %.1f ms, %zu nodes (%zu valid), %zu settled, %zu edges evaluated", ms,
          grid_->size(), grid_->validCount(), grid_->settledCount(), grid_->evaluatedEdges());
}

bool DijkstraSampler::sampleUniform(ompl::base::State *state, const ompl::base::Cost & /*cost*/) {
  size_t sampled_col = 0;
  size_t sampled_row = 0;
  double sampled_theta;
  const double pi = boost::math::constants::pi<double>();

  const double randomValue = rng_.uniformReal(0.0, 1.0);
  // At a bias_ % probability, choose a row, col from the dijkstra path
  const bool biased = randomValue < bias_ && !path_.empty();
  if (biased) {
    const auto idx = static_cast<size_t>(rng_.uniformInt(0, static_cast<int>(path_.size()) - 1));
    const size_t node = path_[idx];
    sampled_col = grid_->col(node);
    sampled_row = grid_->row(node);

    // Heading: towards the next path cell; the last cell looks back from the previous cell (Paper IV, step 3).
    if (idx == path_.size() - 1) {
      const size_t prev = path_[idx - 1];
      sampled_theta = std::atan2(grid_->y(node) - grid_->y(prev), grid_->x(node) - grid_->x(prev));
    } else {
      const size_t next = path_[idx + 1];
      sampled_theta = std::atan2(grid_->y(next) - grid_->y(node), grid_->x(next) - grid_->x(node));
    }
    sampled_theta = rng_.uniformReal(sampled_theta - pi / 8.0, sampled_theta + pi / 8.0);
  } else {
    sampled_col = static_cast<size_t>(rng_.uniformInt(0, static_cast<int>(grid_->cols()) - 1));
    sampled_row = static_cast<size_t>(rng_.uniformInt(0, static_cast<int>(grid_->rows()) - 1));
    sampled_theta = rng_.uniformReal(-pi, pi);
  }

  const double half = cell_size_ / 2.0;
  double sampled_x = rng_.uniformReal(colToX(sampled_col) - half, colToX(sampled_col) + half);
  double sampled_y = rng_.uniformReal(rowToY(sampled_row) - half, rowToY(sampled_row) + half);
  sampled_x = std::clamp(sampled_x, x_min_, x_max_);
  sampled_y = std::clamp(sampled_y, y_min_, y_max_);

  auto *se2 = state->as<ompl::base::SE2StateSpace::StateType>();
  se2->setX(sampled_x);
  se2->setY(sampled_y);
  se2->setYaw(sampled_theta);
  if (sink_)
    sink_->record(sampled_x, sampled_y, sampled_theta,
                  biased ? ::MoD::SampleSource::dijkstra : ::MoD::SampleSource::uniform);
  return true;
}

}  // namespace ompl::MoD
