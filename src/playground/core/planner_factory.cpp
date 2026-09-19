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
 *
 * Objective-per-type wiring and the Paper IV defaults follow bench-mr (MIT, Eric Heiden; MoD additions by
 * Chittaranjan Swaminathan), src/base/src/PlannerSettings.cpp, initializeSteering(). No bench-mr code is included.
 */

#include "core/planner_factory.hpp"

#include <ompl/base/goals/GoalState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/mod/objectives/DTCOptimizationObjective.h>
#include <ompl/mod/objectives/IntensityMapOptimizationObjective.h>
#include <ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h>

#include <algorithm>
#include <mod/log.hpp>
#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace MoD::playground {

ompl::MoD::MoDOptimizationObjectivePtr PlannerFactory::buildObjective(::MoD::RunConfig &config, MapCache &maps,
                                                                      const ob::SpaceInformationPtr &si) {
  const auto &op = config.objective;
  ::MoD::IntensityMapConstPtr intensity;
  if (!op.intensity_map_file.empty()) intensity = maps.intensity(op.intensity_map_file);

  ompl::MoD::MoDOptimizationObjectivePtr objective;
  double mod_cell = 0.0;
  switch (op.type) {
    case ::MoD::ObjectiveType::cliff: {
      auto cliff = maps.cliff(op.cliff_map_file);
      objective = std::make_shared<ompl::MoD::UpstreamCriterionOptimizationObjective>(si, op, config.sampler, cliff,
                                                                                      nullptr, intensity);
      mod_cell = cliff->getResolution();
      break;
    }
    case ::MoD::ObjectiveType::dtc: {
      auto cliff = maps.cliff(op.cliff_map_file);
      objective = std::make_shared<ompl::MoD::DTCOptimizationObjective>(si, op, config.sampler, cliff, intensity);
      mod_cell = cliff->getResolution();
      break;
    }
    case ::MoD::ObjectiveType::gmmt: {
      auto gmmt = maps.gmmt(op.gmmt_map_file);
      objective = std::make_shared<ompl::MoD::UpstreamCriterionOptimizationObjective>(si, op, config.sampler, nullptr,
                                                                                      gmmt, intensity);
      break;
    }
    case ::MoD::ObjectiveType::intensity: {
      objective = std::make_shared<ompl::MoD::IntensityMapOptimizationObjective>(si, op, config.sampler, intensity);
      mod_cell = intensity ? intensity->getCellSize() : 0.0;
      break;
    }
    case ::MoD::ObjectiveType::path_length:
      return nullptr;
  }

  // The samplers' intensity map, shared through the cache.
  if (!config.sampler.intensity_map_file.empty()) {
    objective->setSamplerIntensityMap(maps.intensity(config.sampler.intensity_map_file));
  }
  if (mod_cell <= 0.0) {
    // GMMT has no grid: fall back to the intensity map's cell (objective or sampler), else the pixel.
    const auto &sampler_map = objective->getSamplerIntensityMap();
    mod_cell = intensity ? intensity->getCellSize() : (sampler_map ? sampler_map->getCellSize() : 0.0);
  }
  const double pixel = config.derived.occupancy_pixel_m;
  if (mod_cell <= 0.0) mod_cell = pixel;
  config.derived.mod_cell_m = mod_cell;
  config.derived.mod_cost_step_m = std::min(mod_cell, pixel);
  objective->setCostStep(config.derived.mod_cost_step_m);
  return objective;
}

ob::PlannerPtr PlannerFactory::buildPlanner(const ::MoD::PlannerParameters &params, const ob::SpaceInformationPtr &si) {
  switch (params.type) {
    case ::MoD::PlannerType::rrt_star: {
      auto planner = std::make_shared<og::RRTstar>(si);
      planner->setInformedSampling(params.informed_sampling);
      if (params.range > 0.0) planner->setRange(params.range);
      planner->setGoalBias(params.goal_bias);
      return planner;
    }
    case ::MoD::PlannerType::ait_star: {
      auto planner = std::make_shared<og::AITstar>(si);
      planner->setBatchSize(params.batch_size);
      return planner;
    }
  }
  throw std::invalid_argument("PlannerFactory: unknown planner type");
}

PlannerSetup PlannerFactory::build(::MoD::RunConfig &config, MapCache &maps) {
  PlannerSetup setup;
  setup.occupancy = maps.occupancy(config.scenario.map_yaml);
  const Bounds b = setup.occupancy->bounds();

  // State space over the occupancy bounds.
  const double r = config.vehicle.turning_radius;
  if (config.vehicle.state_space == ::MoD::StateSpaceType::reeds_shepp)
    setup.space = std::make_shared<ob::ReedsSheppStateSpace>(r);
  else
    setup.space = std::make_shared<ob::DubinsStateSpace>(r);
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, b.x_min);
  bounds.setHigh(0, b.x_max);
  bounds.setLow(1, b.y_min);
  bounds.setHigh(1, b.y_max);
  setup.space->as<ob::SE2StateSpace>()->setBounds(bounds);

  // Derived quantities: both steps are inferred, never entered.
  config.derived.occupancy_pixel_m = setup.occupancy->pixel_size();
  config.derived.collision_step_m = setup.occupancy->pixel_size();
  config.derived.circumscribed_radius_m = config.vehicle.circumscribedRadius();

  setup.si = std::make_shared<ob::SpaceInformation>(setup.space);
  setup.checker = std::make_shared<FootprintChecker>(setup.si, setup.occupancy, config.derived.circumscribed_radius_m);
  setup.si->setStateValidityChecker(setup.checker);
  setup.si->setStateValidityCheckingResolution(config.derived.collision_step_m / setup.space->getMaximumExtent());
  setup.si->setup();

  // Objective.
  setup.mod_objective = buildObjective(config, maps, setup.si);
  if (setup.mod_objective) {
    setup.objective = setup.mod_objective;
  } else {
    setup.objective = std::make_shared<ob::PathLengthOptimizationObjective>(setup.si);
    config.derived.mod_cell_m = 0.0;
    config.derived.mod_cost_step_m = 0.0;
  }

  // Problem definition.
  setup.pdef = std::make_shared<ob::ProblemDefinition>(setup.si);
  ob::ScopedState<ob::SE2StateSpace> start(setup.space), goal(setup.space);
  start->setXY(config.scenario.start[0], config.scenario.start[1]);
  start->setYaw(config.scenario.start[2]);
  goal->setXY(config.scenario.goal[0], config.scenario.goal[1]);
  goal->setYaw(config.scenario.goal[2]);
  setup.pdef->addStartState(start);
  auto goal_state = std::make_shared<ob::GoalState>(setup.si);
  goal_state->setState(goal);
  setup.pdef->setGoal(goal_state);
  setup.pdef->setOptimizationObjective(setup.objective);

  if (!setup.checker->isValid(start.get()))
    MOD_LOG("PlannerFactory: start (%.2f, %.2f) is not valid", config.scenario.start[0], config.scenario.start[1]);
  if (!setup.checker->isValid(goal.get()))
    MOD_LOG("PlannerFactory: goal (%.2f, %.2f) is not valid", config.scenario.goal[0], config.scenario.goal[1]);

  setup.planner = buildPlanner(config.planner, setup.si);
  setup.planner->setProblemDefinition(setup.pdef);
  MOD_LOG("PlannerFactory: %s on %s, %s / %s / %s, pixel %.3f m, cost step %.3f m, radius %.3f m",
          config.scenario.name.c_str(), ::MoD::to_string(config.vehicle.state_space).c_str(),
          ::MoD::to_string(config.planner.type).c_str(), ::MoD::to_string(config.sampler.type).c_str(),
          ::MoD::to_string(config.objective.type).c_str(), config.derived.occupancy_pixel_m,
          config.derived.mod_cost_step_m, config.derived.circumscribed_radius_m);
  return setup;
}

}  // namespace MoD::playground
