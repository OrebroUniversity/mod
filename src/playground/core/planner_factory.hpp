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

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/mod/objectives/MoDOptimizationObjective.h>

#include <memory>
#include <mod/parameters.hpp>
#include <mod/planners/hybrid_astar.hpp>

#include "core/footprint_checker.hpp"
#include "core/map_cache.hpp"
#include "core/occupancy_map.hpp"

namespace MoD::playground {

/// Everything one planner run needs. Per run; only the maps are shared.
struct PlannerSetup {
  OccupancyMapConstPtr occupancy;
  ompl::base::StateSpacePtr space;
  ompl::base::SpaceInformationPtr si;
  std::shared_ptr<FootprintChecker> checker;
  ompl::base::ProblemDefinitionPtr pdef;
  ompl::base::OptimizationObjectivePtr objective;
  /// The objective as an MoD objective; null for `path_length`.
  ompl::MoD::MoDOptimizationObjectivePtr mod_objective;
  /// The OMPL planner (RRT* / AIT*); null for `hybrid_astar`.
  ompl::base::PlannerPtr planner;
  /// Hybrid A* (not an ompl::base::Planner); null for the OMPL planners.
  ::MoD::HybridAStarPtr hybrid_astar;
};

/**
 * Builds a PlannerSetup from a RunConfig: state space (Dubins / Reeds-Shepp) over the occupancy bounds,
 * FootprintChecker with the circumscribed radius, collision resolution = pixel / maximum extent, objective
 * from OptObjParameters with cost step min(MoD cell, pixel), sampler dispatch through the objective, and the
 * planner (RRT*, AIT* or Hybrid A*). Fills `config.derived` and, for Hybrid A*, writes the effective
 * `allow_reverse` back into `config.hybrid_astar`. Maps come from the cache. `ompl::RNG::setSeed` is NOT
 * called here (it is global); callers seed before building.
 */
class PlannerFactory {
 public:
  static PlannerSetup build(::MoD::RunConfig &config, MapCache &maps);

  /// The objective only (with `config.derived` filled), for tools that wrap it.
  static ompl::MoD::MoDOptimizationObjectivePtr buildObjective(::MoD::RunConfig &config, MapCache &maps,
                                                               const ompl::base::SpaceInformationPtr &si);

  /// The OMPL planner for `rrt_star` / `ait_star`; null for `hybrid_astar` (see `build`).
  static ompl::base::PlannerPtr buildPlanner(const ::MoD::PlannerParameters &params,
                                             const ompl::base::SpaceInformationPtr &si);
};

}  // namespace MoD::playground
