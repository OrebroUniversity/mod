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
 * Timing solutions through OMPL's intermediate-solution callback follows bench-mr (MIT, Eric Heiden),
 * src/planners/OMPLPlanner.hpp, OMPLPlanner::run(). No bench-mr code is included.
 */

#include "core/solver.hpp"

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <chrono>
#include <mod/log.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace MoD::playground {

namespace {
nlohmann::json numberOrNull(double v) {
  if (std::isnan(v)) return nullptr;
  return v;
}
double numberOrNan(const nlohmann::json &j, const char *key) {
  if (!j.contains(key) || j.at(key).is_null()) return std::nan("");
  return j.at(key).get<double>();
}
}  // namespace

void to_json(nlohmann::json &j, const Solution &s) {
  j = nlohmann::json{{"success", s.success},
                     {"planning_time_s", s.planning_time_s},
                     {"time_to_first_solution_s", numberOrNull(s.time_to_first_solution_s)},
                     {"cost",
                      {{"total", numberOrNull(s.cost_total)},
                       {"d", numberOrNull(s.cost_d)},
                       {"q", numberOrNull(s.cost_q)},
                       {"c", numberOrNull(s.cost_c)}}},
                     {"path_length_m", numberOrNull(s.path_length_m)},
                     {"path", s.path}};
}

void from_json(const nlohmann::json &j, Solution &s) {
  s.success = j.value("success", false);
  s.planning_time_s = j.value("planning_time_s", 0.0);
  s.time_to_first_solution_s = numberOrNan(j, "time_to_first_solution_s");
  if (j.contains("cost")) {
    const auto &c = j.at("cost");
    s.cost_total = numberOrNan(c, "total");
    s.cost_d = numberOrNan(c, "d");
    s.cost_q = numberOrNan(c, "q");
    s.cost_c = numberOrNan(c, "c");
  }
  s.path_length_m = numberOrNan(j, "path_length_m");
  s.path.clear();
  if (j.contains("path")) s.path = j.at("path").get<std::vector<std::array<double, 3>>>();
}

Solution Solver::evaluate(const PlannerSetup &setup) {
  Solution sol;
  sol.success = setup.pdef->hasExactSolution();
  if (!sol.success) return sol;
  const auto path = std::dynamic_pointer_cast<og::PathGeometric>(setup.pdef->getSolutionPath());
  if (!path) return sol;

  sol.path_length_m = path->length();
  sol.cost_total = path->cost(setup.objective).value();
  if (setup.mod_objective) {
    ompl::MoD::CostComponents cc;
    const auto &states = path->getStates();
    for (size_t i = 0; i + 1 < states.size(); ++i)
      cc += setup.mod_objective->motionCostComponents(states[i], states[i + 1]);
    sol.cost_d = cc.d;
    sol.cost_q = cc.q;
    sol.cost_c = cc.c;
  } else {
    sol.cost_d = sol.path_length_m;
    sol.cost_q = 0.0;
    sol.cost_c = 0.0;
  }
  for (const auto *s : path->getStates()) {
    const auto *se2 = s->as<ob::SE2StateSpace::StateType>();
    sol.path.push_back({se2->getX(), se2->getY(), se2->getYaw()});
  }
  return sol;
}

Solution Solver::solve(double max_time_s) {
  cancel_ = false;
  const auto t0 = std::chrono::steady_clock::now();
  const auto elapsed = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };

  double first_solution = std::nan("");
  setup_.pdef->setIntermediateSolutionCallback(
      [&](const ob::Planner *, const std::vector<const ob::State *> &, const ob::Cost) {
        if (std::isnan(first_solution)) first_solution = elapsed();
      });

  ob::PlannerTerminationCondition ptc([&]() { return cancel_.load() || elapsed() >= max_time_s; });

  setup_.planner->setup();
  const ob::PlannerStatus status = setup_.planner->solve(ptc);
  Solution sol = evaluate(setup_);
  sol.planning_time_s = elapsed();
  if (sol.success && std::isnan(first_solution)) first_solution = sol.planning_time_s;
  sol.time_to_first_solution_s = sol.success ? first_solution : std::nan("");
  setup_.pdef->setIntermediateSolutionCallback(nullptr);

  MOD_LOG("Solver: %s in %.2f s (%s), first solution %.2f s, cost %.3f, length %.2f m", status.asString().c_str(),
          sol.planning_time_s, sol.success ? "exact" : "no exact solution", sol.time_to_first_solution_s,
          sol.cost_total, sol.path_length_m);
  return sol;
}

}  // namespace MoD::playground
