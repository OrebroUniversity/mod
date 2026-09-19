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

/// Measures what the planners hand to `motionCost`: `check_interpolation <atc data dir> [--time S]
/// [--objective cliff|gmmt|dtc|intensity]`. Wraps the objective in a counting decorator, runs one RRT* and one
/// AIT* solve on ATC scenario 1 (Reeds-Shepp r = 1, inferred steps, default range) and prints the histograms
/// of edge length and cost points per call, plus calls per planner iteration.

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/RandomNumbers.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <numeric>
#include <string>
#include <vector>

#include "core/planner_factory.hpp"
#include "core/solver.hpp"

namespace ob = ompl::base;
using namespace MoD::playground;

namespace {

/// Forwards to an MoD objective and records, per `motionCost` call, the edge length and the cost point count.
class CountingObjective : public ob::OptimizationObjective {
 public:
  CountingObjective(const ob::SpaceInformationPtr &si, ompl::MoD::MoDOptimizationObjectivePtr inner)
      : ob::OptimizationObjective(si), inner_(std::move(inner)) {
    description_ = "counting(" + inner_->getDescription() + ")";
    setCostToGoHeuristic(ob::goalRegionCostToGo);
  }

  ob::Cost stateCost(const ob::State *s) const override { return inner_->stateCost(s); }
  ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override {
    const double d = si_->distance(s1, s2);
    lengths_.push_back(d);
    points_.push_back(std::max(1.0, std::ceil(d / inner_->getCostStep())));
    return inner_->motionCost(s1, s2);
  }
  ob::Cost motionCostHeuristic(const ob::State *s1, const ob::State *s2) const override {
    ++heuristic_calls_;
    heuristic_lengths_.push_back(si_->distance(s1, s2));
    return inner_->motionCostHeuristic(s1, s2);
  }
  bool isSymmetric() const override { return inner_->isSymmetric(); }
  ob::InformedSamplerPtr allocInformedStateSampler(const ob::ProblemDefinitionPtr &pdef,
                                                   unsigned int maxCalls) const override {
    return inner_->allocInformedStateSampler(pdef, maxCalls);
  }

  const std::vector<double> &lengths() const { return lengths_; }
  const std::vector<double> &points() const { return points_; }
  const std::vector<double> &heuristicLengths() const { return heuristic_lengths_; }
  size_t heuristicCalls() const { return heuristic_calls_; }

 private:
  ompl::MoD::MoDOptimizationObjectivePtr inner_;
  mutable std::vector<double> lengths_;
  mutable std::vector<double> points_;
  mutable std::vector<double> heuristic_lengths_;
  mutable size_t heuristic_calls_{0};
};

/// A bin width giving about 20 bins: 1, 2, 5 x 10^k.
double niceBin(double max_value) {
  const double raw = std::max(1e-9, max_value / 20.0);
  const double p = std::pow(10.0, std::floor(std::log10(raw)));
  for (double m : {1.0, 2.0, 5.0, 10.0})
    if (m * p >= raw) return m * p;
  return 10.0 * p;
}

void histogram(const char *title, const std::vector<double> &v, const char *unit) {
  if (v.empty()) {
    std::printf("%s: no calls\n", title);
    return;
  }
  const double bin = niceBin(*std::max_element(v.begin(), v.end()));
  std::map<long, size_t> bins;
  for (double x : v) ++bins[static_cast<long>(std::floor(x / bin))];
  std::vector<double> sorted(v);
  std::sort(sorted.begin(), sorted.end());
  const double mean = std::accumulate(sorted.begin(), sorted.end(), 0.0) / static_cast<double>(sorted.size());
  std::printf("%s: %zu calls, mean %.3f %s, median %.3f, p90 %.3f, max %.3f\n", title, v.size(), mean, unit,
              sorted[sorted.size() / 2], sorted[static_cast<size_t>(0.9 * static_cast<double>(sorted.size() - 1))],
              sorted.back());
  for (const auto &kv : bins) {
    const double pct = 100.0 * static_cast<double>(kv.second) / static_cast<double>(v.size());
    std::printf("  [%7.2f, %7.2f) %8zu %5.1f%% %s\n", static_cast<double>(kv.first) * bin,
                static_cast<double>(kv.first + 1) * bin, kv.second, pct,
                std::string(static_cast<size_t>(pct / 2.0), '#').c_str());
  }
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <atc data dir> [--time S] [--objective cliff|gmmt|dtc|intensity]\n", argv[0]);
    return 2;
  }
  const std::string data = argv[1];
  double time_s = 5.0;
  std::string objective_name = "cliff";
  for (int i = 2; i < argc; ++i) {
    if (std::strcmp(argv[i], "--time") == 0 && i + 1 < argc)
      time_s = std::stod(argv[++i]);
    else if (std::strcmp(argv[i], "--objective") == 0 && i + 1 < argc)
      objective_name = argv[++i];
  }

  MoD::RunConfig base;
  base.scenario.name = "atc-scenario1";
  base.scenario.map_yaml = data + "/atc.yaml";
  base.scenario.start = {47.690, -18.848, -2.356};
  base.scenario.goal = {-19.575, 12.390, 2.313};
  base.vehicle.state_space = MoD::StateSpaceType::reeds_shepp;
  base.vehicle.turning_radius = 1.0;
  base.objective.type = MoD::objectiveTypeFromString(objective_name);
  base.objective.cliff_map_file = data + "/atc_cliff.xml";
  base.objective.gmmt_map_file = data + "/atc_gmmt.xml";
  base.objective.intensity_map_file = data + "/atc_intensity1m.xml";
  base.objective.w_c = base.objective.type == MoD::ObjectiveType::dtc         ? 0.02
                       : base.objective.type == MoD::ObjectiveType::intensity ? 0.2
                                                                              : 0.1;
  base.sampler.type = MoD::SamplerType::iid;  // its setup makes no motionCost calls
  base.planner.max_planning_time = time_s;

  ompl::RNG::setSeed(1);
  MapCache maps;
  for (const auto planner_type : {MoD::PlannerType::rrt_star, MoD::PlannerType::ait_star}) {
    MoD::RunConfig config = base;
    config.planner.type = planner_type;
    PlannerSetup setup = PlannerFactory::build(config, maps);
    auto counting = std::make_shared<CountingObjective>(setup.si, setup.mod_objective);
    setup.objective = counting;
    setup.pdef->setOptimizationObjective(counting);
    setup.planner->setProblemDefinition(setup.pdef);

    Solver solver(setup);
    const Solution sol = solver.solve(time_s);
    std::printf("\n=== %s, %s objective, %.1f s, pixel %.3f m, cost step %.3f m, range %.2f m ===\n",
                MoD::to_string(planner_type).c_str(), objective_name.c_str(), time_s,
                config.derived.occupancy_pixel_m, config.derived.mod_cost_step_m,
                planner_type == MoD::PlannerType::rrt_star
                    ? std::dynamic_pointer_cast<ompl::geometric::RRTstar>(setup.planner)->getRange()
                    : 0.0);
    std::printf("solved: %s, first solution %.2f s, cost %.3f (d %.3f, q %.3f, c %.3f), length %.2f m\n",
                sol.success ? "yes" : "no", sol.time_to_first_solution_s, sol.cost_total, sol.cost_d, sol.cost_q,
                sol.cost_c, sol.path_length_m);
    histogram("edge length per motionCost call", counting->lengths(), "m");
    histogram("cost points per motionCost call", counting->points(), "points");
    histogram("edge length per motionCostHeuristic call (= motionCost in the MoD objectives)",
              counting->heuristicLengths(), "m");
    if (planner_type == MoD::PlannerType::rrt_star) {
      const auto n = std::dynamic_pointer_cast<ompl::geometric::RRTstar>(setup.planner)->numIterations();
      std::printf("motionCost calls per RRT* iteration: %.2f (%zu calls / %u iterations)\n",
                  static_cast<double>(counting->lengths().size()) / std::max(1u, n), counting->lengths().size(), n);
    } else {
      ob::PlannerData pd(setup.si);
      setup.planner->getPlannerData(pd);
      std::printf("motionCost calls per AIT* graph vertex: %.2f (%zu calls / %u vertices)\n",
                  static_cast<double>(counting->lengths().size()) / std::max(1u, pd.numVertices()),
                  counting->lengths().size(), pd.numVertices());
    }
    std::printf("motionCost calls per second: %.0f; motionCostHeuristic calls per second: %.0f\n",
                static_cast<double>(counting->lengths().size()) / std::max(1e-9, sol.planning_time_s),
                static_cast<double>(counting->heuristicCalls()) / std::max(1e-9, sol.planning_time_s));
  }
  return 0;
}
