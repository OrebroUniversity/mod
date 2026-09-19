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
 * The scenario x planner x sampler x objective x repeats expansion follows bench-mr (MIT, Eric Heiden; MoD
 * additions by Chittaranjan Swaminathan), python/MoD-planning.py. No bench-mr code is included.
 */

#include "core/batch.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "core/planner_factory.hpp"
#include "core/run_logger.hpp"

namespace fs = std::filesystem;

namespace MoD::playground {

void to_json(nlohmann::json &j, const BatchSpec &b) {
  j = nlohmann::json{{"log_dir", b.log_dir},     {"threads", b.threads},       {"scenarios", b.scenarios},
                     {"planners", b.planners},   {"samplers", b.samplers},     {"objectives", b.objectives},
                     {"vehicle", b.vehicle},     {"hybrid_astar", b.hybrid_astar}, {"repeats", b.repeats},
                     {"seed0", b.seed0}};
}

void from_json(const nlohmann::json &j, BatchSpec &b) {
  if (j.contains("log_dir")) b.log_dir = j.at("log_dir").get<std::string>();
  if (j.contains("threads")) b.threads = j.at("threads").get<unsigned int>();
  if (j.contains("scenarios")) b.scenarios = j.at("scenarios").get<std::vector<::MoD::Scenario>>();
  if (j.contains("planners")) b.planners = j.at("planners").get<std::vector<::MoD::PlannerParameters>>();
  if (j.contains("samplers")) b.samplers = j.at("samplers").get<std::vector<::MoD::SamplerParameters>>();
  if (j.contains("objectives")) b.objectives = j.at("objectives").get<std::vector<::MoD::OptObjParameters>>();
  if (j.contains("vehicle")) b.vehicle = j.at("vehicle").get<::MoD::VehicleParameters>();
  if (j.contains("hybrid_astar")) b.hybrid_astar = j.at("hybrid_astar").get<::MoD::HybridAStarParameters>();
  if (j.contains("repeats")) b.repeats = j.at("repeats").get<unsigned int>();
  if (j.contains("seed0")) b.seed0 = j.at("seed0").get<unsigned int>();
}

namespace {
void absolutize(std::string &path, const fs::path &base) {
  if (path.empty()) return;
  fs::path p(path);
  if (p.is_relative()) path = (base / p).lexically_normal().string();
}
}  // namespace

void resolvePaths(BatchSpec &spec, const std::string &base_dir) {
  const fs::path base(base_dir);
  absolutize(spec.log_dir, base);
  for (auto &s : spec.scenarios) absolutize(s.map_yaml, base);
  for (auto &o : spec.objectives) {
    absolutize(o.cliff_map_file, base);
    absolutize(o.gmmt_map_file, base);
    absolutize(o.intensity_map_file, base);
  }
  for (auto &s : spec.samplers) absolutize(s.intensity_map_file, base);
}

BatchSpec loadBatch(const std::string &json_path) {
  std::ifstream in(json_path);
  if (!in) throw std::runtime_error("loadBatch: cannot open " + json_path);
  BatchSpec spec = nlohmann::json::parse(in).get<BatchSpec>();
  resolvePaths(spec, fs::absolute(fs::path(json_path)).parent_path().string());
  return spec;
}

std::vector<::MoD::RunConfig> expand(const BatchSpec &spec) {
  std::vector<::MoD::RunConfig> runs;
  unsigned int index = 0;
  for (const auto &scenario : spec.scenarios)
    for (const auto &planner : spec.planners)
      for (const auto &sampler : spec.samplers)
        for (const auto &objective : spec.objectives)
          for (unsigned int r = 0; r < spec.repeats; ++r) {
            ::MoD::RunConfig c;
            c.vehicle = spec.vehicle;
            c.hybrid_astar = spec.hybrid_astar;
            c.scenario = scenario;
            c.planner = planner;
            c.planner.seed = spec.seed0 + index;
            c.sampler = sampler;
            c.objective = objective;
            runs.push_back(c);
            ++index;
          }
  return runs;
}

Solution runOne(::MoD::RunConfig config, const std::string &log_dir, MapCache &maps, std::string *run_dir) {
  PlannerSetup setup = PlannerFactory::build(config, maps);
  RunLogger logger(log_dir, config);
  if (run_dir) *run_dir = logger.dir();
  if (setup.mod_objective) setup.mod_objective->setSampleSink(logger.sampleSink());
  Solver solver(setup);
  const Solution solution = solver.solve(config.planner.max_planning_time);
  logger.writeSolution(solution);
  logger.finish();
  return solution;
}

}  // namespace MoD::playground
