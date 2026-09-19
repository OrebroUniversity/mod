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

#include <mod/parameters.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "core/map_cache.hpp"
#include "core/solver.hpp"

namespace MoD::playground {

/// Input of run_batch: the product of scenarios x planners x samplers x objectives x repeats.
struct BatchSpec {
  std::string log_dir{"runs"};
  unsigned int threads{1};
  std::vector<::MoD::Scenario> scenarios;
  std::vector<::MoD::PlannerParameters> planners;
  std::vector<::MoD::SamplerParameters> samplers;
  std::vector<::MoD::OptObjParameters> objectives;
  ::MoD::VehicleParameters vehicle;
  ::MoD::HybridAStarParameters hybrid_astar;  ///< one scope per batch, copied into every `hybrid_astar` run
  unsigned int repeats{1};
  unsigned int seed0{0};
};

void to_json(nlohmann::json &j, const BatchSpec &b);
void from_json(const nlohmann::json &j, BatchSpec &b);

/// Loads a batch JSON; relative file paths (maps, log_dir) resolve against the JSON file's directory.
BatchSpec loadBatch(const std::string &json_path);

/// Makes every relative map path / log_dir in `spec` absolute against `base_dir`.
void resolvePaths(BatchSpec &spec, const std::string &base_dir);

/// Expands the product; run i gets `planner.seed = seed0 + i`.
std::vector<::MoD::RunConfig> expand(const BatchSpec &spec);

/// Runs one configured run end to end: factory, logger, solve, solution.json. Returns the solution and the
/// run folder. Exceptions propagate.
Solution runOne(::MoD::RunConfig config, const std::string &log_dir, MapCache &maps, std::string *run_dir = nullptr);

}  // namespace MoD::playground
