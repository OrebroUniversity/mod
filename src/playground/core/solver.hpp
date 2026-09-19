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

#include <array>
#include <atomic>
#include <cmath>
#include <nlohmann/json.hpp>
#include <vector>

#include "core/planner_factory.hpp"

namespace MoD::playground {

/// The content of solution.json.
struct Solution {
  bool success{false};
  double planning_time_s{0.0};
  double time_to_first_solution_s{std::nan("")};  ///< NaN (null in JSON) when no solution was found
  double cost_total{std::nan("")};
  double cost_d{std::nan("")};
  double cost_q{std::nan("")};
  double cost_c{std::nan("")};
  double path_length_m{std::nan("")};
  std::vector<std::array<double, 3>> path;  ///< empty on failure
};

void to_json(nlohmann::json &j, const Solution &s);
void from_json(const nlohmann::json &j, Solution &s);

/**
 * Runs the planner of a PlannerSetup for at most `max_time_s` seconds and evaluates the result with the
 * setup's objective. `cancel()` may be called from another thread. The first solution time comes from the
 * planners' intermediate-solution callback.
 */
class Solver {
 public:
  explicit Solver(PlannerSetup &setup) : setup_(setup) {}

  Solution solve(double max_time_s);
  void cancel() { cancel_ = true; }
  bool cancelled() const { return cancel_; }

  /// Evaluates a solved problem definition: costs, length and the path states.
  static Solution evaluate(const PlannerSetup &setup);

 private:
  PlannerSetup &setup_;
  std::atomic<bool> cancel_{false};
};

}  // namespace MoD::playground
