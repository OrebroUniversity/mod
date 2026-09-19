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

/// Headless batch runner: `run_batch <batch.json> [--threads N] [--log-dir DIR] [--dry-run]`.
/// Expands scenarios x planners x samplers x objectives x repeats and runs them `threads` at a time, one
/// thread per run (own SpaceInformation / objective / sampler / planner; only the maps are shared).

#include <ompl/util/RandomNumbers.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "core/batch.hpp"

using namespace MoD::playground;

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <batch.json> [--threads N] [--log-dir DIR] [--dry-run]\n", argv[0]);
    return 2;
  }
  BatchSpec spec;
  try {
    spec = loadBatch(argv[1]);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "run_batch: %s\n", e.what());
    return 2;
  }
  bool dry_run = false;
  for (int i = 2; i < argc; ++i) {
    if (std::strcmp(argv[i], "--threads") == 0 && i + 1 < argc)
      spec.threads = static_cast<unsigned int>(std::stoul(argv[++i]));
    else if (std::strcmp(argv[i], "--log-dir") == 0 && i + 1 < argc)
      spec.log_dir = argv[++i];
    else if (std::strcmp(argv[i], "--dry-run") == 0)
      dry_run = true;
    else {
      std::fprintf(stderr, "run_batch: unknown argument %s\n", argv[i]);
      return 2;
    }
  }
  if (spec.threads == 0) spec.threads = 1;

  const std::vector<MoD::RunConfig> runs = expand(spec);
  std::printf("run_batch: %zu runs (%zu scenarios x %zu planners x %zu samplers x %zu objectives x %u repeats), "
              "%u threads, log dir %s\n",
              runs.size(), spec.scenarios.size(), spec.planners.size(), spec.samplers.size(),
              spec.objectives.size(), spec.repeats, spec.threads, spec.log_dir.c_str());
  if (dry_run) {
    for (size_t i = 0; i < runs.size(); ++i) {
      const auto &c = runs[i];
      std::printf("  [%zu] %s %s %s %s seed %u\n", i, c.scenario.name.c_str(), MoD::to_string(c.planner.type).c_str(),
                  MoD::to_string(c.sampler.type).c_str(), MoD::to_string(c.objective.type).c_str(), c.planner.seed);
    }
    return 0;
  }

  // ompl::RNG::setSeed is global: once, before any thread starts.
  ompl::RNG::setSeed(spec.seed0 == 0 ? 1 : spec.seed0);

  MapCache maps;
  std::atomic<size_t> next{0};
  std::atomic<size_t> successes{0};
  std::atomic<size_t> failures{0};
  std::mutex print_mutex;
  const auto t0 = std::chrono::steady_clock::now();

  auto worker = [&]() {
    for (size_t i = next++; i < runs.size(); i = next++) {
      const MoD::RunConfig &c = runs[i];
      std::string dir;
      try {
        const Solution s = runOne(c, spec.log_dir, maps, &dir);
        if (s.success) ++successes;
        std::lock_guard<std::mutex> lock(print_mutex);
        std::printf("[%zu/%zu] %-16s %-9s %-9s %-11s seed %-5u %s first %6.2f s cost %9.3f len %7.2f m -> %s\n",
                    i + 1, runs.size(), c.scenario.name.c_str(), MoD::to_string(c.planner.type).c_str(),
                    MoD::to_string(c.sampler.type).c_str(), MoD::to_string(c.objective.type).c_str(), c.planner.seed,
                    s.success ? "ok  " : "FAIL", s.time_to_first_solution_s, s.cost_total, s.path_length_m,
                    dir.c_str());
        std::fflush(stdout);
      } catch (const std::exception &e) {
        ++failures;
        std::lock_guard<std::mutex> lock(print_mutex);
        std::printf("[%zu/%zu] %s %s %s %s: ERROR %s\n", i + 1, runs.size(), c.scenario.name.c_str(),
                    MoD::to_string(c.planner.type).c_str(), MoD::to_string(c.sampler.type).c_str(),
                    MoD::to_string(c.objective.type).c_str(), e.what());
        std::fflush(stdout);
      }
    }
  };

  std::vector<std::thread> threads;
  for (unsigned int t = 0; t < spec.threads; ++t) threads.emplace_back(worker);
  for (auto &t : threads) t.join();

  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("run_batch: done, %zu/%zu runs solved, %zu errors, %.1f s wall\n", successes.load(), runs.size(),
              failures.load(), wall);
  return failures == 0 ? 0 : 1;
}
