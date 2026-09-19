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
#include <memory>
#include <mod/parameters.hpp>
#include <mod/sample_sink.hpp>
#include <string>
#include <vector>

#include "core/solver.hpp"

namespace MoD::playground {

/// In-memory sample sink written to samples.json by the logger.
class SampleBuffer : public ::MoD::SampleSink {
 public:
  struct Row {
    double x, y, theta;
    ::MoD::SampleSource source;
  };
  void record(double x, double y, double theta, ::MoD::SampleSource source) override {
    rows_.push_back({x, y, theta, source});
  }
  const std::vector<Row> &rows() const { return rows_; }
  size_t size() const { return rows_.size(); }

 private:
  std::vector<Row> rows_;
};

/**
 * One folder per run: `<log_dir>/<YYYYMMDD-HHMMSS.mmm>_<scenario>_<planner>_<sampler>_<objective>/` with
 * `config.json` (written on construction), `solution.json` (writeSolution) and `samples.json` (finish, only
 * if `config.sampler.log_samples`). Fills `config.meta` (version, git hash, hostname, start time).
 */
class RunLogger {
 public:
  RunLogger(const std::string &log_dir, ::MoD::RunConfig &config);
  ~RunLogger();

  const std::string &dir() const { return dir_; }

  /// Non-null iff samples are logged for this run. Hand it to the objective before solving.
  ::MoD::SampleSink *sampleSink() { return samples_ ? samples_.get() : nullptr; }

  /// Rewrites config.json (e.g. after the factory filled `Derived`).
  void writeConfig(const ::MoD::RunConfig &config) const;
  void writeSolution(const Solution &solution) const;
  /// Writes samples.json if enabled; idempotent, also called by the destructor.
  void finish();

  static std::string timestampNow();  ///< YYYYMMDD-HHMMSS.mmm local time
  static std::string isoNow();        ///< ISO 8601 UTC with milliseconds
  static ::MoD::RunMeta makeMeta();

 private:
  std::string dir_;
  std::unique_ptr<SampleBuffer> samples_;
  bool finished_{false};
};

}  // namespace MoD::playground
