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

#include "core/run_logger.hpp"

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mod/log.hpp>
#include <mod/version.h>
#include <mutex>

namespace fs = std::filesystem;

namespace MoD::playground {

namespace {

std::mutex g_dir_mutex;  // folder-name uniqueness across threads of one process

std::string formatTime(const char *fmt, bool utc, bool with_ms, const char *ms_sep) {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  if (utc)
    gmtime_r(&t, &tm);
  else
    localtime_r(&t, &tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), fmt, &tm);
  std::string out(buf);
  if (with_ms) {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    char msbuf[8];
    std::snprintf(msbuf, sizeof(msbuf), "%s%03lld", ms_sep, static_cast<long long>(ms));
    out += msbuf;
  }
  return out;
}

void writeJson(const std::string &path, const nlohmann::json &j) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("RunLogger: cannot write " + path);
  out << j.dump(2) << "\n";
}

}  // namespace

std::string RunLogger::timestampNow() { return formatTime("%Y%m%d-%H%M%S", false, true, "."); }

std::string RunLogger::isoNow() { return formatTime("%Y-%m-%dT%H:%M:%S", true, true, ".") + "Z"; }

::MoD::RunMeta RunLogger::makeMeta() {
  ::MoD::RunMeta meta;
  meta.mod_version = MOD_VERSION;
  meta.git_hash = MOD_GIT_HASH;
  char host[256] = "unknown";
  if (gethostname(host, sizeof(host)) == 0) host[sizeof(host) - 1] = '\0';
  meta.hostname = host;
  meta.started_at = isoNow();
  return meta;
}

RunLogger::RunLogger(const std::string &log_dir, ::MoD::RunConfig &config) {
  config.meta = makeMeta();
  const std::string base = timestampNow() + "_" + config.scenario.name + "_" + ::MoD::to_string(config.planner.type) +
                           "_" + ::MoD::to_string(config.sampler.type) + "_" +
                           ::MoD::to_string(config.objective.type);
  {
    std::lock_guard<std::mutex> lock(g_dir_mutex);
    fs::create_directories(log_dir);
    fs::path dir = fs::path(log_dir) / base;
    for (int k = 1; fs::exists(dir); ++k) dir = fs::path(log_dir) / (base + "_" + std::to_string(k));
    fs::create_directory(dir);
    dir_ = dir.string();
  }
  if (config.sampler.log_samples) samples_ = std::make_unique<SampleBuffer>();
  writeConfig(config);
  MOD_LOG("RunLogger: %s", dir_.c_str());
}

RunLogger::~RunLogger() {
  try {
    finish();
  } catch (const std::exception &e) {
    MOD_LOG("RunLogger: %s", e.what());
  }
}

void RunLogger::writeConfig(const ::MoD::RunConfig &config) const {
  writeJson((fs::path(dir_) / "config.json").string(), config);
}

void RunLogger::writeSolution(const Solution &solution) const {
  writeJson((fs::path(dir_) / "solution.json").string(), solution);
}

void RunLogger::finish() {
  if (finished_) return;
  finished_ = true;
  if (!samples_) return;
  nlohmann::json rows = nlohmann::json::array();
  for (const auto &r : samples_->rows()) rows.push_back({r.x, r.y, r.theta, ::MoD::to_string(r.source)});
  std::ofstream out((fs::path(dir_) / "samples.json").string());
  if (!out) throw std::runtime_error("RunLogger: cannot write samples.json in " + dir_);
  out << rows.dump() << "\n";
  MOD_LOG("RunLogger: %zu samples written", samples_->size());
}

}  // namespace MoD::playground
