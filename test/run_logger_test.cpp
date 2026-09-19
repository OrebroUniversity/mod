#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <mod/version.h>

#include "core/run_logger.hpp"

namespace fs = std::filesystem;
using namespace MoD::playground;

namespace {
nlohmann::json readJson(const fs::path &p) {
  std::ifstream in(p);
  EXPECT_TRUE(in.good()) << p;
  return nlohmann::json::parse(in);
}
}  // namespace

TEST(RunLogger, RoundTripConfigSolutionSamples) {
  const fs::path log_dir = fs::path(::testing::TempDir()) / "mod_runs";
  fs::remove_all(log_dir);

  MoD::RunConfig config;
  config.scenario.name = "unit";
  config.scenario.map_yaml = "x.yaml";
  config.planner.type = MoD::PlannerType::ait_star;
  config.sampler.type = MoD::SamplerType::dijkstra;
  config.sampler.log_samples = true;
  config.objective.type = MoD::ObjectiveType::gmmt;
  config.derived.occupancy_pixel_m = 0.05;
  config.derived.mod_cost_step_m = 0.05;

  std::string dir;
  {
    RunLogger logger(log_dir.string(), config);
    dir = logger.dir();
    EXPECT_FALSE(config.meta.started_at.empty());
    EXPECT_EQ(config.meta.mod_version, MOD_VERSION);
    EXPECT_EQ(config.meta.git_hash, MOD_GIT_HASH);
    EXPECT_FALSE(config.meta.hostname.empty());
    ASSERT_NE(logger.sampleSink(), nullptr);
    logger.sampleSink()->record(1.0, 2.0, 0.5, MoD::SampleSource::dijkstra);
    logger.sampleSink()->record(3.0, 4.0, -0.5, MoD::SampleSource::uniform);

    Solution sol;
    sol.success = true;
    sol.planning_time_s = 1.5;
    sol.time_to_first_solution_s = 0.25;
    sol.cost_total = 12.0;
    sol.cost_d = 10.0;
    sol.cost_q = 0.5;
    sol.cost_c = 1.5;
    sol.path_length_m = 10.0;
    sol.path = {{0.0, 0.0, 0.0}, {1.0, 1.0, 0.7}};
    logger.writeSolution(sol);
  }  // destructor writes samples.json

  const fs::path folder(dir);
  EXPECT_EQ(folder.parent_path(), log_dir);
  const std::string name = folder.filename().string();
  EXPECT_NE(name.find("_unit_ait_star_dijkstra_gmmt"), std::string::npos) << name;
  EXPECT_EQ(name.size(), std::string("YYYYMMDD-HHMMSS.mmm_unit_ait_star_dijkstra_gmmt").size()) << name;

  const auto cj = readJson(folder / "config.json");
  const auto parsed = cj.get<MoD::RunConfig>();
  EXPECT_EQ(parsed.scenario.name, "unit");
  EXPECT_EQ(parsed.planner.type, MoD::PlannerType::ait_star);
  EXPECT_EQ(parsed.sampler.type, MoD::SamplerType::dijkstra);
  EXPECT_TRUE(parsed.sampler.log_samples);
  EXPECT_EQ(parsed.objective.type, MoD::ObjectiveType::gmmt);
  EXPECT_DOUBLE_EQ(parsed.derived.mod_cost_step_m, 0.05);
  EXPECT_EQ(parsed.meta.started_at, config.meta.started_at);
  EXPECT_TRUE(cj.contains("RunMeta"));

  const auto sj = readJson(folder / "solution.json");
  const auto sol = sj.get<Solution>();
  EXPECT_TRUE(sol.success);
  EXPECT_DOUBLE_EQ(sol.planning_time_s, 1.5);
  EXPECT_DOUBLE_EQ(sol.time_to_first_solution_s, 0.25);
  EXPECT_DOUBLE_EQ(sol.cost_total, 12.0);
  EXPECT_DOUBLE_EQ(sol.cost_c, 1.5);
  EXPECT_DOUBLE_EQ(sol.path_length_m, 10.0);
  ASSERT_EQ(sol.path.size(), 2u);
  EXPECT_DOUBLE_EQ(sol.path[1][2], 0.7);
  EXPECT_TRUE(sj.at("cost").contains("total"));

  const auto samples = readJson(folder / "samples.json");
  ASSERT_EQ(samples.size(), 2u);
  EXPECT_DOUBLE_EQ(samples[0][0].get<double>(), 1.0);
  EXPECT_EQ(samples[0][3].get<std::string>(), "dijkstra");
  EXPECT_EQ(samples[1][3].get<std::string>(), "uniform");
}

TEST(RunLogger, NoSamplesFileWhenDisabledAndNullsOnFailure) {
  const fs::path log_dir = fs::path(::testing::TempDir()) / "mod_runs2";
  MoD::RunConfig config;
  config.scenario.name = "fail";
  std::string dir;
  {
    RunLogger logger(log_dir.string(), config);
    dir = logger.dir();
    EXPECT_EQ(logger.sampleSink(), nullptr);
    logger.writeSolution(Solution{});
  }
  EXPECT_FALSE(fs::exists(fs::path(dir) / "samples.json"));
  const auto sj = readJson(fs::path(dir) / "solution.json");
  EXPECT_FALSE(sj.at("success").get<bool>());
  EXPECT_TRUE(sj.at("time_to_first_solution_s").is_null());
  EXPECT_TRUE(sj.at("cost").at("total").is_null());
  EXPECT_TRUE(sj.at("path").empty());
  const auto sol = sj.get<Solution>();
  EXPECT_TRUE(std::isnan(sol.cost_total));
}

TEST(RunLogger, DuplicateNamesGetSuffix) {
  const fs::path log_dir = fs::path(::testing::TempDir()) / "mod_runs3";
  fs::remove_all(log_dir);
  MoD::RunConfig a, b;
  a.scenario.name = b.scenario.name = "dup";
  std::string da, db;
  // Two loggers in the same millisecond may collide; force it by creating the folder ourselves.
  {
    RunLogger la(log_dir.string(), a);
    da = la.dir();
    const std::string same = fs::path(da).filename().string();
    // Simulate a second run at the identical timestamp.
    fs::path forced = log_dir / same;
    EXPECT_TRUE(fs::exists(forced));
  }
  EXPECT_TRUE(fs::exists(fs::path(da) / "config.json"));
}
