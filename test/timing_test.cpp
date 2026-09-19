#include <gtest/gtest.h>
#include <ompl/mod/objectives/IntensityMapOptimizationObjective.h>
#include <ompl/mod/samplers/DijkstraSampler.h>
#include <ompl/mod/samplers/IntensityMapSampler.h>

#include <chrono>
#include <cstdio>

#include "test_helpers.hpp"

using namespace mod_test;

namespace {
double seconds(const std::chrono::steady_clock::time_point &t0) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}
}  // namespace

// Office-cubicles bounds (301 x 301 px at 0.1 m from (-10, -10)) at a 0.5 m Dijkstra cell: 61 x 61 nodes.
TEST(Timing, DijkstraSetup61x61WithIntensityObjective) {
  std::vector<double> values(31 * 31);
  for (size_t i = 0; i < values.size(); ++i) values[i] = 0.1 + 0.8 * static_cast<double>(i % 10) / 10.0;
  const std::string file = writeIntensityXml("office_q.xml", 1.0, -10.0, -10.0, 31, 31, values);
  auto si = makeSE2(-10.0, 20.1, -10.0, 20.1, [](double, double) { return true; });

  MoD::OptObjParameters op;
  op.type = MoD::ObjectiveType::intensity;
  op.intensity_map_file = file;
  op.w_c = 0.2;
  auto objective = std::make_shared<ompl::MoD::IntensityMapOptimizationObjective>(si, op, MoD::SamplerParameters{});
  objective->setCostStep(0.1);  // the office pixel size
  auto pdef = makeProblem(si, {-9.0, -9.0, 0.0}, {19.0, 19.0, 0.0}, objective);

  MoD::SamplerParameters sp;
  sp.type = MoD::SamplerType::dijkstra;
  sp.dijkstra_cell_size = 0.5;
  const auto t0 = std::chrono::steady_clock::now();
  ompl::MoD::DijkstraSampler sampler(pdef, 100, sp, nullptr);
  const double dt = seconds(t0);
  std::printf("[timing] Dijkstra setup on %zu x %zu nodes: %.3f s (%zu edges evaluated)\n", sampler.rows(),
              sampler.cols(), dt, sampler.grid().evaluatedEdges());
  EXPECT_EQ(sampler.rows(), 61u);
  EXPECT_EQ(sampler.cols(), 61u);
  EXPECT_FALSE(sampler.path().empty());
  EXPECT_LT(dt, 5.0);
}

TEST(Timing, OneMillionIntensityDraws) {
  std::vector<double> values(140 * 60);
  for (size_t i = 0; i < values.size(); ++i) values[i] = static_cast<double>(i % 100) / 100.0;
  const std::string file = writeIntensityXml("atc_like_q.xml", 1.0, -60.0, -40.0, 60, 140, values);
  auto map = std::make_shared<const ::MoD::IntensityMap>(file);
  auto si = makeSE2(-60.0, 80.0, -40.0, 20.0, [](double, double) { return true; });
  auto pdef = makeProblem(si, {0.0, 0.0, 0.0}, {10.0, 10.0, 0.0});
  MoD::SamplerParameters sp;
  sp.type = MoD::SamplerType::intensity;
  sp.bias = 0.5;
  ompl::MoD::IntensityMapSampler sampler(pdef, 100, sp, map);

  ob::ScopedState<ob::SE2StateSpace> s(si);
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 1000000; ++i) sampler.sampleUniform(s.get(), ob::Cost(1e9));
  const double dt = seconds(t0);
  std::printf("[timing] 1M intensity draws over %zu cells: %.3f s\n", sampler.size(), dt);
  EXPECT_LT(dt, 1.0);
}
