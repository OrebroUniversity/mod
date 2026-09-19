#include <gtest/gtest.h>
#include <ompl/mod/samplers/HybridSampler.h>

#include "test_helpers.hpp"

using namespace mod_test;

TEST(HybridSampler, BranchProportionsMatchBiases) {
  // 12 x 12 m world with an intensity grid of 13 x 13 cells at 1 m and no obstacles.
  std::vector<double> values(13 * 13, 0.3);
  const std::string file = writeIntensityXml("q13x13.xml", 1.0, 0.0, 0.0, 13, 13, values);
  auto map = std::make_shared<const ::MoD::IntensityMap>(file);
  auto si = makeSE2(0.0, 12.0, 0.0, 12.0, [](double, double) { return true; });
  auto pdef = makeProblem(si, {1.0, 1.0, 0.0}, {11.0, 11.0, 0.0});

  MoD::SamplerParameters p;
  p.type = MoD::SamplerType::hybrid;
  p.bias = 0.05;
  p.hybrid_intensity_bias = 0.01;
  p.dijkstra_cell_size = 1.0;
  ompl::MoD::HybridSampler sampler(pdef, 100, p, map);

  size_t n_dijkstra = 0, n_intensity = 0, n_ellipse = 0;
  sampler.setBranchHook([&](ompl::MoD::HybridSampler::Branch b) {
    switch (b) {
      case ompl::MoD::HybridSampler::Branch::dijkstra:
        ++n_dijkstra;
        break;
      case ompl::MoD::HybridSampler::Branch::intensity:
        ++n_intensity;
        break;
      case ompl::MoD::HybridSampler::Branch::ellipse:
        ++n_ellipse;
        break;
    }
  });

  const size_t n = 100000;
  ob::ScopedState<ob::SE2StateSpace> s(si);
  for (size_t i = 0; i < n; ++i) {
    ASSERT_TRUE(sampler.sampleUniform(s.get(), ob::Cost(std::numeric_limits<double>::infinity())));
    EXPECT_GE(s->getX(), 0.0);
    EXPECT_LE(s->getX(), 12.0);
    EXPECT_GE(s->getY(), 0.0);
    EXPECT_LE(s->getY(), 12.0);
  }
  EXPECT_EQ(n_dijkstra + n_intensity + n_ellipse, n);
  EXPECT_NEAR(static_cast<double>(n_dijkstra) / n, 0.05, 0.01);
  EXPECT_NEAR(static_cast<double>(n_intensity) / n, 0.01, 0.01);
  EXPECT_NEAR(static_cast<double>(n_ellipse) / n, 0.94, 0.01);
  EXPECT_GT(n_intensity, 0u);
}
