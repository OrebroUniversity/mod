#include <gtest/gtest.h>
#include <ompl/mod/samplers/DijkstraSampler.h>

#include "test_helpers.hpp"

using namespace mod_test;

namespace {

// 12 x 12 m world, 1 m cells (13 x 13 nodes). A wall along x = 6 with one gap at y in [5, 7].
bool wallWithGap(double x, double y) {
  const bool in_wall = x > 5.5 && x < 6.5;
  const bool in_gap = y >= 4.5 && y <= 7.5;
  return !(in_wall && !in_gap);
}

struct Fixture {
  ob::SpaceInformationPtr si;
  ob::ProblemDefinitionPtr pdef;
  std::shared_ptr<ompl::MoD::DijkstraSampler> sampler;

  explicit Fixture(double bias, XYChecker checker = wallWithGap) {
    si = makeSE2(0.0, 12.0, 0.0, 12.0, std::move(checker));
    pdef = makeProblem(si, {1.0, 1.0, 0.0}, {11.0, 11.0, 0.0});
    MoD::SamplerParameters p;
    p.type = MoD::SamplerType::dijkstra;
    p.bias = bias;
    p.dijkstra_cell_size = 1.0;
    sampler = std::make_shared<ompl::MoD::DijkstraSampler>(pdef, 100, p, nullptr);
  }
};

}  // namespace

TEST(DijkstraSampler, PathPassesTheGap) {
  Fixture f(1.0);
  const auto &path = f.sampler->path();
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(f.sampler->rows(), 13u);
  EXPECT_EQ(f.sampler->cols(), 13u);
  EXPECT_EQ(path.front(), 1u * 13u + 1u);
  EXPECT_EQ(path.back(), 11u * 13u + 11u);
  bool through_gap = false;
  for (size_t node : path) {
    const size_t col = node % 13, row = node / 13;
    EXPECT_TRUE(wallWithGap(f.sampler->colToX(col), f.sampler->rowToY(row))) << "path node in the wall";
    if (col == 6 && row >= 5 && row <= 7) through_gap = true;
  }
  EXPECT_TRUE(through_gap);
}

TEST(DijkstraSampler, BiasedDrawsLieOnThePathWithHeadingTowardsNextCell) {
  Fixture f(1.0);
  std::vector<size_t> path(f.sampler->path().begin(), f.sampler->path().end());
  ASSERT_GE(path.size(), 2u);
  const double half = f.sampler->cellSize() / 2.0 + 1e-9;

  ob::ScopedState<ob::SE2StateSpace> s(f.si);
  for (int i = 0; i < 5000; ++i) {
    ASSERT_TRUE(f.sampler->sampleUniform(s.get(), ob::Cost(std::numeric_limits<double>::infinity())));
    const double x = s->getX(), y = s->getY(), yaw = s->getYaw();
    EXPECT_GE(x, 0.0);
    EXPECT_LE(x, 12.0);
    EXPECT_GE(y, 0.0);
    EXPECT_LE(y, 12.0);

    // Some path node whose cell contains the draw and whose heading (to the next cell, or from the previous
    // cell for the last node) is within pi/8 of the drawn yaw.
    bool matched = false;
    for (size_t k = 0; k < path.size() && !matched; ++k) {
      const double cx = f.sampler->colToX(path[k] % 13), cy = f.sampler->rowToY(path[k] / 13);
      if (std::abs(x - cx) > half || std::abs(y - cy) > half) continue;
      double heading;
      if (k + 1 < path.size()) {
        const double nx = f.sampler->colToX(path[k + 1] % 13), ny = f.sampler->rowToY(path[k + 1] / 13);
        heading = std::atan2(ny - cy, nx - cx);
      } else {
        const double px = f.sampler->colToX(path[k - 1] % 13), py = f.sampler->rowToY(path[k - 1] / 13);
        heading = std::atan2(cy - py, cx - px);
      }
      if (angleDiff(yaw, heading) <= M_PI / 8.0 + 1e-9) matched = true;
    }
    EXPECT_TRUE(matched) << "draw (" << x << ", " << y << ", " << yaw << ") is not on the path with a path heading";
  }
}

TEST(DijkstraSampler, LastNodeHeadingComesFromPreviousCell) {
  // Only the last path node can be drawn when the path has the goal at its end; force draws there by
  // checking that at least some draws land in the goal cell with the previous-cell heading.
  Fixture f(1.0);
  std::vector<size_t> path(f.sampler->path().begin(), f.sampler->path().end());
  const size_t last = path.back(), prev = path[path.size() - 2];
  const double gx = f.sampler->colToX(last % 13), gy = f.sampler->rowToY(last / 13);
  const double px = f.sampler->colToX(prev % 13), py = f.sampler->rowToY(prev / 13);
  const double heading = std::atan2(gy - py, gx - px);
  ob::ScopedState<ob::SE2StateSpace> s(f.si);
  int hits = 0;
  for (int i = 0; i < 20000; ++i) {
    f.sampler->sampleUniform(s.get(), ob::Cost(1e9));
    if (std::abs(s->getX() - gx) <= 0.5 && std::abs(s->getY() - gy) <= 0.5) {
      ++hits;
      EXPECT_LE(angleDiff(s->getYaw(), heading), M_PI / 8.0 + 1e-9);
    }
  }
  EXPECT_GT(hits, 0);
}

TEST(DijkstraSampler, UniformDrawsStayInBounds) {
  Fixture f(0.0);
  ob::ScopedState<ob::SE2StateSpace> s(f.si);
  for (int i = 0; i < 20000; ++i) {
    f.sampler->sampleUniform(s.get(), ob::Cost(1e9));
    EXPECT_GE(s->getX(), 0.0);
    EXPECT_LE(s->getX(), 12.0);
    EXPECT_GE(s->getY(), 0.0);
    EXPECT_LE(s->getY(), 12.0);
  }
}

TEST(DijkstraSampler, UnreachableGoalFallsBackToUniform) {
  // Full wall, no gap: no path; sampling must still work.
  Fixture f(1.0, [](double x, double) { return !(x > 5.5 && x < 6.5); });
  EXPECT_TRUE(f.sampler->path().empty());
  ob::ScopedState<ob::SE2StateSpace> s(f.si);
  for (int i = 0; i < 1000; ++i) {
    EXPECT_TRUE(f.sampler->sampleUniform(s.get(), ob::Cost(1e9)));
    EXPECT_GE(s->getX(), 0.0);
    EXPECT_LE(s->getX(), 12.0);
  }
}
