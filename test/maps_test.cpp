#include <gtest/gtest.h>

#include <cmath>
#include <mod/cliffmap.hpp>
#include <mod/gmmtmap.hpp>
#include <string>

namespace {
const std::string kData = MOD_TEST_DATA_DIR;
}

TEST(Maps, IntensityMapATC) {
  ::MoD::IntensityMap map(kData + "/atc/atc_intensity1m.xml");
  EXPECT_DOUBLE_EQ(map.getXMin(), -60.0);
  EXPECT_DOUBLE_EQ(map.getYMin(), -40.0);
  EXPECT_DOUBLE_EQ(map.getXMax(), 79.0);
  EXPECT_DOUBLE_EQ(map.getYMax(), 19.0);
  EXPECT_DOUBLE_EQ(map.getCellSize(), 1.0);
  EXPECT_EQ(map.getRows(), 60u);
  EXPECT_EQ(map.getColumns(), 140u);
  // Known cell: row 13, col 103 -> (x, y) = (43, -27).
  const auto xy = map.getXYatIndex(13 * 140 + 103);
  EXPECT_DOUBLE_EQ(xy[0], 43.0);
  EXPECT_DOUBLE_EQ(xy[1], -27.0);
  EXPECT_NEAR(map(43.0, -27.0), 0.158412749959, 1e-12);
  EXPECT_NEAR(map(43.4, -26.6), 0.158412749959, 1e-12);
  EXPECT_DOUBLE_EQ(map(-1000.0, 0.0), 0.0);  // outside
}

TEST(Maps, CLiFFMapATC) {
  ::MoD::CLiFFMap map(kData + "/atc/atc_cliff.xml", true);
  EXPECT_TRUE(map.isOrganized());
  EXPECT_DOUBLE_EQ(map.getXMin(), -60.0);
  EXPECT_DOUBLE_EQ(map.getXMax(), 80.0);
  EXPECT_DOUBLE_EQ(map.getYMin(), -40.0);
  EXPECT_DOUBLE_EQ(map.getYMax(), 20.0);
  EXPECT_DOUBLE_EQ(map.getResolution(), 1.0);
  EXPECT_EQ(map.getLocations().size(), 8601u);  // 141 x 61
  EXPECT_DOUBLE_EQ(map.rows_, 61.0);
  EXPECT_DOUBLE_EQ(map.columns_, 141.0);

  // First location of the file is id 1 at (-60, -40) with no distributions.
  const auto &first = map.atId(1);
  EXPECT_EQ(first.id, 1u);
  EXPECT_DOUBLE_EQ(first.position[0], -60.0);
  EXPECT_DOUBLE_EQ(first.position[1], -40.0);
  EXPECT_TRUE(first.distributions.empty());

  // Some location carries a distribution; count matches the file (3409 <distribution> tags).
  size_t n_dist = 0;
  for (const auto &loc : map.getLocations()) n_dist += loc.distributions.size();
  EXPECT_EQ(n_dist, 3409u);

  // Reference return is stable across calls and across at / operator().
  const auto &a = map(0.0, 0.0);
  const auto &b = map(0.0, 0.0);
  EXPECT_EQ(&a, &b);
  EXPECT_EQ(&map.at(40, 60), &a);
  EXPECT_DOUBLE_EQ(a.position[0], 0.0);
  EXPECT_DOUBLE_EQ(a.position[1], 0.0);

  // Out of range yields the same static empty location every time.
  const auto &e1 = map.at(10000, 10000);
  const auto &e2 = map(-1000.0, -1000.0);
  EXPECT_EQ(&e1, &e2);
  EXPECT_TRUE(e1.distributions.empty());
  EXPECT_EQ(&map.atId(0), &e1);
}

TEST(Maps, GMMTMapATC) {
  ::MoD::GMMTMap map(kData + "/atc/atc_gmmt.xml");
  EXPECT_EQ(map.getM(), 21);
  EXPECT_EQ(map.getK(), 30);
  EXPECT_DOUBLE_EQ(map.getStdDev(), 1.5);
  ASSERT_EQ(map.getClusters().size(), 21u);
  const auto &c0 = map.getClusters()[0];
  EXPECT_NEAR(c0.mixing_factor, 0.0161690859844, 1e-12);
  ASSERT_EQ(c0.mean.size(), 30u);
  ASSERT_EQ(c0.heading.size(), 30u);
  EXPECT_NEAR(c0.mean[0][0], 7.45063184949, 1e-9);
  EXPECT_NEAR(c0.mean[0][1], -7.08765481118, 1e-9);
  // Heading of the first mean points to the second mean.
  const double expected = std::atan2(c0.mean[1][1] - c0.mean[0][1], c0.mean[1][0] - c0.mean[0][0]);
  EXPECT_NEAR(c0.heading[0], expected, 1e-12);
  // Heading of the last mean points from the previous mean (no out-of-range read).
  const double last = std::atan2(c0.mean[29][1] - c0.mean[28][1], c0.mean[29][0] - c0.mean[28][0]);
  EXPECT_NEAR(c0.heading[29], last, 1e-12);
  for (double h : c0.heading) EXPECT_TRUE(std::isfinite(h));
  // Query near the first mean returns at most one point per cluster.
  const auto near = map(c0.mean[0][0], c0.mean[0][1]);
  EXPECT_FALSE(near.empty());
  EXPECT_EQ(near.front().second[0], 0u);
  EXPECT_DOUBLE_EQ(map.getMixingFactorByClusterID(0), c0.mixing_factor);
  EXPECT_DOUBLE_EQ(map.getHeadingAtDist(0, 0), c0.heading[0]);
}
