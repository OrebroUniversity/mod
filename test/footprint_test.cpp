#include <gtest/gtest.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <cmath>

#include "core/footprint_checker.hpp"
#include "core/occupancy_map.hpp"

namespace ob = ompl::base;
using MoD::playground::FootprintChecker;
using MoD::playground::OccupancyMap;

namespace {

// 4 x 4 m at 0.1 m (40 x 40 px) with a wall of one pixel column at x in [2.0, 2.1).
std::shared_ptr<const OccupancyMap> wallMap() {
  std::vector<uint8_t> occ(40 * 40, 0);
  for (size_t row = 0; row < 40; ++row) occ[row * 40 + 20] = 1;
  return std::make_shared<const OccupancyMap>(40, 40, 0.1, 0.0, 0.0, occ);
}

struct Fixture {
  ob::SpaceInformationPtr si;
  std::shared_ptr<FootprintChecker> checker;
  explicit Fixture(double radius) {
    auto space = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds b(2);
    b.setLow(0.0);
    b.setHigh(4.0);
    space->setBounds(b);
    si = std::make_shared<ob::SpaceInformation>(space);
    checker = std::make_shared<FootprintChecker>(si, wallMap(), radius);
    si->setStateValidityChecker(checker);
    si->setup();
  }
  bool valid(double x, double y, double yaw) const {
    ob::ScopedState<ob::SE2StateSpace> s(si);
    s->setXY(x, y);
    s->setYaw(yaw);
    return checker->isValid(s.get());
  }
};

}  // namespace

TEST(Footprint, DiscSizeMatchesRadius) {
  Fixture f(0.3);
  // pi r^2 / res^2 = 28.3; the rasterized disc of pixel centres has 29 pixels.
  EXPECT_NEAR(static_cast<double>(f.checker->discPixels()), M_PI * 0.3 * 0.3 / 0.01, 3.0);
  EXPECT_DOUBLE_EQ(f.checker->radius(), 0.3);
}

TEST(Footprint, WallClearanceAndYawIrrelevance) {
  Fixture f(0.3);
  for (double yaw : {0.0, 1.0, -2.5, 3.1}) {
    EXPECT_TRUE(f.valid(1.0, 1.0, yaw));
    EXPECT_TRUE(f.valid(1.6, 1.0, yaw));    // 0.4 m from the wall face at 2.0
    EXPECT_FALSE(f.valid(1.85, 1.0, yaw));  // 0.15 m from the wall face
    EXPECT_FALSE(f.valid(2.05, 1.0, yaw));  // inside the wall
    EXPECT_TRUE(f.valid(2.5, 1.0, yaw));    // 0.4 m past the wall (wall spans 2.0-2.1)
    EXPECT_FALSE(f.valid(2.25, 1.0, yaw));
  }
}

TEST(Footprint, OutsideTheMapIsOccupied) {
  Fixture f(0.3);
  EXPECT_FALSE(f.valid(0.1, 0.1, 0.0));   // disc leaves the map
  EXPECT_TRUE(f.valid(0.35, 0.35, 0.0));  // disc fits
  EXPECT_FALSE(f.valid(-1.0, 1.0, 0.0));  // outside the bounds
  EXPECT_FALSE(f.checker->isValidXY(5.0, 1.0));
}

TEST(Footprint, PointRobot) {
  Fixture f(0.0);
  EXPECT_EQ(f.checker->discPixels(), 1u);
  EXPECT_TRUE(f.valid(1.95, 1.0, 0.0));
  EXPECT_FALSE(f.valid(2.05, 1.0, 0.0));
}
