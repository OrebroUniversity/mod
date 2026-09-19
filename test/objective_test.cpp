#include <gtest/gtest.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/mod/objectives/IntensityMapOptimizationObjective.h>

#include <cstdio>
#include <fstream>
#include <mod/cliffmap.hpp>
#include <string>

namespace ob = ompl::base;

namespace {

/// Writes an intensity XML with a constant value q over a rows x cols grid; returns the file name.
std::string writeConstantQ(double q, double cell, double x_min, double y_min, size_t rows, size_t cols) {
  const std::string file = std::string(::testing::TempDir()) + "/const_q.xml";
  std::ofstream out(file);
  out << "<map><parameters><cell_size>" << cell << "</cell_size><x_min>" << x_min << "</x_min><y_min>" << y_min
      << "</y_min><x_max>" << x_min + cell * (cols - 1) << "</x_max><y_max>" << y_min + cell * (rows - 1)
      << "</y_max></parameters><cells>";
  for (size_t r = 0; r < rows; ++r)
    for (size_t c = 0; c < cols; ++c)
      out << "<cell><row>" << r << "</row><col>" << c << "</col><value>" << q << "</value></cell>";
  out << "</cells></map>";
  return file;
}

ob::SpaceInformationPtr makeSE2(double lo, double hi) {
  auto space = std::make_shared<ob::SE2StateSpace>();
  ob::RealVectorBounds bounds(2);
  bounds.setLow(lo);
  bounds.setHigh(hi);
  space->setBounds(bounds);
  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker([](const ob::State *) { return true; });
  si->setup();
  return si;
}

}  // namespace

TEST(Objective, CostStepPerPointCost) {
  const double q = 0.3;
  const std::string file = writeConstantQ(q, 1.0, -5.0, -5.0, 11, 11);
  auto si = makeSE2(-5.0, 5.0);
  MoD::OptObjParameters p;
  p.type = MoD::ObjectiveType::intensity;
  p.intensity_map_file = file;
  p.w_d = 1.5;
  p.w_q = 1.0;
  p.w_c = 2.0;
  ompl::MoD::IntensityMapOptimizationObjective obj(si, p, MoD::SamplerParameters{});
  EXPECT_DOUBLE_EQ(obj.getCostStep(), 1.0);  // library default: the map's cell size
  obj.setCostStep(0.25);

  ob::ScopedState<ob::SE2StateSpace> a(si), b(si);
  a->setXY(0.0, 0.0);
  a->setYaw(0.0);
  b->setXY(2.0, 0.0);
  b->setYaw(0.0);

  const auto cc = obj.motionCostComponents(a.get(), b.get());
  EXPECT_NEAR(cc.d, 2.0, 1e-9);
  EXPECT_NEAR(cc.q, 0.0, 1e-12);
  EXPECT_NEAR(cc.c, q * 8, 1e-9);  // 8 cost points at 0.25 m
  EXPECT_NEAR(obj.motionCost(a.get(), b.get()).value(), p.w_d * 2.0 + p.w_c * q * 8, 1e-9);

  // Zero-length edges still cost one point.
  const auto zero = obj.motionCostComponents(a.get(), a.get());
  EXPECT_NEAR(zero.d, 0.0, 1e-12);
  EXPECT_NEAR(zero.c, q, 1e-12);

  // Larger step: n = ceil(2 / 0.6) = 4.
  obj.setCostStep(0.6);
  EXPECT_NEAR(obj.motionCostComponents(a.get(), b.get()).c, q * 4, 1e-9);
  std::remove(file.c_str());
}

TEST(Objective, DubinsInterpolationFollowsSteering) {
  const double q = 0.5;
  const std::string file = writeConstantQ(q, 1.0, -10.0, -10.0, 21, 21);
  auto space = std::make_shared<ob::DubinsStateSpace>(1.0);
  ob::RealVectorBounds bounds(2);
  bounds.setLow(-10.0);
  bounds.setHigh(10.0);
  space->setBounds(bounds);
  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker([](const ob::State *) { return true; });
  si->setup();

  MoD::OptObjParameters p;
  p.type = MoD::ObjectiveType::intensity;
  p.intensity_map_file = file;
  ompl::MoD::IntensityMapOptimizationObjective obj(si, p, MoD::SamplerParameters{});
  obj.setCostStep(0.1);

  ob::ScopedState<ob::SE2StateSpace> a(si), b(si);
  a->setXY(0.0, 0.0);
  a->setYaw(0.0);
  b->setXY(0.0, 3.0);
  b->setYaw(M_PI);  // needs a turn: Dubins length > Euclidean 3
  const double dubins = si->distance(a.get(), b.get());
  EXPECT_GT(dubins, 3.0);
  const auto cc = obj.motionCostComponents(a.get(), b.get());
  EXPECT_NEAR(cc.d, dubins, 1e-6);  // sum of sub-segment distances along the Dubins path
  EXPECT_NEAR(cc.c, q * std::ceil(dubins / 0.1), 1e-9);
  EXPECT_GT(cc.q, 0.0);
  std::remove(file.c_str());
}
