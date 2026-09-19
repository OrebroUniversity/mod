#include <gtest/gtest.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/mod/objectives/IntensityMapOptimizationObjective.h>

#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <mod/planners/hybrid_astar.hpp>
#include <vector>

#include "core/footprint_checker.hpp"
#include "core/occupancy_map.hpp"
#include "test_helpers.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using MoD::HybridAStar;
using MoD::HybridAStarParameters;
using MoD::playground::FootprintChecker;
using MoD::playground::OccupancyMap;
using MoD::playground::OccupancyMapConstPtr;

namespace {

typedef std::function<bool(double x, double y)> Occupied;

/// A `size_m` x `size_m` map at `res` metres per pixel; `occupied(x, y)` tested at every pixel centre.
OccupancyMapConstPtr makeMap(double size_m, double res, const Occupied &occupied) {
  const auto n = static_cast<size_t>(std::lround(size_m / res));
  std::vector<uint8_t> occ(n * n, 0);
  for (size_t row = 0; row < n; ++row)
    for (size_t col = 0; col < n; ++col)
      occ[row * n + col] = occupied((static_cast<double>(col) + 0.5) * res, (static_cast<double>(row) + 0.5) * res) ? 1 : 0;
  return std::make_shared<const OccupancyMap>(n, n, res, 0.0, 0.0, occ);
}

/// Everything a Hybrid A* run needs over an in-memory map, as the playground builds it.
struct World {
  OccupancyMapConstPtr map;
  ob::StateSpacePtr space;
  ob::SpaceInformationPtr si;
  std::shared_ptr<FootprintChecker> checker;
  double turning_radius;

  World(OccupancyMapConstPtr m, bool reeds_shepp = false, double radius = 0.3, double r = 1.0)
      : map(std::move(m)), turning_radius(r) {
    if (reeds_shepp)
      space = std::make_shared<ob::ReedsSheppStateSpace>(r);
    else
      space = std::make_shared<ob::DubinsStateSpace>(r);
    const auto b = map->bounds();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, b.x_min);
    bounds.setHigh(0, b.x_max);
    bounds.setLow(1, b.y_min);
    bounds.setHigh(1, b.y_max);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);
    si = std::make_shared<ob::SpaceInformation>(space);
    checker = std::make_shared<FootprintChecker>(si, map, radius);
    si->setStateValidityChecker(checker);
    si->setStateValidityCheckingResolution(map->pixel_size() / space->getMaximumExtent());
    si->setup();
  }

  ob::ScopedState<> state(double x, double y, double yaw) const {
    ob::ScopedState<ob::SE2StateSpace> s(space);
    s->setXY(x, y);
    s->setYaw(yaw);
    return ob::ScopedState<>(s);
  }
  ob::OptimizationObjectivePtr pathLength() const { return std::make_shared<ob::PathLengthOptimizationObjective>(si); }
};

bool allValid(const World &w, const og::PathGeometric &path) {
  for (size_t i = 0; i < path.getStateCount(); ++i)
    if (!w.si->isValid(path.getState(i))) return false;
  return true;
}

bool allMotionsValid(const World &w, const og::PathGeometric &path) {
  for (size_t i = 0; i + 1 < path.getStateCount(); ++i)
    if (!w.si->checkMotion(path.getState(i), path.getState(i + 1))) return false;
  return true;
}

double objectiveCost(const ob::OptimizationObjectivePtr &obj, const og::PathGeometric &path) {
  double c = 0.0;
  for (size_t i = 0; i + 1 < path.getStateCount(); ++i)
    c += obj->motionCost(path.getState(i), path.getState(i + 1)).value();
  return c;
}

size_t cuspCount(const std::vector<HybridAStar::Direction> &dirs) {
  size_t c = 0;
  for (size_t i = 1; i < dirs.size(); ++i)
    if (dirs[i] != dirs[i - 1]) ++c;
  return c;
}

/// x at which the path (densely interpolated) first crosses x = x0; returns the y there.
double yAtCrossing(const World &w, const og::PathGeometric &path, double x0) {
  og::PathGeometric dense(path);
  dense.interpolate(2000);
  const auto &s = dense.getStates();
  for (size_t i = 0; i + 1 < s.size(); ++i) {
    const auto *a = s[i]->as<ob::SE2StateSpace::StateType>();
    const auto *b = s[i + 1]->as<ob::SE2StateSpace::StateType>();
    if ((a->getX() < x0) != (b->getX() < x0)) return 0.5 * (a->getY() + b->getY());
  }
  (void)w;
  return std::nan("");
}

bool samePath(const og::PathGeometric &a, const og::PathGeometric &b) {
  if (a.getStateCount() != b.getStateCount()) return false;
  for (size_t i = 0; i < a.getStateCount(); ++i) {
    const auto *p = a.getState(i)->as<ob::SE2StateSpace::StateType>();
    const auto *q = b.getState(i)->as<ob::SE2StateSpace::StateType>();
    if (p->getX() != q->getX() || p->getY() != q->getY() || p->getYaw() != q->getYaw()) return false;
  }
  return true;
}

// Maps -------------------------------------------------------------------------------------------------------

OccupancyMapConstPtr emptyMap(double size_m = 20.0) {
  return makeMap(size_m, 0.1, [](double, double) { return false; });
}

/// Wall at x in [9.9, 10.1] with gaps at the given y intervals.
OccupancyMapConstPtr wallMap(const std::vector<std::pair<double, double>> &gaps) {
  return makeMap(20.0, 0.1, [gaps](double x, double y) {
    if (x < 9.9 || x > 10.1) return false;
    for (const auto &g : gaps)
      if (y >= g.first && y <= g.second) return false;
    return true;
  });
}

/// Two dead-end corridors of width 1.2 m joined by a room: A x in [1, 8], room x in [8, 14] x y in [4, 16],
/// B x in [14, 19]; corridors at y in [9.4, 10.6].
OccupancyMapConstPtr deadEndMap() {
  return makeMap(20.0, 0.1, [](double x, double y) {
    const bool corridor = y >= 9.4 && y <= 10.6 && x >= 1.0 && x <= 19.0;
    const bool room = x >= 8.0 && x <= 14.0 && y >= 4.0 && y <= 16.0;
    return !(corridor || room);
  });
}

/// T: stem x in [8, 12] x y in [1, 12]; bar x in [1, 19] x y in [12, 17].
OccupancyMapConstPtr tMap() {
  return makeMap(20.0, 0.1, [](double x, double y) {
    const bool stem = x >= 8.0 && x <= 12.0 && y >= 1.0 && y <= 12.0;
    const bool bar = x >= 1.0 && x <= 19.0 && y >= 12.0 && y <= 17.0;
    return !(stem || bar);
  });
}

}  // namespace

// HA1 -------------------------------------------------------------------------------------------------------

TEST(HybridAStar, EmptyMapCostNearDubins) {
  World w(emptyMap());
  auto obj = w.pathLength();
  HybridAStar planner(w.si, obj, HybridAStarParameters{}, w.turning_radius);
  const auto start = w.state(2.0, 2.0, 0.0), goal = w.state(18.0, 18.0, 0.0);
  const auto path = planner.solve(start.get(), goal.get(), 30.0);
  ASSERT_TRUE(planner.result().solved) << planner.result().termination;
  EXPECT_GE(path.getStateCount(), 2u);
  EXPECT_TRUE(allValid(w, path));
  EXPECT_TRUE(allMotionsValid(w, path));
  const double dubins = w.si->distance(start.get(), goal.get());
  const double cost = objectiveCost(obj, path);
  EXPECT_NEAR(planner.result().cost, cost, 1e-6);
  EXPECT_LE(cost, 1.10 * dubins) << "cost " << cost << " dubins " << dubins;
  EXPECT_GE(cost, dubins - 1e-6);
  EXPECT_EQ(planner.result().cusps, 0u);
  EXPECT_EQ(planner.pathDirections().size(), path.getStateCount() - 1);
  // Ends exactly at the goal.
  const auto *last = path.getState(path.getStateCount() - 1)->as<ob::SE2StateSpace::StateType>();
  EXPECT_NEAR(last->getX(), 18.0, 1e-9);
  EXPECT_NEAR(last->getY(), 18.0, 1e-9);
}

TEST(HybridAStar, WallGap) {
  World w(wallMap({{9.25, 10.75}}));
  HybridAStar planner(w.si, w.pathLength(), HybridAStarParameters{}, w.turning_radius);
  const auto start = w.state(3.0, 10.0, 0.0), goal = w.state(17.0, 10.0, 0.0);
  const auto path = planner.solve(start.get(), goal.get(), 30.0);
  ASSERT_TRUE(planner.result().solved) << planner.result().termination;
  EXPECT_TRUE(allValid(w, path));
  EXPECT_TRUE(allMotionsValid(w, path));
  const double y = yAtCrossing(w, path, 10.0);
  EXPECT_GT(y, 9.25);
  EXPECT_LT(y, 10.75);
}

TEST(HybridAStar, HeuristicSanity) {
  World w(wallMap({{9.25, 10.75}}));
  auto obj = w.pathLength();
  HybridAStar planner(w.si, obj, HybridAStarParameters{}, w.turning_radius);
  const auto start = w.state(3.0, 10.0, 0.0), goal = w.state(17.0, 10.0, 0.0);
  const auto path = planner.solve(start.get(), goal.get(), 30.0);
  ASSERT_TRUE(planner.result().solved);
  EXPECT_DOUBLE_EQ(planner.heuristicGrid(goal.get()), 0.0);
  const double h = planner.heuristic(start.get());
  const double cost = objectiveCost(obj, path);
  EXPECT_GT(h, 0.0);
  EXPECT_LE(h, 1.15 * cost) << "h " << h << " cost " << cost;
  // The kinematic term alone is the Dubins distance (w_d = 1 for path length).
  EXPECT_NEAR(planner.heuristicKinematic(start.get()), w.si->distance(start.get(), goal.get()), 1e-9);
}

TEST(HybridAStar, IntensityCorridorAvoidsHighQ) {
  // Two gaps in the wall: the lower one (nearer the straight line) is a high-intensity corridor.
  World w(wallMap({{5.0, 6.5}, {13.5, 15.0}}));
  std::vector<double> q(40 * 40, 0.0);  // 0.5 m cells over 20 x 20 m
  for (size_t r = 0; r < 40; ++r)
    for (size_t c = 0; c < 40; ++c) {
      const double x = 0.5 * static_cast<double>(c), y = 0.5 * static_cast<double>(r);
      if (x >= 7.0 && x <= 13.0 && y >= 4.0 && y <= 7.5) q[r * 40 + c] = 1.0;
    }
  const std::string file = mod_test::writeIntensityXml("ha_corridor.xml", 0.5, 0.0, 0.0, 40, 40, q);
  MoD::OptObjParameters p;
  p.type = MoD::ObjectiveType::intensity;
  p.intensity_map_file = file;
  p.w_d = 1.0;
  p.w_c = 50.0;
  auto obj = std::make_shared<ompl::MoD::IntensityMapOptimizationObjective>(w.si, p, MoD::SamplerParameters{});
  obj->setCostStep(0.1);
  HybridAStar planner(w.si, obj, HybridAStarParameters{}, w.turning_radius);
  const auto start = w.state(3.0, 8.0, 0.0), goal = w.state(17.0, 8.0, 0.0);
  const auto path = planner.solve(start.get(), goal.get(), 60.0);
  std::remove(file.c_str());
  ASSERT_TRUE(planner.result().solved) << planner.result().termination;
  EXPECT_TRUE(allValid(w, path));
  const double y = yAtCrossing(w, path, 10.0);
  EXPECT_GT(y, 13.5) << "path took the high-q corridor (y = " << y << ")";
  EXPECT_LT(y, 15.0);
}

TEST(HybridAStar, Deterministic) {
  World w(wallMap({{9.25, 10.75}}));
  const auto start = w.state(3.0, 10.0, 0.0), goal = w.state(17.0, 10.0, 0.0);
  HybridAStar a(w.si, w.pathLength(), HybridAStarParameters{}, w.turning_radius);
  HybridAStar b(w.si, w.pathLength(), HybridAStarParameters{}, w.turning_radius);
  const auto pa = a.solve(start.get(), goal.get(), 30.0);
  const auto pb = b.solve(start.get(), goal.get(), 30.0);
  ASSERT_TRUE(a.result().solved);
  ASSERT_TRUE(b.result().solved);
  EXPECT_TRUE(samePath(pa, pb));
  EXPECT_EQ(a.result().expansions, b.result().expansions);
  // Solving again with the same instance gives the same answer too.
  const auto pa2 = a.solve(start.get(), goal.get(), 30.0);
  EXPECT_TRUE(samePath(pa, pa2));
}

TEST(HybridAStar, TimeBudgetReturnsNoSolution) {
  World w(emptyMap(100.0));
  HybridAStar planner(w.si, w.pathLength(), HybridAStarParameters{}, w.turning_radius);
  const auto start = w.state(5.0, 5.0, 0.0), goal = w.state(95.0, 95.0, 0.0);
  og::PathGeometric path(w.si);
  EXPECT_NO_THROW(path = planner.solve(start.get(), goal.get(), 0.01));
  EXPECT_FALSE(planner.result().solved);
  EXPECT_EQ(path.getStateCount(), 0u);
  EXPECT_EQ(planner.result().termination, "time");
}

TEST(HybridAStar, InvalidStartOrGoal) {
  World w(wallMap({}));
  HybridAStar planner(w.si, w.pathLength(), HybridAStarParameters{}, w.turning_radius);
  const auto start = w.state(3.0, 10.0, 0.0), goal = w.state(17.0, 10.0, 0.0), wall = w.state(10.0, 10.0, 0.0);
  EXPECT_EQ(planner.solve(start.get(), wall.get(), 1.0).getStateCount(), 0u);
  EXPECT_EQ(planner.result().termination, "invalid");
  // A full wall: the search exhausts the reachable half.
  EXPECT_EQ(planner.solve(start.get(), goal.get(), 30.0).getStateCount(), 0u);
  EXPECT_EQ(planner.result().termination, "exhausted");
}

// HA3 -------------------------------------------------------------------------------------------------------

TEST(HybridAStarReverse, ForcedOffUnderDubins) {
  World w(emptyMap());
  HybridAStarParameters p;
  p.allow_reverse = true;
  HybridAStar planner(w.si, w.pathLength(), p, w.turning_radius);
  EXPECT_FALSE(planner.reverseEnabled());
  World rs(emptyMap(), true);
  HybridAStar planner_rs(rs.si, rs.pathLength(), p, rs.turning_radius);
  EXPECT_TRUE(planner_rs.reverseEnabled());
}

TEST(HybridAStarReverse, DeadEndNeedsOneCusp) {
  World w(deadEndMap(), true);
  const auto start = w.state(1.6, 10.0, -M_PI), goal = w.state(18.4, 10.0, 0.0);  // SO(2) bounds: [-pi, pi)
  ASSERT_TRUE(w.si->isValid(start.get()));
  ASSERT_TRUE(w.si->isValid(goal.get()));

  HybridAStarParameters fwd;
  fwd.allow_reverse = false;
  HybridAStar forward_only(w.si, w.pathLength(), fwd, w.turning_radius);
  EXPECT_EQ(forward_only.solve(start.get(), goal.get(), 30.0).getStateCount(), 0u);
  EXPECT_FALSE(forward_only.result().solved);

  HybridAStarParameters rev;
  rev.allow_reverse = true;
  HybridAStar planner(w.si, w.pathLength(), rev, w.turning_radius);
  const auto path = planner.solve(start.get(), goal.get(), 60.0);
  ASSERT_TRUE(planner.result().solved) << planner.result().termination;
  EXPECT_TRUE(allValid(w, path));
  EXPECT_TRUE(allMotionsValid(w, path));
  EXPECT_EQ(planner.result().cusps, 1u);
  EXPECT_EQ(cuspCount(planner.pathDirections()), 1u);
  EXPECT_EQ(planner.pathDirections().front(), HybridAStar::Direction::reverse);
  EXPECT_EQ(planner.pathDirections().back(), HybridAStar::Direction::forward);
  EXPECT_NEAR(planner.result().cost, planner.result().objective_cost + rev.change_penalty, 1e-6);
}

TEST(HybridAStarReverse, OpenMapSamePathWithAndWithoutReverse) {
  World w(emptyMap(), true);
  const auto start = w.state(2.0, 2.0, 0.0), goal = w.state(18.0, 18.0, 0.0);
  HybridAStarParameters fwd, rev;
  fwd.allow_reverse = false;
  rev.allow_reverse = true;
  HybridAStar a(w.si, w.pathLength(), fwd, w.turning_radius);
  HybridAStar b(w.si, w.pathLength(), rev, w.turning_radius);
  const auto pa = a.solve(start.get(), goal.get(), 30.0);
  const auto pb = b.solve(start.get(), goal.get(), 30.0);
  ASSERT_TRUE(a.result().solved);
  ASSERT_TRUE(b.result().solved);
  EXPECT_EQ(b.result().cusps, 0u);
  EXPECT_TRUE(samePath(pa, pb));
  for (auto d : b.pathDirections()) EXPECT_EQ(d, HybridAStar::Direction::forward);
}

TEST(HybridAStarReverse, ZeroPenaltyPrefersCuspsWhenCheaper) {
  World w(tMap(), true);
  auto obj = w.pathLength();
  const auto start = w.state(10.0, 2.0, M_PI_2), goal = w.state(3.0, 14.5, 0.0);
  ASSERT_TRUE(w.si->isValid(start.get()));
  ASSERT_TRUE(w.si->isValid(goal.get()));

  HybridAStarParameters fwd;
  fwd.allow_reverse = false;
  HybridAStar forward_only(w.si, obj, fwd, w.turning_radius);
  const auto pf = forward_only.solve(start.get(), goal.get(), 60.0);
  ASSERT_TRUE(forward_only.result().solved) << forward_only.result().termination;

  HybridAStarParameters rev;
  rev.allow_reverse = true;
  rev.change_penalty = 0.0;
  HybridAStar planner(w.si, obj, rev, w.turning_radius);
  const auto pr = planner.solve(start.get(), goal.get(), 60.0);
  ASSERT_TRUE(planner.result().solved) << planner.result().termination;
  EXPECT_TRUE(allValid(w, pr));
  EXPECT_TRUE(allMotionsValid(w, pr));
  EXPECT_GE(planner.result().cusps, 1u);
  EXPECT_LT(objectiveCost(obj, pr), objectiveCost(obj, pf));
  EXPECT_NEAR(planner.result().cost, planner.result().objective_cost, 1e-9);
}
