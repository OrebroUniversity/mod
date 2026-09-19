#pragma once

#include <gtest/gtest.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace mod_test {

namespace ob = ompl::base;

/// Writes an intensity-map XML (rows x cols, row-major `values`) and returns the file name.
inline std::string writeIntensityXml(const std::string &name, double cell, double x_min, double y_min, size_t rows,
                                     size_t cols, const std::vector<double> &values) {
  const std::string file = std::string(::testing::TempDir()) + "/" + name;
  std::ofstream out(file);
  out << "<map><parameters><cell_size>" << cell << "</cell_size><x_min>" << x_min << "</x_min><y_min>" << y_min
      << "</y_min><x_max>" << x_min + cell * static_cast<double>(cols - 1) << "</x_max><y_max>"
      << y_min + cell * static_cast<double>(rows - 1) << "</y_max></parameters><cells>";
  for (size_t r = 0; r < rows; ++r)
    for (size_t c = 0; c < cols; ++c)
      out << "<cell><row>" << r << "</row><col>" << c << "</col><value>" << values[r * cols + c]
          << "</value></cell>";
  out << "</cells></map>";
  return file;
}

typedef std::function<bool(double, double)> XYChecker;

/// SE2 space over [x_lo, x_hi] x [y_lo, y_hi] with a validity checker on (x, y).
inline ob::SpaceInformationPtr makeSE2(double x_lo, double x_hi, double y_lo, double y_hi, XYChecker valid) {
  auto space = std::make_shared<ob::SE2StateSpace>();
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, x_lo);
  bounds.setHigh(0, x_hi);
  bounds.setLow(1, y_lo);
  bounds.setHigh(1, y_hi);
  space->setBounds(bounds);
  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker([valid](const ob::State *s) {
    const auto *se2 = s->as<ob::SE2StateSpace::StateType>();
    return valid(se2->getX(), se2->getY());
  });
  si->setup();
  return si;
}

/// Problem definition with a start state, a GoalState and the given objective (path length if null).
inline ob::ProblemDefinitionPtr makeProblem(const ob::SpaceInformationPtr &si, const std::array<double, 3> &start,
                                            const std::array<double, 3> &goal,
                                            ob::OptimizationObjectivePtr objective = nullptr) {
  auto pdef = std::make_shared<ob::ProblemDefinition>(si);
  ob::ScopedState<ob::SE2StateSpace> s(si), g(si);
  s->setXY(start[0], start[1]);
  s->setYaw(start[2]);
  g->setXY(goal[0], goal[1]);
  g->setYaw(goal[2]);
  pdef->addStartState(s);
  auto goal_state = std::make_shared<ob::GoalState>(si);
  goal_state->setState(g);
  pdef->setGoal(goal_state);
  if (!objective) objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
  pdef->setOptimizationObjective(objective);
  return pdef;
}

inline double angleDiff(double a, double b) { return std::abs(std::atan2(std::sin(a - b), std::cos(a - b))); }

}  // namespace mod_test
