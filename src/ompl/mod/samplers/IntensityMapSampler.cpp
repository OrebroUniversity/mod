/*
 *   Copyright (c) 2019 Chittaranjan Srinivas Swaminathan
 *   This file is part of Maps of Dynamics library (libmod).
 *
 *   ompl_mod_objectives is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ompl_mod_objectives is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with ompl_planners_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/mod/samplers/IntensityMapSampler.h>

#include <boost/log/trivial.hpp>
#include <boost/math/constants/constants.hpp>

namespace ompl {
namespace MoD {

IntensityMapSampler::IntensityMapSampler(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                                         const ::MoD::IntensityMap &qmap, double bias, bool debug)
    : ompl::base::InformedSampler(pdef, maxCalls), bias_(bias), debug_(debug) {
  this->numIters_ = maxCalls;
  setup(qmap);

  if (debug_) {
    sampledPosesFile_.open("/home/ksatyaki/samples.csv", std::fstream::out);
    if (sampledPosesFile_.is_open()) {
      OMPL_INFORM("Debug Enabled.");
      sampledPosesFile_ << "x,y,choice" << std::endl;
    } else {
      OMPL_INFORM("Couldn't open file for debug: %s", strerror(errno));
    }
  } else {
    OMPL_INFORM("Debug disabled.");
  }
}

IntensityMapSampler::IntensityMapSampler(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                                         const std::string &intensity_map_file_name, double bias, bool debug)
    : ompl::base::InformedSampler(pdef, maxCalls), bias_(bias), debug_(debug) {
  this->numIters_ = maxCalls;
  setup(::MoD::IntensityMap(intensity_map_file_name));

  if (debug_) {
    sampledPosesFile_.open("/home/ksatyaki/samples.csv", std::fstream::out);
    if (sampledPosesFile_.is_open()) {
      OMPL_INFORM("Debug Enabled.");
      sampledPosesFile_ << "x,y,choice" << std::endl;
    } else {
      OMPL_INFORM("Couldn't open file for debug: %s", strerror(errno));
    }
  } else {
    OMPL_INFORM("Debug disabled.");
  }
}

bool IntensityMapSampler::checkValidity(double xi, double yi) {
  ompl::base::State *first = probDefn_->getSpaceInformation()->allocState();

  (first->as<ompl::base::SE2StateSpace::StateType>())->setX(xi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setY(yi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setYaw(0.0);

  auto checker = probDefn_->getSpaceInformation()->getStateValidityChecker();
  bool valid = true;
  if (checker != nullptr) {
    valid = checker->isValid(first);
  } else {
    std::cout << "SHITE";
  }
  probDefn_->getSpaceInformation()->freeState(first);

  return valid;
}

void IntensityMapSampler::setup(const ::MoD::IntensityMap &intensity_map) {
  for (size_t i = 0; i < (intensity_map.getRows() * intensity_map.getColumns()); i++) {
    auto xy = intensity_map.getXYatIndex(i);
    if (this->checkValidity(xy[0], xy[1])) {
      q_map.emplace_back(xy[0], xy[1], 1.0 - intensity_map(xy[0], xy[1]));
      nonq_map.emplace_back(xy[0], xy[1], 1.0);
    }
  }
  BOOST_LOG_TRIVIAL(info) << "Intensity Map has " << q_map.size() << " locations.";

  // Sort it by value. Not necessary really.
  std::sort(q_map.begin(), q_map.end(), [](QMap a, QMap b) { return a.getValue() < b.getValue(); });

  // Compute weighted sum.
  this->value_sum = (std::accumulate(q_map.begin(), q_map.end(), QMap(0.0, 0.0, 0.0), [](QMap a, QMap b) {
                      return QMap(0.0, 0.0, a.getValue() + b.getValue());
                    })).getValue();

  // Don't forget to set the cell size...
  this->half_cell_size = intensity_map.getCellSize() / 2.0;
}

bool IntensityMapSampler::sampleUniform(ompl::base::State *state, const ompl::base::Cost &maxCost) {
  sampleNecessarilyValid(state);
  return true;
}

void IntensityMapSampler::sampleNecessarilyValid(ompl::base::State *state) {
  // Sample theta first. This is the easiest part.
  double theta = rng_.uniformReal(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());

  // Sample position next.
  auto sampled_value = rng_.uniformReal(0.0, this->value_sum);

  // Move to the appropriate value in the list ...
  double accum_sum = 0.0;
  size_t index = 0;
  size_t result_index = 0;

  double sampled_x, sampled_y;

  auto random_num = rng_.uniformReal(0.0, 1.0);
  bool uniform = false;
  if (random_num < bias_) {
    while (index < q_map.size()) {
      if (sampled_value < accum_sum) {
        // This is the right index!
        result_index = index;
        break;
      }
      accum_sum = accum_sum + q_map[index].getValue();
      index = index + 1;
    }
    sampled_x =
        rng_.uniformReal(q_map[result_index].getX() - half_cell_size, q_map[result_index].getX() + half_cell_size);
    sampled_y =
        rng_.uniformReal(q_map[result_index].getY() - half_cell_size, q_map[result_index].getY() + half_cell_size);
  } else {
    while (index < nonq_map.size()) {
      if (sampled_value < accum_sum) {
        // This is the right index!
        result_index = index;
        break;
      }
      accum_sum = accum_sum + nonq_map[index].getValue();
      index = index + 1;
    }
    sampled_x = rng_.uniformReal(nonq_map[result_index].getX() - half_cell_size,
                                 nonq_map[result_index].getX() + half_cell_size);
    sampled_y = rng_.uniformReal(nonq_map[result_index].getY() - half_cell_size,
                                 nonq_map[result_index].getY() + half_cell_size);
    uniform = true;
  }

  (state->as<ompl::base::SE2StateSpace::StateType>())->setX(sampled_x);
  (state->as<ompl::base::SE2StateSpace::StateType>())->setY(sampled_y);
  (state->as<ompl::base::SE2StateSpace::StateType>())->setYaw(theta);

  if (debug_) {
    sampledPosesFile_ << sampled_x << "," << sampled_y << "," << (uniform ? "uniform" : "intensity") << std::endl;
    sampledPosesFile_.flush();
  }
}

}  // namespace MoD
}  // namespace ompl