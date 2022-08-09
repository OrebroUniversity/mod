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

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/mod/samplers/DijkstraSampler.h>

#include <utility>

namespace ompl::MoD {
DijkstraSampler::DijkstraSampler(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls, double cell_size,
                                 double bias, bool debug)
    : ompl::base::InformedSampler(pdef, maxCalls), bias_(bias), debug_(debug) {
  this->props_.cell_size = cell_size;

  start_ = {(probDefn_->getStartState(0)->as<ompl::base::SE2StateSpace::StateType>())->getX(),
            (probDefn_->getStartState(0)->as<ompl::base::SE2StateSpace::StateType>())->getY(),
            (probDefn_->getStartState(0)->as<ompl::base::SE2StateSpace::StateType>())->getYaw()};

  ompl::base::State *goal_state = probDefn_->getGoal()->as<ompl::base::GoalState>()->getState();
  goal_ = {(goal_state->as<ompl::base::SE2StateSpace::StateType>())->getX(),
           (goal_state->as<ompl::base::SE2StateSpace::StateType>())->getY(),
           (goal_state->as<ompl::base::SE2StateSpace::StateType>())->getYaw()};

  setup();

  if (debug_) {
    sampledPosesFile_.open(
        "/home/ksatyaki/samples-dijkstra-" + pdef->getOptimizationObjective()->getDescription() + ".csv",
        std::fstream::out);
    if (sampledPosesFile_.is_open()) {
      OMPL_INFORM("Debug Enabled.");
      sampledPosesFile_ << "x,y,choice" << std::endl;
    } else {
      OMPL_INFORM("Couldn't open file for debug.");
    }
  } else {
    OMPL_INFORM("Debug disabled.");
  }
}

double DijkstraSampler::getCost(double xi, double yi, double xf, double yf) {
  ompl::base::State *first = probDefn_->getSpaceInformation()->allocState();
  ompl::base::State *next = probDefn_->getSpaceInformation()->allocState();

  (first->as<ompl::base::SE2StateSpace::StateType>())->setX(xi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setY(yi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setYaw(atan2(yf - yi, xf - xi));

  (next->as<ompl::base::SE2StateSpace::StateType>())->setX(xf);
  (next->as<ompl::base::SE2StateSpace::StateType>())->setY(yf);
  (next->as<ompl::base::SE2StateSpace::StateType>())->setYaw(atan2(yf - yi, xf - xi));

  auto cost = opt_->motionCost(first, next).value();

  probDefn_->getSpaceInformation()->freeState(first);
  probDefn_->getSpaceInformation()->freeState(next);

  return cost;
}

bool DijkstraSampler::checkValidity(double xi, double yi, double xf, double yf) {
  ompl::base::State *first = probDefn_->getSpaceInformation()->allocState();
  ompl::base::State *next = probDefn_->getSpaceInformation()->allocState();

  (first->as<ompl::base::SE2StateSpace::StateType>())->setX(xi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setY(yi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setYaw(atan2(yf - yi, xf - xi));

  (next->as<ompl::base::SE2StateSpace::StateType>())->setX(xf);
  (next->as<ompl::base::SE2StateSpace::StateType>())->setY(yf);
  (next->as<ompl::base::SE2StateSpace::StateType>())->setYaw(atan2(yf - yi, xf - xi));

  auto checker = probDefn_->getSpaceInformation()->getStateValidityChecker();
  bool valid = true;
  if (checker != nullptr) {
    valid = checker->isValid(first) && checker->isValid(next);
  } else {
    std::cout << "SHITE";
  }
  probDefn_->getSpaceInformation()->freeState(first);
  probDefn_->getSpaceInformation()->freeState(next);

  return valid;
}

bool DijkstraSampler::checkValidity(size_t row_i, size_t col_i, size_t row_f, size_t col_f) {
  return checkValidity(colToX(col_i), rowToY(row_i), colToX(col_f), rowToY(row_f));
}

double DijkstraSampler::getCost(size_t row_i, size_t col_i, size_t row_f, size_t col_f) {
  return getCost(colToX(col_i), rowToY(row_i), colToX(col_f), rowToY(row_f));
}

double DijkstraSampler::distance(size_t row_i, size_t col_i, size_t row_f, size_t col_f) {
  double xi = colToX(col_i);
  double yi = rowToY(row_i);
  double xf = colToX(col_f);
  double yf = rowToY(row_f);

  return sqrt((xi - xf) * (xi - xf)) + ((yi - yf) * (yi - yf));
}

void DijkstraSampler::addEdgeAndWeight(size_t row_i, size_t col_i, size_t row_f, size_t col_f) {
  if (!checkValidity(row_i, col_i, row_f, col_f)) return;

  edges_.emplace_back(row_i * this->props_.cols + col_i, row_f * this->props_.cols + col_f);
  weights_.push_back(getCost(row_i, col_i, row_f, col_f));
  // weights_.push_back(distance(row_i, col_i, row_f, col_f));
}

void DijkstraSampler::setup() {
  const ompl::base::RealVectorBounds state_bounds =
      probDefn_->getSpaceInformation()->getStateSpace()->as<ompl::base::SE2StateSpace>()->getBounds();
  const auto x_min = state_bounds.low[0];
  const auto x_max = state_bounds.high[0];
  const auto y_min = state_bounds.low[1];
  const auto y_max = state_bounds.high[1];

  auto cols = static_cast<size_t>((x_max - x_min) / this->props_.cell_size) + 1u;
  auto rows = static_cast<size_t>((y_max - y_min) / this->props_.cell_size) + 1u;
  auto num_nodes = rows * cols;

  // Total edges are eight per cell - 5 missing per corner - 3 missing along the edges of the map.
  size_t total_edges = (rows * cols * 8) - 20 - (3 * 2 * (cols - 2)) - (3 * 2 * (rows - 2));

  this->props_ = props(this->props_.cell_size, x_min, x_max, y_min, y_max, rows, cols, total_edges);
  BOOST_LOG_TRIVIAL(info) << "Properties: " << std::endl
                          << "Bias: " << this->bias_ << std::endl
                          << "Cell size: " << this->props_.cell_size;

  for (size_t row = 0; row < rows; row++) {
    for (size_t col = 0; col < cols; col++) {
      if (row == 0 and col == 0) {
        addEdgeAndWeight(row, col, (row + 0), (col + 1));
        addEdgeAndWeight(row, col, (row + 1), (col + 0));
        addEdgeAndWeight(row, col, (row + 1), (col + 1));
      } else if (row == 0 and col == cols - 1) {
        addEdgeAndWeight(row, col, (row + 0), (col - 1));
        addEdgeAndWeight(row, col, (row + 1), (col - 1));
        addEdgeAndWeight(row, col, (row + 1), (col + 0));
      } else if (row == rows - 1 and col == 0) {
        addEdgeAndWeight(row, col, (row - 1), (col - 0));
        addEdgeAndWeight(row, col, (row - 1), (col + 1));
        addEdgeAndWeight(row, col, (row + 0), (col + 1));
      } else if (row == rows - 1 and col == cols - 1) {
        addEdgeAndWeight(row, col, (row - 1), (col - 1));
        addEdgeAndWeight(row, col, (row - 1), (col - 0));
        addEdgeAndWeight(row, col, (row + 0), (col - 1));
      } else if (row == 0) {
        addEdgeAndWeight(row, col, (row + 0), (col - 1));
        addEdgeAndWeight(row, col, (row + 0), (col + 1));
        addEdgeAndWeight(row, col, (row + 1), (col - 1));
        addEdgeAndWeight(row, col, (row + 1), (col + 0));
        addEdgeAndWeight(row, col, (row + 1), (col + 1));
      } else if (col == 0) {
        addEdgeAndWeight(row, col, (row - 1), (col - 0));
        addEdgeAndWeight(row, col, (row - 1), (col + 1));
        addEdgeAndWeight(row, col, (row + 0), (col + 1));
        addEdgeAndWeight(row, col, (row + 1), (col + 0));
        addEdgeAndWeight(row, col, (row + 1), (col + 1));
      } else if (row == rows - 1) {
        addEdgeAndWeight(row, col, (row - 1), (col - 1));
        addEdgeAndWeight(row, col, (row - 1), (col - 0));
        addEdgeAndWeight(row, col, (row - 1), (col + 1));
        addEdgeAndWeight(row, col, (row + 0), (col - 1));
        addEdgeAndWeight(row, col, (row + 0), (col + 1));
      } else if (col == cols - 1) {
        addEdgeAndWeight(row, col, (row - 1), (col - 1));
        addEdgeAndWeight(row, col, (row - 1), (col - 0));
        addEdgeAndWeight(row, col, (row + 0), (col - 1));
        addEdgeAndWeight(row, col, (row + 1), (col - 1));
        addEdgeAndWeight(row, col, (row + 1), (col + 0));
      } else {
        addEdgeAndWeight(row, col, (row - 1), (col - 1));
        addEdgeAndWeight(row, col, (row - 1), (col - 0));
        addEdgeAndWeight(row, col, (row - 1), (col + 1));
        addEdgeAndWeight(row, col, (row + 0), (col - 1));
        addEdgeAndWeight(row, col, (row + 0), (col + 1));
        addEdgeAndWeight(row, col, (row + 1), (col - 1));
        addEdgeAndWeight(row, col, (row + 1), (col + 0));
        addEdgeAndWeight(row, col, (row + 1), (col + 1));
      }
    }
  }

  if (edges_.size() != this->props_.total_edges) {
    BOOST_LOG_TRIVIAL(info) << "Surely, the number of edges has reduced due to invalid ones not being added. We added: "
                            << edges_.size() << " edges and " << weights_.size() << " weights, but would have added "
                            << total_edges << " if we considered the bad apples.";
  }

  SamplingGraph graph_(edges_.begin(), edges_.end(), weights_.begin(), num_nodes);

  std::vector<SamplingGraphVertexDescriptor> p(boost::num_vertices(graph_));
  std::vector<size_t> d(boost::num_vertices(graph_));

  BOOST_LOG_TRIVIAL(info) << "Vertices in the graph are: " << boost::num_vertices(graph_);
  BOOST_LOG_TRIVIAL(info) << "Total vertices would have been " << rows * cols;
  BOOST_LOG_TRIVIAL(info) << "Rows = " << rows << ", Cols = " << cols;

  // Find out where the start and goal are in terms of row, col.
  auto start_row = static_cast<size_t>((this->start_[1] - y_min) / this->props_.cell_size);
  auto start_col = static_cast<size_t>((this->start_[0] - x_min) / this->props_.cell_size);
  auto goal_row = static_cast<size_t>((this->goal_[1] - y_min) / this->props_.cell_size);
  auto goal_col = static_cast<size_t>((this->goal_[0] - x_min) / this->props_.cell_size);

  BOOST_LOG_TRIVIAL(info) << "Start: (" << start_row << ", " << start_col << ") = (" << this->start_[0] << ", "
                          << this->start_[1] << ")";
  BOOST_LOG_TRIVIAL(info) << "Goal: (" << goal_row << ", " << goal_col << ") = (" << this->goal_[0] << ", "
                          << this->goal_[1] << ")";
  SamplingGraphVertexDescriptor sVertexDescriptor = boost::vertex(start_row * this->props_.cols + start_col, graph_);
  SamplingGraphVertexDescriptor gVertexDescriptor = boost::vertex(goal_row * this->props_.cols + goal_col, graph_);

  boost::dijkstra_shortest_paths(
      graph_, sVertexDescriptor,
      boost::predecessor_map(boost::make_iterator_property_map(p.begin(), boost::get(boost::vertex_index, graph_)))
          .distance_map(boost::make_iterator_property_map(d.begin(), boost::get(boost::vertex_index, graph_))));

  BOOST_LOG_TRIVIAL(info) << "Ran Dijkstra...";
  path_.clear();
  SamplingGraphVertexDescriptor current = gVertexDescriptor;
  SamplingGraphVertexDescriptor prev;
  while (current != sVertexDescriptor) {
    prev = current;
    path_.emplace_front(current);
    current = p[current];
    if (prev == current) {
      BOOST_LOG_TRIVIAL(error) << "Dijkstra Sampler failed to find a path!";
      return;
    }
  }
  path_.emplace_front(sVertexDescriptor);

  double x_prev{0.0};
  double y_prev{0.0};
  SamplingGraphVertexDescriptorIterator it;
  double cost = 0.0;
  for (it = path_.begin(); it != path_.end(); ++it) {
    size_t col = (*it) % this->props_.cols;
    size_t row = static_cast<size_t>((*it) / this->props_.cols);

    double x_this = colToX(col);
    double y_this = rowToY(row);

    if (it != path_.begin()) cost += getCost(x_prev, y_prev, x_this, y_this);

    x_prev = x_this;
    y_prev = y_this;
  }
  BOOST_LOG_TRIVIAL(info) << "Found a path: " << path_.size() << " nodes... "
                          << "Cost: " << cost;
}

bool DijkstraSampler::sampleUniform(ompl::base::State *state, const ompl::base::Cost &cost) {
  size_t sampled_col = 0;
  size_t sampled_row = 0;
  double sampled_theta;

  double randomValue = rng_.uniformReal(0.0, 1.0);
  bool uniform = false;
  // At a bias_ % probability, choose a row, col from the dijkstra path
  if (randomValue < bias_) {
    uniform = false;
    auto idx = rng_.uniformInt(0, this->path_.size() - 1);
    auto iter = path_.begin();
    std::advance(iter, idx);
    sampled_col = (*iter) % this->props_.cols;
    sampled_row = static_cast<size_t>((*iter) / this->props_.cols);

    if (idx == this->path_.size() - 1) {
      auto prev_iter = path_.begin();
      std::advance(iter, idx - 1);
      size_t prev_col = (*prev_iter) % this->props_.cols;
      size_t prev_row = static_cast<size_t>((*prev_iter) / this->props_.cols);
      sampled_theta = atan2(rowToY(sampled_row) - rowToY(prev_row), colToX(sampled_col) - colToX(prev_col));
    } else {
      auto next_iter = path_.begin();
      std::advance(iter, idx + 1);
      size_t next_col = (*next_iter) % this->props_.cols;
      size_t next_row = static_cast<size_t>((*next_iter) / this->props_.cols);
      sampled_theta = atan2(-rowToY(sampled_row) + rowToY(next_row), -colToX(sampled_col) + colToX(next_col));
    }

    sampled_theta = rng_.uniformReal(sampled_theta - (boost::math::constants::pi<double>() / 8.0),
                                     sampled_theta + (boost::math::constants::pi<double>() / 8.0));

  } else {
    uniform = true;
    sampled_col = rng_.uniformInt(0, this->props_.cols - 1);
    sampled_row = rng_.uniformInt(0, this->props_.rows - 1);
    sampled_theta = rng_.uniformReal(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());
  }

  double sampled_x = rng_.uniformReal(colToX(sampled_col) - this->props_.cell_size / 2.0,
                                      colToX(sampled_col) + this->props_.cell_size / 2.0);
  double sampled_y = rng_.uniformReal(rowToY(sampled_row) - this->props_.cell_size / 2.0,
                                      rowToY(sampled_row) + this->props_.cell_size / 2.0);

  (state->as<ompl::base::SE2StateSpace::StateType>())->setX(sampled_x);
  (state->as<ompl::base::SE2StateSpace::StateType>())->setY(sampled_y);
  (state->as<ompl::base::SE2StateSpace::StateType>())->setYaw(sampled_theta);

  if (debug_) {
    sampledPosesFile_ << sampled_x << "," << sampled_y << "," << (uniform ? "uniform" : "intensity") << std::endl;
    sampledPosesFile_.flush();
  }
  return true;
}

}  // namespace ompl::MoD
