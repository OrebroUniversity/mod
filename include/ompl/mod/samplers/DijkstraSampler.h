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

#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/util/RandomNumbers.h>

#include <array>
#include <memory>
#include <mod/cliffmap.hpp>
#include <mod/grid_dijkstra.hpp>
#include <mod/parameters.hpp>
#include <mod/sample_sink.hpp>
#include <vector>

namespace ompl {
namespace MoD {

/**
 * Informed sampler biased towards the Dijkstra shortest path (under the planner's objective) on a grid over the
 * state bounds. With probability `bias` a cell of the path is drawn, the heading points to the next path cell
 * (previous cell for the last node) +/- pi/8; otherwise a uniform cell and heading. Positions are uniform within
 * the cell and clamped to the bounds.
 *
 * Setup: one validity check per node (yaw 0; the footprint is a circle), Dijkstra on the implicit 8-neighbour
 * grid with `motionCost(a, b)` as the edge weight, both yaws set to the edge heading.
 */
class DijkstraSampler : public ompl::base::InformedSampler {
 protected:
  std::unique_ptr<::MoD::GridDijkstra> grid_;

  double x_min_{0.0}, x_max_{0.0}, y_min_{0.0}, y_max_{0.0};

  std::array<double, 3> start_{0.0, 0.0, 0.0};
  std::array<double, 3> goal_{0.0, 0.0, 0.0};

  /// Node indices (row * cols + col) of the path from start to goal; empty if none was found.
  std::vector<size_t> path_;

  ompl::RNG rng_;

  /// Optional receiver of every draw (not owned).
  ::MoD::SampleSink *sink_{nullptr};

  double bias_{0.05};

  double cell_size_{0.5};

  /// Edge weight for the grid: `motionCost` between the two node positions with yaws along the edge.
  double edgeCost(size_t from, size_t to, ompl::base::State *a, ompl::base::State *b) const;

 public:
  DijkstraSampler(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                  const ::MoD::SamplerParameters &params, ::MoD::IntensityMapConstPtr intensity_map = nullptr);

  static ompl::base::InformedSamplerPtr allocate(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls,
                                                 const ::MoD::SamplerParameters &params,
                                                 ::MoD::IntensityMapConstPtr intensity_map = nullptr) {
    return std::make_shared<DijkstraSampler>(pdef, maxCalls, params, std::move(intensity_map));
  }

  ~DijkstraSampler() override = default;

  inline void setBias(double bias) { this->bias_ = bias; }
  inline double getBias() const { return bias_; }

  inline double colToX(size_t col) const { return grid_->colToX(col); }
  inline double rowToY(size_t row) const { return grid_->rowToY(row); }
  inline size_t rows() const { return grid_->rows(); }
  inline size_t cols() const { return grid_->cols(); }
  inline double cellSize() const { return cell_size_; }
  inline const std::vector<size_t> &path() const { return path_; }
  inline const ::MoD::GridDijkstra &grid() const { return *grid_; }

  void setup();

  bool sampleUniform(ompl::base::State *state, const ompl::base::Cost &maxCost) override;

  inline bool sampleUniform(ompl::base::State *state, const ompl::base::Cost & /*minCost*/,
                            const ompl::base::Cost &maxCost) override {
    return sampleUniform(state, maxCost);
  }

  inline void setSampleSink(::MoD::SampleSink *sink) { sink_ = sink; }

  inline bool hasInformedMeasure() const override { return false; }

  inline double getInformedMeasure(const ompl::base::Cost & /*currentCost*/) const override {
    return this->space_->getMeasure();
  }
};

}  // namespace MoD
}  // namespace ompl
