/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of mod.
 *
 *   mod is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   mod is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with mod.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <array>
#include <cmath>
#include <vector>

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>

namespace MoD {

class IntensityMap {
  double x_max_;
  double y_max_;
  double x_min_;
  double y_min_;
  size_t rows_;
  size_t columns_;
  double cell_size_;
  std::vector<double> values_;

  void readFromXML(const std::string &fileName);

public:
  inline IntensityMap() = default;
  inline explicit IntensityMap(const std::string &fileName) {
    this->readFromXML(fileName);
  }

  inline double getXMax() const { return x_max_; }
  inline double getYMax() const { return y_max_; }
  inline double getXMin() const { return x_min_; }
  inline double getYMin() const { return y_min_; }
  inline size_t getRows() const { return rows_; }
  inline size_t getColumns() const { return columns_; }
  inline double getCellSize() const { return cell_size_; }

  IntensityMap(const IntensityMap &intensityMap);
  virtual ~IntensityMap() = default;

  inline double operator()(double x, double y) const {
    size_t row = size_t(std::floor(y - this->y_min_) / this->cell_size_);
    size_t col = size_t(std::floor(x - this->x_min_) / this->cell_size_);
    return this->values_[row * this->columns_ + col];
  }

  inline std::array<double, 2> getXYatIndex(size_t index) const {
    size_t row = index % this->columns_;
    size_t col = index - (row * this->columns_);

    return {(((double)col * this->cell_size_) + this->x_min_),
            (((double)row * this->cell_size_) + this->y_min_)};
  }
};

/**
 * A Single semi-wrapped Gaussian distribution.
 *
 */
struct CLiFFMapDistribution {
  double mixing_factor;
  std::array<double, 2> mean;
  std::array<double, 4> covariance;

  /**
   * \brief Returns the mixing factor for this distribution component.
   *
   * @return
   */
  inline double getMixingFactor() const { return mixing_factor; }

  /**
   * \brief Returns the mean for this distribution component.
   *
   * @return
   */
  inline std::array<double, 2> getMean() const { return mean; }

  /**
   * \brief Returns the covariance for this distribution component.
   *
   * @return
   */
  inline std::array<double, 4> getCovariance() const { return covariance; }

  /**
   * \brief Returns the mean heading for this distribution component.
   *
   * @return
   */
  inline double getMeanHeading() const { return mean[0]; }

  /**
   * \brief Returns the mean speed for this distribution component.
   *
   * @return
   */
  inline double getMeanSpeed() const { return mean[1]; }
};

/**
 * A location in the cliffmap. Might contain multiple distributions.
 *
 */
struct CLiFFMapLocation {
  size_t id;
  std::array<double, 2> position;
  double p;
  double q;
  std::vector<CLiFFMapDistribution> distributions;
};

class CLiFFMap {
public:
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double radius_;

  // Used only if we want to interpret this cliffmap as a grid.
  double resolution_;

  // (rows_ - 1) * resolution_ = y_max_ - y_min_
  double rows_;
  // (columns_ - 1) * resolution_ = x_max_ - x_min_
  double columns_;
  bool organized_{false};

  std::string frame_id_{std::string("default")};

  inline void setFrameID(const std::string &frame_id) { frame_id_ = frame_id; }

  inline std::string getFrameID() const { return frame_id_; }

  std::vector<CLiFFMapLocation> locations_;

  inline double index2x(size_t col) const {
    return (((double)col * resolution_) + x_min_);
  }
  inline double index2y(size_t row) const {
    return (((double)row * resolution_) + y_min_);
  }
  inline size_t x2index(double x) const {
    return std::round((x - x_min_) / resolution_);
  }
  inline size_t y2index(double y) const {
    return std::round((y - y_min_) / resolution_);
  }

  CLiFFMap() = default;

  inline CLiFFMap(const std::string &fileName, bool organize = false) {
    readFromXML(fileName);
    if (organize)
      organizeAsGrid();
  }
  /**
   * Calling this function makes the locations accessible as a grid.
   * Hence locations is modified such that it can be accessed as a row major
   * grid.
   * Use appropriate functions to query the grid.
   */
  void organizeAsGrid();

  /**
   * Get the CLiFFMapLocation at (row,col). Need to call organizeAsGrid() first.
   */
  CLiFFMapLocation at(size_t row, size_t col) const;
  CLiFFMapLocation atId(size_t id) const;
  CLiFFMapLocation operator()(double x, double y) const;
  double getLikelihood(double x, double y, double heading, double speed) const;
  double getBestHeading(double x, double y) const;

  void readFromXML(const std::string &fileName);

  inline bool isOrganized() const { return organized_; }

  inline double getXMin() const { return x_min_; }
  inline double getYMin() const { return y_min_; }
  inline double getXMax() const { return x_max_; }
  inline double getYMax() const { return y_max_; }
  inline double getRadius() const { return radius_; }
  inline double getResolution() const { return resolution_; }
  inline const std::vector<CLiFFMapLocation> &getLocations() const {
    return locations_;
  }
};

typedef std::shared_ptr<CLiFFMap> CLiFFMapPtr;
typedef std::shared_ptr<const CLiFFMap> CLiFFMapConstPtr;

} // namespace MoD

std::ostream &operator<<(std::ostream &, const MoD::CLiFFMap &);
std::ostream &operator<<(std::ostream &, const MoD::CLiFFMapLocation &);
std::ostream &operator<<(std::ostream &, const MoD::CLiFFMapDistribution &);
