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

#include <Eigen/Dense>
#include <cmath>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <mod/cliffmap.hpp>

#include <iostream>

#include <boost/log/trivial.hpp>

namespace MoD {

IntensityMap::IntensityMap(const IntensityMap &intensityMap) {
  this->rows_ = intensityMap.rows_;
  this->columns_ = intensityMap.columns_;
  this->x_max_ = intensityMap.x_max_;
  this->y_max_ = intensityMap.y_max_;
  this->x_min_ = intensityMap.x_min_;
  this->y_min_ = intensityMap.y_min_;
  this->cell_size_ = intensityMap.cell_size_;
  this->values_ = intensityMap.values_;
}

void IntensityMap::readFromXML(const std::string &fileName) {
  using boost::property_tree::ptree;
  ptree pTree;

  boost::property_tree::read_xml(fileName, pTree);

  this->x_min_ = pTree.get<double>("map.parameters.x_min");
  this->y_min_ = pTree.get<double>("map.parameters.y_min");
  this->x_max_ = pTree.get<double>("map.parameters.x_max");
  this->y_max_ = pTree.get<double>("map.parameters.y_max");
  this->cell_size_ = pTree.get<double>("map.parameters.cell_size");
  this->rows_ = size_t((this->y_max_ - this->y_min_) / this->cell_size_) + 1;
  this->columns_ = size_t((this->x_max_ - this->x_min_) / this->cell_size_) + 1;

  this->values_.resize(this->rows_ * this->columns_);

  for (const auto &cell : pTree.get_child("map.cells")) {
    if (cell.second.get<size_t>("row") * this->columns_ +
            cell.second.get<size_t>("col") <
        this->rows_ * this->columns_) {
      this->values_[cell.second.get<size_t>("row") * this->columns_ +
                    cell.second.get<size_t>("col")] =
          cell.second.get<double>("value");
    }
  }
}

void CLiFFMap::readFromXML(const std::string &fileName) {
  using boost::property_tree::ptree;
  ptree pTree;

  boost::property_tree::read_xml(fileName, pTree);

  // Read top-most tag
  for (const auto &value : pTree.get_child("map")) {
    if (value.first == "parameters") {
      this->x_min_ = value.second.get<double>("x_min");
      this->y_min_ = value.second.get<double>("y_min");
      this->x_max_ = value.second.get<double>("x_max");
      this->y_max_ = value.second.get<double>("y_max");
      this->radius_ = value.second.get<double>("radious");
      this->resolution_ = value.second.get<double>("step");
    }
  }
  for (const auto &vLocation : pTree.get_child("map.locations")) {
    CLiFFMapLocation location;
    location.id = vLocation.second.get<size_t>("id");

    for (const auto &vLocProperty : vLocation.second.get_child("")) {
      if (vLocProperty.first == "p")
        try {
          location.p = vLocation.second.get<double>("p");
        } catch (std::exception &ex) {
          location.p = 1.0;
        }

      if (vLocProperty.first == "q")
        try {
          location.q = vLocation.second.get<double>("q");
        } catch (std::exception &ex) {
          location.q = 1.0;
        }

      if (vLocProperty.first == "pose") {
        location.position[0] = vLocProperty.second.get<double>("x");
        location.position[1] = vLocProperty.second.get<double>("y");
      }

      if (vLocProperty.first == "distribution") {
        CLiFFMapDistribution dist;
        dist.mixing_factor = vLocProperty.second.get<double>("P");
        for (const auto &vDistribution : vLocProperty.second.get_child("")) {
          if (vDistribution.first == "M") {
            dist.mean[0] = vDistribution.second.get<double>("th");
            dist.mean[1] = vDistribution.second.get<double>("r");
          }

          if (vDistribution.first == "Cov") {
            dist.covariance[0] = vDistribution.second.get<double>("e_11");
            dist.covariance[1] = vDistribution.second.get<double>("e_12");
            dist.covariance[2] = vDistribution.second.get<double>("e_21");
            dist.covariance[3] = vDistribution.second.get<double>("e_22");
          }
        }
        location.distributions.push_back(dist);
      }
    }
    this->locations_.push_back(location);
  }
  BOOST_LOG_TRIVIAL(info) << "Read a cliffmap from XML" << std::endl;

  // Frame ID:
  BOOST_LOG_TRIVIAL(info) << "Frame ID for cliffmap is: " << frame_id_;
}

CLiFFMapLocation CLiFFMap::at(size_t row, size_t col) const {
  if (row >= rows_ || col >= columns_) {
    return CLiFFMapLocation();
  }

  return locations_[row * columns_ + col];
}

CLiFFMapLocation CLiFFMap::atId(size_t id) const {
  return locations_[id - (size_t)1];
}

CLiFFMapLocation CLiFFMap::operator()(double x, double y) const {
  size_t row = y2index(y);
  size_t col = x2index(x);
  return this->at(row, col);
}

double CLiFFMap::getLikelihood(double x, double y, double heading,
                               double speed) const {
  CLiFFMapLocation loc = (*this)(x, y);
  Eigen::Vector2d V;
  V[0] = heading;
  V[1] = speed;

  double likelihood = 0.0;

  for (const auto &dist : loc.distributions) {
    Eigen::Matrix2d Sigma;
    std::array<double, 4> sigma_array = dist.getCovariance();
    Sigma(0, 0) = sigma_array[0];
    Sigma(0, 1) = sigma_array[1];
    Sigma(1, 0) = sigma_array[2];
    Sigma(1, 1) = sigma_array[3];

    Eigen::Vector2d myu;
    myu[0] = atan2(sin(dist.getMeanHeading()), cos(dist.getMeanHeading()));
    myu[1] = dist.getMeanSpeed();

    double mahalanobis_sq = (V - myu).transpose() * Sigma.inverse() * (V - myu);

    likelihood += (1 / (2 * M_PI)) * (1 / sqrt(Sigma.determinant())) *
                  exp(-0.5 * mahalanobis_sq) * dist.getMixingFactor();
  }
  return likelihood;
}

double CLiFFMap::getBestHeading(double x, double y) const {
  CLiFFMapLocation loc = (*this)(x, y);

  double best_likelihood = 0.0;
  double best_heading = 0.0;
  for (const auto &dist : loc.distributions) {
    Eigen::Matrix2d Sigma;
    std::array<double, 4> sigma_array = dist.getCovariance();
    Sigma(0, 0) = sigma_array[0];
    Sigma(0, 1) = sigma_array[1];
    Sigma(1, 0) = sigma_array[2];
    Sigma(1, 1) = sigma_array[3];

    Eigen::Vector2d myu;
    myu[0] = atan2(sin(dist.getMeanHeading()), cos(dist.getMeanHeading()));
    myu[1] = dist.getMeanSpeed();

    double likelihood = (1 / (2 * M_PI)) * (1 / sqrt(Sigma.determinant())) *
                        dist.getMixingFactor();
    if (likelihood > best_likelihood) {
      best_likelihood = likelihood;
      best_heading = myu[0];
    }
  }
  return best_heading;
}

void CLiFFMap::organizeAsGrid() {
  std::vector<CLiFFMapLocation> organizedLocations;

  columns_ = round((x_max_ - x_min_) / resolution_) + 1;
  rows_ = round((y_max_ - y_min_) / resolution_) + 1;

  organizedLocations.resize(rows_ * columns_);

  if (organizedLocations.size() != locations_.size()) {
    BOOST_LOG_TRIVIAL(warning)
        << "[CLiFFMap] Error in number of locations. We thought it was "
        << organizedLocations.size() << ", but it was " << locations_.size()
        << ".";
    if (organizedLocations.size() < locations_.size())
      return;
    BOOST_LOG_TRIVIAL(warning)
        << "Less is more. We have the space. Let's continue... ";
  }

  for (const CLiFFMapLocation &location : locations_) {
    size_t r = y2index(location.position[1]);
    size_t c = x2index(location.position[0]);

    size_t idx = r * columns_ + c;
    if (idx < organizedLocations.size()) {
      organizedLocations[idx].distributions = location.distributions;
      organizedLocations[idx].id = location.id;
      organizedLocations[idx].p = location.p;
      organizedLocations[idx].q = location.q;
      organizedLocations[idx].position = location.position;
    } else {
      BOOST_LOG_TRIVIAL(info) << 1,
          "Some new locations were added while organizing...";
      organizedLocations.push_back(location);
    }
  }
  locations_ = organizedLocations;
  organized_ = true;

  BOOST_LOG_TRIVIAL(info) << "[CLiFFMap] Organized a cliffmap with resolution: "
                          << getResolution() << " m/cell.";
}

} // namespace MoD

std::ostream &operator<<(std::ostream &out,
                         const MoD::CLiFFMapDistribution &dist) {
  out << "Mixing Factor: " << dist.mixing_factor << "\t";
  out << "Mean: [" << dist.mean[0] << "," << dist.mean[1] << "]" << std::endl;
  return out;
}

std::ostream &operator<<(std::ostream &out, const MoD::CLiFFMapLocation &loc) {
  out << "Position: [" << loc.position[0] << ", " << loc.position[1] << "]\n";
  for (const auto &dist : loc.distributions)
    out << "Distribution: " << dist;
  return out;
}

std::ostream &operator<<(std::ostream &out, const MoD::CLiFFMap &map) {
  out << "XMin: " << map.getXMin() << "\n"
      << "XMax: " << map.getXMax() << "\n"
      << "YMin: " << map.getYMin() << "\n"
      << "YMax: " << map.getYMax() << "\n";

  for (const auto &loc : map.getLocations()) {
    out << "Location: " << loc;
  }
  return out;
}
