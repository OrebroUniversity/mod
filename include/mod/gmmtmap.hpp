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
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with mod.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <Eigen/Core>
#include <array>
#include <boost/chrono.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/log/trivial.hpp>
#include <mod/base.hpp>
#include <vector>

namespace MoD {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::d2::point_xy<double> Point2D;
typedef bg::model::box<Point2D> Box;
typedef std::pair<Point2D, std::array<size_t, 2>> TreeValue;

struct GMMTMapCluster {
  /// Mixing factor
  double mixing_factor;

  /// Cluster means
  std::vector<std::array<double, 2>> mean;

  /// Approximate heading computed using an average.
  std::vector<double> heading;

  /// Default constructor.
  inline GMMTMapCluster() = default;

  /**
   * \brief Constructor of a cluster.
   * \param pi Mixing factor.
   * \param mean A vector of 2D means for the @K_ Gaussians.
   * \param heading A vector of approximate headings.
   */
  inline GMMTMapCluster(double pi, const std::vector<std::array<double, 2>> &mean, std::vector<double> heading) {
    this->mixing_factor = pi;
    this->mean = mean;
    this->heading = heading;
  }
};

class GMMTMap : public Base {
 public:
  /**
   * \brief Constructor that reads a GMMTMap from an xml file.
   * \param fileName Filename of XML file.
   */
  explicit GMMTMap(const std::string &fileName) { readFromXML(fileName); }

  /**
   * \brief Read a GMMTMap from an xml file.
   * \param fileName Filename of XML file.
   */
  void readFromXML(const std::string &fileName);

  /**
   * \brief Prepares the Boost RTree and computes all headings as well.
   */
  void computeHeadingAndConstructRTree();

  /**
   * \brief Computes closest points (within a radius of @stddev_) from each
   * motion pattern.
   * \param x The x coordinate of the position for which the
   * query needs to be done.
   * \param y The y coordinate of the position for which
   * the query needs to be done.
   * \return Closest points with their cluster_id
   * and 'k'.
   */
  std::vector<TreeValue> getNearestNeighbors(double x, double y) const;

  inline std::vector<TreeValue> operator()(double x, double y) const { return this->getNearestNeighbors(x, y); };

  /**
   * \brief Get the number of motion patterns in this GMMT-map.
   * \return The number of motion patterns.
   */
  inline int getM() const { return M_; }

  /**
   * \brief Get the number of Gaussians in each motion pattern.
   * \return The number of Gaussians.
   */
  inline int getK() const { return K_; }

  /**
   * \brief Get the standard deviation of each Gaussian.
   * \return The standard deviation.
   */
  inline double getStdDev() const { return stddev_; }

  /**
   * \brief Get the mixing factor for a certain cluster index.
   * @param cluster_id The index of the cluster.
   * @return The mixing factor for that cluster.
   */
  inline double getMixingFactorByClusterID(size_t cluster_idx) {
    if (cluster_idx >= this->clusters_.size()) {
      BOOST_LOG_TRIVIAL(error) << "getMixingFactorByClusterID() called with "
                                  "cluster_id >= number of clusters.";
      return 1.0;
    }

    return this->clusters_[cluster_idx].mixing_factor;
  }

  inline double getHeadingAtDist(size_t cluster_idx, size_t mean_idx) {
    if (cluster_idx >= this->clusters_.size()) {
      BOOST_LOG_TRIVIAL(error) << "getHeadingAtDist() called with cluster_idx "
                                  ">= number of clusters.";
      BOOST_LOG_TRIVIAL(error) << "Total clusters: " << this->clusters_.size() << ", Cluster ID: " << cluster_idx;
      return 0.0;
    }

    if (mean_idx >= this->clusters_[cluster_idx].heading.size()) {
      BOOST_LOG_TRIVIAL(error) << "getHeadingAtDist() called with mean_idx >= "
                                  "number of traj-means in cluster.";
      BOOST_LOG_TRIVIAL(error) << "Total means: " << this->clusters_[cluster_idx].heading.size()
                               << ", Cluster ID and Mean ID: " << cluster_idx << ", " << mean_idx;
      return 0.0;
    }

    return this->clusters_[cluster_idx].heading[mean_idx];
  }

 protected:
  /// The number of motion patterns in the GMM Trajectory Map.
  int M_;

  /// The number of Gaussians per motion pattern in the GMM Trajectory Map.
  int K_;

  /// The standard deviation used in the GMM Trajectory Map. The 2D Gaussians in
  /// each motion pattern are circular.
  double stddev_;

  /// A vector containing all the clusters (motion patterns).
  std::vector<GMMTMapCluster> clusters_;

  /// A tree used for
  bgi::rtree<TreeValue, bgi::quadratic<16>> rtree_;
};

typedef std::shared_ptr<GMMTMap> GMMTMapPtr;
typedef std::shared_ptr<const GMMTMap> GMMTMapConstPtr;

}  // namespace MoD