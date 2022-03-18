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

#include <boost/log/trivial.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <mod/gmmtmap.hpp>

namespace mod {

std::vector<TreeValue> GMMTMap::getNearestNeighbors(double x, double y) const {
  std::vector<TreeValue> returned;
  Point2D query_pt(x, y);
  Box query_box(Point2D(query_pt.get<0>() - this->stddev_,
                        query_pt.get<1>() - this->stddev_),
                Point2D(query_pt.get<0>() + this->stddev_,
                        query_pt.get<1>() + this->stddev_));

  rtree_.query(bgi::within(query_box) && bgi::satisfies([&](TreeValue V) {
                 return bg::distance(V.first, query_pt) < this->stddev_;
               }),
               std::back_inserter(returned));

  std::sort(returned.begin(), returned.end(), [&](TreeValue a, TreeValue b) {
    return bg::distance(a.first, query_pt) < bg::distance(b.first, query_pt);
  });

  std::vector<int> mps;
  for (auto value = returned.begin(); value != returned.end();) {
    if (std::find(mps.begin(), mps.end(), value->second[0]) == mps.end()) {
      mps.push_back(value->second[0]);
      ++value;
    } else {
      value = returned.erase(value);
    }
  }

  return returned;
}

void GMMTMap::computeHeadingAndConstructRTree() {
  for (size_t index = 0; index < this->clusters_.size(); index++) {
    auto cluster = this->clusters_[index];

    for (size_t i = 0; i < cluster.mean.size(); i++) {

      const auto &point = cluster.mean[i];
      double heading;

      if (i == 0) {
        heading = atan2(cluster.mean[i + 1][1] - cluster.mean[i][1],
                        cluster.mean[i + 1][0] - cluster.mean[i][0]);
      } else if (i + 1 == this->clusters_.size()) {
        heading = atan2(cluster.mean[i][1] - cluster.mean[i - 1][1],
                        cluster.mean[i][0] - cluster.mean[i - 1][0]);
      } else {
        heading = atan2(cluster.mean[i + 1][1] - cluster.mean[i - 1][1],
                        cluster.mean[i + 1][0] - cluster.mean[i - 1][0]);
      }

      cluster.heading.push_back(heading);
      this->rtree_.insert(std::make_pair(Point2D(point[0], point[1]),
                                         std::array<size_t, 2>({index, i})));
    }
  }
}

void GMMTMap::readFromXML(const std::string &fileName) {
  using boost::property_tree::ptree;
  ptree pTree;

  boost::property_tree::read_xml(fileName, pTree);

  this->K_ = pTree.get<int>("map.parameters.K");
  this->M_ = pTree.get<int>("map.parameters.M");
  this->stddev_ = pTree.get<double>("map.parameters.stddev");

  for (const auto &vCluster : pTree.get_child("map.clusters")) {
    GMMTMapCluster cluster;
    cluster.mixing_factor = vCluster.second.get<double>("pi");
    for (const auto &vPoint : vCluster.second.get_child("mean")) {
      cluster.mean.push_back(
          {vPoint.second.get<double>("x"), vPoint.second.get<double>("y")});
    }
    this->clusters_.push_back(cluster);
  }

  this->computeHeadingAndConstructRTree();
  BOOST_LOG_TRIVIAL(info)
      << "Read a GMMT-map with %d clusters each containing %d gaussians",
      this->M_, this->K_;
}
} // namespace mod