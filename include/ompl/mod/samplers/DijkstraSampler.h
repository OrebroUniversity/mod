#pragma once

#include <boost/log/trivial.hpp>
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/property_map/property_map.hpp>

#include <memory>
#include <vector>

#include <mod/gmmtmap.hpp>
#include <mod/cliffmap.hpp>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/util/RandomNumbers.h>

namespace ompl {
namespace MoD {

class DijkstraSampler : public ompl::base::ValidStateSampler {

  typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS,
                                boost::no_property,
                                boost::property<boost::edge_weight_t, double>>
      graph_t;
  typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
  typedef boost::graph_traits<graph_t>::edge_descriptor edge_descriptor;
  typedef std::pair<int, int> Edge;

 protected:
  std::shared_ptr<::MoD::Base> mod_ptr_;

 public:
  DijkstraSampler(const ompl::base::SpaceInformation *si,
                  const std::string &mod_map_file_name,
                  const std::string &mod_type);

  static ompl::base::ValidStateSamplerPtr allocate(const ompl::base::SpaceInformation *si,
                                                   const std::string &mod_map_file_name,
                                                   const std::string &mod_type) {
    return std::make_shared<DijkstraSampler>(si, mod_map_file_name, mod_type);
  }

  void setup();

  bool sample(ompl::base::State *state) override;

  inline bool sampleNear(ompl::base::State *state, const ompl::base::State *near, double distance) override {
    return false;
  }
};

} // namespace MoD
} // namespace ompl