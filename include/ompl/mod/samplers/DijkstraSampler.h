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
#include <ompl/mod/objectives/MoDOptimizationObjective.h>
#include <ompl/util/RandomNumbers.h>

namespace ompl {
namespace MoD {

class DijkstraSampler : public ompl::base::ValidStateSampler {

  typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, boost::no_property,
                                boost::property<boost::edge_weight_t, double>>
      SamplingGraph;
  typedef boost::graph_traits<SamplingGraph>::vertex_descriptor SamplingGraphVertexDescriptor;
  typedef std::list<boost::graph_traits<SamplingGraph>::vertex_descriptor>::iterator
      SamplingGraphVertexDescriptorIterator;
  typedef boost::graph_traits<SamplingGraph>::edge_descriptor SamplingGraphEdgeDescriptor;
  typedef std::pair<size_t, size_t> Edge;

  struct props {
    double cell_size;

    double x_min;
    double x_max;
    double y_min;
    double y_max;

    size_t rows;
    size_t cols;

    size_t total_edges;

    props() = default;

    inline props(double cell_size,
                 double x_min,
                 double x_max,
                 double y_min,
                 double y_max,
                 size_t rows,
                 size_t cols,
                 size_t total_edges) {
      this->cell_size = cell_size;
      this->x_min = x_min;
      this->x_max = x_max;
      this->y_min = y_min;
      this->y_max = y_max;
      this->rows = rows;
      this->cols = cols;
      this->total_edges = total_edges;
    }
  };

  double getCost(double xi, double yi, double xf, double yf);
  double getCost(size_t row_i, size_t col_i, size_t row_f, size_t col_f);
  double distance(size_t row_i, size_t col_i, size_t row_f, size_t col_f);
  bool checkValidity(double xi, double yi, double xf, double yf);
  bool checkValidity(size_t row_i, size_t col_i, size_t row_f, size_t col_f);

  inline double colToX(size_t col) {
    return static_cast<double>(col) * this->props_.cell_size + this->props_.x_min;
  }

  inline double rowToY(size_t row) {
    return static_cast<double>(row) * this->props_.cell_size + this->props_.y_min;
  }
 protected:
  std::shared_ptr<::MoD::Base> mod_ptr_{nullptr};
  ompl::base::OptimizationObjectivePtr opt_obj_{nullptr};

  props props_;

  std::array<double, 3> start_{0.0, 0.0, 0.0};
  std::array<double, 3> goal_{0.0, 0.0, 0.0};

  std::list<Edge> edges_;
  std::vector<double> weights_;

  std::list<SamplingGraphVertexDescriptor> path_;

  ompl::RNG rng_;

  double bias_{0.05};

  void addEdgeAndWeight(size_t row_i, size_t col_i, size_t row_f, size_t col_f);

 public:
  DijkstraSampler(const ompl::base::SpaceInformation *si,
                  const std::string &mod_map_file_name,
                  const std::string &mod_type,
                  const ompl::base::OptimizationObjectivePtr &opt,
                  const std::array<double, 3> &start_state,
                  const std::array<double, 3> &goal_state,
                  double cell_size = 0.5,
                  double bias = 0.05);

  static ompl::base::ValidStateSamplerPtr allocate(const ompl::base::SpaceInformation *si,
                                                   const std::string &mod_map_file_name,
                                                   const std::string &mod_type,
                                                   const ompl::base::OptimizationObjectivePtr &opt,
                                                   const std::array<double, 3> &start_state,
                                                   const std::array<double, 3> &goal_state,
                                                   double cell_size = 0.5, double bias = 0.05) {
    return std::make_shared<DijkstraSampler>(si,
                                             mod_map_file_name,
                                             mod_type,
                                             opt,
                                             start_state,
                                             goal_state,
                                             cell_size,
                                             0.05);
  }

  inline void setStart(const std::array<double, 3> &start) { this->start_ = start; }
  inline void setGoal(const std::array<double, 3> &goal) { this->goal_ = goal; }
  inline void setCellSize(double cell_size) { this->props_.cell_size = cell_size; }
  inline void setBias(double bias) { this->bias_ = bias; }

  void setup();

  bool sample(ompl::base::State *state) override;

  inline bool sampleNear(ompl::base::State *state, const ompl::base::State *near, double distance) override {
    return false;
  }
};

} // namespace MoD
} // namespace ompl