#include <ompl/mod/samplers/DijkstraSampler.h>

#include <ompl/base/SpaceInformation.h>

#include <utility>

namespace ompl {
namespace MoD {
DijkstraSampler::DijkstraSampler(const ompl::base::SpaceInformation *si,
                                 const std::string &mod_map_file_name,
                                 const std::string &mod_type,
                                 const ompl::base::OptimizationObjectivePtr &opt,
                                 const std::array<double, 3> &start_state,
                                 const std::array<double, 3> &goal_state,
                                 double cell_size, double bias) :
    ompl::base::ValidStateSampler(si),
    opt_obj_(opt),
    start_(start_state),
    goal_(goal_state),
    bias_(bias) {
  this->props_.cell_size = cell_size;
  if (mod_type == "cliffmap" || mod_type == "CLiFF-map") {
    mod_ptr_ = std::make_shared<::MoD::CLiFFMap>(mod_map_file_name);
  } else if (mod_type == "gmmtmap" || mod_type == "GMMT-map") {
    mod_ptr_ = std::make_shared<::MoD::GMMTMap>(mod_map_file_name);
  } else {
    BOOST_LOG_TRIVIAL(error) << "This isn't implemented yet for your mod type: " << mod_type;
  }

  this->name_ = "Dijkstra Sampler with " + mod_type;
  setup();
}

double DijkstraSampler::getCost(double xi, double yi, double xf, double yf) {
  ompl::base::State *first = si_->allocState();
  ompl::base::State *next = si_->allocState();

  (first->as<ompl::base::SE2StateSpace::StateType>())->setX(xi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setY(yi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setYaw(atan2(yf - yi, xf - xi));

  (next->as<ompl::base::SE2StateSpace::StateType>())->setX(xf);
  (next->as<ompl::base::SE2StateSpace::StateType>())->setY(yf);
  (next->as<ompl::base::SE2StateSpace::StateType>())->setYaw(atan2(yf - yi, xf - xi));

  auto cost = opt_obj_->motionCost(first, next).value();

  si_->freeState(first);
  si_->freeState(next);

  return cost;
}

bool DijkstraSampler::checkValidity(double xi, double yi, double xf, double yf) {
  ompl::base::State *first = si_->allocState();
  ompl::base::State *next = si_->allocState();

  (first->as<ompl::base::SE2StateSpace::StateType>())->setX(xi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setY(yi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setYaw(atan2(yf - yi, xf - xi));

  (next->as<ompl::base::SE2StateSpace::StateType>())->setX(xf);
  (next->as<ompl::base::SE2StateSpace::StateType>())->setY(yf);
  (next->as<ompl::base::SE2StateSpace::StateType>())->setYaw(atan2(yf - yi, xf - xi));

  auto checker = si_->getStateValidityChecker();
  bool valid = true;
  if (checker != nullptr) {
    valid = checker->isValid(first) && checker->isValid(next);
  } else {
    std::cout << "SHITE";
  }
  si_->freeState(first);
  si_->freeState(next);

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
  //weights_.push_back(distance(row_i, col_i, row_f, col_f));
}

void DijkstraSampler::setup() {
  const ompl::base::RealVectorBounds state_bounds = si_->getStateSpace()->as<ompl::base::SE2StateSpace>()->getBounds();
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
    BOOST_LOG_TRIVIAL(error)
      << "Surely, the number of edges has reduced due to invalid ones not being added. We added: " << edges_.size()
      << " edges and " << weights_.size() << " weights, but would have added " << total_edges
      << " if we considered the bad apples.";
  }

  SamplingGraph graph_(edges_.begin(), edges_.end(), weights_.begin(), num_nodes);

  std::vector<SamplingGraphVertexDescriptor> p(boost::num_vertices(graph_));
  std::vector<int> d(boost::num_vertices(graph_));

  BOOST_LOG_TRIVIAL(info) << "Vertices in the graph are: " << boost::num_vertices(graph_);

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

  boost::dijkstra_shortest_paths(graph_, sVertexDescriptor, boost::predecessor_map(boost::make_iterator_property_map(
      p.begin(), boost::get(boost::vertex_index, graph_))).distance_map(boost::make_iterator_property_map(
      d.begin(), boost::get(boost::vertex_index, graph_))));

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
  BOOST_LOG_TRIVIAL(info) << "Found a path: " << path_.size() << " nodes... " << "Cost: " << cost;
}

bool DijkstraSampler::sample(ompl::base::State *state) {
  size_t sampled_col = 0;
  size_t sampled_row = 0;
  double sampled_theta;

  double randomValue = rng_.uniformReal(0.0, 1.0);

  // At a bias_ % probability, choose a row, col from the dijkstra path
  if (randomValue < bias_) {

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

  } else {
    sampled_col = rng_.uniformInt(0, this->props_.cols - 1);
    sampled_row = rng_.uniformInt(0, this->props_.rows - 1);
    sampled_theta = rng_.uniformReal(-boost::math::constants::pi<double>(),
                                     boost::math::constants::pi<double>());
  }

  double sampled_x =
      rng_.uniformReal(colToX(sampled_col) - this->props_.cell_size / 2.0,
                       colToX(sampled_col) + this->props_.cell_size / 2.0);
  double sampled_y =
      rng_.uniformReal(rowToY(sampled_row) - this->props_.cell_size / 2.0,
                       rowToY(sampled_row) + this->props_.cell_size / 2.0);

  (state->as<ompl::base::SE2StateSpace::StateType>())->setX(sampled_x);
  (state->as<ompl::base::SE2StateSpace::StateType>())->setY(sampled_y);
  (state->as<ompl::base::SE2StateSpace::StateType>())->setYaw(sampled_theta);
  return true;
}

}
}