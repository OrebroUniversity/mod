#include <boost/log/trivial.hpp>
#include <boost/math/constants/constants.hpp>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/mod/samplers/IntensityMapSampler.h>

namespace ompl {
namespace MoD {

IntensityMapSampler::IntensityMapSampler(const ompl::base::SpaceInformation *si,
                                         const ::MoD::IntensityMap &qmap,
                                         double bias)
    : ompl::base::ValidStateSampler(si), bias_(bias) {
  this->attempts_ = 100;
  setup(qmap);
}

IntensityMapSampler::IntensityMapSampler(
    const ompl::base::SpaceInformation *si, const std::string &intensity_map_file_name, double bias)
    : ompl::base::ValidStateSampler(si), bias_(bias) {
  this->attempts_ = 100;
  setup(::MoD::IntensityMap(intensity_map_file_name));
}

bool IntensityMapSampler::checkValidity(double xi, double yi) {
  ompl::base::State *first = si_->allocState();

  (first->as<ompl::base::SE2StateSpace::StateType>())->setX(xi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setY(yi);
  (first->as<ompl::base::SE2StateSpace::StateType>())->setYaw(0.0);

  auto checker = si_->getStateValidityChecker();
  bool valid = true;
  if (checker != nullptr) {
    valid = checker->isValid(first);
  } else {
    std::cout << "SHITE";
  }
  si_->freeState(first);

  return valid;
}

void IntensityMapSampler::setup(const ::MoD::IntensityMap &intensity_map) {

  this->name_ = "Intensity Map Sampler";

  BOOST_LOG_TRIVIAL(info) << "Intensity Map has " << intensity_map.getRows() * intensity_map.getColumns()
                          << " locations.";
  for (size_t i = 0; i < intensity_map.getRows() * intensity_map.getColumns(); i++) {

    auto xy = intensity_map.getXYatIndex(i);
    if (this->checkValidity(xy[0], xy[1])) {
      q_map.emplace_back(xy[0], xy[1], 1.0 - intensity_map(xy[0], xy[1]));
      nonq_map.emplace_back(xy[0], xy[1], 1.0);
    }
  }

  // Sort it by value. Not necessary really.
  std::sort(q_map.begin(), q_map.end(), [](QMap a, QMap b) { return a.getValue() < b.getValue(); });

  // Compute weighted sum.
  this->value_sum =
      (std::accumulate(q_map.begin(), q_map.end(), QMap(0.0, 0.0, 0.0),
                       [](QMap a, QMap b) {
                         return QMap(0.0, 0.0, a.getValue() + b.getValue());
                       }))
          .getValue();
}

bool IntensityMapSampler::sample(ompl::base::State *state) {
  unsigned int attempts = 0;
  double dist = 0.0;
  sampleNecessarilyValid(state);
  return true;
}

void IntensityMapSampler::sampleNecessarilyValid(ompl::base::State *state) {

  // Sample theta first. This is the easiest part.
  double theta = rng_.uniformReal(-boost::math::constants::pi<double>(),
                                  boost::math::constants::pi<double>());

  // Sample position next.
  auto sampled_value = rng_.uniformReal(0.0, this->value_sum);

  // Move to the appropriate value in the list ...
  double accum_sum = 0.0;
  size_t index = 0;
  size_t result_index = 0;

  double sampled_x, sampled_y;

  auto random_num = rng_.uniformReal(0.0, 1.0);
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
    rng_.uniformReal(q_map[result_index].getX() - half_cell_size,
                     q_map[result_index].getX() + half_cell_size);
    rng_.uniformReal(q_map[result_index].getX() - half_cell_size,
                     q_map[result_index].getX() + half_cell_size);
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
    rng_.uniformReal(nonq_map[result_index].getX() - half_cell_size,
                     nonq_map[result_index].getX() + half_cell_size);
    rng_.uniformReal(nonq_map[result_index].getX() - half_cell_size,
                     nonq_map[result_index].getX() + half_cell_size);
  }

  (state->as<ompl::base::SE2StateSpace::StateType>())->setX(sampled_x);
  (state->as<ompl::base::SE2StateSpace::StateType>())->setY(sampled_y);
  (state->as<ompl::base::SE2StateSpace::StateType>())->setYaw(theta);
}

} // namespace MoD
} // namespace ompl