#include <boost/log/trivial.hpp>
#include <boost/math/constants/constants.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/mod/samplers/IntensityMapSampler.h>

namespace ompl {
namespace MoD {

IntensityMapSampler::IntensityMapSampler(const ob::SpaceInformation *si,
                                         const ::MoD::IntensityMap &qmap)
    : ob::ValidStateSampler(si) {
  setup(qmap);
}

IntensityMapSampler::IntensityMapSampler(
    const ob::SpaceInformation *si, const std::string &intensity_map_file_name)
    : ob::ValidStateSampler(si) {
  setup(::MoD::IntensityMap(intensity_map_file_name));
}

void IntensityMapSampler::setup(const ::MoD::IntensityMap &intensity_map) {

  this->name_ = "Intensity Map Sampler";

  BOOST_LOG_TRIVIAL(info) << "Intensity Map has "
                          << intensity_map.getRows() *
                                 intensity_map.getColumns()
                          << " locations.";
  for (size_t i = 0; i < intensity_map.getRows() * intensity_map.getColumns();
       i++) {
    auto xy = intensity_map.getXYatIndex(i);
    q_map.emplace_back(xy[0], xy[1], 1.0 - intensity_map(xy[0], xy[1]));
  }

  // Sort it by value. Not necessary really.
  std::sort(q_map.begin(), q_map.end(),
            [](QMap a, QMap b) { return a.getValue() < b.getValue(); });

  // Compute weighted sum.
  this->value_sum =
      (std::accumulate(q_map.begin(), q_map.end(), QMap(0.0, 0.0, 0.0),
                       [](QMap a, QMap b) {
                         return QMap(0.0, 0.0, a.getValue() + b.getValue());
                       }))
          .getValue();
}

bool IntensityMapSampler::sample(ob::State *state) {

  // Sample theta first. This is the easiest part.
  double theta = rng_.uniformReal(-boost::math::constants::pi<double>(),
                                  boost::math::constants::pi<double>());

  // Sample position next.
  auto sampled_value = rng_.uniformReal(0.0, this->value_sum);

  // Move to the appropriate value in the list ...
  double accum_sum = 0.0;
  size_t index = 0;
  size_t result_index = 0;

  while (index < q_map.size()) {
    if (sampled_value < accum_sum) {
      // This is the right index!
      result_index = index;
      break;
    }
    accum_sum = accum_sum + q_map[index].getValue();
    index = index + 1;
  }

  double sampled_x =
      rng_.uniformReal(q_map[result_index].getX() - half_cell_size,
                       q_map[result_index].getX() + half_cell_size);
  double sampled_y =
      rng_.uniformReal(q_map[result_index].getY() - half_cell_size,
                       q_map[result_index].getY() + half_cell_size);

  (state->as<ompl::base::SE2StateSpace::StateType>())->setX(sampled_x);
  (state->as<ompl::base::SE2StateSpace::StateType>())->setY(sampled_y);
  (state->as<ompl::base::SE2StateSpace::StateType>())->setYaw(theta);

  return true;
}

} // namespace MoD
} // namespace ompl