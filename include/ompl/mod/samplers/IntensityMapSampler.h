#pragma once

#include <memory>
#include <mod/cliffmap.hpp>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/util/RandomNumbers.h>

namespace ompl {
namespace MoD {

class IntensityMapSampler : public ompl::base::ValidStateSampler {

 protected:
  class QMap {
    std::array<double, 2> position;
    double value;

   public:
    QMap(double x, double y, double value) {
      this->position[0] = x;
      this->position[1] = y;
      this->value = value;
    }

    double getX() { return position[0]; }
    double getY() { return position[1]; }
    double getValue() { return value; }
  };

  std::vector<QMap> q_map;

  double half_cell_size{0.0};

  double value_sum{0.0};

  ompl::RNG rng_;

 public:
  IntensityMapSampler(const ompl::base::SpaceInformation *si,
                      const ::MoD::IntensityMap &q_map);

  IntensityMapSampler(const ompl::base::SpaceInformation *si,
                      const std::string &intensity_map_file_name);

  void setup(const ::MoD::IntensityMap &intensity_map);

  bool sample(ompl::base::State *state) override;

  void sampleNotNecessarilyValid(ompl::base::State *state);

  inline bool sampleNear(ompl::base::State *state, const ompl::base::State *near,
                         double distance) override {
    return false;
  }

  static ompl::base::ValidStateSamplerPtr
  allocate(const ompl::base::SpaceInformation *si,
           const std::string &intensity_map_file_name) {
    return std::make_shared<IntensityMapSampler>(si, intensity_map_file_name);
  }
};

} // namespace MoD
} // namespace ompl