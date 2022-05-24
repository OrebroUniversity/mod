#pragma once

#include <memory>
#include <mod/cliffmap.hpp>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/util/RandomNumbers.h>

namespace ompl {
namespace MoD {

class IntensityMapSampler : public ompl::base::ValidStateSampler {

 private:
  bool checkValidity(double xi, double yi);

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

    double getX() const { return position[0]; }
    double getY() const { return position[1]; }
    double getValue() const { return value; }
  };

  std::vector<QMap> q_map;

  std::vector<QMap> nonq_map;

  double half_cell_size{0.0};

  double bias_{0.5};

  double value_sum{0.0};

  ompl::RNG rng_;

 public:
  IntensityMapSampler(const ompl::base::SpaceInformation *si, const ::MoD::IntensityMap &q_map, double bias);

  IntensityMapSampler(const ompl::base::SpaceInformation *si, const std::string &intensity_map_file_name, double bias);

  void setup(const ::MoD::IntensityMap &intensity_map);

  bool sample(ompl::base::State *state) override;

  void sampleNecessarilyValid(ompl::base::State *state);

  inline bool sampleNear(ompl::base::State *state, const ompl::base::State *near,
                         double distance) override {
    return false;
  }

  static ompl::base::ValidStateSamplerPtr
  allocate(const ompl::base::SpaceInformation *si,
           const std::string &intensity_map_file_name, double bias) {
    return std::make_shared<IntensityMapSampler>(si, intensity_map_file_name, bias);
  }
};

} // namespace MoD
} // namespace ompl