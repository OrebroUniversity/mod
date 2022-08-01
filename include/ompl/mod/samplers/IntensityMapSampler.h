#pragma once

#include <memory>
#include <fstream>
#include <mod/cliffmap.hpp>

#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/util/RandomNumbers.h>

namespace ompl {
namespace MoD {

class IntensityMapSampler : public ompl::base::InformedSampler {

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

    inline ~QMap() = default;
  };

  std::vector<QMap> q_map;

  std::vector<QMap> nonq_map;

  double half_cell_size{0.0};

  double bias_{0.5};

  double value_sum{0.0};

  ompl::RNG rng_;

  bool debug_{false};

  std::fstream sampledPosesFile_;

 public:
  IntensityMapSampler(const ompl::base::ProblemDefinitionPtr &pdef,
                      unsigned int maxCalls,
                      const ::MoD::IntensityMap &q_map,
                      double bias,
                      bool debug = false);

  IntensityMapSampler(const ompl::base::ProblemDefinitionPtr &pdef,
                      unsigned int maxCalls,
                      const std::string &intensity_map_file_name,
                      double bias = 0.5,
                      bool debug = false);

  inline ~IntensityMapSampler() = default;

  void setup(const ::MoD::IntensityMap &intensity_map);

  bool sampleUniform(ompl::base::State *state, const ompl::base::Cost &maxCost) override;

  inline bool sampleUniform(ompl::base::State *state,
                            const ompl::base::Cost &minCost,
                            const ompl::base::Cost &maxCost) override {
    return sampleUniform(state, maxCost);
  }

  inline bool hasInformedMeasure() const override { return false; }

  inline double getInformedMeasure(const ompl::base::Cost &currentCost) const override { return this->space_->getMeasure(); }

  void sampleNecessarilyValid(ompl::base::State *state);

  static ompl::base::InformedSamplerPtr allocate(const ompl::base::ProblemDefinitionPtr &pdef,
                                                 unsigned int maxCalls,
                                                 const std::string &intensity_map_file_name,
                                                 double bias = 0.5,
                                                 bool debug = false) {
    return std::make_shared<IntensityMapSampler>(pdef, maxCalls, intensity_map_file_name, bias, debug);
  }
};

} // namespace MoD
} // namespace ompl