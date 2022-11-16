#pragma once

#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <ompl/mod/samplers/DijkstraSampler.h>
#include <ompl/mod/samplers/IntensityMapSampler.h>

namespace ompl::MoD {
class HybridSampler : public ompl::base::InformedSampler {
protected:
  std::shared_ptr<ompl::base::InformedSampler> intensity_map_sampler_;
  std::shared_ptr<ompl::base::InformedSampler> dijkstra_sampler_;
  std::shared_ptr<ompl::base::InformedSampler> ellipse_sampler_;

  double intensity_bias_{0.0};
  double dijkstra_bias_{0.0};

  ompl::RNG rng_;

public:
  HybridSampler(const ompl::base::ProblemDefinitionPtr &pdef,
                unsigned int maxCalls = 100,
                const std::string &intensity_map_file_name = "",
                double cell_size = 0.5, double bias_a = 0.05,
                double bias_b = 0.05, bool debug = false);

  static ompl::base::InformedSamplerPtr allocate(
      const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxCalls = 100,
      const std::string &intensity_map_file_name = "", double cell_size = 0.5,
      double bias_a = 0.05, double bias_b = 0.05, bool debug = false) {
    return std::make_shared<HybridSampler>(pdef, maxCalls,
                                           intensity_map_file_name, cell_size,
                                           bias_a, bias_b, debug);
  }

  bool sampleUniform(ompl::base::State *state,
                     const ompl::base::Cost &maxCost) override;

  inline bool sampleUniform(ompl::base::State *state,
                            const ompl::base::Cost &minCost,
                            const ompl::base::Cost &maxCost) override {
    return sampleUniform(state, maxCost);
  }

  inline bool hasInformedMeasure() const override { return false; }

  inline double
  getInformedMeasure(const ompl::base::Cost &currentCost) const override {
    return this->space_->getMeasure();
  }
};
} // namespace ompl::MoD