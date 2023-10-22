#include <ompl/mod/samplers/HybridSampler.h>

namespace ompl::MoD {
HybridSampler::HybridSampler(const ompl::base::ProblemDefinitionPtr &pdef,
                             unsigned int maxCalls,
                             const std::string &intensity_map_file_name,
                             double cell_size, double bias_a, double bias_b,
                             bool debug)
    : ompl::base::InformedSampler(pdef, maxCalls), dijkstra_bias_(bias_a),
      intensity_bias_(bias_b) {
  this->dijkstra_sampler_ = ompl::MoD::DijkstraSampler::allocate(
      pdef, maxCalls, cell_size, 1.0, debug);
  this->intensity_map_sampler_ = ompl::MoD::IntensityMapSampler::allocate(
      pdef, maxCalls, intensity_map_file_name, 1.0, debug);
  this->ellipse_sampler_ = std::shared_ptr<ompl::base::InformedSampler>(
      new ompl::base::PathLengthDirectInfSampler(pdef, maxCalls));
}

bool HybridSampler::sampleUniform(ompl::base::State *state,
                                  const ompl::base::Cost &maxCost) {
  auto rndm = rng_.uniformReal(0.0, 1.0);

  if (rndm < dijkstra_bias_) {
    return dijkstra_sampler_->sampleUniform(state, maxCost);
  } else if (rndm < (intensity_bias_ + dijkstra_bias_)) {
    return intensity_map_sampler_->sampleUniform(state, maxCost);
  } else {
    std::dynamic_pointer_cast<IntensityMapSampler>(intensity_map_sampler_)->setBias(0.0);
    auto result = ellipse_sampler_->sampleUniform(state, maxCost);
    std::dynamic_pointer_cast<IntensityMapSampler>(intensity_map_sampler_)->setBias(intensity_bias_);
    return result;
  }
}
} // namespace ompl::MoD