#include <ompl/mod/samplers/DijkstraSampler.h>

#include <ompl/base/SpaceInformation.h>

namespace ompl {
namespace MoD {
DijkstraSampler::DijkstraSampler(const ompl::base::SpaceInformation *si,
                                 const std::string &mod_map_file_name,
                                 const std::string &mod_type) :
    ompl::base::ValidStateSampler(si) {

  if (mod_type == "cliffmap" || mod_type == "CLiFF-map") {
    mod_ptr_ = std::make_shared<::MoD::CLiFFMap>(mod_map_file_name);
  } else if (mod_type == "gmmtmap" || mod_type == "GMMT-map") {
    mod_ptr_ = std::make_shared<::MoD::GMMTMap>(mod_map_file_name);
  } else {
    BOOST_LOG_TRIVIAL(error) << "This isn't implemented yet for your mod type: " << mod_type;
  }
  setup();
}

void DijkstraSampler::setup() {
  const ompl::base::RealVectorBounds state_bounds = si_->getStateSpace()->as<ompl::base::SE2StateSpace>()->getBounds();
  const auto x_min = state_bounds.low[0];
  const auto x_max = state_bounds.high[0];
  const auto y_min = state_bounds.low[1];
  const auto y_max = state_bounds.high[1];

  const auto resolution = si_->getStateValidityCheckingResolution();
}

}
}
