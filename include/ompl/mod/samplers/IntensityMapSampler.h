#pragma once

#include <mod/cliffmap.hpp>
#include <ompl/base/ValidStateSampler.h>

namespace ompl {
namespace mod {

namespace ob = ompl::base;

class IntensityMapSampler : public ob::ValidStateSampler {

protected:
  ::MoD::IntensityMap q_map;

public:
  IntensityMapSampler(const ob::SpaceInformation *si,
                      const MoD::IntensityMap &q_map);

  IntensityMapSampler(const ob::SpaceInformation *si,
                      const std::string &intensity_map_file_name);

  bool sample(ob::State *state) override;

};

} // namespace mod
} // namespace ompl