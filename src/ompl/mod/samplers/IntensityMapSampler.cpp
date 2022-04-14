#include <ompl/mod/samplers/IntensityMapSampler.h>

namespace ompl {
namespace mod {
IntensityMapSampler::IntensityMapSampler(const ob::SpaceInformation *si,
                                         const MoD::IntensityMap &qmap)
    : ob::ValidStateSampler(si), q_map(qmap) {}

IntensityMapSampler::IntensityMapSampler(
    const ob::SpaceInformation *si, const std::string &intensity_map_file_name)
    : ob::ValidStateSampler(si), q_map(intensity_map_file_name) {}

bool IntensityMapSampler::sample(ob::State *state) {

  return false;
}

} // namespace mod
} // namespace ompl