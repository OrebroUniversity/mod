/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of mod.
 *
 *   mod is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   mod is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with mod.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>

#include <utility>
#include <vector>

#include "core/occupancy_map.hpp"

namespace MoD::playground {

/**
 * Footprint test with one circumscribed circle: the rasterised disc of pixels of the given radius around the
 * pose is tested against the occupancy map (offset list precomputed once). Yaw is irrelevant. A pose is valid
 * iff it satisfies the state bounds and no pixel of the disc is occupied (pixels outside the map count as
 * occupied).
 */
class FootprintChecker : public ompl::base::StateValidityChecker {
 public:
  FootprintChecker(const ompl::base::SpaceInformationPtr &si, OccupancyMapConstPtr map, double radius);

  bool isValid(const ompl::base::State *state) const override;
  bool isValidXY(double x, double y) const;

  double radius() const { return radius_; }
  size_t discPixels() const { return offsets_.size(); }
  const OccupancyMapConstPtr &map() const { return map_; }

 private:
  OccupancyMapConstPtr map_;
  double radius_;
  std::vector<std::pair<int, int>> offsets_;  ///< (dcol, drow) of every pixel of the disc
};

}  // namespace MoD::playground
