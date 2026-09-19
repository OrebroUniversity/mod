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

#include "core/footprint_checker.hpp"

#include <ompl/base/spaces/SE2StateSpace.h>

#include <cmath>
#include <mod/log.hpp>

namespace MoD::playground {

FootprintChecker::FootprintChecker(const ompl::base::SpaceInformationPtr &si, OccupancyMapConstPtr map,
                                   double radius)
    : ompl::base::StateValidityChecker(si), map_(std::move(map)), radius_(radius) {
  const double res = map_->pixel_size();
  const double r_px = radius_ / res;  // radius in pixels
  const int r_int = static_cast<int>(std::ceil(r_px));
  const double r2 = r_px * r_px + 1e-9;  // tolerance keeps the axis pixels at exactly the radius
  for (int dr = -r_int; dr <= r_int; ++dr) {
    for (int dc = -r_int; dc <= r_int; ++dc) {
      // Pixel centre offsets; the centre pixel is always included.
      if (static_cast<double>(dc * dc + dr * dr) <= r2) offsets_.emplace_back(dc, dr);
    }
  }
  MOD_LOG("FootprintChecker: radius %.3f m = %zu pixels at %.3f m", radius_, offsets_.size(), res);
}

bool FootprintChecker::isValidXY(double x, double y) const {
  long col, row;
  if (!map_->worldToPixel(x, y, col, row)) return false;
  for (const auto &o : offsets_) {
    if (map_->occupiedPixel(col + o.first, row + o.second)) return false;
  }
  return true;
}

bool FootprintChecker::isValid(const ompl::base::State *state) const {
  if (!si_->satisfiesBounds(state)) return false;
  const auto *se2 = state->as<ompl::base::SE2StateSpace::StateType>();
  return isValidXY(se2->getX(), se2->getY());
}

}  // namespace MoD::playground
