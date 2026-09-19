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

#include "core/map_cache.hpp"

namespace MoD::playground {

OccupancyMapConstPtr MapCache::occupancy(const std::string &yaml_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto &slot = occupancy_[yaml_path];
  if (!slot) slot = std::make_shared<const OccupancyMap>(yaml_path);
  return slot;
}

::MoD::CLiFFMapConstPtr MapCache::cliff(const std::string &xml_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto &slot = cliff_[xml_path];
  if (!slot) slot = std::make_shared<const ::MoD::CLiFFMap>(xml_path, true);
  return slot;
}

::MoD::GMMTMapConstPtr MapCache::gmmt(const std::string &xml_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto &slot = gmmt_[xml_path];
  if (!slot) slot = std::make_shared<const ::MoD::GMMTMap>(xml_path);
  return slot;
}

::MoD::IntensityMapConstPtr MapCache::intensity(const std::string &xml_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto &slot = intensity_[xml_path];
  if (!slot) slot = std::make_shared<const ::MoD::IntensityMap>(xml_path);
  return slot;
}

void MapCache::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  occupancy_.clear();
  cliff_.clear();
  gmmt_.clear();
  intensity_.clear();
}

}  // namespace MoD::playground
