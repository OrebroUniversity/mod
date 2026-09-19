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

#include <map>
#include <memory>
#include <mod/cliffmap.hpp>
#include <mod/gmmtmap.hpp>
#include <mutex>
#include <string>

#include "core/occupancy_map.hpp"

namespace MoD::playground {

/// Loads every map once per process and shares it as shared_ptr<const>. Thread-safe.
class MapCache {
 public:
  OccupancyMapConstPtr occupancy(const std::string &yaml_path);
  ::MoD::CLiFFMapConstPtr cliff(const std::string &xml_path);
  ::MoD::GMMTMapConstPtr gmmt(const std::string &xml_path);
  ::MoD::IntensityMapConstPtr intensity(const std::string &xml_path);

  void clear();

 private:
  std::mutex mutex_;
  std::map<std::string, OccupancyMapConstPtr> occupancy_;
  std::map<std::string, ::MoD::CLiFFMapConstPtr> cliff_;
  std::map<std::string, ::MoD::GMMTMapConstPtr> gmmt_;
  std::map<std::string, ::MoD::IntensityMapConstPtr> intensity_;
};

}  // namespace MoD::playground
