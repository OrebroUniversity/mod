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

#include <mod/parameters.hpp>

#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace MoD {

using nlohmann::json;

namespace {

template <typename E>
E enumFromString(const std::string &s, const char *what) {
  const E v = json(s).get<E>();
  if (json(v).get<std::string>() != s) throw std::invalid_argument(std::string("unknown ") + what + ": '" + s + "'");
  return v;
}

/// Read `key` into `out` if present; keep the current value otherwise.
template <typename T>
void get(const json &j, const char *key, T &out) {
  if (!j.contains(key)) return;
  if constexpr (std::is_enum_v<T>) {
    out = enumFromString<T>(j.at(key).get<std::string>(), key);
  } else {
    out = j.at(key).get<T>();
  }
}

}  // namespace

std::string to_string(Shape v) { return json(v).get<std::string>(); }
std::string to_string(StateSpaceType v) { return json(v).get<std::string>(); }
std::string to_string(SamplerType v) { return json(v).get<std::string>(); }
std::string to_string(ObjectiveType v) { return json(v).get<std::string>(); }
std::string to_string(PlannerType v) { return json(v).get<std::string>(); }

Shape shapeFromString(const std::string &s) { return enumFromString<Shape>(s, "shape"); }
StateSpaceType stateSpaceFromString(const std::string &s) { return enumFromString<StateSpaceType>(s, "state space"); }
SamplerType samplerTypeFromString(const std::string &s) { return enumFromString<SamplerType>(s, "sampler type"); }
ObjectiveType objectiveTypeFromString(const std::string &s) {
  return enumFromString<ObjectiveType>(s, "objective type");
}
PlannerType plannerTypeFromString(const std::string &s) { return enumFromString<PlannerType>(s, "planner type"); }

double VehicleParameters::circumscribedRadius() const {
  if (shape == Shape::circle) return radius;
  return std::sqrt((length / 2.0) * (length / 2.0) + (width / 2.0) * (width / 2.0));
}

void to_json(json &j, const VehicleParameters &p) {
  j = json{{"shape", p.shape},   {"radius", p.radius},
           {"length", p.length}, {"width", p.width},
           {"state_space", p.state_space}, {"turning_radius", p.turning_radius}};
}
void from_json(const json &j, VehicleParameters &p) {
  get(j, "shape", p.shape);
  get(j, "radius", p.radius);
  get(j, "length", p.length);
  get(j, "width", p.width);
  get(j, "state_space", p.state_space);
  get(j, "turning_radius", p.turning_radius);
}

void to_json(json &j, const Derived &p) {
  j = json{{"occupancy_pixel_m", p.occupancy_pixel_m},
           {"mod_cell_m", p.mod_cell_m},
           {"collision_step_m", p.collision_step_m},
           {"mod_cost_step_m", p.mod_cost_step_m},
           {"circumscribed_radius_m", p.circumscribed_radius_m}};
}
void from_json(const json &j, Derived &p) {
  get(j, "occupancy_pixel_m", p.occupancy_pixel_m);
  get(j, "mod_cell_m", p.mod_cell_m);
  get(j, "collision_step_m", p.collision_step_m);
  get(j, "mod_cost_step_m", p.mod_cost_step_m);
  get(j, "circumscribed_radius_m", p.circumscribed_radius_m);
}

void to_json(json &j, const SamplerParameters &p) {
  j = json{{"type", p.type},
           {"bias", p.bias},
           {"dijkstra_cell_size", p.dijkstra_cell_size},
           {"hybrid_intensity_bias", p.hybrid_intensity_bias},
           {"intensity_map_file", p.intensity_map_file},
           {"log_samples", p.log_samples}};
}
void from_json(const json &j, SamplerParameters &p) {
  get(j, "type", p.type);
  get(j, "bias", p.bias);
  get(j, "dijkstra_cell_size", p.dijkstra_cell_size);
  get(j, "hybrid_intensity_bias", p.hybrid_intensity_bias);
  get(j, "intensity_map_file", p.intensity_map_file);
  get(j, "log_samples", p.log_samples);
}

void to_json(json &j, const OptObjParameters &p) {
  j = json{{"type", p.type},
           {"w_d", p.w_d},
           {"w_q", p.w_q},
           {"w_c", p.w_c},
           {"cliff_map_file", p.cliff_map_file},
           {"gmmt_map_file", p.gmmt_map_file},
           {"intensity_map_file", p.intensity_map_file},
           {"max_vehicle_speed", p.max_vehicle_speed},
           {"mahalanobis_threshold", p.mahalanobis_threshold},
           {"use_mixing_factor", p.use_mixing_factor}};
}
void from_json(const json &j, OptObjParameters &p) {
  get(j, "type", p.type);
  get(j, "w_d", p.w_d);
  get(j, "w_q", p.w_q);
  get(j, "w_c", p.w_c);
  get(j, "cliff_map_file", p.cliff_map_file);
  get(j, "gmmt_map_file", p.gmmt_map_file);
  get(j, "intensity_map_file", p.intensity_map_file);
  get(j, "max_vehicle_speed", p.max_vehicle_speed);
  get(j, "mahalanobis_threshold", p.mahalanobis_threshold);
  get(j, "use_mixing_factor", p.use_mixing_factor);
}

void to_json(json &j, const PlannerParameters &p) {
  j = json{{"type", p.type},
           {"max_planning_time", p.max_planning_time},
           {"seed", p.seed},
           {"range", p.range},
           {"goal_bias", p.goal_bias},
           {"batch_size", p.batch_size},
           {"informed_sampling", p.informed_sampling}};
}
void from_json(const json &j, PlannerParameters &p) {
  get(j, "type", p.type);
  get(j, "max_planning_time", p.max_planning_time);
  get(j, "seed", p.seed);
  get(j, "range", p.range);
  get(j, "goal_bias", p.goal_bias);
  get(j, "batch_size", p.batch_size);
  get(j, "informed_sampling", p.informed_sampling);
}

void to_json(json &j, const Scenario &p) {
  j = json{{"name", p.name}, {"map_yaml", p.map_yaml}, {"start", p.start}, {"goal", p.goal}};
}
void from_json(const json &j, Scenario &p) {
  get(j, "name", p.name);
  get(j, "map_yaml", p.map_yaml);
  get(j, "start", p.start);
  get(j, "goal", p.goal);
}

void to_json(json &j, const RunMeta &p) {
  j = json{{"mod_version", p.mod_version},
           {"git_hash", p.git_hash},
           {"hostname", p.hostname},
           {"started_at", p.started_at}};
}
void from_json(const json &j, RunMeta &p) {
  get(j, "mod_version", p.mod_version);
  get(j, "git_hash", p.git_hash);
  get(j, "hostname", p.hostname);
  get(j, "started_at", p.started_at);
}

void to_json(json &j, const RunConfig &p) {
  j = json{{"VehicleParameters", p.vehicle}, {"Derived", p.derived},
           {"SamplerParameters", p.sampler}, {"OptObjParameters", p.objective},
           {"PlannerParameters", p.planner}, {"Scenario", p.scenario},
           {"RunMeta", p.meta}};
}
void from_json(const json &j, RunConfig &p) {
  get(j, "VehicleParameters", p.vehicle);
  get(j, "Derived", p.derived);
  get(j, "SamplerParameters", p.sampler);
  get(j, "OptObjParameters", p.objective);
  get(j, "PlannerParameters", p.planner);
  get(j, "Scenario", p.scenario);
  get(j, "RunMeta", p.meta);
}

}  // namespace MoD
