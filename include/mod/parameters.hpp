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

#include <array>
#include <nlohmann/json.hpp>
#include <string>

/// Parameter structs for the mod library and its playground. Every struct has nlohmann `to_json` / `from_json`;
/// `from_json` starts from the defaults and overrides only the keys present, so partial JSON is accepted.
/// Defaults equal the Paper IV settings.
namespace MoD {

enum class Shape { circle, rectangle };
enum class StateSpaceType { dubins, reeds_shepp };
enum class SamplerType { iid, ellipse, intensity, dijkstra, hybrid };
enum class ObjectiveType { cliff, gmmt, dtc, intensity, path_length };
enum class PlannerType { rrt_star, ait_star };

std::string to_string(Shape v);
std::string to_string(StateSpaceType v);
std::string to_string(SamplerType v);
std::string to_string(ObjectiveType v);
std::string to_string(PlannerType v);

/// Parse from the strings produced by `to_string`; throws std::invalid_argument on unknown text.
Shape shapeFromString(const std::string &s);
StateSpaceType stateSpaceFromString(const std::string &s);
SamplerType samplerTypeFromString(const std::string &s);
ObjectiveType objectiveTypeFromString(const std::string &s);
PlannerType plannerTypeFromString(const std::string &s);

NLOHMANN_JSON_SERIALIZE_ENUM(Shape, {{Shape::circle, "circle"}, {Shape::rectangle, "rectangle"}})
NLOHMANN_JSON_SERIALIZE_ENUM(StateSpaceType,
                             {{StateSpaceType::dubins, "dubins"}, {StateSpaceType::reeds_shepp, "reeds_shepp"}})
NLOHMANN_JSON_SERIALIZE_ENUM(SamplerType, {{SamplerType::iid, "iid"},
                                           {SamplerType::ellipse, "ellipse"},
                                           {SamplerType::intensity, "intensity"},
                                           {SamplerType::dijkstra, "dijkstra"},
                                           {SamplerType::hybrid, "hybrid"}})
NLOHMANN_JSON_SERIALIZE_ENUM(ObjectiveType, {{ObjectiveType::cliff, "cliff"},
                                             {ObjectiveType::gmmt, "gmmt"},
                                             {ObjectiveType::dtc, "dtc"},
                                             {ObjectiveType::intensity, "intensity"},
                                             {ObjectiveType::path_length, "path_length"}})
NLOHMANN_JSON_SERIALIZE_ENUM(PlannerType, {{PlannerType::rrt_star, "rrt_star"}, {PlannerType::ait_star, "ait_star"}})

/// The robot. No resolution field: both the collision step and the MoD cost step are inferred from the maps.
struct VehicleParameters {
  Shape shape{Shape::rectangle};
  double radius{0.4};  ///< circle only [m]
  double length{0.7};  ///< rectangle only [m], along the heading (Paper IV robot: x in [-0.2, 0.5])
  double width{0.4};   ///< rectangle only [m]
  StateSpaceType state_space{StateSpaceType::dubins};
  double turning_radius{1.0};  ///< [m]

  /// Radius of the circumscribed circle used for the footprint test.
  double circumscribedRadius() const;
};

/// Values the playground computes from the loaded maps and the vehicle. Written to config.json for the record,
/// never read back.
struct Derived {
  double occupancy_pixel_m{0.0};
  double mod_cell_m{0.0};
  double collision_step_m{0.0};
  double mod_cost_step_m{0.0};
  double circumscribed_radius_m{0.0};
};

struct SamplerParameters {
  SamplerType type{SamplerType::iid};
  double bias{0.05};                  ///< probability of the informed branch (dijkstra / intensity)
  double dijkstra_cell_size{0.5};     ///< [m]
  double hybrid_intensity_bias{0.01}; ///< hybrid: probability of the intensity branch
  std::string intensity_map_file;     ///< empty: the objective's intensity map is used, if it has one
  bool log_samples{false};
};

struct OptObjParameters {
  ObjectiveType type{ObjectiveType::cliff};
  double w_d{1.0};
  double w_q{1.0};
  double w_c{0.1};  ///< Paper IV: cliff 0.1, gmmt 0.1, intensity 0.2, dtc 0.02
  std::string cliff_map_file;
  std::string gmmt_map_file;
  std::string intensity_map_file;
  double max_vehicle_speed{1.0};
  double mahalanobis_threshold{10.0};
  bool use_mixing_factor{true};
};

struct PlannerParameters {
  PlannerType type{PlannerType::rrt_star};
  double max_planning_time{64.0};  ///< [s]
  unsigned int seed{0};
  double range{0.0};  ///< RRT* steer range [m]; 0 lets OMPL pick from the space extent
  double goal_bias{0.05};
  unsigned int batch_size{100};  ///< AIT*
  bool informed_sampling{true};  ///< RRT*: use the objective's informed sampler
};

struct Scenario {
  std::string name;
  std::string map_yaml;
  std::array<double, 3> start{0.0, 0.0, 0.0};
  std::array<double, 3> goal{0.0, 0.0, 0.0};
};

/// Filled by the run logger, never by hand.
struct RunMeta {
  std::string mod_version;
  std::string git_hash;
  std::string hostname;
  std::string started_at;  ///< ISO 8601
};

/// One object per scope; config.json is its serialization.
struct RunConfig {
  VehicleParameters vehicle;
  Derived derived;
  SamplerParameters sampler;
  OptObjParameters objective;
  PlannerParameters planner;
  Scenario scenario;
  RunMeta meta;
};

void to_json(nlohmann::json &j, const VehicleParameters &p);
void from_json(const nlohmann::json &j, VehicleParameters &p);
void to_json(nlohmann::json &j, const Derived &p);
void from_json(const nlohmann::json &j, Derived &p);
void to_json(nlohmann::json &j, const SamplerParameters &p);
void from_json(const nlohmann::json &j, SamplerParameters &p);
void to_json(nlohmann::json &j, const OptObjParameters &p);
void from_json(const nlohmann::json &j, OptObjParameters &p);
void to_json(nlohmann::json &j, const PlannerParameters &p);
void from_json(const nlohmann::json &j, PlannerParameters &p);
void to_json(nlohmann::json &j, const Scenario &p);
void from_json(const nlohmann::json &j, Scenario &p);
void to_json(nlohmann::json &j, const RunMeta &p);
void from_json(const nlohmann::json &j, RunMeta &p);
void to_json(nlohmann::json &j, const RunConfig &p);
void from_json(const nlohmann::json &j, RunConfig &p);

}  // namespace MoD
