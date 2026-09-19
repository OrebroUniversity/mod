#include <gtest/gtest.h>

#include <mod/parameters.hpp>

using nlohmann::json;

namespace {

template <typename T>
T roundTrip(const T &in) {
  json j = in;
  const std::string text = j.dump();
  return json::parse(text).get<T>();
}

}  // namespace

TEST(Parameters, VehicleRoundTrip) {
  MoD::VehicleParameters p;
  p.shape = MoD::Shape::circle;
  p.radius = 0.33;
  p.length = 1.2;
  p.width = 0.8;
  p.state_space = MoD::StateSpaceType::reeds_shepp;
  p.turning_radius = 2.5;
  const auto r = roundTrip(p);
  EXPECT_EQ(r.shape, p.shape);
  EXPECT_DOUBLE_EQ(r.radius, p.radius);
  EXPECT_DOUBLE_EQ(r.length, p.length);
  EXPECT_DOUBLE_EQ(r.width, p.width);
  EXPECT_EQ(r.state_space, p.state_space);
  EXPECT_DOUBLE_EQ(r.turning_radius, p.turning_radius);
}

TEST(Parameters, VehicleCircumscribedRadius) {
  MoD::VehicleParameters p;
  p.shape = MoD::Shape::circle;
  p.radius = 0.5;
  EXPECT_DOUBLE_EQ(p.circumscribedRadius(), 0.5);
  p.shape = MoD::Shape::rectangle;
  p.length = 0.6;
  p.width = 0.8;
  EXPECT_DOUBLE_EQ(p.circumscribedRadius(), 0.5);
}

TEST(Parameters, DerivedRoundTrip) {
  MoD::Derived p{0.05, 1.0, 0.05, 0.05, 0.4031};
  const auto r = roundTrip(p);
  EXPECT_DOUBLE_EQ(r.occupancy_pixel_m, 0.05);
  EXPECT_DOUBLE_EQ(r.mod_cell_m, 1.0);
  EXPECT_DOUBLE_EQ(r.collision_step_m, 0.05);
  EXPECT_DOUBLE_EQ(r.mod_cost_step_m, 0.05);
  EXPECT_DOUBLE_EQ(r.circumscribed_radius_m, 0.4031);
}

TEST(Parameters, SamplerRoundTrip) {
  MoD::SamplerParameters p;
  p.type = MoD::SamplerType::hybrid;
  p.bias = 0.1;
  p.dijkstra_cell_size = 1.0;
  p.hybrid_intensity_bias = 0.02;
  p.intensity_map_file = "q.xml";
  p.log_samples = true;
  const auto r = roundTrip(p);
  EXPECT_EQ(r.type, p.type);
  EXPECT_DOUBLE_EQ(r.bias, p.bias);
  EXPECT_DOUBLE_EQ(r.dijkstra_cell_size, p.dijkstra_cell_size);
  EXPECT_DOUBLE_EQ(r.hybrid_intensity_bias, p.hybrid_intensity_bias);
  EXPECT_EQ(r.intensity_map_file, p.intensity_map_file);
  EXPECT_EQ(r.log_samples, p.log_samples);
}

TEST(Parameters, ObjectiveRoundTrip) {
  MoD::OptObjParameters p;
  p.type = MoD::ObjectiveType::dtc;
  p.w_d = 2.0;
  p.w_q = 0.5;
  p.w_c = 0.02;
  p.cliff_map_file = "c.xml";
  p.gmmt_map_file = "g.xml";
  p.intensity_map_file = "q.xml";
  p.max_vehicle_speed = 1.5;
  p.mahalanobis_threshold = 7.0;
  p.use_mixing_factor = false;
  const auto r = roundTrip(p);
  EXPECT_EQ(r.type, p.type);
  EXPECT_DOUBLE_EQ(r.w_d, p.w_d);
  EXPECT_DOUBLE_EQ(r.w_q, p.w_q);
  EXPECT_DOUBLE_EQ(r.w_c, p.w_c);
  EXPECT_EQ(r.cliff_map_file, p.cliff_map_file);
  EXPECT_EQ(r.gmmt_map_file, p.gmmt_map_file);
  EXPECT_EQ(r.intensity_map_file, p.intensity_map_file);
  EXPECT_DOUBLE_EQ(r.max_vehicle_speed, p.max_vehicle_speed);
  EXPECT_DOUBLE_EQ(r.mahalanobis_threshold, p.mahalanobis_threshold);
  EXPECT_EQ(r.use_mixing_factor, p.use_mixing_factor);
}

TEST(Parameters, PlannerRoundTrip) {
  MoD::PlannerParameters p;
  p.type = MoD::PlannerType::ait_star;
  p.max_planning_time = 12.5;
  p.seed = 42;
  p.range = 3.0;
  p.goal_bias = 0.1;
  p.batch_size = 200;
  p.informed_sampling = false;
  const auto r = roundTrip(p);
  EXPECT_EQ(r.type, p.type);
  EXPECT_DOUBLE_EQ(r.max_planning_time, p.max_planning_time);
  EXPECT_EQ(r.seed, p.seed);
  EXPECT_DOUBLE_EQ(r.range, p.range);
  EXPECT_DOUBLE_EQ(r.goal_bias, p.goal_bias);
  EXPECT_EQ(r.batch_size, p.batch_size);
  EXPECT_EQ(r.informed_sampling, p.informed_sampling);
}

TEST(Parameters, HybridAStarRoundTrip) {
  MoD::HybridAStarParameters d;
  EXPECT_DOUBLE_EQ(d.cell_size_m, 0.25);
  EXPECT_EQ(d.angle_bins, 72u);
  EXPECT_DOUBLE_EQ(d.change_penalty, 1000.0);
  MoD::HybridAStarParameters p;
  p.cell_size_m = 0.5;
  p.angle_bins = 36;
  p.primitive_length_m = 0.9;
  p.analytic_ratio = 2.0;
  p.analytic_max_length_m = 8.0;
  p.max_expansions = 12345;
  p.allow_reverse = false;
  p.change_penalty = 5.0;
  const auto r = roundTrip(p);
  EXPECT_DOUBLE_EQ(r.cell_size_m, p.cell_size_m);
  EXPECT_EQ(r.angle_bins, p.angle_bins);
  EXPECT_DOUBLE_EQ(r.primitive_length_m, p.primitive_length_m);
  EXPECT_DOUBLE_EQ(r.analytic_ratio, p.analytic_ratio);
  EXPECT_DOUBLE_EQ(r.analytic_max_length_m, p.analytic_max_length_m);
  EXPECT_EQ(r.max_expansions, p.max_expansions);
  EXPECT_EQ(r.allow_reverse, p.allow_reverse);
  EXPECT_DOUBLE_EQ(r.change_penalty, p.change_penalty);
  // Own scope in RunConfig; the planner type string round-trips.
  MoD::RunConfig c;
  c.planner.type = MoD::PlannerType::hybrid_astar;
  c.hybrid_astar = p;
  const nlohmann::json j = c;
  EXPECT_TRUE(j.contains("HybridAStarParameters"));
  EXPECT_EQ(j.at("PlannerParameters").at("type").get<std::string>(), "hybrid_astar");
  const auto rc = j.get<MoD::RunConfig>();
  EXPECT_EQ(rc.planner.type, MoD::PlannerType::hybrid_astar);
  EXPECT_DOUBLE_EQ(rc.hybrid_astar.change_penalty, 5.0);
  EXPECT_EQ(MoD::plannerTypeFromString("hybrid_astar"), MoD::PlannerType::hybrid_astar);
}

TEST(Parameters, ScenarioRoundTrip) {
  MoD::Scenario p;
  p.name = "atc-scenario1";
  p.map_yaml = "atc.yaml";
  p.start = {47.69, -18.848, -2.356};
  p.goal = {-19.575, 12.39, 2.313};
  const auto r = roundTrip(p);
  EXPECT_EQ(r.name, p.name);
  EXPECT_EQ(r.map_yaml, p.map_yaml);
  EXPECT_EQ(r.start, p.start);
  EXPECT_EQ(r.goal, p.goal);
}

TEST(Parameters, RunMetaRoundTrip) {
  MoD::RunMeta p{"2.0.0", "abc123", "host", "2026-09-19T08:00:00.000Z"};
  const auto r = roundTrip(p);
  EXPECT_EQ(r.mod_version, p.mod_version);
  EXPECT_EQ(r.git_hash, p.git_hash);
  EXPECT_EQ(r.hostname, p.hostname);
  EXPECT_EQ(r.started_at, p.started_at);
}

TEST(Parameters, RunConfigRoundTripAndScopes) {
  MoD::RunConfig c;
  c.vehicle.shape = MoD::Shape::circle;
  c.sampler.type = MoD::SamplerType::dijkstra;
  c.objective.type = MoD::ObjectiveType::gmmt;
  c.planner.type = MoD::PlannerType::ait_star;
  c.scenario.name = "s";
  c.meta.hostname = "h";
  const json j = c;
  for (const char *scope : {"VehicleParameters", "Derived", "SamplerParameters", "OptObjParameters",
                            "PlannerParameters", "Scenario", "RunMeta"}) {
    EXPECT_TRUE(j.contains(scope)) << scope;
    EXPECT_TRUE(j.at(scope).is_object()) << scope;
  }
  const auto r = roundTrip(c);
  EXPECT_EQ(r.vehicle.shape, MoD::Shape::circle);
  EXPECT_EQ(r.sampler.type, MoD::SamplerType::dijkstra);
  EXPECT_EQ(r.objective.type, MoD::ObjectiveType::gmmt);
  EXPECT_EQ(r.planner.type, MoD::PlannerType::ait_star);
  EXPECT_EQ(r.scenario.name, "s");
  EXPECT_EQ(r.meta.hostname, "h");
}

TEST(Parameters, PartialJsonKeepsDefaults) {
  const auto p = json::parse(R"({"type": "intensity", "bias": 0.2})").get<MoD::SamplerParameters>();
  EXPECT_EQ(p.type, MoD::SamplerType::intensity);
  EXPECT_DOUBLE_EQ(p.bias, 0.2);
  EXPECT_DOUBLE_EQ(p.dijkstra_cell_size, 0.5);
  EXPECT_FALSE(p.log_samples);
}

TEST(Parameters, EnumStrings) {
  EXPECT_EQ(MoD::to_string(MoD::SamplerType::dijkstra), "dijkstra");
  EXPECT_EQ(MoD::samplerTypeFromString("hybrid"), MoD::SamplerType::hybrid);
  EXPECT_EQ(MoD::objectiveTypeFromString("path_length"), MoD::ObjectiveType::path_length);
  EXPECT_EQ(MoD::plannerTypeFromString("ait_star"), MoD::PlannerType::ait_star);
  EXPECT_EQ(MoD::stateSpaceFromString("reeds_shepp"), MoD::StateSpaceType::reeds_shepp);
  EXPECT_EQ(MoD::shapeFromString("rectangle"), MoD::Shape::rectangle);
  EXPECT_THROW(MoD::samplerTypeFromString("bogus"), std::invalid_argument);
  EXPECT_THROW(json::parse(R"({"type": "bogus"})").get<MoD::PlannerParameters>(), std::invalid_argument);
}

TEST(Parameters, DefaultsArePaperIV) {
  MoD::SamplerParameters s;
  EXPECT_DOUBLE_EQ(s.bias, 0.05);
  EXPECT_DOUBLE_EQ(s.dijkstra_cell_size, 0.5);
  EXPECT_DOUBLE_EQ(s.hybrid_intensity_bias, 0.01);
  MoD::OptObjParameters o;
  EXPECT_DOUBLE_EQ(o.w_d, 1.0);
  EXPECT_DOUBLE_EQ(o.w_q, 1.0);
  EXPECT_DOUBLE_EQ(o.max_vehicle_speed, 1.0);
  EXPECT_DOUBLE_EQ(o.mahalanobis_threshold, 10.0);
  EXPECT_TRUE(o.use_mixing_factor);
  MoD::PlannerParameters pl;
  EXPECT_DOUBLE_EQ(pl.goal_bias, 0.05);
  EXPECT_EQ(pl.batch_size, 100u);
  EXPECT_TRUE(pl.informed_sampling);
  MoD::VehicleParameters v;
  EXPECT_EQ(v.state_space, MoD::StateSpaceType::dubins);
  EXPECT_DOUBLE_EQ(v.turning_radius, 1.0);
}
