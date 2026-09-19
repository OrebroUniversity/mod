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

#include "gui/app.hpp"

#include <GL/gl.h>
#include <imgui_stdlib.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/RandomNumbers.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mod/log.hpp>

#include "core/planner_factory.hpp"
#include "core/run_logger.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace fs = std::filesystem;

namespace MoD::playground::gui {

namespace {

const ImU32 kStartColour = IM_COL32(40, 200, 60, 255);
const ImU32 kGoalColour = IM_COL32(230, 60, 60, 255);
const ImU32 kPathColour = IM_COL32(30, 90, 255, 255);
const ImU32 kTreeColour = IM_COL32(255, 140, 0, 110);
const ImU32 kCliffColour = IM_COL32(120, 40, 160, 200);
const double kZero = 0.0, kOne = 1.0, kRadiusMin = 0.1, kRadiusMax = 10.0;

template <typename E>
int enumIndex(E v, const std::vector<E> &values) {
  for (size_t i = 0; i < values.size(); ++i)
    if (values[i] == v) return static_cast<int>(i);
  return 0;
}

template <typename E>
bool enumCombo(const char *label, E &value, const std::vector<E> &values) {
  std::string items;
  for (const auto &v : values) items += ::MoD::to_string(v) + '\0';
  int idx = enumIndex(value, values);
  if (ImGui::Combo(label, &idx, items.c_str())) {
    value = values[static_cast<size_t>(idx)];
    return true;
  }
  return false;
}

ImU32 clusterColour(size_t i) {
  const float h = std::fmod(0.61803398875f * static_cast<float>(i), 1.0f);
  float r, g, b;
  ImGui::ColorConvertHSVtoRGB(h, 0.8f, 0.9f, r, g, b);
  return IM_COL32(static_cast<int>(r * 255), static_cast<int>(g * 255), static_cast<int>(b * 255), 220);
}

}  // namespace

App::App(Options options) : options_(std::move(options)), log_dir_(options_.log_dir) {
  config_.scenario.name = "gui";
  if (!options_.config_file.empty()) {
    std::ifstream in(options_.config_file);
    if (!in) {
      status_ = "cannot open " + options_.config_file;
    } else {
      config_ = nlohmann::json::parse(in).get<::MoD::RunConfig>();
      status_ = "loaded " + options_.config_file;
    }
  }
  scanMapsDir();
  if (!options_.map_yaml.empty()) config_.scenario.map_yaml = options_.map_yaml;
  resolveConfigPaths(config_);
  if (!config_.scenario.map_yaml.empty()) loadMap(config_.scenario.map_yaml);
  if (options_.solve_on_start) startSolve();
  if (!options_.screenshot.empty() && !options_.solve_on_start) screenshot_pending_ = true;
}

App::~App() {
  cancelSolve();
  joinWorker();
  if (map_texture_) glDeleteTextures(1, &map_texture_);
}

// ---------------------------------------------------------------------------------------------------------------
// Map files under the maps dir

void App::scanMapsDir() {
  for (auto &list : map_files_) list.clear();
  map_file_count_ = 0;
  const fs::path root(options_.maps_dir);
  std::error_code ec;
  if (options_.maps_dir.empty() || !fs::is_directory(root, ec)) return;
  for (const auto &entry : fs::recursive_directory_iterator(root, fs::directory_options::skip_permission_denied, ec)) {
    if (!entry.is_regular_file(ec)) continue;
    const fs::path &p = entry.path();
    const std::string ext = p.extension().string();
    FileKind kind;
    if (ext == ".yaml" || ext == ".yml") {
      kind = FileKind::occupancy;
    } else if (ext == ".xml") {
      // Sniff the head: GMMT has <M>/<clusters>, intensity <cell_size>, CLiFF <map version=...>.
      std::ifstream in(p);
      std::string head(512, '\0');
      in.read(head.data(), static_cast<std::streamsize>(head.size()));
      head.resize(static_cast<size_t>(std::max<std::streamsize>(0, in.gcount())));
      if (head.find("<clusters>") != std::string::npos || head.find("<M>") != std::string::npos)
        kind = FileKind::gmmt;
      else if (head.find("<cell_size>") != std::string::npos)
        kind = FileKind::intensity;
      else if (head.find("<map version") != std::string::npos)
        kind = FileKind::cliff;
      else
        continue;
    } else {
      continue;
    }
    map_files_[static_cast<size_t>(kind)].push_back({p.lexically_relative(root).generic_string(), p.string()});
    ++map_file_count_;
  }
  for (auto &list : map_files_)
    std::sort(list.begin(), list.end(), [](const MapFile &a, const MapFile &b) { return a.rel < b.rel; });
  MOD_LOG("GUI: %zu map files under %s (%zu yaml, %zu cliff, %zu gmmt, %zu intensity)", map_file_count_,
          options_.maps_dir.c_str(), map_files_[0].size(), map_files_[1].size(), map_files_[2].size(),
          map_files_[3].size());
}

std::string App::resolvePath(const std::string &path) const {
  if (path.empty() || options_.maps_dir.empty()) return path;
  std::error_code ec;
  if (fs::exists(path, ec)) return path;
  const fs::path p(path);
  if (p.is_absolute()) return path;
  const fs::path under = fs::path(options_.maps_dir) / p;
  if (fs::exists(under, ec)) return under.lexically_normal().string();
  const std::string name = p.filename().string();
  for (const auto &list : map_files_)
    for (const auto &f : list)
      if (fs::path(f.abs).filename().string() == name) return f.abs;
  return path;
}

void App::resolveConfigPaths(::MoD::RunConfig &config) const {
  config.scenario.map_yaml = resolvePath(config.scenario.map_yaml);
  config.objective.cliff_map_file = resolvePath(config.objective.cliff_map_file);
  config.objective.gmmt_map_file = resolvePath(config.objective.gmmt_map_file);
  config.objective.intensity_map_file = resolvePath(config.objective.intensity_map_file);
  config.sampler.intensity_map_file = resolvePath(config.sampler.intensity_map_file);
}

bool App::fileField(const char *label, std::string &value, FileKind kind) {
  const std::string id = std::string("##") + label;
  bool changed = false;
  ImGui::SetNextItemWidth(-170.f);
  if (ImGui::InputText(id.c_str(), &value, ImGuiInputTextFlags_EnterReturnsTrue)) changed = true;
  ImGui::SameLine();
  const std::string popup = id + "_picker";
  if (ImGui::Button((std::string("v") + id).c_str())) ImGui::OpenPopup(popup.c_str());
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("pick from %s", options_.maps_dir.c_str());
  if (ImGui::BeginPopup(popup.c_str())) {
    const auto &files = map_files_[static_cast<size_t>(kind)];
    if (files.empty()) ImGui::TextDisabled("no files found under %s", options_.maps_dir.c_str());
    for (const auto &f : files) {
      if (ImGui::Selectable(f.rel.c_str(), f.abs == value)) {
        value = f.abs;
        changed = true;
      }
    }
    if (ImGui::Selectable("(clear)", false)) {
      value.clear();
      changed = true;
    }
    ImGui::EndPopup();
  }
  ImGui::SameLine();
  ImGui::TextUnformatted(label);
  return changed;
}

// ---------------------------------------------------------------------------------------------------------------
// Loading

void App::loadMap(const std::string &yaml_in) {
  const std::string yaml = resolvePath(yaml_in);
  try {
    occupancy_ = maps_.occupancy(yaml);
    config_.scenario.map_yaml = yaml;
    uploadTexture();
    camera_fitted_ = false;
    status_ = "loaded " + yaml;
    // Default start / goal inside the map if they are at the origin of an empty config.
    const Bounds b = occupancy_->bounds();
    auto inside = [&](const std::array<double, 3> &p) {
      return p[0] > b.x_min && p[0] < b.x_max && p[1] > b.y_min && p[1] < b.y_max;
    };
    if (!inside(config_.scenario.start))
      config_.scenario.start = {b.x_min + 0.25 * (b.x_max - b.x_min), 0.5 * (b.y_min + b.y_max), 0.0};
    if (!inside(config_.scenario.goal))
      config_.scenario.goal = {b.x_min + 0.75 * (b.x_max - b.x_min), 0.5 * (b.y_min + b.y_max), 0.0};
  } catch (const std::exception &e) {
    status_ = std::string("map: ") + e.what();
  }
}

void App::uploadTexture() {
  if (!occupancy_) return;
  const size_t w = occupancy_->width(), h = occupancy_->height();
  std::vector<unsigned char> rgba(w * h * 4);
  const auto &gray = occupancy_->gray();
  for (size_t img_row = 0; img_row < h; ++img_row) {
    for (size_t col = 0; col < w; ++col) {
      unsigned char v;
      if (!gray.empty()) {
        v = gray[img_row * w + col];
      } else {
        const size_t row = h - 1 - img_row;
        v = occupancy_->occupancy()[row * w + col] ? 0 : 255;
      }
      unsigned char *px = &rgba[(img_row * w + col) * 4];
      px[0] = px[1] = px[2] = v;
      px[3] = 255;
    }
  }
  if (!map_texture_) glGenTextures(1, &map_texture_);
  glBindTexture(GL_TEXTURE_2D, map_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, static_cast<GLsizei>(w), static_cast<GLsizei>(h), 0, GL_RGBA,
               GL_UNSIGNED_BYTE, rgba.data());
}

void App::loadCliffOverlay() {
  const std::string file = resolvePath(config_.objective.cliff_map_file);
  if (file.empty() || file == cliff_loaded_) return;
  try {
    auto map = maps_.cliff(file);
    cliff_arrows_.clear();
    for (const auto &loc : map->getLocations()) {
      for (const auto &d : loc.distributions) {
        const float len = static_cast<float>(0.8 * d.getMixingFactor() * map->getResolution());
        cliff_arrows_.push_back({static_cast<float>(loc.position[0]), static_cast<float>(loc.position[1]),
                                 static_cast<float>(len * std::cos(d.getMeanHeading())),
                                 static_cast<float>(len * std::sin(d.getMeanHeading()))});
      }
    }
    cliff_loaded_ = file;
    status_ = "loaded " + file;
  } catch (const std::exception &e) {
    status_ = std::string("cliff: ") + e.what();
    show_cliff_ = false;
  }
}

void App::loadGmmtOverlay() {
  const std::string file = resolvePath(config_.objective.gmmt_map_file);
  if (file.empty() || file == gmmt_loaded_) return;
  try {
    auto map = maps_.gmmt(file);
    gmmt_polylines_.clear();
    gmmt_weights_.clear();
    for (const auto &c : map->getClusters()) {
      std::vector<ImVec2> line;
      for (const auto &m : c.mean) line.emplace_back(static_cast<float>(m[0]), static_cast<float>(m[1]));
      gmmt_polylines_.push_back(std::move(line));
      gmmt_weights_.push_back(static_cast<float>(c.mixing_factor));
    }
    gmmt_loaded_ = file;
    status_ = "loaded " + file;
  } catch (const std::exception &e) {
    status_ = std::string("gmmt: ") + e.what();
    show_gmmt_ = false;
  }
}

void App::loadIntensityOverlay() {
  std::string file = config_.objective.intensity_map_file;
  if (file.empty()) file = config_.sampler.intensity_map_file;
  file = resolvePath(file);
  if (file.empty() || file == intensity_loaded_) return;
  try {
    auto map = maps_.intensity(file);
    intensity_cells_.clear();
    const float half = static_cast<float>(map->getCellSize() / 2.0);
    for (size_t i = 0; i < map->getRows() * map->getColumns(); ++i) {
      const double q = map->valueAt(i);
      if (q <= 0.0) continue;
      const auto xy = map->getXYatIndex(i);
      intensity_cells_.push_back({static_cast<float>(xy[0]), static_cast<float>(xy[1]), half, static_cast<float>(q)});
    }
    intensity_loaded_ = file;
    status_ = "loaded " + file;
  } catch (const std::exception &e) {
    status_ = std::string("intensity: ") + e.what();
    show_intensity_ = false;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Solving on a worker thread

void App::joinWorker() {
  if (worker_.joinable()) worker_.join();
}

void App::cancelSolve() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (solver_) solver_->cancel();
}

void App::startSolve() {
  if (solving_) return;
  joinWorker();
  if (!occupancy_) {
    status_ = "load a map first";
    return;
  }
  solving_ = true;
  status_ = "solving...";
  resolveConfigPaths(config_);  // config.json records the resolved paths
  ::MoD::RunConfig config = config_;
  const std::string log_dir = log_dir_;
  worker_ = std::thread([this, config, log_dir]() mutable {
    auto result = std::make_shared<SolveResult>();
    try {
      ompl::RNG::setSeed(config.planner.seed == 0 ? 1 : config.planner.seed);
      PlannerSetup setup = PlannerFactory::build(config, maps_);
      RunLogger logger(log_dir, config);
      result->run_dir = logger.dir();
      if (setup.mod_objective) setup.mod_objective->setSampleSink(logger.sampleSink());
      auto solver = std::make_shared<Solver>(setup);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        solver_ = solver;
      }
      result->solution = solver->solve(config.planner.max_planning_time);
      logger.writeSolution(result->solution);
      logger.finish();
      result->config = config;

      if (setup.planner) {
        ob::PlannerData pd(setup.si);
        setup.planner->getPlannerData(pd);
        std::vector<unsigned int> edges;
        for (unsigned int v = 0; v < pd.numVertices(); ++v) {
          const auto *a = pd.getVertex(v).getState()->as<ob::SE2StateSpace::StateType>();
          edges.clear();
          pd.getEdges(v, edges);
          for (unsigned int w : edges) {
            const auto *b = pd.getVertex(w).getState()->as<ob::SE2StateSpace::StateType>();
            result->tree_edges.push_back({static_cast<float>(a->getX()), static_cast<float>(a->getY()),
                                          static_cast<float>(b->getX()), static_cast<float>(b->getY())});
          }
        }
      }
      if (setup.hybrid_astar) {
        result->angle_bins = setup.hybrid_astar->parameters().angle_bins;
        for (const auto &n : setup.hybrid_astar->expandedNodes())
          result->expanded.push_back({static_cast<float>(n.x), static_cast<float>(n.y), n.bin,
                                      n.dir == ::MoD::HybridAStar::Direction::reverse});
      }
      if (result->solution.success) {
        auto path = std::dynamic_pointer_cast<og::PathGeometric>(setup.pdef->getSolutionPath());
        if (path) {
          og::PathGeometric dense(*path);
          dense.interpolate();
          for (const auto *s : dense.getStates()) {
            const auto *se2 = s->as<ob::SE2StateSpace::StateType>();
            result->path_dense.push_back({static_cast<float>(se2->getX()), static_cast<float>(se2->getY())});
          }
        }
      }
    } catch (const std::exception &e) {
      result->error = e.what();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    result_ = result;
    solver_.reset();
    solving_ = false;
  });
}

// ---------------------------------------------------------------------------------------------------------------
// UI

bool App::frame(int fb_width, int fb_height) {
  const float panel_width = 400.f;
  const ImVec2 view_min(panel_width, 0.f);
  const ImVec2 view_max(static_cast<float>(fb_width), static_cast<float>(fb_height));

  if (!solving_ && worker_.joinable()) {
    joinWorker();
    std::lock_guard<std::mutex> lock(mutex_);
    if (result_) {
      if (!result_->error.empty()) {
        status_ = "error: " + result_->error;
      } else {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s: cost %.3f, length %.2f m, first solution %.2f s -> %s",
                      result_->solution.success ? "solved" : "no solution", result_->solution.cost_total,
                      result_->solution.path_length_m, result_->solution.time_to_first_solution_s,
                      result_->run_dir.c_str());
        status_ = buf;
      }
      if (!options_.screenshot.empty()) screenshot_pending_ = true;
      if (options_.exit_after_solve) quit_ = true;
    }
  }

  if (occupancy_ && !camera_fitted_) {
    camera_.fit(occupancy_->bounds(), view_min, view_max);
    camera_fitted_ = true;
  }

  drawPanel(static_cast<float>(fb_height));
  handleCanvasInput(view_min, view_max);
  drawCanvas(view_min, view_max);
  return !quit_;
}

void App::drawPanel(float height) {
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2(400.f, height));
  ImGui::Begin("mod playground", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                   ImGuiWindowFlags_NoTitleBar);
  ImGui::PushItemWidth(-140.f);

  if (ImGui::CollapsingHeader("Map & scenario", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (fileField("map yaml", config_.scenario.map_yaml, FileKind::occupancy)) loadMap(config_.scenario.map_yaml);
    if (ImGui::Button("Load")) loadMap(config_.scenario.map_yaml);
    ImGui::SameLine();
    if (ImGui::Button("Rescan maps")) scanMapsDir();
    ImGui::SameLine();
    ImGui::TextDisabled("%zu files under maps dir", map_file_count_);
    ImGui::InputText("scenario name", &config_.scenario.name);
    ImGui::InputDouble("start x", &config_.scenario.start[0], 0, 0, "%.3f");
    ImGui::InputDouble("start y", &config_.scenario.start[1], 0, 0, "%.3f");
    ImGui::InputDouble("start yaw", &config_.scenario.start[2], 0, 0, "%.3f");
    ImGui::InputDouble("goal x", &config_.scenario.goal[0], 0, 0, "%.3f");
    ImGui::InputDouble("goal y", &config_.scenario.goal[1], 0, 0, "%.3f");
    ImGui::InputDouble("goal yaw", &config_.scenario.goal[2], 0, 0, "%.3f");
    ImGui::TextDisabled("hold S / G + left click: set start / goal; drag: heading");
    ImGui::TextDisabled("right drag: pan, wheel: zoom, F: fit");
  }

  if (ImGui::CollapsingHeader("Planner", ImGuiTreeNodeFlags_DefaultOpen)) {
    enumCombo("planner", config_.planner.type,
              {::MoD::PlannerType::rrt_star, ::MoD::PlannerType::ait_star, ::MoD::PlannerType::hybrid_astar});
    ImGui::InputDouble("max time [s]", &config_.planner.max_planning_time, 1.0, 10.0, "%.1f");
    int seed = static_cast<int>(config_.planner.seed);
    if (ImGui::InputInt("seed", &seed)) config_.planner.seed = static_cast<unsigned int>(std::max(0, seed));
    if (config_.planner.type == ::MoD::PlannerType::rrt_star) {
      ImGui::InputDouble("range [m] (0=auto)", &config_.planner.range, 0.5, 1.0, "%.2f");
      ImGui::SliderScalar("goal bias", ImGuiDataType_Double, &config_.planner.goal_bias, &kZero, &kOne, "%.3f");
      ImGui::Checkbox("informed sampling", &config_.planner.informed_sampling);
    } else if (config_.planner.type == ::MoD::PlannerType::ait_star) {
      int batch = static_cast<int>(config_.planner.batch_size);
      if (ImGui::InputInt("batch size", &batch)) config_.planner.batch_size = static_cast<unsigned int>(std::max(1, batch));
    } else {
      auto &h = config_.hybrid_astar;
      ImGui::InputDouble("cell size [m]", &h.cell_size_m, 0.05, 0.25, "%.3f");
      int bins = static_cast<int>(h.angle_bins);
      if (ImGui::InputInt("angle bins", &bins)) h.angle_bins = static_cast<unsigned int>(std::max(1, bins));
      ImGui::InputDouble("primitive [m] (0=cell*sqrt2)", &h.primitive_length_m, 0.05, 0.25, "%.3f");
      ImGui::InputDouble("analytic ratio", &h.analytic_ratio, 0.5, 1.0, "%.2f");
      ImGui::InputDouble("analytic max [m]", &h.analytic_max_length_m, 1.0, 5.0, "%.2f");
      double max_exp = static_cast<double>(h.max_expansions);
      if (ImGui::InputDouble("max expansions", &max_exp, 100000.0, 1000000.0, "%.0f"))
        h.max_expansions = static_cast<size_t>(std::max(1.0, max_exp));
      if (config_.vehicle.state_space == ::MoD::StateSpaceType::reeds_shepp) {
        ImGui::Checkbox("allow reverse", &h.allow_reverse);
        ImGui::InputDouble("change penalty", &h.change_penalty, 10.0, 100.0, "%.1f");
      } else {
        ImGui::TextDisabled("forward only (Dubins); reverse needs Reeds-Shepp");
      }
    }
  }

  if (ImGui::CollapsingHeader("Vehicle", ImGuiTreeNodeFlags_DefaultOpen)) {
    enumCombo("state space", config_.vehicle.state_space,
              {::MoD::StateSpaceType::dubins, ::MoD::StateSpaceType::reeds_shepp});
    ImGui::SliderScalar("turning radius [m]", ImGuiDataType_Double, &config_.vehicle.turning_radius, &kRadiusMin,
                        &kRadiusMax, "%.2f");
    enumCombo("shape", config_.vehicle.shape, {::MoD::Shape::circle, ::MoD::Shape::rectangle});
    if (config_.vehicle.shape == ::MoD::Shape::circle) {
      float r = static_cast<float>(config_.vehicle.radius);
      if (ImGui::SliderFloat("radius [m]", &r, 0.05f, 2.0f)) config_.vehicle.radius = r;
    } else {
      float l = static_cast<float>(config_.vehicle.length), w = static_cast<float>(config_.vehicle.width);
      if (ImGui::SliderFloat("length [m]", &l, 0.1f, 4.0f)) config_.vehicle.length = l;
      if (ImGui::SliderFloat("width [m]", &w, 0.1f, 3.0f)) config_.vehicle.width = w;
    }
    ImGui::Text("circumscribed radius: %.3f m", config_.vehicle.circumscribedRadius());
  }

  if (ImGui::CollapsingHeader("Objective", ImGuiTreeNodeFlags_DefaultOpen)) {
    enumCombo("objective", config_.objective.type,
              {::MoD::ObjectiveType::cliff, ::MoD::ObjectiveType::gmmt, ::MoD::ObjectiveType::dtc,
               ::MoD::ObjectiveType::intensity, ::MoD::ObjectiveType::path_length});
    ImGui::InputDouble("w_d", &config_.objective.w_d, 0.1, 1.0, "%.3f");
    ImGui::InputDouble("w_q", &config_.objective.w_q, 0.1, 1.0, "%.3f");
    ImGui::InputDouble("w_c", &config_.objective.w_c, 0.01, 0.1, "%.3f");
    if (fileField("cliff map", config_.objective.cliff_map_file, FileKind::cliff)) {
      cliff_loaded_.clear();
      if (show_cliff_) loadCliffOverlay();
    }
    if (fileField("gmmt map", config_.objective.gmmt_map_file, FileKind::gmmt)) {
      gmmt_loaded_.clear();
      if (show_gmmt_) loadGmmtOverlay();
    }
    if (fileField("intensity map", config_.objective.intensity_map_file, FileKind::intensity)) {
      intensity_loaded_.clear();
      if (show_intensity_) loadIntensityOverlay();
    }
    if (config_.objective.type == ::MoD::ObjectiveType::dtc) {
      ImGui::InputDouble("max speed", &config_.objective.max_vehicle_speed, 0.1, 1.0, "%.2f");
      ImGui::InputDouble("mahalanobis thr.", &config_.objective.mahalanobis_threshold, 1.0, 5.0, "%.1f");
      ImGui::Checkbox("use mixing factor", &config_.objective.use_mixing_factor);
    }
  }

  if (ImGui::CollapsingHeader("Sampler", ImGuiTreeNodeFlags_DefaultOpen)) {
    enumCombo("sampler", config_.sampler.type,
              {::MoD::SamplerType::iid, ::MoD::SamplerType::ellipse, ::MoD::SamplerType::intensity,
               ::MoD::SamplerType::dijkstra, ::MoD::SamplerType::hybrid});
    ImGui::SliderScalar("bias", ImGuiDataType_Double, &config_.sampler.bias, &kZero, &kOne, "%.3f");
    ImGui::InputDouble("dijkstra cell [m]", &config_.sampler.dijkstra_cell_size, 0.1, 0.5, "%.2f");
    if (config_.sampler.type == ::MoD::SamplerType::hybrid)
      ImGui::InputDouble("intensity bias", &config_.sampler.hybrid_intensity_bias, 0.01, 0.05, "%.3f");
    if (fileField("sampler q map", config_.sampler.intensity_map_file, FileKind::intensity)) {
      intensity_loaded_.clear();
      if (show_intensity_) loadIntensityOverlay();
    }
    ImGui::Checkbox("log samples", &config_.sampler.log_samples);
  }

  if (ImGui::CollapsingHeader("Overlays", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Checkbox("CLiFF mean arrows", &show_cliff_) && show_cliff_) loadCliffOverlay();
    if (ImGui::Checkbox("GMMT cluster polylines", &show_gmmt_) && show_gmmt_) loadGmmtOverlay();
    if (ImGui::Checkbox("intensity heat", &show_intensity_) && show_intensity_) loadIntensityOverlay();
    ImGui::Checkbox("planner tree", &show_tree_);
    ImGui::Checkbox("expanded nodes (Hybrid A*)", &show_expanded_);
    ImGui::Checkbox("solution path", &show_path_);
  }

  ImGui::Separator();
  ImGui::InputText("log dir", &log_dir_);
  if (solving_) {
    if (ImGui::Button("Cancel", ImVec2(-1, 0))) cancelSolve();
  } else {
    if (ImGui::Button("Solve", ImVec2(-1, 0))) startSolve();
  }
  ImGui::TextWrapped("%s", status_.c_str());
  ImGui::PopItemWidth();
  ImGui::End();
}

void App::handleCanvasInput(ImVec2 view_min, ImVec2 view_max) {
  ImGuiIO &io = ImGui::GetIO();
  const ImVec2 mouse = io.MousePos;
  const bool over_canvas = !io.WantCaptureMouse && mouse.x >= view_min.x && mouse.x < view_max.x &&
                           mouse.y >= view_min.y && mouse.y < view_max.y;

  if (over_canvas && io.MouseWheel != 0.f) camera_.zoomAbout(mouse, std::pow(1.15, io.MouseWheel));
  if (ImGui::IsMouseDragging(ImGuiMouseButton_Right) && !io.WantCaptureMouse) {
    camera_.origin.x += io.MouseDelta.x;
    camera_.origin.y += io.MouseDelta.y;
  }
  if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_F) && occupancy_) {
    camera_.fit(occupancy_->bounds(), view_min, view_max);
  }

  // Start / goal placement: hold S or G and left-click; drag while held sets the heading.
  if (over_canvas && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureKeyboard) {
    if (ImGui::IsKeyDown(ImGuiKey_S))
      placing_ = Placing::start;
    else if (ImGui::IsKeyDown(ImGuiKey_G))
      placing_ = Placing::goal;
    if (placing_ != Placing::none) {
      place_anchor_ = mouse;
      const auto w = camera_.toWorld(mouse);
      auto &pose = placing_ == Placing::start ? config_.scenario.start : config_.scenario.goal;
      pose[0] = w[0];
      pose[1] = w[1];
    }
  }
  if (placing_ != Placing::none) {
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
      const float dx = mouse.x - place_anchor_.x, dy = mouse.y - place_anchor_.y;
      if (dx * dx + dy * dy > 25.f) {
        auto &pose = placing_ == Placing::start ? config_.scenario.start : config_.scenario.goal;
        pose[2] = std::atan2(-static_cast<double>(dy), static_cast<double>(dx));
      }
    } else {
      placing_ = Placing::none;
    }
  }
}

void App::drawArrow(ImDrawList *dl, double x, double y, double heading, double length_m, ImU32 colour,
                    float thickness) const {
  const ImVec2 a = camera_.toScreen(x, y);
  const ImVec2 b = camera_.toScreen(x + length_m * std::cos(heading), y + length_m * std::sin(heading));
  dl->AddLine(a, b, colour, thickness);
  const double head = std::min(length_m * 0.35, 0.6);
  const ImVec2 l = camera_.toScreen(x + (length_m - head) * std::cos(heading) + head * 0.6 * std::cos(heading + M_PI_2),
                                    y + (length_m - head) * std::sin(heading) + head * 0.6 * std::sin(heading + M_PI_2));
  const ImVec2 r = camera_.toScreen(x + (length_m - head) * std::cos(heading) - head * 0.6 * std::cos(heading + M_PI_2),
                                    y + (length_m - head) * std::sin(heading) - head * 0.6 * std::sin(heading + M_PI_2));
  dl->AddTriangleFilled(b, l, r, colour);
}

void App::drawCanvas(ImVec2 view_min, ImVec2 view_max) {
  ImDrawList *dl = ImGui::GetBackgroundDrawList();
  dl->AddRectFilled(view_min, view_max, IM_COL32(60, 60, 64, 255));
  dl->PushClipRect(view_min, view_max, true);

  if (occupancy_ && map_texture_) {
    const Bounds b = occupancy_->bounds();
    dl->AddImage(static_cast<ImTextureID>(map_texture_), camera_.toScreen(b.x_min, b.y_max),
                 camera_.toScreen(b.x_max, b.y_min));
  }

  if (show_intensity_) {
    for (const auto &c : intensity_cells_) {
      const int alpha = static_cast<int>(std::min(1.f, c.q) * 180.f);
      dl->AddRectFilled(camera_.toScreen(c.x - c.half, c.y + c.half), camera_.toScreen(c.x + c.half, c.y - c.half),
                        IM_COL32(255, 60, 30, alpha));
    }
  }
  if (show_cliff_) {
    for (const auto &a : cliff_arrows_) {
      const double len = std::hypot(a.dx, a.dy);
      if (len < 1e-4) continue;
      drawArrow(dl, a.x, a.y, std::atan2(a.dy, a.dx), len, kCliffColour, 1.5f);
    }
  }
  if (show_gmmt_) {
    std::vector<ImVec2> pts;
    for (size_t i = 0; i < gmmt_polylines_.size(); ++i) {
      pts.clear();
      for (const auto &p : gmmt_polylines_[i]) pts.push_back(camera_.toScreen(p.x, p.y));
      if (pts.size() > 1)
        dl->AddPolyline(pts.data(), static_cast<int>(pts.size()), clusterColour(i), ImDrawFlags_None,
                        1.f + 20.f * gmmt_weights_[i]);
    }
  }

  std::shared_ptr<SolveResult> result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result = result_;
  }
  if (result) {
    if (show_tree_) {
      for (const auto &e : result->tree_edges)
        dl->AddLine(camera_.toScreen(e[0], e[1]), camera_.toScreen(e[2], e[3]), kTreeColour, 1.f);
    }
    if (show_expanded_ && !result->expanded.empty()) {
      // Closed set coloured by heading bin (hue = bin / bins); reverse arrivals as hollow rings.
      const float radius = std::max(1.5f, static_cast<float>(0.08 * camera_.scale));
      const float bins = static_cast<float>(std::max(1u, result->angle_bins));
      for (const auto &n : result->expanded) {
        float r, g, b;
        ImGui::ColorConvertHSVtoRGB(static_cast<float>(n.bin) / bins, 0.85f, 0.95f, r, g, b);
        const ImU32 colour = IM_COL32(static_cast<int>(r * 255), static_cast<int>(g * 255), static_cast<int>(b * 255), 170);
        const ImVec2 c = camera_.toScreen(n.x, n.y);
        if (n.reverse)
          dl->AddCircle(c, radius * 1.4f, colour, 0, 1.5f);
        else
          dl->AddCircleFilled(c, radius, colour);
      }
    }
    if (show_path_ && !result->path_dense.empty()) {
      std::vector<ImVec2> pts;
      pts.reserve(result->path_dense.size());
      for (const auto &p : result->path_dense) pts.push_back(camera_.toScreen(p[0], p[1]));
      dl->AddPolyline(pts.data(), static_cast<int>(pts.size()), kPathColour, ImDrawFlags_None, 3.f);
      for (const auto &p : result->solution.path) dl->AddCircleFilled(camera_.toScreen(p[0], p[1]), 3.f, kPathColour);
    }
  }

  // Start and goal with their footprint circle.
  if (occupancy_) {
    const double r = config_.vehicle.circumscribedRadius();
    const auto &s = config_.scenario.start;
    const auto &g = config_.scenario.goal;
    dl->AddCircle(camera_.toScreen(s[0], s[1]), static_cast<float>(r * camera_.scale), kStartColour, 0, 1.5f);
    drawArrow(dl, s[0], s[1], s[2], std::max(1.5, 2.5 * r), kStartColour, 3.f);
    dl->AddCircle(camera_.toScreen(g[0], g[1]), static_cast<float>(r * camera_.scale), kGoalColour, 0, 1.5f);
    drawArrow(dl, g[0], g[1], g[2], std::max(1.5, 2.5 * r), kGoalColour, 3.f);
  }

  // Cursor position.
  ImGuiIO &io = ImGui::GetIO();
  if (!io.WantCaptureMouse) {
    const auto w = camera_.toWorld(io.MousePos);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "(%.2f, %.2f)", w[0], w[1]);
    dl->AddText(ImVec2(view_min.x + 8.f, view_max.y - 20.f), IM_COL32(255, 255, 255, 220), buf);
  }
  dl->PopClipRect();
}

}  // namespace MoD::playground::gui
