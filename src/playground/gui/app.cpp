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
#include <fstream>
#include <mod/log.hpp>

#include "core/planner_factory.hpp"
#include "core/run_logger.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

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
  if (!options_.map_yaml.empty()) config_.scenario.map_yaml = options_.map_yaml;
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
// Loading

void App::loadMap(const std::string &yaml) {
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
  const std::string &file = config_.objective.cliff_map_file;
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
  const std::string &file = config_.objective.gmmt_map_file;
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
    ImGui::InputText("map yaml", &config_.scenario.map_yaml);
    ImGui::SameLine();
    if (ImGui::Button("Load")) loadMap(config_.scenario.map_yaml);
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
    enumCombo("planner", config_.planner.type, {::MoD::PlannerType::rrt_star, ::MoD::PlannerType::ait_star});
    ImGui::InputDouble("max time [s]", &config_.planner.max_planning_time, 1.0, 10.0, "%.1f");
    int seed = static_cast<int>(config_.planner.seed);
    if (ImGui::InputInt("seed", &seed)) config_.planner.seed = static_cast<unsigned int>(std::max(0, seed));
    if (config_.planner.type == ::MoD::PlannerType::rrt_star) {
      ImGui::InputDouble("range [m] (0=auto)", &config_.planner.range, 0.5, 1.0, "%.2f");
      ImGui::SliderScalar("goal bias", ImGuiDataType_Double, &config_.planner.goal_bias, &kZero, &kOne, "%.3f");
      ImGui::Checkbox("informed sampling", &config_.planner.informed_sampling);
    } else {
      int batch = static_cast<int>(config_.planner.batch_size);
      if (ImGui::InputInt("batch size", &batch)) config_.planner.batch_size = static_cast<unsigned int>(std::max(1, batch));
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
    ImGui::InputText("cliff map", &config_.objective.cliff_map_file);
    ImGui::InputText("gmmt map", &config_.objective.gmmt_map_file);
    ImGui::InputText("intensity map", &config_.objective.intensity_map_file);
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
    ImGui::InputText("sampler q map", &config_.sampler.intensity_map_file);
    ImGui::Checkbox("log samples", &config_.sampler.log_samples);
  }

  if (ImGui::CollapsingHeader("Overlays", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Checkbox("CLiFF mean arrows", &show_cliff_) && show_cliff_) loadCliffOverlay();
    if (ImGui::Checkbox("GMMT cluster polylines", &show_gmmt_) && show_gmmt_) loadGmmtOverlay();
    if (ImGui::Checkbox("intensity heat", &show_intensity_) && show_intensity_) loadIntensityOverlay();
    ImGui::Checkbox("planner tree", &show_tree_);
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
