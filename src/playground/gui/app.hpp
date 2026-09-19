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

#include <imgui.h>

#include <array>
#include <atomic>
#include <memory>
#include <mod/parameters.hpp>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "core/map_cache.hpp"
#include "core/occupancy_map.hpp"
#include "core/solver.hpp"

namespace MoD::playground::gui {

/// World <-> screen transform: y up in the world, down on the screen. Pan and zoom about the cursor.
struct Camera {
  double scale{8.0};  ///< pixels per metre
  ImVec2 origin{0.f, 0.f};  ///< screen position of the world origin

  ImVec2 toScreen(double x, double y) const {
    return ImVec2(static_cast<float>(origin.x + x * scale), static_cast<float>(origin.y - y * scale));
  }
  std::array<double, 2> toWorld(ImVec2 p) const { return {(p.x - origin.x) / scale, (origin.y - p.y) / scale}; }
  void zoomAbout(ImVec2 cursor, double factor) {
    const auto w = toWorld(cursor);
    scale *= factor;
    origin.x = static_cast<float>(cursor.x - w[0] * scale);
    origin.y = static_cast<float>(cursor.y + w[1] * scale);
  }
  void fit(const Bounds &b, ImVec2 view_min, ImVec2 view_max, float margin = 20.f) {
    const double w = view_max.x - view_min.x - 2 * margin, h = view_max.y - view_min.y - 2 * margin;
    scale = std::min(w / (b.x_max - b.x_min), h / (b.y_max - b.y_min));
    const double cx = 0.5 * (b.x_min + b.x_max), cy = 0.5 * (b.y_min + b.y_max);
    const ImVec2 centre((view_min.x + view_max.x) * 0.5f, (view_min.y + view_max.y) * 0.5f);
    origin.x = static_cast<float>(centre.x - cx * scale);
    origin.y = static_cast<float>(centre.y + cy * scale);
  }
};

/// Result of one solve, produced on the worker thread and drawn on the UI thread.
struct SolveResult {
  ::MoD::RunConfig config;
  Solution solution;
  std::string run_dir;
  std::string error;
  std::vector<std::array<float, 4>> tree_edges;   ///< x1, y1, x2, y2 from PlannerData
  std::vector<std::array<float, 2>> path_dense;   ///< interpolated solution path
  /// Hybrid A*: closed nodes (x, y, heading bin, reverse flag) in expansion order.
  struct Expanded {
    float x, y;
    unsigned int bin;
    bool reverse;
  };
  std::vector<Expanded> expanded;
  unsigned int angle_bins{72};
};

struct Options {
  std::string config_file;
  std::string map_yaml;
  std::string log_dir{"runs"};
  bool solve_on_start{false};
  bool exit_after_solve{false};
  std::string screenshot;  ///< PPM written after the solve (or first frame if not solving)
};

class App {
 public:
  explicit App(Options options);
  ~App();

  /// One frame: panel, canvas input and drawing. Returns false when the app wants to quit.
  bool frame(int fb_width, int fb_height);
  bool wantsScreenshot() const { return screenshot_pending_; }
  void screenshotTaken() { screenshot_pending_ = false; }
  const std::string &screenshotFile() const { return options_.screenshot; }

 private:
  // --- state ---
  Options options_;
  ::MoD::RunConfig config_;
  std::string log_dir_;
  MapCache maps_;
  OccupancyMapConstPtr occupancy_;
  unsigned int map_texture_{0};
  Camera camera_;
  bool camera_fitted_{false};
  std::string status_;

  // overlays
  bool show_cliff_{false}, show_gmmt_{false}, show_intensity_{false}, show_tree_{true}, show_path_{true},
      show_expanded_{true};
  struct Arrow {
    float x, y, dx, dy;
  };
  std::vector<Arrow> cliff_arrows_;
  std::string cliff_loaded_;
  std::vector<std::vector<ImVec2>> gmmt_polylines_;  // world coordinates stored as ImVec2
  std::vector<float> gmmt_weights_;
  std::string gmmt_loaded_;
  struct Cell {
    float x, y, half, q;
  };
  std::vector<Cell> intensity_cells_;
  std::string intensity_loaded_;

  // interaction
  enum class Placing { none, start, goal } placing_{Placing::none};
  ImVec2 place_anchor_;

  // solving
  std::thread worker_;
  std::atomic<bool> solving_{false};
  std::mutex mutex_;
  std::shared_ptr<Solver> solver_;
  std::shared_ptr<SolveResult> result_;
  bool screenshot_pending_{false};
  bool quit_{false};

  // --- steps ---
  void loadMap(const std::string &yaml);
  void uploadTexture();
  void loadCliffOverlay();
  void loadGmmtOverlay();
  void loadIntensityOverlay();
  void startSolve();
  void cancelSolve();
  void joinWorker();

  void drawPanel(float height);
  void handleCanvasInput(ImVec2 view_min, ImVec2 view_max);
  void drawCanvas(ImVec2 view_min, ImVec2 view_max);
  void drawArrow(ImDrawList *dl, double x, double y, double heading, double length_m, ImU32 colour,
                 float thickness) const;
};

}  // namespace MoD::playground::gui
