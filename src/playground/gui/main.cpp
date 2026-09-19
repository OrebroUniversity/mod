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

/// mod playground GUI: `mod_playground_gui [--config run.json] [--map map.yaml] [--log-dir DIR] [--maps-dir DIR]
/// [--solve] [--exit-after-solve] [--screenshot out.ppm]`. `--maps-dir` defaults to the source tree's `maps/`.

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

#include "gui/app.hpp"

namespace {

void writePpm(const std::string &file, int w, int h) {
  std::vector<unsigned char> rgb(static_cast<size_t>(w) * static_cast<size_t>(h) * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
  std::ofstream out(file, std::ios::binary);
  out << "P6\n" << w << " " << h << "\n255\n";
  for (int y = h - 1; y >= 0; --y)
    out.write(reinterpret_cast<const char *>(&rgb[static_cast<size_t>(y) * static_cast<size_t>(w) * 3]),
              static_cast<std::streamsize>(w) * 3);
}

}  // namespace

int main(int argc, char **argv) {
  MoD::playground::gui::Options options;
  options.maps_dir = MOD_MAPS_DIR;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--config")
      options.config_file = next();
    else if (a == "--map")
      options.map_yaml = next();
    else if (a == "--log-dir")
      options.log_dir = next();
    else if (a == "--maps-dir")
      options.maps_dir = next();
    else if (a == "--solve")
      options.solve_on_start = true;
    else if (a == "--exit-after-solve")
      options.exit_after_solve = true;
    else if (a == "--screenshot")
      options.screenshot = next();
    else {
      std::fprintf(stderr,
                   "usage: %s [--config run.json] [--map map.yaml] [--log-dir DIR] [--maps-dir DIR] [--solve] "
                   "[--exit-after-solve] [--screenshot out.ppm]\n",
                   argv[0]);
      return 2;
    }
  }

  if (!glfwInit()) {
    std::fprintf(stderr, "glfwInit failed\n");
    return 1;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  GLFWwindow *window = glfwCreateWindow(1600, 900, "mod playground", nullptr, nullptr);
  if (!window) {
    std::fprintf(stderr, "glfwCreateWindow failed\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 130");

  int exit_code = 0;
  {
    MoD::playground::gui::App app(options);
    while (!glfwWindowShouldClose(window)) {
      glfwPollEvents();
      int w, h;
      glfwGetFramebufferSize(window, &w, &h);
      if (w == 0 || h == 0) {
        glfwWaitEventsTimeout(0.1);
        continue;
      }
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();
      const bool keep_going = app.frame(w, h);
      ImGui::Render();
      glViewport(0, 0, w, h);
      glClearColor(0.2f, 0.2f, 0.22f, 1.f);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      if (app.wantsScreenshot()) {
        writePpm(app.screenshotFile(), w, h);
        app.screenshotTaken();
        std::printf("screenshot written to %s\n", app.screenshotFile().c_str());
        if (!options.solve_on_start) glfwSetWindowShouldClose(window, GLFW_TRUE);
      }
      glfwSwapBuffers(window);
      if (!keep_going) glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return exit_code;
}
