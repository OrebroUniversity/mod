# ImGui app: mod_playground_gui. ImGui is a git submodule (third_party/imgui, MIT), GLFW from the system.
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/third_party/imgui)
if(NOT EXISTS ${IMGUI_DIR}/imgui.cpp)
  message(FATAL_ERROR "ImGui submodule missing: run `git submodule update --init` (or set MOD_BUILD_PLAYGROUND=OFF)")
endif()

add_library(mod_imgui STATIC
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(mod_imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends ${IMGUI_DIR}/misc/cpp)
target_link_libraries(mod_imgui PUBLIC glfw OpenGL::GL)
target_compile_options(mod_imgui PRIVATE -w)

add_executable(mod_playground_gui main.cpp app.cpp)
target_link_libraries(mod_playground_gui PRIVATE mod::playground mod_imgui Threads::Threads)
target_compile_definitions(mod_playground_gui PRIVATE MOD_MAPS_DIR="${CMAKE_SOURCE_DIR}/maps")
target_compile_options(mod_playground_gui PRIVATE -Wall -Wextra)
