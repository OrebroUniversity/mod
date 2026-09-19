# Getting started

## Dependencies

| Need | Package (Fedora) | Package (Ubuntu 24.04) | Used by |
|---|---|---|---|
| C++17 compiler, CMake ≥ 3.16, git | `gcc-c++ cmake git` | `build-essential cmake git` | everything |
| OMPL 2.0 (brings Boost) | build from source, see below | build from source (the distro OMPL is older) | library |
| Boost headers (property_tree, geometry) | `boost-devel` | `libboost-all-dev` | library |
| Eigen 3 | `eigen3-devel` | `libeigen3-dev` | library |
| nlohmann_json | `json-devel` | `nlohmann-json3-dev` | library |
| GoogleTest | `gtest-devel` | `libgtest-dev` | tests (`MOD_BUILD_TESTS`) |
| GLFW ≥ 3.3, OpenGL | `glfw-devel mesa-libGL-devel` | `libglfw3-dev libgl1-mesa-dev` | GUI (`MOD_BUILD_PLAYGROUND`) |
| Python 3 + numpy, pandas, matplotlib | `python3-numpy python3-pandas python3-matplotlib` | same names | `analysis/` |

The development machine is Fedora with OMPL 2.0 in `/usr/local`, Boost 1.90, Eigen 5, gtest 1.17,
nlohmann_json 3.12, glfw 3.4. Other recent versions should work; OMPL must be 2.x because the Dubins and
Reeds-Shepp spaces' `getPath` API is used.

OMPL from source:

```bash
git clone https://github.com/ompl/ompl.git && cd ompl
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j && sudo cmake --install build
```

## Build

```bash
git clone --recurse-submodules https://github.com/OrebroUniversity/mod.git
cd mod
cmake -S . -B build            # Release by default
cmake --build build -j
ctest --test-dir build
```

If you cloned without `--recurse-submodules`, run `git submodule update --init` (it fetches Dear ImGui for the
GUI), or configure with `-DMOD_BUILD_PLAYGROUND=OFF` to build only the library.

Options:

| Option | Default | Effect |
|---|---|---|
| `MOD_BUILD_TESTS` | `ON` | GoogleTest suite (`ctest`) |
| `MOD_BUILD_PLAYGROUND` | `ON` | batch runner, tools and the ImGui app |
| `CMAKE_BUILD_TYPE` | `Release` | use `RelWithDebInfo` when profiling |

Build outputs (flat tree):

```
build/lib/libmod.so                 the library (SOVERSION 2)
build/lib/libmod_playground.a       playground core (map loader, factory, solver, logger, batch)
build/bin/run_batch                 headless batch runner
build/bin/check_interpolation       diagnostic: what the planners hand to motionCost
build/bin/mod_playground_gui        the app
build/bin/tests/*                   test executables
build/generated/mod/version.h       MOD_VERSION, MOD_GIT_HASH (git describe at configure time)
```

## Install and use from another project

```bash
cmake --install build --prefix ~/.local     # or /usr/local
```

Then in your `CMakeLists.txt`:

```cmake
find_package(mod REQUIRED)          # provides the imported target mod::mod
target_link_libraries(my_planner PRIVATE mod::mod)
```

`mod::mod` carries its include directories and its dependencies (OMPL, Eigen, Boost headers, nlohmann_json).
Adding the repository with `add_subdirectory(mod)` works too. A `package.xml` (`ament_cmake`) is present so the
repository can sit in a ROS 2 workspace; nothing in the code depends on ROS.

## First run

```bash
./build/bin/mod_playground_gui --map maps/atc/atc.yaml
```

or, without a display:

```bash
./build/bin/run_batch maps/atc/batch_atc_smoke.json --threads 4
python3 analysis/runs.py runs/atc_smoke
```

`runs/` is git-ignored; every run folder there is self-describing (`config.json` records the exact settings, the
version and the git hash).

## Troubleshooting

- **`ImGui submodule missing`** at configure time: `git submodule update --init`, or `-DMOD_BUILD_PLAYGROUND=OFF`.
- **`find_package(ompl)` fails**: OMPL's config lives in `<prefix>/share/ompl/cmake`; pass
  `-DCMAKE_PREFIX_PATH=/usr/local` if you installed it elsewhere.
- **A start or goal is reported "not valid"**: the pose's circumscribed circle (see
  [concepts.md](concepts.md#footprint-and-collision-checking)) touches an occupied pixel or leaves the map. Also
  note that OMPL's yaw range is `[-pi, pi)`, so a heading of exactly `pi` must be written as `-pi`.
- **The GUI shows a blank canvas**: load a map first (`--map` or the picker next to "map yaml"); the file
  fields resolve bare names such as `atc.yaml` against `maps/` (or `--maps-dir`).
- **Runs are slow with many threads**: the sampling planners are budget-bound, so they just do fewer iterations
  under load; Hybrid A*'s reported planning time grows with contention (0.05-0.2 s alone on ATC, ~0.6 s with 20
  parallel runs).
