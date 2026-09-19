# mod — Maps of Dynamics for motion planning

`mod` is a C++17 library of Maps-of-Dynamics (MoD) representations (CLiFF-map, GMMT-map, intensity map) and the
OMPL optimization objectives and informed samplers that use them (Down-The-CLiFF, upstream criterion, intensity,
Dijkstra / intensity / hybrid samplers), plus a *playground* for experiments: a headless batch runner with JSON run
logs, an ImGui app, and Python analysis scripts. Version 2.0.0 replaces the bench-mr based experiment pipeline of
the earlier papers; see `CHANGELOG.rst` for what changed and `AI-PLANS/` for the implementation plans.

## Build

Host build (Fedora, OMPL 2.0, Boost, Eigen, nlohmann_json, gtest, glfw):

```bash
git submodule update --init          # third_party/imgui (GUI only)
cmake -S . -B build
cmake --build build -j
ctest --test-dir build
```

Outputs: `build/lib/libmod.so`, tools in `build/bin` (`run_batch`, `check_interpolation`, `mod_playground_gui`),
tests in `build/bin/tests`. `-DMOD_BUILD_PLAYGROUND=OFF` builds the library alone.

## Use

- Library: link `mod::mod`; construct an objective from `MoD::OptObjParameters` + `MoD::SamplerParameters`
  (`include/mod/parameters.hpp`) and set it on an OMPL problem definition; the objective allocates the informed
  sampler the planner asks for.
- Batch: `./build/bin/run_batch test/data/atc/batch_atc_smoke.json --threads 4 --log-dir runs/atc`; one folder per run
  with `config.json`, `solution.json`, optional `samples.json`.
- GUI: `./build/bin/mod_playground_gui --map test/data/atc/atc.yaml`.
- Plots: see `analysis/README.md`.

## Acknowledgements

The experiment pipeline in `src/playground` and `analysis` is inspired by, and partly modelled on,
[bench-mr](https://github.com/robot-motion/bench-mr) (MIT, Eric Heiden and contributors) and the MoD additions in
[ksatyaki/bench-mr](https://github.com/ksatyaki/bench-mr), which ran the published experiments. No bench-mr source is
included, but the following are derived from it and are marked as such in the files:

- the ATC maps and the six start/goal pairs under `test/data/atc` (copied from bench-mr `maps/` and
  `python/sg-pairs-atc.yaml`); the occupancy map itself comes from the ATC pedestrian dataset (Brščić et al., 2013);
- the objective-per-type wiring and the Paper IV parameter defaults (`src/playground/core/planner_factory.cpp`,
  `include/mod/parameters.hpp`), from bench-mr `src/base/src/PlannerSettings.cpp`;
- timing solutions through OMPL's intermediate-solution callback (`src/playground/core/solver.cpp`), from bench-mr
  `src/planners/OMPLPlanner.hpp`;
- the scenario x planner x sampler x objective batch expansion (`src/playground/core/batch.cpp`), from bench-mr
  `python/MoD-planning.py`;
- the success-rate-versus-time and cost figures (`analysis/plot_success.py`, `analysis/plot_cost.py`), from bench-mr
  `python/plot_convergence_mod.py` and `plot_stats.py`.

Third-party code and data are listed with their licences in `3rd_party_licenses.md`. `mod` itself is LGPL-3.0
(`LICENSE`).
