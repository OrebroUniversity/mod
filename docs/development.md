# Development guide

## Layout

```
include/mod/                 library headers: parameters, maps, grid Dijkstra, planners/hybrid_astar, log, sample_sink
include/ompl/mod/            OMPL objectives and samplers (namespace ompl::MoD)
src/                         implementations (mirrors include/); src/planners/hybrid_astar.cpp
src/playground/core/         occupancy map, footprint checker, map cache, planner factory, solver, run logger, batch
src/playground/tools/        run_batch, check_interpolation
src/playground/gui/          the ImGui app (app.hpp/cpp, main.cpp)
test/                        GoogleTest suites (one executable per file) and test_helpers.hpp
analysis/                    Python: runs.py loader and the plot scripts
maps/                        bundled environments (occupancy, MoD maps, scenarios, batches)
docs/                        this documentation
AI-PLANS/                    implementation plans: PLAN.md (2.0), PLAN-hybrid-astar.md (2.1)
cmake/, CMakeModules/        package config template, version header template, uninstall, CPack
third_party/imgui            git submodule, GUI only
```

## Building for development

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`compile_commands.json` is git-ignored; enable it with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`. The library and
every playground target compile with `-Wall -Wextra` and are expected to stay warning-free. `MOD_MAPS_DIR`
(the GUI's default maps folder) and `MOD_TEST_DATA_DIR` are absolute paths into the source tree, baked in at
configure time.

## Tests

| Executable | Covers |
|---|---|
| `parameters_test` | JSON round trip of every struct, enum strings, `RunConfig` scopes |
| `maps_test` | the three ATC MoD maps: bounds, sizes, known cells, stable references |
| `objective_test` | per-point cost definition, Dubins interpolation along the steering curve |
| `dijkstra_sampler_test`, `intensity_sampler_test`, `hybrid_sampler_test` | the paper-faithful sampling behaviour and the 2.0 bug fixes |
| `timing_test` | sampler setup / draw budgets |
| `occupancy_map_test`, `footprint_test`, `run_logger_test` | playground core |
| `hybrid_astar_test` | Hybrid A* on synthetic in-memory maps: empty map, wall gap, heuristic, intensity corridor, determinism, budget, invalid goals, reverse motion (dead end, identical open-map path, zero penalty) |

Synthetic maps are built in the tests with the in-memory `OccupancyMap` constructor; real data comes from
`maps/` through `MOD_TEST_DATA_DIR`. Keep new tests deterministic (Hybrid A* is; sampling tests fix the OMPL
seed).

## Conventions

- **Style**: `.clang-format` (Google, 120 columns). Run `clang-format -i` on touched files.
- **Namespaces**: library code in `MoD`, OMPL-derived classes in `ompl::MoD`, playground in `MoD::playground`.
  Never introduce a global `mod` namespace: it shadows the unqualified `mod()` inside Boost.Geometry's
  `math.hpp` and breaks every translation unit that includes the GMMT map.
- **Logging**: `MOD_LOG(fmt, ...)` only (one `fprintf(stderr, "[mod] ...")`, no levels, no Boost.Log, no OMPL
  log-level changes). Log what a run needs to be understood afterwards: sizes, timings, counts, fallbacks.
- **Parameters**: every tunable is a field of a struct in `parameters.hpp` with a default, `to_json`/`from_json`
  through the `get(j, key, out)` helper (partial JSON accepted), and a row in [parameters.md](parameters.md).
  Nothing is inferred silently without being written to `Derived`.
- **Thread safety**: no mutable per-call state in objectives, samplers or planners; maps as
  `shared_ptr<const>`; one planner stack per thread; `ompl::RNG::setSeed` once per process.
- **Errors**: throw `std::runtime_error` / `std::invalid_argument` from constructors and loaders; the batch runner
  catches per run and reports.
- **Attribution**: any copied snippet gets the original licence header in-file and a row in
  `3rd_party_licenses.md`. Design-only borrowings (bench-mr, Nav2) are named in the file header of what follows
  them, with the source file they follow.

## Plans and decisions

Features are planned before they are built, in `AI-PLANS/*.md`: a goal, a milestone checklist, a table of
*settled decisions* and per-milestone bullet lists with acceptance criteria. The decisions table is binding; the
plan's own "implementation notes" record where the build deviated and why. Two decisions worth knowing before
touching the objectives:

- `motionCostHeuristic` returns the exact `motionCost` (as in the papers and in OMPL's path-length objective).
  Do not replace it with a cheaper bound.
- The MoD cost is evaluated for the velocity direction, never the heading, and there is no reverse penalty
  anywhere. Hybrid A* charges direction *changes* (cusps), not reversing.

Agent protocol used to implement the plans: take the first unchecked milestone, implement only it, keep every
test green, one commit per milestone, tick it, stop.

## Versioning and changelog

Semantic versions; `project(mod VERSION ...)` in `CMakeLists.txt` and `<version>` in `package.xml` are bumped
together. `CHANGELOG.rst` (reStructuredText) gets an entry per version, grouped by Library / Playground / data,
and holds the measured results that justify a change (the interpolation record, the ATC comparison table).
`MOD_GIT_HASH` in `version.h` comes from `git describe --always --dirty --tags` at configure time; tag releases
so run logs carry a readable version.

## How to add things

**A new objective.** Derive from `ompl::MoD::MoDOptimizationObjective`, implement `modCost(x, y, alpha)` (the
MoD term at one point for motion direction `alpha`), pass the right `MapType` and the intensity map to the base
constructor. Add the enum value and string to `ObjectiveType`, a case in `PlannerFactory::buildObjective`
(with the map's cell size for `Derived.mod_cell_m`), the GUI combo, `OBJECTIVE_ORDER` in `analysis/runs.py`, and a
test on a constant map that pins the per-point value.

**A new sampler.** Derive from `ompl::base::InformedSampler`, take `(pdef, maxCalls, SamplerParameters,
IntensityMapConstPtr)`, record draws into the sink, clamp positions to the bounds. Add the `SamplerType` value, a
case in `MoDOptimizationObjective::allocInformedStateSampler`, a `SampleSource` if it is a new kind of draw, the
GUI combo and `SAMPLER_ORDER` / colours in `runs.py`.

**A new planner.** An OMPL planner needs a `PlannerType` value and a case in `PlannerFactory::buildPlanner`
(plus its fields in `PlannerParameters` and the GUI panel). A non-OMPL planner follows Hybrid A*: its own class
under `include/mod/planners/`, a member of `PlannerSetup`, a branch in `Solver::solve` that adds the path to the
problem definition so `Solver::evaluate` and the logger stay unchanged, and its own parameter scope in
`RunConfig`, `BatchSpec` and `runs.py`.

**A new environment.** A folder under `maps/` with the yaml + pgm, the MoD XMLs, a `README.md` with provenance,
optionally `scenarios_<env>.json` and a batch; a row in `maps/README.md` and, if the data is not yours, in
`3rd_party_licenses.md`. The GUI finds it on the next scan.

## Release checklist

1. `ctest` green, playground built warning-free, the ATC smoke batch runs.
2. Bump the version in `CMakeLists.txt` and `package.xml`; finish the `CHANGELOG.rst` entry.
3. Tag (`git tag -a vX.Y.Z`), so `MOD_GIT_HASH` in new run logs names the release.
