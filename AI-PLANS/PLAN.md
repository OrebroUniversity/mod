# Plan: mod 2.0 — fixed and fast MoD samplers, run logging, playground

**Goal:** Turn `mod` into a self-contained, thread-safe library of MoD objectives and samplers with correct (paper-faithful) sampling, fast Dijkstra setup, JSON run logs, and a playground (headless batch runner + ImGui app) that replaces bench-mr for experiments.

## Milestone progress
- [x] M1 — foundation: C++17 CMake, thread safety, parameter structs, printf logging, gtest scaffold
- [x] M2 — bug fixes with tests that pin the paper's behaviour
- [x] M3 — sampler optimizations (Dijkstra flat array + heap, intensity prefix sums)
- [x] M4 — playground core: map loader, planner factory, run logger, headless batch runner, interpolation check
- [x] M5 — ImGui app
- [x] M6 — Python log readers and plots

## Agent protocol
1. Pick the first unchecked milestone above.
2. Implement only that milestone.
3. Run its tests until they pass, plus all previously passing tests (`cmake --build build && ctest --test-dir build`).
4. Commit once: `M<n>: <what was built>`. One commit per milestone.
5. Tick the milestone here and commit that edit as `M<n>: done`.
6. Stop. One milestone per session.

## Decisions (settled; do not reopen)
| Topic | Decision |
|---|---|
| bench-mr | No API compatibility. Constructors take parameter structs. Version 2.0.0. bench-mr is not built or used again; ATC start/goal pairs are copied from its `python/sg-pairs-atc.yaml`, maps from its `maps/`. |
| Bugs | Fix Dijkstra heading (neighbour iterator) and intensity uniform-valid truncation. Hybrid stays as in Paper IV (US until a solution, then EH; IIS 1 %; DGS α); its `uniform_valid` flag stays as is. STeF remnants stay. |
| Grid edge cost | `motionCost` of the planner's objective, exactly as the planner calls it. RRT* and AIT* call it on the whole steered edge (`RRTstar.cpp:287`, `AITstar.cpp:745`); the objective interpolates internally. Whether that internal interpolation is redundant is decided by the M4 check, not by assumption. |
| Validity cache | Per node (N checks, cached); an edge is valid iff both endpoint nodes are valid. Yaw is irrelevant because the footprint is a circle (below). If an asymmetric footprint is ever added, the cache becomes per node × 8 headings. |
| Collision step | Inferred, never entered: the occupancy map's pixel size. The playground sets OMPL's `setStateValidityCheckingResolution(pixel / maxExtent)` from it. Consecutive footprint tests one pixel apart cannot skip a pixel, which is the same argument grid planners such as Nav2 use. |
| MoD cost step | Inferred, never entered: `min(MoD cell size, occupancy pixel size)`. The objectives interpolate at this step themselves (`n = ceil(distance / step)` via `space->interpolate`) and no longer call `validSegmentCount`, so collision checking and cost integration are decoupled. Cost stays **per point** as in Paper IV, so the step is part of the cost definition and is recorded in `config.json` under `Derived`. |
| Footprint test | One circumscribed circle for every shape: circle → its radius; rectangle → `sqrt((L/2)² + (W/2)²)`. Tested as the rasterised disc of pixels around the pose (offset list precomputed once per map). No distance transform, no polygon test; sophistication later if needed. |
| Interpolation question | Closed by source reading: RRT* (`RRTstar.cpp:287, 323, 422`) and AIT* (`AITstar.cpp:745, 1279`) pass only the two endpoints of an edge, up to `range` / the RGG radius long; `checkMotion` interpolates separately (`DiscreteMotionValidator.cpp:103`). The objectives' interpolation is therefore necessary and stays. The M4 tool only measures edge-length and point-count distributions. |
| Thread safety | Delete `last_cost_` and `getLastCost*` (no readers exist); add `motionCostComponents()`; make `GMMTMap` getters const; maps are shared as `shared_ptr<const>`; samplers and objectives are per-planner objects. |
| Logging (console) | Plain `fprintf(stderr, ...)` with a fixed `[mod] ` prefix, no levels, no Boost.Log, no OMPL log-level changes. |
| Logging (runs) | nlohmann JSON. One folder per run, `config.json` + `solution.json`, optional `samples.json` (off by default). No intermediate solutions. |
| Sampling opt | Intensity sampler: prefix sums + `upper_bound`, OMPL RNG kept. Dijkstra: implicit grid, `std::vector` state, binary heap. |
| Cleanups | Clamp samples to state bounds; delete dead `DijkstraSampler::distance()`. |
| CLiFFMap | `at`, `atId`, `operator()` return `const CLiFFMapLocation&`. |
| Playground | `src/playground/`. Own yaml+pgm loader and footprint checker (no MRPT, no yaml-cpp). Planners RRT* and AIT*. State spaces Dubins (default, as the published runs) and Reeds-Shepp. Robot circle or rectangle. ImGui as git submodule `third_party/imgui` pinned to a release tag, GLFW + OpenGL3 backends, glfw from the system. Overlays: CLiFF mean arrows, GMMT cluster polylines, intensity heat, planner tree; each with a toggle. |
| Python | Plain scripts in `analysis/` (numpy, pandas, matplotlib). |
| Attribution | Any borrowed code gets its licence header in-file and an entry in `3rd_party_licenses.md`. ImGui is MIT. |
| Related plans | `PLAN-hybrid-astar.md` (starts after M4; adds the `hybrid_astar` planner type to the factory and GUI; its HA3 covers reverse motion). No third plan: the objectives already cost motion by velocity direction, which is the decided semantics (facing direction is irrelevant for a robot), so no objective change is needed for reversing. |
| State space default | Dubins. bench-mr's `CarStateSpace` derives from `DubinsStateSpace`, so the published runs were Dubins although the paper text says Reeds-Shepp. Reeds-Shepp stays selectable. |
| AIT* heuristic | `motionCostHeuristic` returns `motionCost` (the exact per-point integral), exactly as in Paper IV and as OMPL's own `PathLengthOptimizationObjective` does. Not to be changed: the paper never claimed a heuristic that ignores MoD costs. AIT*'s reverse search therefore pays the full integral on every RGG edge (record in `CHANGELOG.rst`); that is the cost of the method, not a bug. |
| Ellipse sampler | `ellipse` (and the hybrid's ellipse branch) is OMPL's `PathLengthDirectInfSampler`, and the ellipse is built from the planner's current best **full MoD cost** (`RRTstar.cpp:1135` passes `bestCost_`; the sampler uses it as the transverse diameter, `PathLengthDirectInfSampler.cpp:550`), not from the path length alone. Consistent with the paper; not to be changed. Uniform over the bounds until a first solution exists. |
| Infra | Host build only (Fedora 44, OMPL 2.0 in /usr/local, Boost 1.90, Eigen 5, gtest 1.17, nlohmann_json 3.12, glfw 3.4). `.clang-format` Google/120. No Docker, no CI. |

## Parameter structs (core library, `include/mod/parameters.hpp`)
All have `to_json`/`from_json` (nlohmann) and defaults equal to the Paper IV settings.
- `VehicleParameters { shape: circle|rectangle; radius; length; width; state_space: dubins|reeds_shepp = dubins; turning_radius = 1.0 }` (no resolution field; both steps are inferred)
- `Derived { occupancy_pixel_m; mod_cell_m; collision_step_m; mod_cost_step_m; circumscribed_radius_m }` — computed by the playground from the loaded maps and vehicle, written to `config.json` for the record, never read back.
- `SamplerParameters { type: iid|ellipse|intensity|dijkstra|hybrid; bias = 0.05; dijkstra_cell_size = 0.5; hybrid_intensity_bias = 0.01; intensity_map_file; log_samples = false }`
- `OptObjParameters { type: cliff|gmmt|dtc|intensity|path_length; w_d = 1; w_q = 1; w_c; cliff_map_file; gmmt_map_file; intensity_map_file; max_vehicle_speed = 1.0; mahalanobis_threshold = 10; use_mixing_factor = true }`
- `PlannerParameters { type: rrt_star|ait_star; max_planning_time; seed; range; goal_bias = 0.05; batch_size = 100; informed_sampling = true }`
- `Scenario { name; map_yaml; start[3]; goal[3] }`
- `RunMeta { mod_version; git_hash; hostname; started_at (ISO 8601) }` — filled by the logger, never by hand.
`RunConfig` aggregates all seven; `config.json` is its serialization, one object per scope, no flat key salad.

## Log layout
`<log_dir>/<YYYYMMDD-HHMMSS.mmm>_<scenario>_<planner>_<sampler>_<objective>/`
- `config.json` — `RunConfig`.
- `solution.json` — `{ success, planning_time_s, time_to_first_solution_s, cost: { total, d, q, c }, path_length_m, path: [[x, y, theta], ...] }`; `path` empty on failure.
- `samples.json` (only if `log_samples`) — `[[x, y, theta, source], ...]`, `source ∈ {uniform, ellipse, intensity, dijkstra}`.

---

## M1 — foundation
**Touches:** `CMakeLists.txt`, `package.xml`, `include/mod/*.hpp`, `src/*.cpp`, all objective and sampler headers/sources, `test/`.
- [ ] CMake: `project(mod VERSION 2.0.0)`, C++17, target-based includes/links, `find_package(nlohmann_json REQUIRED)`, drop Boost.Log and `BOOST_LOG_DYN_LINK` (Boost stays for property_tree XML and geometry rtree), `option(MOD_BUILD_TESTS ON)` + `find_package(GTest)` + `enable_testing()` + `add_subdirectory(test)`, `option(MOD_BUILD_PLAYGROUND ON)` placeholder. Keep install/export; export a proper `mod::mod` target.
- [ ] `include/mod/log.hpp`: `MOD_LOG(fmt, ...)` → one `fprintf(stderr, "[mod] " fmt "\n", ...)`. Replace every `BOOST_LOG_TRIVIAL` and `OMPL_INFORM/ERROR` in the library with it. Remove `std::cout << "SHITE"` sites (log a real message).
- [ ] Thread safety: delete `Cost last_cost_`, `getLastCost*`; add `virtual CostComponents motionCostComponents(s1, s2) const` on `MoDOptimizationObjective` (struct `{d, q, c}`), implement in the three objectives by factoring the existing loops so `motionCost` = weighted sum of components. `GMMTMap::getMixingFactorByClusterID`, `getHeadingAtDist` become const. `allocInformedStateSampler` stops re-reading the intensity map from disk: objectives hold `shared_ptr<const IntensityMap>` and hand it to the sampler.
- [ ] CLiFFMap: `at`, `atId`, `operator()` return `const CLiFFMapLocation&` with a static empty location for out-of-range; DTC and upstream call sites bind `const auto&`.
- [ ] Objectives interpolate at their own step: `MoDOptimizationObjective` gets `setCostStep(double metres)` (set by the playground to `min(MoD cell, pixel)`; library default = the MoD cell size of the map given); `motionCost` uses `n = max(1, ceil(si_->distance(s1,s2) / step))` sub-segments through `space->interpolate`, replacing every `validSegmentCount` call. Per-point summation unchanged. Test: on a constant-q map a 2 m straight edge with step 0.25 m costs `w_d·2 + w_c·q·8`.
- [ ] `include/mod/parameters.hpp` + `src/parameters.cpp`: the structs above with json (de)serialization and string↔enum helpers.
- [ ] Constructors: `DTCOptimizationObjective(si, OptObjParameters, SamplerParameters)`, `UpstreamCriterionOptimizationObjective(si, OptObjParameters, SamplerParameters)`, `IntensityMapOptimizationObjective(si, OptObjParameters, SamplerParameters)`; samplers take `(pdef, maxCalls, SamplerParameters, shared_ptr<const IntensityMap>)`. Delete the old overloads and the `debug` CSV code (file streams, hard-coded `/home/ksatyaki` paths). `sampleUniform` gains no new behaviour in M1.
- [ ] `test/CMakeLists.txt` + `test/parameters_test.cpp` (json round-trip of every struct) + `test/maps_test.cpp` (load the three ATC maps from `test/data/` copied from bench-mr, check bounds/rows/cols and one known cell each; CLiFFMap reference return is stable across calls).
- [ ] `3rd_party_licenses.md` created (empty list + header).
- **Acceptance:** library builds warning-free with `-Wall -Wextra`, `ctest` passes, no `BOOST_LOG` or `OMPL_INFORM` symbol left in `src/`.

## M2 — bug fixes
**Touches:** `DijkstraSampler.cpp`, `IntensityMapSampler.cpp`, `test/`.
- [ ] Dijkstra heading: advance `prev_iter`/`next_iter` (not `iter`); last cell uses the previous cell, every other cell the next cell, as Paper IV Sampling step 3.
- [ ] Intensity uniform-valid branch: draw `sampled_value ~ U(0, N)` for the uniform branch (its own weight sum), keep `U(0, value_sum)` for the q branch; selection returns the cell whose cumulative interval contains `sampled_value` (fix the check-before-add off-by-one in both branches).
- [ ] Clamp sampled x, y to the state bounds. Delete `distance()`.
- [ ] `test/dijkstra_sampler_test.cpp` (synthetic 12×12 grid, wall with one gap, path-length objective): path passes the gap; every biased draw lies within ±cell/2 of a path node inside the bounds; heading of each biased draw is within ±π/8 of the direction to the next path cell (previous cell for the last node) — computed from the sampler's exposed `path()` accessor.
- [ ] `test/intensity_sampler_test.cpp` (synthetic 6×5 intensity XML, three invalid cells): draws never hit invalid cells; uniform-branch hit counts over 200k draws are uniform over all valid cells (chi-square p > 0.01); q-branch hit frequencies ∝ (1−q) within 1 %.
- [ ] `test/hybrid_sampler_test.cpp`: branch proportions over 100k draws match α and the intensity bias within 1 pp (identify by sub-sampler through a test-only hook that records which sub-sampler was called).
- **Acceptance:** the three new test files pass; M1 tests still pass.

## M3 — sampler optimizations
**Touches:** `DijkstraSampler.{h,cpp}`, `IntensityMapSampler.{h,cpp}`, `test/`.
- [ ] Dijkstra: replace Boost.Graph, `std::list`, `props`, edge/weight lists with `rows_`, `cols_`, `cell_size_`, `x_min_`, `y_min_`, `std::vector<uint8_t> valid_` (one entry per node), `std::vector<size_t> path_`. Written from scratch; if any snippet is borrowed, attribute it.
- [ ] Setup: one pass filling `valid_` with the checker at `(colToX, rowToY, yaw 0)` (one preallocated state; yaw is irrelevant for the circumscribed-circle footprint). Dijkstra with `std::priority_queue<pair<double,size_t>>` and lazy deletion over the implicit 8-neighbour grid; edge weight evaluated on pop via `edgeCost(r0,c0,r1,c1)` = `opt_->motionCost(a,b)` with both yaws = edge heading (unchanged semantics; two preallocated states). Stop when the goal pops. Unreachable goal → `path_` empty, log once, sampler falls back to the uniform branch (no UB).
- [ ] Factor the grid search into `include/mod/grid_dijkstra.hpp` (`mod::GridDijkstra`): implicit 8-neighbour grid over given bounds and cell size, per-node validity cache from a checker callback, edge-weight callback `w(n, m)`, forward mode (root = start, distances from root) and reverse mode (root = goal, cost-to-go, relaxing `w(n, m)` on predecessors), lazy `costTo(node)` that expands until that node settles, `pathTo(node)`. `DijkstraSampler` uses it in forward mode with full expansion to the goal. PLAN-hybrid-astar.md uses reverse + lazy mode as its heuristic.
- [ ] Sampling: O(1) index into `path_`; RNG call order identical to M2.
- [ ] Log setup wall time, node count, evaluated edge count.
- [ ] Intensity: struct-of-arrays valid cells, `prefix_q_` (sorted ascending by 1−q as before), uniform branch indexes directly by `floor(U(0,N))`; `upper_bound` for the q branch.
- [ ] `test/timing_test.cpp`: 61×61 grid setup (office bounds at 0.5 m) with the intensity objective; prints setup time and asserts it is below 5 s; 1M intensity draws below 1 s.
- [ ] Remove Boost.Graph includes; Boost.Geometry stays for GMMT.
- **Acceptance:** all M2 tests pass unchanged on the rewritten samplers; timing test passes.

## M4 — playground core, batch runner, interpolation check
**Touches:** `src/playground/core/` (library `mod_playground`), `src/playground/tools/`, `test/`.
- [ ] `OccupancyMap`: parse map yaml (image, resolution, origin, negate, occupied_thresh) and P5 pgm; `bounds()`, `pixel_size()`, `occupied(x,y)`. `FootprintChecker`: circumscribed radius from `VehicleParameters` (circle → radius; rectangle → half-diagonal); precomputes the pixel-offset list of the disc once, tests every pixel of the disc around the pose; `isValid(State*)` implements `ob::StateValidityChecker`. No polygon test, no distance transform.
- [ ] `PlannerFactory::build(RunConfig, shared maps)` → `SpaceInformation` with `setStateValidityCheckingResolution(pixel_size / space->getMaximumExtent())`, objective (from `OptObjParameters`) with `setCostStep(min(mod_cell, pixel_size))`, sampler dispatch (from `SamplerParameters`), planner (`og::RRTstar` with `setInformedSampling`, range, goal bias; `og::AITstar` with batch size), seeded via `ompl::RNG::setSeed`. Fills the `Derived` scope. Maps loaded once per process and shared as `shared_ptr<const>`.
- [ ] `RunLogger`: creates the run folder, writes `config.json` before solving, `solution.json` after, `samples.json` if enabled (samplers get an optional `SampleSink*`). `RunMeta` from `git describe` baked in at configure time (`mod_version.h.in`), `gethostname`, `std::chrono`.
- [ ] `tools/run_batch.cpp`: input JSON `{ log_dir, threads, scenarios: [Scenario], planners: [PlannerParameters], samplers: [SamplerParameters], objectives: [OptObjParameters], vehicle: VehicleParameters, repeats, seed0 }`; expands the product, runs `threads` runs concurrently (one thread per run, each with its own SpaceInformation/objective/sampler/planner; only maps are shared). Prints a one-line summary per run.
- [ ] `tools/check_interpolation.cpp`: wraps the chosen objective in a counting decorator recording, per `motionCost` call, `si->distance(s1,s2)` and the number of cost points `ceil(distance / step)`; runs one RRT* and one AIT* solve on ATC scenario 1 (Reeds-Shepp r=1, inferred steps, range default) and prints both histograms plus calls per planner iteration. Output goes into `CHANGELOG.rst` as the documented magnitude of the per-point cost. No decision hangs on it (see the Interpolation row in the decisions table).
- [ ] `test/data/atc/` gets `atc.yaml`, `atc_white_fixed.pgm`, the three ATC MoD XMLs, and `scenarios_atc.json` with the six Paper IV pairs; `test/occupancy_map_test.cpp`, `test/footprint_test.cpp`, `test/run_logger_test.cpp` (round trip config → folder → parse).
- [ ] `CHANGELOG.rst` 2.0.0 entry.
- **Acceptance:** `run_batch` completes 6 scenarios × 2 planners × 2 samplers × 1 objective × 2 repeats on ATC with 4 threads and produces valid JSON folders; `check_interpolation` prints its histograms; all tests pass.

## M5 — ImGui app
**Touches:** `third_party/imgui` (submodule), `src/playground/gui/`.
- [ ] Submodule ImGui at its newest release tag; compile `imgui*.cpp`, `backends/imgui_impl_glfw.cpp`, `backends/imgui_impl_opengl3.cpp` into `mod_playground_gui`; add the MIT notice to `3rd_party_licenses.md`.
- [ ] Canvas: world↔screen transform with pan (right-drag) and zoom (scroll, about the cursor). Map drawn as a texture from the pgm once.
- [ ] Start/goal: hold `S` or `G` + left-click sets position, drag while held sets heading; drawn as arrows.
- [ ] Side panel from `RunConfig`: map yaml file picker (text field + load), planner, state space, robot shape with sliders (radius / length / width), turning radius, objective + weights, sampler + bias + cell size, max planning time, seed, `log_samples`, log dir. Solve button runs on a worker thread; UI stays responsive; a Cancel calls `planner->terminate()`. Each solve writes a run folder through `RunLogger` (same code as the batch runner).
- [ ] Overlays with toggles: CLiFF mean arrows (per location, one arrow per distribution scaled by mixing factor), GMMT cluster polylines, intensity heat (translucent per-cell fill), planner tree/graph from `ob::PlannerData` after solve, solution path.
- **Acceptance:** app opens ATC, sets start/goal by mouse, solves RRT* + dijkstra, shows the path and tree, and the run folder appears with valid JSON.

## M6 — Python analysis
**Touches:** `analysis/`.
- [ ] `analysis/runs.py`: load every run folder under a directory into one pandas DataFrame (config scopes flattened with scope prefixes, e.g. `sampler.type`).
- [ ] `analysis/plot_success.py`: success rate vs planning time per sampler/planner (the Paper IV Fig. style) from `solution.json` `time_to_first_solution_s`.
- [ ] `analysis/plot_cost.py`: median cost at the final time per sampler/objective; box plots.
- [ ] `analysis/plot_samples.py`: sample cloud by source over the map for one run with `samples.json`.
- [ ] `analysis/README.md`: the batch-run → plot workflow in 6 lines.
- **Acceptance:** scripts run on the M4 acceptance batch output and produce the three figure types.

## Edge cases & risks
- Per-point cost with a pixel-size step gives 20 cost points per metre on ATC (0.05 m) versus about 7 in Paper IV (0.001 of the extent ≈ 0.15 m), so the MoD component is roughly 3× larger relative to the distance component; the weights in `OptObjParameters` defaults may need one rescaling pass after the first batch. The `check_interpolation` histogram is the record for that.
- A rasterised disc for a 0.4 m radius at 0.05 m is about 800 pixel lookups per pose; on the 34k-node ATC Dijkstra grid that is ~27M lookups at setup, well under a second. If a much finer map appears, the disc offset list is where a distance transform would slot in.
- Collision checking at exactly one pixel per step is the finest OMPL will go for that map; edge collision checks cost `length / pixel` disc tests, so a 5 m RRT* edge on ATC is 100 poses × 800 lookups. Acceptable, and the same order as bench-mr's polygon checks at 0.15 m.
- Thread-per-run batches share nothing but const maps; OMPL's `ompl::RNG` seeds are per instance, but `ompl::RNG::setSeed` is global and must be called once before threads start, then per-run generators seeded from `seed0 + run_index`.
- Fedora's OMPL is a source build in `/usr/local`; the playground finds it through the same `find_package(ompl)` as the library.
