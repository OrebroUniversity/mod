# Plan 2: Hybrid A* on OMPL components

**Goal:** A Hybrid A* planner in `mod` that uses the same OMPL state space, validity checker, resolution and MoD objective as the sampling planners, so its solutions are directly comparable in the playground logs. Own implementation, not derived from `ompl::base::Planner`.

**Depends on:** PLAN.md M1–M4 (parameter structs, thread-safe objectives with `setCostStep`, `GridDijkstra` factored out in M3, playground core in M4). Start only after PLAN.md M4 is ticked. M5/M6 of PLAN.md may run before or after this plan.

**Motion direction.** HA1 and HA2 are forward-only. HA3 adds reverse motion for Reeds-Shepp runs. No objective change is needed for that: the objectives already take the MoD direction from the position delta between interpolated points, i.e. the velocity direction, and that is the decided semantics (which way the robot faces does not matter; which way it moves does).

## Milestone progress
- [x] HA1 — `HybridAStar` core with heuristic, primitives, analytic expansion, tests
- [x] HA2 — playground integration: factory, batch runner, GUI, ATC comparison batch
- [x] HA3 — reverse motion under Reeds-Shepp: reverse primitives, RS shot, RS heuristic, cusp penalty

## Implementation notes (deviations from the text below, decided while building)
- Constructor: `HybridAStar(si, ompl::base::OptimizationObjectivePtr, HybridAStarParameters, turning_radius)`. The
  objective is the OMPL base type so the `path_length` objective (a plain `PathLengthOptimizationObjective`) works
  as in the tests; `w_d` for `h_kin` is read from the objective when it is a MoD objective, else 1. The turning
  radius is a constructor argument because OMPL's Dubins / Reeds-Shepp spaces do not expose theirs.
- `allow_reverse` defaults to `true` in the struct and is forced off unless the space is Reeds-Shepp, which gives
  the intended "false under Dubins, true under Reeds-Shepp" without a tri-state; the factory writes the effective
  value back into `config.json`.
- `h_grid` is `+inf` for poses whose cell centre is invalid or unreachable; the search then falls back to `h_kin`
  alone instead of pruning the node (a valid pose can sit in a cell whose centre is not).
- HA3 `h_kin` under reverse motion is `w_d * min(Dubins [+ penalty if arriving in reverse], RS length + penalty x
  RS cusps relative to the arriving direction)` rather than the bare RS distance: with the default penalty this
  equals the forward-only heuristic wherever the RS optimum has a cusp, so the open-map path is identical with and
  without reverse (the HA3 test); with `change_penalty = 0` it is the RS distance.
- The first primitive out of the start pays no cusp penalty (the robot is stationary; its direction is free).
- Duplicate detection keeps one node per key with the best g (updated in place, lazy deletion in the open list);
  the goal test ignores the direction bit.
- OMPL's SO(2) bounds are `[-pi, pi)`: the planner normalises yaws into that range; a scenario yaw of exactly `pi`
  must be given as `-pi`.
- Batch: `BatchSpec` has one `hybrid_astar` scope copied into every run (`maps/atc/batch_atc_hybrid.json`).

## Agent protocol
Same as PLAN.md: first unchecked milestone, only that milestone, tests green, one commit `HA<n>: …`, tick and commit `HA<n>: done`, stop.

## Decisions (settled)
| Topic | Decision |
|---|---|
| Class | `mod::HybridAStar` in `include/mod/planners/hybrid_astar.hpp`, constructed from `ompl::base::SpaceInformationPtr`, `MoDOptimizationObjectivePtr`, `HybridAStarParameters`. `solve(start, goal, time_budget_s) → ompl::geometric::PathGeometric`. |
| Heuristic | `h = max(h_grid, h_kin)`. `h_grid`: `GridDijkstra` rooted at the goal in reverse mode with the objective's `motionCost` as edge weight (obstacle- and MoD-aware). `h_kin`: `w_d × dubins_space->distance(state, goal)`, memoised per (cell, bin). Inadmissibility of `h_grid` (octile overestimate up to 8 %, eight-heading MoD term) is accepted, as Nav2 accepts its own. |
| Discretisation | Search cell 0.25 m, 72 angle bins (5°); both parameters. Nodes keep their exact continuous pose; the cell × bin key is only for duplicate detection. |
| Primitives | Three per node: straight, left, right at the vehicle's minimum turning radius; arc length `= cell × √2` so every primitive leaves its cell (Nav2's rule). Endpoint pose computed analytically; validity by `si->checkMotion(a, b)` at the inferred pixel step; g-cost by `objective->motionCost(a, b)` (the Dubins/RS interpolation between two poses on one minimum-radius arc is that arc, so the objective integrates along the primitive). |
| Analytic expansion | Nav2 scheme: attempt a Dubins shot from the expanded node to the goal when `expansions_since_last_attempt ≥ max(1, floor(h_kin / (ratio × primitive_length)))`, `ratio = 3.5`; only if the shot length ≤ `analytic_max_length_m` (default 5). Shot checked with `checkMotion`, costed with `motionCost`. First valid shot ends the search. |
| Goal test | Analytic shot success, or a node whose cell and bin equal the goal's. |
| Penalties | No reverse penalty ever (facing direction is irrelevant for a robot). One cusp penalty `change_penalty` added to g per direction flip, in cost units, default 1000: on ATC that exceeds any forward path's cost, so cusps appear only when no forward solution exists; lower it per run to allow them. Forward-only milestones never pay it, so their g-cost is exactly the objective and comparable with RRT*/AIT*. |
| State space default | Dubins. bench-mr's `CarStateSpace` derives from `DubinsStateSpace`, so the published runs were Dubins although the paper text says Reeds-Shepp. Reeds-Shepp is selectable and is what HA3 targets. |
| Output | `PathGeometric` with the primitive endpoints and the shot endpoint; the logger's `path` is that list. Single solution; `time_to_first_solution_s = planning_time_s`. |
| Termination | First solution, `time_budget_s` exceeded, `max_expansions` reached, or open list empty. |
| Threading | One `HybridAStar` per run; shares only const maps; safe for the batch runner's thread-per-run. |
| Attribution | Implemented from Nav2's SmacPlannerHybrid design description, not its code. Any copied snippet gets the Apache-2.0 notice in-file and an entry in `3rd_party_licenses.md`. |

## `HybridAStarParameters` (added to `parameters.hpp`, own scope in `config.json`)
`{ cell_size_m = 0.25; angle_bins = 72; primitive_length_m = 0 (0 → cell × √2); analytic_ratio = 3.5; analytic_max_length_m = 5.0; max_expansions = 2'000'000; allow_reverse = false (HA3: true when the state space is Reeds-Shepp); change_penalty = 1000.0 }`

## HA1 — core
**Touches:** `include/mod/planners/hybrid_astar.hpp`, `src/planners/hybrid_astar.cpp`, `include/mod/grid_dijkstra.hpp` (from PLAN.md M3), `include/mod/parameters.hpp`, `test/`.
- [x] `GridDijkstra` (already factored in PLAN.md M3) is used in reverse mode: popping node m relaxes each neighbour n with `w(n→m) = motionCost(n, m)`; lazy expansion: `costTo(node)` runs the queue until that node is settled, so only the region the search touches is ever costed. Cell size = `cell_size_m`. Nodes outside bounds or invalid → `+inf`.
- [x] Node store: `std::vector<Node{pose[3], g, h, parent, dir}>`; open list `std::priority_queue` of `(f, index)` with lazy deletion; closed set `std::unordered_set<uint64_t>` keyed by `(col, row, bin)`; bin = `round(yaw / (2π / bins)) mod bins`.
- [x] Primitive generator: for a pose and turning radius r, straight `(L)`, left/right arcs of length L on radius r; endpoint pose closed-form. Reject primitives leaving the state bounds.
- [x] Main loop: pop best f; skip if closed; goal test; analytic-expansion schedule; expand three primitives: `checkMotion`, `g' = g + motionCost(a,b).value()`, `h'` as above, push. Wall-clock check every 256 expansions.
- [x] `h_kin` uses a private `DubinsStateSpace(r)` regardless of the run's state space (forward-only, see top); memoised in an `unordered_map<uint64_t,double>` keyed like the closed set.
- [x] Path extraction by parent chain; states allocated from `si`.
- [x] Tests (`test/hybrid_astar_test.cpp`, synthetic maps through the M4 `OccupancyMap` + `FootprintChecker`, path-length and intensity objectives):
  - empty 20×20 m map, start (2,2,0) → goal (18,18,0): solved, every path state valid, cost within 10 % of the Dubins distance × `w_d`.
  - wall with one 1.5 m gap: path passes the gap, all states valid, `checkMotion` holds on every consecutive pair.
  - heuristic sanity: `h_grid(goal cell) == 0`; `h` at the start ≤ 1.15 × returned path cost.
  - intensity map with one high-q corridor and one free corridor: with `w_c` large the path takes the free corridor.
  - determinism: two solves with identical input give identical paths.
  - time budget of 0.01 s on a large map returns "no solution" without throwing.
- **Acceptance:** tests pass; PLAN.md test suite still green.

## HA2 — playground integration and comparison
**Touches:** `src/playground/core/PlannerFactory`, `tools/run_batch.cpp`, `src/playground/gui/`, `analysis/`.
- [x] `PlannerParameters.type` gains `hybrid_astar`; `RunConfig` gains the `HybridAStar` scope; the factory builds `HybridAStar` from the same `SpaceInformation` and objective as the others; `RunLogger` unchanged (fields already fit).
- [x] GUI: planner dropdown lists `hybrid_astar`; overlay toggle "expanded nodes" draws the closed set as dots coloured by heading bin; the solution path draws as for the others. The parameter panel shows the `HybridAStar` scope when selected.
- [x] `run_batch` on the six ATC scenarios: `hybrid_astar` vs `rrt_star` (dijkstra 0.05/0.5) vs `ait_star` with the four objectives, 10 repeats each (Hybrid A* is deterministic; repeats only measure timing noise). `analysis/plot_success.py` and `plot_cost.py` include it without changes.
- [x] `CHANGELOG.rst`: entry with the ATC comparison summary table.
- **Acceptance:** the batch completes, plots show all three planners, GUI solves ATC with Hybrid A* and shows expanded nodes.

## HA3 — reverse motion (Reeds-Shepp runs)
**Touches:** `hybrid_astar.{hpp,cpp}`, `parameters.hpp`, `test/hybrid_astar_test.cpp`, GUI parameter panel.
- [x] `allow_reverse` is forced to `false` under Dubins and defaults to `true` under Reeds-Shepp. When enabled, each node expands six primitives: the three forward ones and their reverse counterparts (same arcs traversed backwards; the endpoint yaw follows the car model, the position moves against the heading). `Node.dir ∈ {fwd, rev}`; a primitive whose direction differs from its parent's adds `change_penalty` to g.
- [x] Validity and cost of a reverse primitive use the same calls, `checkMotion(a, b)` and `motionCost(a, b)`, over the run's `ReedsSheppStateSpace`; the objectives cost it by the velocity direction automatically (no change to any objective).
- [x] Analytic shot uses `reedsShepp(node, goal)` instead of the Dubins path; cusps inside the shot are counted from the RS path's segment signs and each adds `change_penalty`. `h_kin` switches to `w_d × reeds_shepp_space->distance(state, goal)`.
- [x] Duplicate detection key gains the direction bit `(col, row, bin, dir)` so a forward and a reverse arrival at the same bin are distinct nodes.
- [x] Tests: dead-end corridor where the only way out is to back up: no solution with `allow_reverse=false`, solution with exactly one cusp with `allow_reverse=true`; the same open-map scenario as HA1 gives the identical forward path under both settings (the penalty keeps cusps out); `change_penalty = 0` on a T-shaped map yields a path with cusps whose objective cost is lower than the forward-only path's.
- [x] GUI: `allow_reverse` and `change_penalty` in the HybridAStar panel; reverse primitives drawn in the expanded-node overlay with a distinct marker.
- **Acceptance:** HA3 tests pass; HA1/HA2 tests unchanged.

## Edge cases & risks
- With 0.25 m cells the goal-rooted `GridDijkstra` on ATC has 560×240 nodes and up to 1.1M directed edges, each a `motionCost` with ~7 cost points; lazy expansion keeps the evaluated set to what the search touches, but a hard scenario can approach the full grid (a few seconds). If that dominates, the heuristic grid can use its own coarser cell (0.5 m) as a parameter; not added now.
- The cell × bin closed set discards a better continuous pose that lands in an already-closed bin; this is inherent to Hybrid A* and is why the returned cost is compared, not assumed optimal.
- Until HA3, a Reeds-Shepp run of Hybrid A* can fail where RRT* with reversing succeeds (dead-end turnarounds); logged as "no solution".
- With `change_penalty` at its default the search still generates reverse primitives under Reeds-Shepp and pushes them with a huge f; they sit at the bottom of the open list and cost memory, not time. If memory becomes an issue on a huge map, prune pushes whose f exceeds the best known solution bound.
