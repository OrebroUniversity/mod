^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mod
^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2026-09-19)
------------------
Hybrid A* planner (``AI-PLANS/PLAN-hybrid-astar.md``, milestones HA1-HA3).

Library
~~~~~~~
* ``MoD::HybridAStar`` (``include/mod/planners/hybrid_astar.hpp``): own implementation (not an
  ``ompl::base::Planner``) over the same ``SpaceInformation`` (state space, bounds, footprint checker, collision
  resolution) and ``OptimizationObjective`` as the sampling planners, so its solution cost is directly comparable in
  the run logs. Constructed from ``(si, objective, HybridAStarParameters, turning_radius)``;
  ``solve(start, goal, budget_s) -> PathGeometric``.

  - Nodes keep their exact continuous pose; the ``(cell 0.25 m, 72 heading bins[, direction])`` key is only for
    duplicate detection (one node per key with the best g).
  - Three primitives (straight, left, right at the minimum turning radius) of length ``cell * sqrt(2)``; validity by
    ``si->checkMotion`` at the inferred pixel step, g-cost by ``objective->motionCost`` (the objective integrates
    along the arc through the space's own interpolation).
  - ``h = max(h_grid, h_kin)``: ``h_grid`` is a goal-rooted, lazily expanded ``GridDijkstra`` in reverse mode with
    ``motionCost`` as edge weight (obstacle- and MoD-aware); ``h_kin = w_d * Dubins distance``, memoized per key.
  - Analytic expansion on Nav2's schedule (a shot every ``max(1, floor(h_kin / (3.5 * primitive length)))``
    expansions, only if at most 5 m long); the first valid shot ends the search. Goal test: shot, or same cell and
    bin as the goal (then connected to the exact goal when ``checkMotion`` allows).
  - Reverse motion under Reeds-Shepp (HA3): six primitives, direction bit in the key, Reeds-Shepp shot with its
    cusps counted from the segment signs, ``h_kin = w_d * min(Dubins, RS length + change_penalty * cusps)``. No
    reverse penalty ever; one ``change_penalty`` (default 1000) per direction flip, so cusps appear only where no
    forward solution exists unless the penalty is lowered. Forward-only runs never pay it, so their cost is exactly
    the objective's.
  - Termination: first solution, time budget, cancel callback, ``max_expansions``, or empty open list.
    ``result()`` reports termination, expansions, cusps and costs; ``expandedNodes()`` returns the closed set.

* ``HybridAStarParameters`` (own scope ``HybridAStarParameters`` in ``config.json``) and the planner type
  ``hybrid_astar`` in ``PlannerParameters``. Version 2.1.0.
* Design after Nav2's SmacPlannerHybrid description (Apache-2.0); no Nav2 code (``3rd_party_licenses.md``).

Maps
~~~~
* Map data moved from ``test/data/atc`` to ``maps/atc``; ``maps/warehouse`` (``pedsim_warehouse``) and
  ``maps/office_cubicles`` (with the four bench-mr start/goal pairs in ``scenarios_office_cubicles.json`` and
  ``batch_office_smoke.json``) copied from the bench-mr fork's ``maps/``. ``MOD_TEST_DATA_DIR`` points at ``maps/``.
* GUI: the file fields get pickers over the yaml / CLiFF / GMMT / intensity files found under ``maps/``
  (classified by content; compiled-in source path, ``--maps-dir DIR`` overrides, "rescan" button), and a bare
  file name typed or read from a config resolves against that folder (``config.json`` records the resolved path).

Playground
~~~~~~~~~~
* ``PlannerFactory`` builds ``HybridAStar`` from the same ``SpaceInformation`` and objective (``PlannerSetup::
  hybrid_astar``; ``planner`` stays null); ``Solver`` runs it with the run's time budget and cancel flag and adds
  the path to the problem definition, so ``RunLogger`` and ``Solver::evaluate`` are unchanged
  (``time_to_first_solution_s = planning_time_s``). ``BatchSpec`` gains one ``hybrid_astar`` scope.
* GUI: ``hybrid_astar`` in the planner dropdown with its parameter panel (``allow_reverse`` and ``change_penalty``
  shown under Reeds-Shepp); overlay "expanded nodes" draws the closed set coloured by heading bin, reverse
  arrivals as rings.
* ``analysis/runs.py`` flattens the new scope as ``hybrid_astar.*``; the plots already list ``hybrid_astar``.
* Tests: ``test/hybrid_astar_test.cpp`` (synthetic maps through ``OccupancyMap`` + ``FootprintChecker``): empty
  map within 10 % of the Dubins length, wall gap, heuristic sanity, high-intensity corridor avoided, determinism,
  0.01 s budget, invalid / unreachable goal, dead end needing exactly one cusp, identical open-map path with and
  without reverse, zero penalty preferring a cheaper cusp path on a T map.

ATC comparison (``maps/atc/batch_atc_hybrid.json``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Six scenarios x {hybrid_astar, rrt_star, ait_star} x dijkstra sampler (bias 0.05, cell 0.5 m) x four objectives x
10 repeats, Dubins r = 1 m, 30 s budget each, 20 runs in parallel on 24 cores (Hybrid A* is deterministic; its
repeats only measure timing noise, inflated here by the parallel load: alone it takes 0.04-0.2 s per ATC
scenario). Medians over the solved runs; ``c`` is the unweighted MoD component.

=========  ============  ======  =================  ===========  ============  ==========
objective  planner       solved  first solution [s] cost         MoD cost c    length [m]
=========  ============  ======  =================  ===========  ============  ==========
cliff      rrt_star      59/60   0.79               66.82        160.43        50.34
cliff      ait_star      54/60   4.89               65.19        176.75        47.59
cliff      hybrid_astar  60/60   0.63               61.75        157.40        45.79
gmmt       rrt_star      59/60   1.09               49.50        37.37         43.99
gmmt       ait_star      56/60   5.50               53.78        46.90         47.97
gmmt       hybrid_astar  60/60   0.76               49.94        33.66         45.86
dtc        rrt_star      57/60   0.78               63.03        692.92        45.54
dtc        ait_star      54/60   8.70               60.04        747.52        46.25
dtc        hybrid_astar  60/60   0.68               58.55        633.45        45.67
intensity  rrt_star      59/60   0.65               75.07        153.27        44.00
intensity  ait_star      48/60   7.03               74.42        160.07        44.94
intensity  hybrid_astar  60/60   0.57               75.93        152.33        46.46
=========  ============  ======  =================  ===========  ============  ==========

Hybrid A* solves every scenario under every objective within a second and its single solution costs about the
same as, or less than, the sampling planners' best after 30 s (cliff -8 %, dtc -7 %, gmmt and intensity within
1 % of RRT*). The comparison is forward-only (Dubins); the cell x bin closed set means the returned cost is
compared, not assumed optimal. ``analysis/plot_success.py`` and ``plot_cost.py`` on ``runs/atc_hybrid`` show the
three planners without changes.

2.0.0 (2026-09-19)
------------------
Self-contained, thread-safe library of MoD objectives and samplers with paper-faithful sampling, fast Dijkstra
setup, JSON run logs and a playground that replaces bench-mr for experiments. No API compatibility with 1.x:
constructors take parameter structs.

Library
~~~~~~~
* C++17, CMake 3.16 target ``mod::mod`` (exported), ``nlohmann_json`` dependency, Boost.Log dropped (Boost stays
  header-only for property_tree XML and the GMMT rtree). Console logging is ``MOD_LOG`` (``fprintf(stderr)`` with a
  ``[mod]`` prefix, no levels).
* Parameter structs ``MoD::VehicleParameters``, ``Derived``, ``SamplerParameters``, ``OptObjParameters``,
  ``PlannerParameters``, ``Scenario``, ``RunMeta`` and ``RunConfig`` (``include/mod/parameters.hpp``) with nlohmann
  (de)serialization; ``from_json`` accepts partial objects. They live in the existing ``MoD`` namespace: a global
  ``mod`` namespace shadows the unqualified ``mod()`` call inside Boost.Geometry's ``math.hpp`` and breaks every
  translation unit that includes the GMMT map.
* Objectives: ``last_cost_`` / ``getLastCost*`` removed (no readers); ``motionCostComponents(s1, s2)`` returns the
  unweighted ``{d, q, c}``; ``motionCost`` is their weighted sum. The cost integral interpolates at the objective's
  own step (``setCostStep``, default the MoD cell size; the playground sets ``min(MoD cell, pixel)``) through
  ``space->interpolate``; ``validSegmentCount`` is no longer used, so collision checking and cost integration are
  decoupled. Cost stays per point. Maps are held as ``shared_ptr<const>`` and can be shared between threads.
* ``CLiFFMap::at`` / ``atId`` / ``operator()`` return ``const CLiFFMapLocation &`` (static empty location when out of
  range). ``GMMTMap`` getters are const; the heading of the last mean of a cluster no longer reads past the vector.
* ``IntensityMap::operator()`` returns 0 outside the map instead of indexing out of range.
* Samplers take ``(pdef, maxCalls, SamplerParameters, shared_ptr<const IntensityMap>)``; the debug CSV code with
  hard-coded home paths is gone. Optional ``MoD::SampleSink`` receives every draw with its source
  (``uniform | ellipse | intensity | dijkstra``).
* Bug fixes pinned by tests:

  - ``DijkstraSampler`` heading: the previous / next path cell is now actually advanced (the old code advanced the
    current iterator and read ``path_.begin()``), so biased draws point along the path as in Paper IV, step 3.
  - ``IntensityMapSampler`` uniform-valid branch drew its value from ``U(0, sum(1 - q))`` while walking cells of
    weight 1, so only a prefix of the valid cells was reachable (5.8 % of ATC's valid cells could never be drawn).
    Each branch now draws from its own weight sum, and the cell whose cumulative interval contains the value is
    returned (the old check-before-add loop was off by one in both branches).
  - Sampled x, y are clamped to the state bounds.

* ``MoD::GridDijkstra`` (``include/mod/grid_dijkstra.hpp``): implicit 8-neighbour grid, per-node validity cache,
  edge-weight callback evaluated on pop, forward and reverse (cost-to-go) modes, lazy ``costTo``, ``pathTo``.
  ``DijkstraSampler`` is built on it (Boost.Graph removed): ATC at 0.5 m (34k nodes) sets up in ~70 ms; a 61 x 61
  grid in 7 ms. ``IntensityMapSampler`` uses struct-of-arrays, prefix sums and ``upper_bound`` (1M draws in 75 ms).

Playground (``src/playground``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* ``mod_playground`` core: ``OccupancyMap`` (map_server yaml + P5 pgm), ``FootprintChecker`` (one circumscribed
  circle, rasterized disc of pixels), ``MapCache`` (maps loaded once per process), ``PlannerFactory`` (Dubins /
  Reeds-Shepp, RRT* / AIT*, collision resolution = pixel / extent, objective and sampler from the parameter structs,
  fills ``Derived``), ``Solver`` (termination by time or cancel, time to first solution from the intermediate
  solution callback), ``RunLogger`` (one folder per run with ``config.json``, ``solution.json`` and optional
  ``samples.json``; ``RunMeta`` from the configure-time git describe, hostname and clock).
* ``run_batch <batch.json>``: scenarios x planners x samplers x objectives x repeats, one thread per run.
  Acceptance batch on ATC (6 scenarios x RRT*/AIT* x iid/dijkstra x cliff x 2 repeats, 10 s each, 4 threads):
  48 valid run folders, 43/48 solved, 121 s wall.
* ``mod_playground_gui``: ImGui/GLFW app with map texture, pan/zoom, S/G + click/drag for start and goal, side
  panel over ``RunConfig``, solve on a worker thread with cancel, overlays (CLiFF mean arrows, GMMT cluster
  polylines, intensity heat, planner graph, solution path). ``--config``, ``--map``, ``--solve``,
  ``--exit-after-solve``, ``--screenshot`` for scripted use.
* ``analysis/``: ``runs.py`` (run folders -> DataFrame), ``plot_success.py``, ``plot_cost.py``, ``plot_samples.py``.

Interpolation record (``check_interpolation``, ATC scenario 1, Reeds-Shepp r = 1, cliff objective, 5 s each)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The planners hand ``motionCost`` whole edges; the objective's own interpolation is what produces the per-point cost.
With the inferred steps (pixel 0.05 m, cost step 0.05 m):

* RRT* (default range 30.8 m = 20 % of the extent): 3782 ``motionCost`` calls in 5 s, 15.4 per iteration
  (245 iterations). Edge length mean 33.9 m, median 29.1 m, p90 65.4 m, max 89.7 m; cost points per call mean 679,
  median 582, p90 1309, max 1795. ``motionCostHeuristic`` is never called.
* AIT* (batch 100): 7 ``motionCost`` calls but 13 799 ``motionCostHeuristic`` calls in 5 s (2732/s), edge length
  mean 8.8 m, median 7.1 m, p90 12.7 m; the MoD objectives forward ``motionCostHeuristic`` to ``motionCost``, so
  AIT*'s reverse search pays the full per-point integral on every RGG edge (about 176 cost points per call).
* At 0.05 m there are 20 cost points per metre versus about 7 in Paper IV (0.001 of the extent), so the MoD
  component is roughly 3x larger relative to the distance component; the ``w_c`` defaults may need one rescaling
  pass after the first full batch.

Test data
~~~~~~~~~
``maps/atc``: ATC occupancy map, CLiFF / GMMT / intensity maps and the six Paper IV start-goal pairs
(``scenarios_atc.json``), copied from bench-mr.

0.0.1 (2023-01-15)
------------------
* First release
