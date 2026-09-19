^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mod
^^^^^^^^^^^^^^^^^^^^^^^^^

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
  circle, rasterised disc of pixels), ``MapCache`` (maps loaded once per process), ``PlannerFactory`` (Dubins /
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
``test/data/atc``: ATC occupancy map, CLiFF / GMMT / intensity maps and the six Paper IV start-goal pairs
(``scenarios_atc.json``), copied from bench-mr.

0.0.1 (2023-01-15)
------------------
* First release
