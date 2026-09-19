# Concepts

This page explains what the library computes. Symbols: a pose is `(x, y, θ)`; `α` is the *direction of motion*
of a small step (`atan2(Δy, Δx)`), which is not the same as the heading `θ` when the robot reverses.

## Maps of Dynamics

A Map of Dynamics (MoD) summarises how people move through a place, learned from recorded trajectories. Three
representations are supported, each read from an XML file:

| Map | Content per location | Class | Used by |
|---|---|---|---|
| **CLiFF-map** | a mixture of Gaussians over (heading, speed), each with a mixing factor `π_k`, on a grid of resolution `r` | `MoD::CLiFFMap` | `cliff` and `dtc` objectives |
| **GMMT-map** | `M` clusters of typical trajectories, each a polyline of `K` mean points with a heading per point, a mixing factor and a common standard deviation `σ` | `MoD::GMMTMap` (R-tree over the mean points) | `gmmt` objective |
| **Intensity map** | a scalar `q(x, y) ∈ [0, 1]` per cell: how often the cell is occupied by people | `MoD::IntensityMap` | `intensity` objective, samplers, and as a multiplier in `cliff` / `dtc` |

Occupancy maps use the ROS map_server format (yaml + binary P5 pgm). Bundled environments are listed in
[../maps/README.md](../maps/README.md).

## The cost of a motion

Every planner asks the objective for the cost of an edge between two states `s1 → s2`. The objective splits the
edge into `n = max(1, ceil(distance / step))` sub-segments through the state space's own interpolation (a Dubins
or Reeds-Shepp curve, so the points lie on the actual driven path) and sums, per sub-segment `a → b`:

```
cost = w_d · d  +  w_q · q  +  w_c · c
d = steering distance from a to b             (Dubins / Reeds-Shepp length)
q = 1 − cos²((θ_b − θ_a) / 2)                 (heading change)
c = MoD cost at b for the motion direction α   (below)
```

The cost is **per point**, so the step is part of the definition. The playground infers it as
`min(MoD cell size, occupancy pixel size)` and records it in `config.json` under `Derived.mod_cost_step_m`; the
library default is the cell size of the map given to the objective (`setCostStep` changes it). The unweighted
components are available separately (`motionCostComponents`) and are logged as `cost.d / q / c`.

### MoD cost `c` per objective

| Objective | `c(x, y, α)` |
|---|---|
| `cliff` (upstream criterion on a CLiFF-map) | `Σ_k π_k (1 − cos(μ_k − α))` over the Gaussians at the location; times `q(x, y)` when an intensity map is configured. Zero when moving with the flow, 2 when moving against it. |
| `gmmt` (upstream criterion on a GMMT-map) | `Σ π_j (1 − dist_j / σ)(1 − cos(α − heading_j))` over the nearest cluster mean points `j` |
| `dtc` (Down-The-CLiFF) | `Σ_k min(Mahalanobis((α, v_max) ∥ N(μ_k, Σ_k)), threshold) [· π_k]`, times `q(x, y)` if an intensity map is configured. Degenerate covariances cost the threshold. |
| `intensity` | `q(x, y)`, independent of `α` |
| `path_length` | OMPL's `PathLengthOptimizationObjective`: `c = 0`, cost = steering distance |

Because `c` depends on the motion direction and not on the heading, **reversing through a crowd costs the same
as driving through it the same way**; there is no reverse penalty anywhere in the library.

Published defaults for `w_c`: cliff 0.1, gmmt 0.1, intensity 0.2, dtc 0.02 (with `w_d = w_q = 1`). Note that the
playground's inferred step (typically 0.05 m on ATC) puts about 20 cost points per metre where the original
experiments had about 7, so the MoD term weighs roughly 3× more relative to distance than in the papers.

## Samplers

An OMPL *informed sampler* proposes states to RRT* (and is asked for by AIT* only through the objective's
heuristic). The objective allocates the sampler named in `SamplerParameters`:

| Type | Draw |
|---|---|
| `iid` | uniform over the valid cells of the intensity map when one is configured (as in the papers), else OMPL rejection sampling over the bounds |
| `ellipse` | OMPL's `PathLengthDirectInfSampler`: uniform until a solution exists, then inside the ellipsoid whose transverse diameter is the current best *full MoD cost* |
| `intensity` | with probability `bias` a cell drawn proportional to `1 − q` (quiet cells preferred), otherwise a uniform valid cell; position uniform inside the cell, heading uniform |
| `dijkstra` | with probability `bias` a cell of the Dijkstra shortest path under the planner's own objective on a grid of `dijkstra_cell_size`, heading towards the next path cell ± π/8; otherwise uniform |
| `hybrid` | `bias` → Dijkstra, `hybrid_intensity_bias` → intensity, otherwise the ellipse sampler |

`intensity`, `hybrid` and the cell-uniform `iid` need an intensity map: `SamplerParameters.intensity_map_file`,
or, if empty, the objective's. The Dijkstra grid uses one validity check per node (the footprint is a circle) and
`motionCost` as the edge weight, so it is obstacle- and flow-aware; on ATC at 0.5 m it sets up in about 70 ms.
With `log_samples` every draw is written to `samples.json` with its source.

## Planners

| Type | What it is | Key parameters |
|---|---|---|
| `rrt_star` | OMPL RRT* with the objective's informed sampler | `range` (0: OMPL's default, 20 % of the extent), `goal_bias`, `informed_sampling` |
| `ait_star` | OMPL AIT*; its reverse search uses `motionCostHeuristic`, which the MoD objectives make equal to the exact `motionCost` (as in the papers) | `batch_size` |
| `hybrid_astar` | `MoD::HybridAStar`, below | the `HybridAStarParameters` scope |

Solutions are evaluated with the objective at the end of the budget; the time to the first solution comes from
OMPL's intermediate-solution callback (for Hybrid A* it equals the planning time).

### Hybrid A*

A deterministic search that reuses the planner's state space, bounds, footprint checker, collision resolution and
objective, so its cost is comparable with the sampling planners'.

- **Nodes** keep their exact continuous pose. A node is a duplicate of another when they fall in the same
  `cell_size_m` square and the same heading bin (`angle_bins`, default 72 = 5°), and, with reverse motion, the same
  direction. The best `g` per key is kept.
- **Primitives**: straight, left and right arcs at the vehicle's minimum turning radius, of length
  `cell × √2` so every primitive leaves its cell. Validity by `checkMotion` at the pixel step; cost by
  `motionCost` (the Dubins / Reeds-Shepp interpolation between the two endpoints of such an arc is that arc).
- **Heuristic** `h = max(h_grid, h_kin)`. `h_grid` is a goal-rooted Dijkstra on the search grid with `motionCost`
  as edge weight, expanded lazily as the search touches cells, so it knows about walls and flow. `h_kin` is the
  Dubins distance times `w_d`, memoised per key. Neither is strictly admissible (accepted, as Nav2 does); the
  returned cost is measured, not assumed optimal.
- **Analytic expansion** (Nav2's schedule): every `max(1, floor(h_kin / (analytic_ratio × primitive length)))`
  expansions a Dubins (or Reeds-Shepp) shot to the goal is tried when it is at most `analytic_max_length_m` long;
  the first collision-free shot ends the search. A node landing in the goal's cell and bin also ends it.
- **Reverse motion** (Reeds-Shepp only): six primitives, the shot is a Reeds-Shepp path whose cusps are counted
  from its segment signs, and each direction change adds `change_penalty` (default 1000 cost units, which exceeds
  any forward path on the bundled maps, so cusps appear only where no forward path exists; lower it to allow
  them). The first primitive out of the start is free to pick its direction. `h_kin` becomes
  `w_d · min(Dubins, Reeds-Shepp length + penalty × cusps)`.
- **Termination**: first solution, time budget, cancel, `max_expansions`, or an empty open list. The result
  records which, plus expansions, shots, cusps and the objective cost.

On ATC it takes 0.05-0.2 s per scenario and its cost matched or beat RRT*/AIT* after 30 s (table in
`CHANGELOG.rst`).

## Footprint and collision checking

The vehicle is tested as **one circumscribed circle**: a circle robot uses its radius; a rectangle uses
`sqrt((L/2)² + (W/2)²)`. The disc of pixels around a pose (offsets precomputed once per map) must be free and
inside the map; yaw is irrelevant. OMPL's motion validator checks poses one pixel apart along each edge
(`setStateValidityCheckingResolution(pixel / extent)`), which cannot skip a pixel. Both steps, collision and cost,
are inferred from the maps and never entered by hand.

## State spaces

`dubins` (default; forward only) or `reeds_shepp` (forward and reverse), both with the vehicle's turning radius,
over the occupancy map's bounds. The published bench-mr runs used Dubins although the paper text said
Reeds-Shepp, so Dubins stays the default. OMPL's yaw range is `[-π, π)`.

## Where this comes from

| Component | Paper |
|---|---|
| Down-The-CLiFF (`dtc`) objective, intensity-map sampling | Swaminathan et al., *Down the CLiFF: Flow-Aware Trajectory Planning under Motion Pattern Uncertainty*, IROS 2018 |
| Upstream criterion on CLiFF and GMMT maps, benchmark of all four objectives | Swaminathan et al., *Benchmarking the Utility of Maps of Dynamics for Human-Aware Motion Planning*, Frontiers in Robotics and AI 2022 |
| Dijkstra, intensity and hybrid samplers; ellipsoid heuristic with MoD costs; the ATC experiments and their defaults | Swaminathan et al., *Sampling Functions for Global Motion Planning Using Maps of Dynamics for Mobile Robots*, RAS 2025 |
| CLiFF-map | Kucner et al., *Enabling Flow Awareness for Mobile Robots in Partially Observable Environments*, RA-L 2017 |
| GMMT-map | Bennewitz et al., *Learning Motion Patterns of People for Compliant Robot Motion*, IJRR 2005 |
| Dijkstra-graph sampling that the Dijkstra sampler extends (CLiFF-EUC cost) | Palmieri et al., *Kinodynamic Motion Planning on Gaussian Mixture Fields*, ICRA 2017 |
| Hybrid A* design | Nav2 SmacPlannerHybrid (design description only) |

Full references are in the README's [Background reading](../README.md#background-reading).
