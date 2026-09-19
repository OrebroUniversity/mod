# Using the C++ library

Link `mod::mod` (see [getting-started.md](getting-started.md#install-and-use-from-another-project)). Headers:

```
mod/parameters.hpp                       parameter structs, enums, JSON
mod/cliffmap.hpp                         MoD::CLiFFMap, MoD::IntensityMap
mod/gmmtmap.hpp                          MoD::GMMTMap
mod/grid_dijkstra.hpp                    MoD::GridDijkstra (header-only)
mod/planners/hybrid_astar.hpp            MoD::HybridAStar
mod/sample_sink.hpp                      MoD::SampleSink
mod/log.hpp                              MOD_LOG
mod/version.h                            MOD_VERSION, MOD_GIT_HASH (generated)
ompl/mod/objectives/*.h                  ompl::MoD::{DTC,UpstreamCriterion,IntensityMap}OptimizationObjective
ompl/mod/samplers/*.h                    ompl::MoD::{IntensityMap,Dijkstra,Hybrid,Recording}Sampler
```

Two namespaces: `MoD` for the library's own types and `ompl::MoD` for the OMPL-derived objectives and samplers.
(A global `mod` namespace is deliberately avoided: it shadows a call inside Boost.Geometry.)

## Maps

```cpp
MoD::CLiFFMap cliff("maps/atc/atc_cliff.xml", /*organize=*/true);   // organized = grid lookup by (x, y)
const MoD::CLiFFMapLocation &loc = cliff(x, y);                      // static empty location when out of range
for (const auto &d : loc.distributions) d.getMeanHeading(), d.getMeanSpeed(), d.getMixingFactor(), d.getCovariance();
cliff.getBestHeading(x, y); cliff.getLikelihood(x, y, heading, speed);

MoD::GMMTMap gmmt("maps/atc/atc_gmmt.xml");
for (const auto &[point, id] : gmmt(x, y))            // nearest mean points: id = {cluster, mean index}
  gmmt.getMixingFactorByClusterID(id[0]), gmmt.getHeadingAtDist(id[0], id[1]);

MoD::IntensityMap q("maps/atc/atc_intensity1m.xml");
q(x, y);                                              // 0 outside the map
```

Share maps between threads as `shared_ptr<const T>` (`CLiFFMapConstPtr`, `GMMTMapConstPtr`,
`IntensityMapConstPtr`); every getter is const.

## Objectives

```cpp
MoD::OptObjParameters op;  op.type = MoD::ObjectiveType::dtc; op.cliff_map_file = ...; op.w_c = 0.02;
MoD::SamplerParameters sp; sp.type = MoD::SamplerType::hybrid; sp.intensity_map_file = ...;

auto objective = std::make_shared<ompl::MoD::DTCOptimizationObjective>(si, op, sp);          // loads the maps
// or hand in preloaded maps to share them:
auto objective = std::make_shared<ompl::MoD::DTCOptimizationObjective>(si, op, sp, cliff_ptr, intensity_ptr);
auto upstream  = std::make_shared<ompl::MoD::UpstreamCriterionOptimizationObjective>(si, op, sp, cliff_ptr, gmmt_ptr, intensity_ptr);
auto intensity = std::make_shared<ompl::MoD::IntensityMapOptimizationObjective>(si, op, sp, intensity_ptr);

objective->setCostStep(0.05);                        // metres between cost points; default: the map's cell size
pdef->setOptimizationObjective(objective);
```

`UpstreamCriterionOptimizationObjective` picks the GMMT-map when `op.type == gmmt`, else the CLiFF-map. All three
derive from `ompl::MoD::MoDOptimizationObjective`, which provides:

| Member | Meaning |
|---|---|
| `motionCost(s1, s2)` | the weighted per-point integral of [concepts.md](concepts.md#the-cost-of-a-motion) |
| `motionCostComponents(s1, s2)` | the unweighted `{d, q, c}` |
| `motionCostHeuristic(s1, s2)` | equals `motionCost` (AIT*'s reverse search pays the full integral; a settled decision) |
| `stateCost` | 0; `isSymmetric()` is false |
| `setCostStep`, `getCostStep` | the sub-segment length |
| `allocInformedStateSampler(pdef, maxCalls)` | the sampler of `SamplerParameters.type` (falls back to OMPL rejection sampling and logs when a needed intensity map is missing) |
| `setSampleSink(sink)` | samplers allocated afterwards record every draw into `sink` |
| `setSamplerIntensityMap(map)` | replace the samplers' map with a preloaded one |
| `getParameters()`, `getSamplerParameters()`, `getIntensityMap()` | accessors |

## Samplers

Normally you do not construct samplers: RRT* calls `allocInformedStateSampler` on the objective. To use one
directly:

```cpp
auto sampler = ompl::MoD::DijkstraSampler::allocate(pdef, /*maxCalls=*/100, sp, intensity_ptr);
sampler->sampleUniform(state, ompl::base::Cost(std::numeric_limits<double>::infinity()));
```

`DijkstraSampler` exposes its grid and path (`grid()`, `path()`, `cellSize()`); `HybridSampler` has a test hook
for the chosen branch. Implement `MoD::SampleSink::record(x, y, theta, source)` to observe draws.

## Hybrid A*

```cpp
#include <mod/planners/hybrid_astar.hpp>

MoD::HybridAStarParameters hp;                      // defaults in parameters.md
MoD::HybridAStar planner(si, objective, hp, /*turning_radius=*/1.0);
ompl::geometric::PathGeometric path = planner.solve(start, goal, /*time_budget_s=*/10.0,
                                                    [&] { return cancel_requested.load(); });
const auto &r = planner.result();     // solved, cost, objective_cost, cusps, expansions, generated,
                                      // analytic_attempts, solved_by_shot, planning_time_s, termination
planner.expandedNodes();              // closed set: x, y, yaw, heading bin, direction (for drawing)
planner.pathDirections();             // forward / reverse per path segment
planner.heuristic(state);             // h for the last goal (also heuristicGrid / heuristicKinematic)
```

`objective` may be any `ompl::base::OptimizationObjectivePtr` (an MoD objective or OMPL's path length);
`w_d` for the kinematic heuristic is read from an MoD objective's parameters, else 1. The state space must be
SE(2)-based; reverse motion is enabled only on `ReedsSheppStateSpace`. `solve` may be called repeatedly. The
path is empty on failure; `result().termination` says why (`time`, `cancelled`, `max_expansions`,
`exhausted`, `invalid`).

`MoD::GridDijkstra` (header-only) is the 8-neighbour grid search behind both the Dijkstra sampler and the
Hybrid A* heuristic: per-node validity cache, edge weights from a callback evaluated on pop, forward or reverse
(cost-to-go) mode, lazy `costTo(node)`, `pathTo(node)`.

## Thread safety

Objectives, samplers and `HybridAStar` hold no mutable per-call state apart from scratch states owned by the
instance; maps are read-only and shared through `shared_ptr<const>`. The rule is one objective, sampler and
planner per thread; maps may be shared freely. `ompl::RNG::setSeed` is global: seed once before starting
threads. `MOD_LOG` writes single lines with `fprintf`, which is safe from several threads.

## The playground core (optional)

`libmod_playground.a` (namespace `MoD::playground`, headers under `src/playground/core/`) is what the tools and
the GUI are built on and can be reused: `OccupancyMap` (yaml + pgm or in-memory), `FootprintChecker`,
`MapCache`, `PlannerFactory::build(RunConfig&, MapCache&) → PlannerSetup` (state space, checker, objective,
problem definition, planner or Hybrid A*), `Solver`, `RunLogger`, and `batch.hpp` (`loadBatch`, `expand`,
`runOne`). It is not installed; use it through `add_subdirectory`.
