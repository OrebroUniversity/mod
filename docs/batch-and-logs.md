# Batch runs and run logs

## `run_batch`

```
run_batch <batch.json> [--threads N] [--log-dir DIR] [--dry-run]
```

Expands the batch into its runs, prints the count, and runs them `N` at a time, one thread per run (own state
space, objective, sampler and planner; only the loaded maps are shared). `--dry-run` lists the runs and exits.
The exit code is 1 if any run threw (a failed solve is not an error; it is a run with `success: false`). Progress
lines show, per run, the scenario, planner, sampler, objective, seed, ok/FAIL, time to first solution, cost, length
and the run folder.

Relative paths inside the JSON (maps, `log_dir`) are resolved against the JSON file's folder, so the bundled
batches can be run from anywhere.

## Batch JSON

```json
{
  "log_dir": "../../runs/atc_smoke",
  "threads": 4,
  "repeats": 2,
  "seed0": 1,
  "vehicle":      { "shape": "rectangle", "length": 0.7, "width": 0.4, "state_space": "dubins", "turning_radius": 1.0 },
  "hybrid_astar": { "cell_size_m": 0.25, "angle_bins": 72 },
  "scenarios":  [ { "name": "atc-scenario1", "map_yaml": "atc.yaml", "start": [47.69, -18.848, -2.356], "goal": [-19.575, 12.39, 2.313] } ],
  "planners":   [ { "type": "rrt_star", "max_planning_time": 10.0 }, { "type": "hybrid_astar", "max_planning_time": 10.0 } ],
  "samplers":   [ { "type": "dijkstra", "bias": 0.05, "dijkstra_cell_size": 0.5, "intensity_map_file": "atc_intensity1m.xml" } ],
  "objectives": [ { "type": "cliff", "w_c": 0.1, "cliff_map_file": "atc_cliff.xml", "intensity_map_file": "atc_intensity1m.xml" } ]
}
```

| Key | Meaning |
|---|---|
| `log_dir` | where the run folders go (`--log-dir` overrides) |
| `threads` | parallel runs (`--threads` overrides; 0 means 1) |
| `repeats` | runs per combination |
| `seed0` | run `i` of the expansion gets `seed0 + i`; OMPL's global RNG is seeded once with `seed0` |
| `vehicle`, `hybrid_astar` | one scope each, copied into every run |
| `scenarios`, `planners`, `samplers`, `objectives` | lists; the batch is their product |

Each list element is the corresponding parameter object of [parameters.md](parameters.md); keys you leave out
keep their defaults. The loop order is scenario → planner → sampler → objective → repeat. A sampler entry is
required even for `hybrid_astar` (it is ignored by that planner but named in the run folder).

Bundled batches: `maps/atc/batch_atc_smoke.json` (48 runs, about a minute), `maps/atc/batch_atc_hybrid.json`
(720 runs, the Hybrid A* comparison), `maps/office_cubicles/batch_office_smoke.json` (8 runs).

## Run folders

```
<log_dir>/<YYYYMMDD-HHMMSS.mmm>_<scenario>_<planner>_<sampler>_<objective>/
    config.json      the complete RunConfig, one object per scope
    solution.json    the result
    samples.json     only with sampler.log_samples
```

Colliding names in the same millisecond get a numeric suffix. The GUI writes the same folders.

### `config.json`

One object per scope, never a flat key list. The scopes are `VehicleParameters`, `Derived`, `SamplerParameters`,
`OptObjParameters`, `PlannerParameters`, `HybridAStarParameters`, `Scenario` and `RunMeta`; their fields are in
[parameters.md](parameters.md). `Derived` is computed from the maps and the vehicle (pixel size, MoD cell size,
collision step, cost step, circumscribed radius) and `RunMeta` by the logger (library version, `git describe`
of the build, hostname, ISO 8601 start time). A `config.json` can be fed back to the GUI with `--config`, or
copied into a batch's lists.

### `solution.json`

```json
{
  "success": true,
  "planning_time_s": 10.01,
  "time_to_first_solution_s": 0.83,
  "cost": { "total": 66.8, "d": 50.3, "q": 1.9, "c": 145.7 },
  "path_length_m": 50.34,
  "path": [[x, y, theta], ...]
}
```

`success` is true only for an *exact* solution (approximate RRT* solutions count as failures). `cost.total` is
the objective's value of the path; `d`, `q`, `c` are the unweighted components (for `path_length`, `d` is the
length and the others are 0). On failure the numeric fields are `null` and `path` is empty. `path` holds the
planner's states (for Hybrid A*, the primitive endpoints and the goal); interpolate with the state space to get
the driven curve.

### `samples.json`

`[[x, y, theta, source], ...]` for every draw of the informed sampler, `source ∈ {uniform, ellipse, intensity,
dijkstra}`. Off by default (large).

## Analysis

`analysis/runs.py` loads every run folder below a directory into one pandas DataFrame (scopes flattened as
`vehicle.*`, `sampler.*`, `objective.*`, `planner.*`, `hybrid_astar.*`, `scenario.*`, `meta.*`, plus the solution
fields). The plot scripts take that directory:

```bash
python3 analysis/runs.py runs/atc_hybrid                        # summary table per scenario / planner / sampler / objective
python3 analysis/plot_success.py runs/atc_hybrid -o success.png # success rate vs planning time, one panel per objective
python3 analysis/plot_cost.py runs/atc_hybrid -o cost.png [--metric cost_total|cost_c|path_length_m]
python3 analysis/plot_samples.py runs/atc_hybrid/<run> -o samples.png   # sample cloud of one run with samples.json
```

Colours follow the sampler and line styles the planner, in a fixed order, so figures from different batches are
comparable. See [../analysis/README.md](../analysis/README.md).

## `check_interpolation`

```
check_interpolation <maps/atc> [--time S] [--objective cliff|gmmt|dtc|intensity]
```

A diagnostic that wraps the objective in a counting decorator, runs one RRT* and one AIT* solve on ATC
scenario 1 and prints how long the edges handed to `motionCost` are and how many cost points each evaluates. Its
findings are recorded in `CHANGELOG.rst` (2.0.0).
