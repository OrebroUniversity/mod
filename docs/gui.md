# The playground GUI

`mod_playground_gui` is an ImGui/GLFW app for setting up one planning problem interactively, solving it and
looking at what the planner did on top of the map and the MoD overlays. Every solve is logged exactly like a
batch run, so a GUI experiment can be reproduced with `run_batch` from its `config.json`.

```
mod_playground_gui [--config run.json] [--map map.yaml] [--log-dir DIR] [--maps-dir DIR]
                   [--solve] [--exit-after-solve] [--screenshot out.ppm]
```

| Flag | Meaning |
|---|---|
| `--config run.json` | Load a full `RunConfig` (the `config.json` of any run folder, or a hand-written one with the same scopes; partial objects are fine, missing keys keep their defaults) |
| `--map map.yaml` | Load this occupancy map (overrides the config's scenario map) |
| `--log-dir DIR` | Where run folders go (default `runs`) |
| `--maps-dir DIR` | Folder scanned for map files (default: the source tree's `maps/`, compiled in) |
| `--solve` | Start solving immediately |
| `--exit-after-solve` | Quit when the solve ends (scripting) |
| `--screenshot out.ppm` | Write the framebuffer after the solve (or the first frame when not solving) as a binary PPM |

Example of a scripted run that produces a picture:

```bash
./build/bin/mod_playground_gui --config runs/atc_smoke/<run>/config.json --solve --exit-after-solve --screenshot shot.ppm
```

## Canvas controls

| Action | Effect |
|---|---|
| hold **S** + left click | set the start position; keep the button down and drag to set its heading |
| hold **G** + left click | set the goal position and heading the same way |
| right drag | pan |
| mouse wheel | zoom about the cursor |
| **F** | fit the map to the view |

The cursor's world coordinates are shown in the bottom-left corner. Start (green) and goal (red) are drawn with
their heading arrow and the footprint circle.

## The panel

**Map & scenario.** The map yaml (with a `v` picker over every yaml found under the maps folder, a *Load* button,
and *Rescan maps*), the scenario name used in the run folder name, and the start/goal poses as numbers.

**Planner.** `rrt_star` (range, goal bias, informed sampling on/off), `ait_star` (batch size) or `hybrid_astar`
(cell size, angle bins, primitive length, analytic-expansion ratio and maximum length, expansion cap, and under
Reeds-Shepp *allow reverse* and *change penalty*). Plus the time budget and the seed (0 means 1; the seed is global
to OMPL's RNG).

**Vehicle.** State space (Dubins or Reeds-Shepp), turning radius, shape (circle or rectangle) and size; the
resulting circumscribed radius is displayed.

**Objective.** Type (`cliff`, `gmmt`, `dtc`, `intensity`, `path_length`), the weights `w_d`, `w_q`, `w_c`, the
map files (each with a picker that lists only files of the right kind, detected from the file content), and the
DTC extras (max speed, Mahalanobis threshold, mixing factor).

**Sampler.** Type (`iid`, `ellipse`, `intensity`, `dijkstra`, `hybrid`), bias, Dijkstra cell size, hybrid
intensity bias, the sampler's own intensity map (empty: the objective's), and *log samples* (writes
`samples.json`).

**Overlays.** CLiFF mean arrows (length ∝ mixing factor), GMMT cluster polylines (width ∝ mixing factor),
intensity heat, planner tree (RRT*/AIT* graph from `PlannerData`), expanded nodes (Hybrid A*'s closed set, coloured
by heading bin, rings for reverse arrivals), solution path (dense interpolation plus the path's states).

**Solve / Cancel** run on a worker thread; the status line reports cost, length, time to first solution and the
run folder.

## File pickers and bare names

On start the app scans the maps folder recursively. `.yaml` files are occupancy maps; `.xml` files are classified
by content (GMMT: `<clusters>`, intensity: `<cell_size>`, CLiFF: `<map version=...>`). Each file field's `v`
button lists the matching files by their path relative to the folder and fills in the absolute path.

A bare file name such as `atc_cliff.xml`, typed in a field or read from a config, is resolved when the map is
loaded, an overlay is drawn or a solve starts: first as given, then under the maps folder, then by file name among
the scanned files. The resolved absolute path is what ends up in the run's `config.json`.

## What a solve does

1. Copies the panel into a `RunConfig`, resolves the file names and seeds OMPL's RNG.
2. Builds the state space over the map bounds, the footprint checker, the objective and sampler, and the planner
   (see [concepts.md](concepts.md)); fills the `Derived` scope (pixel size, MoD cell, cost step, radius).
3. Creates the run folder and writes `config.json`.
4. Solves for the time budget (or until cancelled); Hybrid A* returns as soon as it has its single solution.
5. Writes `solution.json` (and `samples.json` if enabled) and collects the overlays.
