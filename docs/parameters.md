# Parameter reference

All parameters live in `include/mod/parameters.hpp` as plain structs with nlohmann JSON (de)serialisation.
`from_json` starts from the defaults and overrides only the keys present, so partial objects are accepted
everywhere (batch JSON, `--config`, your own code). Enum values are the strings shown. Defaults equal the settings
of the published experiments unless noted.

## `VehicleParameters` — scope `VehicleParameters`

| Key | Default | Meaning |
|---|---|---|
| `shape` | `rectangle` | `circle` or `rectangle` |
| `radius` | 0.4 | circle only [m] |
| `length` | 0.7 | rectangle only, along the heading [m] (the published robot spans x ∈ [-0.2, 0.5]) |
| `width` | 0.4 | rectangle only [m] |
| `state_space` | `dubins` | `dubins` (forward only) or `reeds_shepp` (reverse allowed) |
| `turning_radius` | 1.0 | minimum turning radius [m] |

The footprint test uses the circumscribed circle: `radius`, or `sqrt((length/2)² + (width/2)²)` = 0.403 m for the
default rectangle. There is no resolution field: the collision step and the cost step are inferred (see
`Derived`).

## `OptObjParameters` — scope `OptObjParameters`

| Key | Default | Meaning |
|---|---|---|
| `type` | `cliff` | `cliff`, `gmmt`, `dtc`, `intensity`, `path_length` |
| `w_d` | 1.0 | weight of the steering distance |
| `w_q` | 1.0 | weight of the heading-change term |
| `w_c` | 0.1 | weight of the MoD term (published: cliff 0.1, gmmt 0.1, intensity 0.2, dtc 0.02) |
| `cliff_map_file` | "" | CLiFF-map XML (`cliff`, `dtc`) |
| `gmmt_map_file` | "" | GMMT-map XML (`gmmt`) |
| `intensity_map_file` | "" | intensity XML: the cost for `intensity`, a multiplier for `cliff` and `dtc`, and the default map of the samplers |
| `max_vehicle_speed` | 1.0 | `dtc`: the speed paired with the motion direction in the Mahalanobis distance [m/s] |
| `mahalanobis_threshold` | 10.0 | `dtc`: cap on the Mahalanobis distance per Gaussian |
| `use_mixing_factor` | true | `dtc`: weight each Gaussian's distance by its mixing factor |

## `SamplerParameters` — scope `SamplerParameters`

| Key | Default | Meaning |
|---|---|---|
| `type` | `iid` | `iid`, `ellipse`, `intensity`, `dijkstra`, `hybrid` |
| `bias` | 0.05 | probability of the informed branch (`intensity`: the `1 − q` draw; `dijkstra` and `hybrid`: the Dijkstra path) |
| `dijkstra_cell_size` | 0.5 | grid cell of the Dijkstra sampler [m] |
| `hybrid_intensity_bias` | 0.01 | `hybrid`: probability of the intensity branch |
| `intensity_map_file` | "" | the samplers' intensity map; empty means the objective's |
| `log_samples` | false | write every draw to `samples.json` |

## `PlannerParameters` — scope `PlannerParameters`

| Key | Default | Meaning |
|---|---|---|
| `type` | `rrt_star` | `rrt_star`, `ait_star`, `hybrid_astar` |
| `max_planning_time` | 64.0 | budget [s]; Hybrid A* returns earlier when it has its solution |
| `seed` | 0 | OMPL RNG seed for the run (0 is treated as 1; batches set it to `seed0 + index`) |
| `range` | 0.0 | RRT* steer range [m]; 0 lets OMPL choose (20 % of the space extent) |
| `goal_bias` | 0.05 | RRT* |
| `batch_size` | 100 | AIT* samples per batch |
| `informed_sampling` | true | RRT*: use the objective's informed sampler |

## `HybridAStarParameters` — scope `HybridAStarParameters`

| Key | Default | Meaning |
|---|---|---|
| `cell_size_m` | 0.25 | search cell for duplicate detection and the heuristic grid [m] |
| `angle_bins` | 72 | heading bins for duplicate detection (5°) |
| `primitive_length_m` | 0.0 | arc length of a primitive; 0 means `cell_size_m × √2` |
| `analytic_ratio` | 3.5 | shot every `floor(h_kin / (ratio × primitive length))` expansions (at least every expansion) |
| `analytic_max_length_m` | 5.0 | shots longer than this are not attempted |
| `max_expansions` | 2000000 | cap on expanded nodes |
| `allow_reverse` | true | reverse primitives; only effective under `reeds_shepp` (forced off under `dubins`; the effective value is written back to `config.json`) |
| `change_penalty` | 1000.0 | added to the cost per direction flip [cost units]; there is never a reverse penalty |

## `Scenario` — scope `Scenario`

| Key | Default | Meaning |
|---|---|---|
| `name` | "" | used in the run folder name |
| `map_yaml` | "" | occupancy map (map_server yaml); relative paths resolve against the batch JSON or, in the GUI, the maps folder |
| `start` | [0, 0, 0] | `[x, y, yaw]`; yaw in `[-π, π)` |
| `goal` | [0, 0, 0] | `[x, y, yaw]` |

## `Derived` — scope `Derived` (written by the playground, never read)

| Key | Meaning |
|---|---|
| `occupancy_pixel_m` | pixel size of the occupancy map |
| `mod_cell_m` | cell size of the objective's MoD map (CLiFF resolution, intensity cell; GMMT has none and falls back to the intensity map's, then the pixel) |
| `collision_step_m` | = pixel size; OMPL's validity resolution is `pixel / extent` |
| `mod_cost_step_m` | = `min(mod_cell_m, occupancy_pixel_m)`; the objective's cost step |
| `circumscribed_radius_m` | the footprint radius actually used |

## `RunMeta` — scope `RunMeta` (written by the logger)

`mod_version`, `git_hash` (`git describe --always --dirty --tags` at configure time), `hostname`, `started_at`
(ISO 8601, UTC, milliseconds).

## `RunConfig`

The aggregate of all scopes above; `config.json` is its serialisation:

```json
{
  "VehicleParameters": {...}, "Derived": {...}, "SamplerParameters": {...}, "OptObjParameters": {...},
  "PlannerParameters": {...}, "HybridAStarParameters": {...}, "Scenario": {...}, "RunMeta": {...}
}
```

String helpers: `MoD::to_string(enum)` and `MoD::shapeFromString`, `stateSpaceFromString`,
`samplerTypeFromString`, `objectiveTypeFromString`, `plannerTypeFromString` (throw `std::invalid_argument` on
unknown text).
