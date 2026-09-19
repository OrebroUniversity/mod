# Office cubicles (simulated)

Copied from bench-mr (https://github.com/ksatyaki/bench-mr, MIT) `maps/`: `office_cubicles.yaml` / `.pgm`
(occupancy map of a simulated office, 0.1 m per pixel, origin (-10, -10)) and the CLiFF-map, GMMT-map and intensity
map learned from simulated pedestrian trajectories (`office_cubicles_cliffmap.xml`, `office_cubicles_gmmtmap.xml`,
`office_cubicles_intensitymap.xml`). `scenarios_office_cubicles.json` holds the four start/goal pairs of bench-mr
`python/sgs/sg-pairs-pedsim.yaml`; `batch_office_smoke.json` runs them (paths relative to the JSON). With the circumscribed-circle footprint (radius
0.403 m) `office_cubicles3a` is infeasible forward-only: its start (19, 19, 0.785) faces the room corner, so no
Dubins planner finds a path (bench-mr used a polygon footprint). Under Reeds-Shepp, Hybrid A* backs out and solves it.
