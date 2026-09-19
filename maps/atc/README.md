# ATC

Copied from bench-mr (https://github.com/ksatyaki/bench-mr, MIT) `maps/` and `python/sg-pairs-atc.yaml`:

- `atc.yaml`, `atc_white_fixed.pgm`: occupancy map of the ATC shopping centre (Osaka), 2800 x 1200 px at 0.05 m,
  origin (-60, -40). Built from the ATC pedestrian dataset (Brščić, Kanda, Ikeda, Miyashita, IEEE THMS 2013).
- `atc_cliff.xml`, `atc_gmmt.xml`, `atc_intensity1m.xml`: CLiFF-map, GMMT-map and intensity map learned from the
  same trajectories (Maps of Dynamics, Örebro University).
- `scenarios_atc.json`: the six Paper IV start/goal pairs; `batch_atc_smoke.json`: the M4 acceptance batch;
  `batch_atc_hybrid.json`: the Hybrid A* / RRT* / AIT* comparison (2.1.0). Batch paths are relative to the JSON.
