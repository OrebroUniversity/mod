# Maps

One folder per environment: the occupancy map (map_server yaml + P5 pgm), the MoD maps (CLiFF, GMMT, intensity XML)
and the scenario / batch JSONs. The tests read `maps/` through `MOD_TEST_DATA_DIR`; the batch runner resolves
relative paths against the JSON's folder; the GUI lists these files in its pickers and resolves bare file names
against this folder (`--maps-dir` overrides the compiled-in path).

| Folder | Occupancy | MoD maps | Scenarios |
|---|---|---|---|
| `atc` | `atc.yaml` (2800 x 1200 px, 0.05 m, origin (-60, -40)) | `atc_cliff.xml`, `atc_gmmt.xml`, `atc_intensity1m.xml` | `scenarios_atc.json` (six Paper IV pairs), `batch_atc_smoke.json`, `batch_atc_hybrid.json` |
| `warehouse` | `pedsim_warehouse.yaml` (0.1 m, origin (-10, -10)) | `pedsim_warehouse_cliff.xml`, `pedsim_warehouse_gmmt.xml`, `pedsim_warehouse_intensity1m.xml` | none in bench-mr |
| `office_cubicles` | `office_cubicles.yaml` (0.1 m, origin (-10, -10)) | `office_cubicles_cliffmap.xml`, `office_cubicles_gmmtmap.xml`, `office_cubicles_intensitymap.xml` | `scenarios_office_cubicles.json` (four bench-mr pairs; `3a` needs Reeds-Shepp, see its README), `batch_office_smoke.json` |

All copied from the bench-mr fork (https://github.com/ksatyaki/bench-mr, MIT); provenance in `3rd_party_licenses.md`.
