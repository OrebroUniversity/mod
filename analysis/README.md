# Batch run to plots

1. Build: `cmake -S . -B build && cmake --build build -j` (needs OMPL, Boost, Eigen, nlohmann_json, gtest, glfw).
2. Write a batch JSON (see `test/data/atc/batch_atc_smoke.json`: scenarios x planners x samplers x objectives x repeats; map paths relative to the JSON).
3. Run it: `./build/bin/run_batch test/data/atc/batch_atc_smoke.json --threads 4 --log-dir runs/atc` (one folder per run with `config.json`, `solution.json`, optional `samples.json`).
4. Load everything: `python analysis/runs.py runs/atc` (or `from runs import load_runs` for a DataFrame with `sampler.type`, `planner.type`, `cost_total`, ...).
5. Figures: `python analysis/plot_success.py runs/atc -o success.png` and `python analysis/plot_cost.py runs/atc -o cost.png`.
6. Sample cloud of one run logged with `log_samples`: `python analysis/plot_samples.py runs/atc/<run folder> -o samples.png`.

`sampler_bug_audit.py` / `results_atc.json` are the pre-2.0 audit of the two sampler bugs fixed in M2.
