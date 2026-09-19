"""Load mod playground run folders into one pandas DataFrame.

Each run folder holds config.json (RunConfig, one object per scope) and solution.json. The config scopes are
flattened with scope prefixes: VehicleParameters -> vehicle.*, Derived -> derived.*, SamplerParameters -> sampler.*,
OptObjParameters -> objective.*, PlannerParameters -> planner.*, HybridAStarParameters -> hybrid_astar.*,
Scenario -> scenario.*, RunMeta -> meta.*.
Solution fields: success, planning_time_s, time_to_first_solution_s, cost_total, cost_d, cost_q, cost_c,
path_length_m, n_path_states, has_samples, run_dir.

Usage: python runs.py <runs_dir>   (prints a summary)
"""
import json
import sys
from pathlib import Path

import pandas as pd

SCOPES = {
    "VehicleParameters": "vehicle",
    "Derived": "derived",
    "SamplerParameters": "sampler",
    "OptObjParameters": "objective",
    "PlannerParameters": "planner",
    "HybridAStarParameters": "hybrid_astar",
    "Scenario": "scenario",
    "RunMeta": "meta",
}

# Fixed categorical order for the samplers and planners (never cycled; colours follow the entity).
SAMPLER_ORDER = ["iid", "ellipse", "intensity", "dijkstra", "hybrid"]
PLANNER_ORDER = ["rrt_star", "ait_star", "hybrid_astar"]
OBJECTIVE_ORDER = ["cliff", "gmmt", "dtc", "intensity", "path_length"]
# Validated categorical palette (light surface), slots 1-6.
PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
SAMPLER_COLOUR = dict(zip(SAMPLER_ORDER, PALETTE))
PLANNER_STYLE = {"rrt_star": "-", "ait_star": "--", "hybrid_astar": ":"}


def load_run(folder: Path) -> dict | None:
    cfg_file, sol_file = folder / "config.json", folder / "solution.json"
    if not cfg_file.is_file() or not sol_file.is_file():
        return None
    cfg = json.loads(cfg_file.read_text())
    sol = json.loads(sol_file.read_text())
    row = {"run_dir": str(folder), "run_name": folder.name}
    for scope, prefix in SCOPES.items():
        for key, value in cfg.get(scope, {}).items():
            if isinstance(value, list):
                for i, v in enumerate(value):
                    row[f"{prefix}.{key}[{i}]"] = v
            else:
                row[f"{prefix}.{key}"] = value
    row["success"] = bool(sol.get("success", False))
    row["planning_time_s"] = sol.get("planning_time_s")
    row["time_to_first_solution_s"] = sol.get("time_to_first_solution_s")
    cost = sol.get("cost", {}) or {}
    for k in ("total", "d", "q", "c"):
        row[f"cost_{k}"] = cost.get(k)
    row["path_length_m"] = sol.get("path_length_m")
    row["n_path_states"] = len(sol.get("path", []) or [])
    row["has_samples"] = (folder / "samples.json").is_file()
    return row


def load_runs(root) -> pd.DataFrame:
    """All run folders below `root` (any depth), one row each."""
    root = Path(root)
    rows = [r for r in (load_run(f.parent) for f in root.rglob("config.json")) if r]
    df = pd.DataFrame(rows)
    for col in ("time_to_first_solution_s", "cost_total", "cost_d", "cost_q", "cost_c", "path_length_m"):
        if col in df:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def load_samples(run_dir) -> pd.DataFrame:
    """samples.json of one run as a DataFrame with columns x, y, theta, source."""
    rows = json.loads((Path(run_dir) / "samples.json").read_text())
    return pd.DataFrame(rows, columns=["x", "y", "theta", "source"])


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    df = load_runs(sys.argv[1])
    print(f"{len(df)} runs")
    if len(df):
        group = ["scenario.name", "planner.type", "sampler.type", "objective.type"]
        print(df.groupby(group).agg(runs=("success", "size"), solved=("success", "sum"),
                                    first_s=("time_to_first_solution_s", "median"),
                                    cost=("cost_total", "median")).to_string())
