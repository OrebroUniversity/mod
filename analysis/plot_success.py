"""Success rate vs planning time per sampler / planner (Paper IV figure style).

For every (objective, planner, sampler) group the curve is the fraction of runs whose time_to_first_solution_s
is <= t, for t from 0 to the largest max_planning_time. One panel per objective, one line per planner x sampler:
colour = sampler (fixed order), line style = planner.

Figure design (success rate vs planning time per sampler / planner) follows bench-mr
(MIT, Eric Heiden; MoD additions by Chittaranjan Swaminathan), python/plot_convergence_mod.py.

Usage: python plot_success.py <runs_dir> [-o success.png]
"""
import argparse

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from runs import OBJECTIVE_ORDER, PLANNER_ORDER, PLANNER_STYLE, SAMPLER_COLOUR, SAMPLER_ORDER, load_runs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("runs_dir")
    ap.add_argument("-o", "--output", default="success.png")
    args = ap.parse_args()
    df = load_runs(args.runs_dir)
    if df.empty:
        raise SystemExit("no runs found")

    objectives = [o for o in OBJECTIVE_ORDER if o in set(df["objective.type"])]
    t_max = float(df["planner.max_planning_time"].max())
    t = np.linspace(0.0, t_max, 200)
    fig, axes = plt.subplots(1, len(objectives), figsize=(5.0 * len(objectives), 3.6), squeeze=False, sharey=True)
    for ax, objective in zip(axes[0], objectives):
        sub = df[df["objective.type"] == objective]
        for planner in [p for p in PLANNER_ORDER if p in set(sub["planner.type"])]:
            for sampler in [s for s in SAMPLER_ORDER if s in set(sub["sampler.type"])]:
                g = sub[(sub["planner.type"] == planner) & (sub["sampler.type"] == sampler)]
                if g.empty:
                    continue
                first = g["time_to_first_solution_s"].to_numpy(dtype=float)
                rate = [(np.nan_to_num(first, nan=np.inf) <= ti).mean() for ti in t]
                ax.plot(t, rate, PLANNER_STYLE[planner], color=SAMPLER_COLOUR[sampler], linewidth=2,
                        label=f"{planner} / {sampler} (n={len(g)})")
        ax.set_title(f"objective: {objective}")
        ax.set_xlabel("planning time [s]")
        ax.set_ylim(0, 1.02)
        ax.set_xlim(0, t_max)
        ax.grid(True, color="#e6e6e3", linewidth=0.8)
        ax.spines[["top", "right"]].set_visible(False)
    axes[0][0].set_ylabel("success rate (first solution found)")
    axes[0][-1].legend(loc="lower right", frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    matplotlib.use("Agg")
    main()
