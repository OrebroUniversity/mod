"""Median cost at the final planning time per sampler / objective, as box plots.

One panel per objective; boxes grouped by sampler (fixed order, fixed colour) and split by planner. Failed runs
(no exact solution) are excluded and counted in the x tick label.

Figure design (cost box plots per sampler / planner) follows bench-mr (MIT, Eric Heiden;
MoD additions by Chittaranjan Swaminathan), python/plot_stats.py.

Usage: python plot_cost.py <runs_dir> [-o cost.png] [--metric cost_total|cost_c|path_length_m]
"""
import argparse

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from runs import OBJECTIVE_ORDER, PLANNER_ORDER, SAMPLER_COLOUR, SAMPLER_ORDER, load_runs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("runs_dir")
    ap.add_argument("-o", "--output", default="cost.png")
    ap.add_argument("--metric", default="cost_total")
    args = ap.parse_args()
    df = load_runs(args.runs_dir)
    if df.empty:
        raise SystemExit("no runs found")

    objectives = [o for o in OBJECTIVE_ORDER if o in set(df["objective.type"])]
    fig, axes = plt.subplots(1, len(objectives), figsize=(5.0 * len(objectives), 3.8), squeeze=False)
    for ax, objective in zip(axes[0], objectives):
        sub = df[df["objective.type"] == objective]
        planners = [p for p in PLANNER_ORDER if p in set(sub["planner.type"])]
        samplers = [s for s in SAMPLER_ORDER if s in set(sub["sampler.type"])]
        width = 0.8 / max(1, len(planners))
        ticks, labels = [], []
        for si, sampler in enumerate(samplers):
            for pi, planner in enumerate(planners):
                g = sub[(sub["sampler.type"] == sampler) & (sub["planner.type"] == planner)]
                ok = g[g["success"]][args.metric].dropna()
                x = si + (pi - (len(planners) - 1) / 2) * width
                if len(ok):
                    box = ax.boxplot([ok.to_numpy()], positions=[x], widths=width * 0.85, patch_artist=True,
                                     showfliers=True, medianprops={"color": "#1a1a19", "linewidth": 1.5})
                    for patch in box["boxes"]:
                        patch.set_facecolor(SAMPLER_COLOUR[sampler])
                        patch.set_alpha(0.55 if planner == planners[0] else 0.25)
                        patch.set_edgecolor(SAMPLER_COLOUR[sampler])
                    ax.text(x, ok.median(), f"{ok.median():.1f}", ha="center", va="bottom", fontsize=7,
                            color="#1a1a19")
                ticks.append(x)
                labels.append(f"{sampler}\n{planner}\n{len(ok)}/{len(g)} ok")
        ax.set_xticks(ticks)
        ax.set_xticklabels(labels, fontsize=7)
        ax.set_title(f"objective: {objective}")
        ax.set_ylabel(args.metric)
        ax.grid(True, axis="y", color="#e6e6e3", linewidth=0.8)
        ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    matplotlib.use("Agg")
    main()
