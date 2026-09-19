"""Sample cloud by source over the map for one run that has samples.json.

Reads the run's config.json for the map yaml (image, resolution, origin), draws the pgm in grey, then the samples
coloured by source (uniform / ellipse / intensity / dijkstra, fixed colours) and the solution path.

Usage: python plot_samples.py <run_dir> [-o samples.png] [--max-points N]
"""
import argparse
import json
import re
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from runs import PALETTE, load_samples

SOURCE_ORDER = ["uniform", "ellipse", "intensity", "dijkstra"]
SOURCE_COLOUR = dict(zip(SOURCE_ORDER, PALETTE))


def read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    tokens, pos = [], 0
    pattern = re.compile(rb"\s*(#[^\n]*\n)*\s*(\S+)")
    while len(tokens) < 4:
        m = pattern.match(data, pos)
        tokens.append(m.group(2))
        pos = m.end()
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[pos + 1:pos + 1 + w * h], dtype=np.uint8).reshape(h, w)


def read_map_yaml(path: Path):
    d = {}
    for line in path.read_text().splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            d[k.strip()] = v.strip()
    origin = json.loads(d["origin"])
    image = Path(d["image"])
    if not image.is_absolute():
        image = path.parent / image
    return image, float(d["resolution"]), origin


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir")
    ap.add_argument("-o", "--output", default="samples.png")
    ap.add_argument("--max-points", type=int, default=50000)
    args = ap.parse_args()
    run = Path(args.run_dir)
    cfg = json.loads((run / "config.json").read_text())
    sol = json.loads((run / "solution.json").read_text())
    samples = load_samples(run)
    if len(samples) > args.max_points:
        samples = samples.sample(args.max_points, random_state=0)

    fig, ax = plt.subplots(figsize=(12, 6))
    image, res, origin = read_map_yaml(Path(cfg["Scenario"]["map_yaml"]))
    img = read_pgm(image)
    h, w = img.shape
    ax.imshow(img, cmap="gray", extent=(origin[0], origin[0] + w * res, origin[1], origin[1] + h * res),
              origin="upper", interpolation="nearest")
    for source in SOURCE_ORDER:
        g = samples[samples["source"] == source]
        if g.empty:
            continue
        ax.scatter(g["x"], g["y"], s=4, color=SOURCE_COLOUR[source], alpha=0.6, linewidths=0,
                   label=f"{source} ({len(g)})")
    path = np.array(sol.get("path") or [])
    if len(path):
        ax.plot(path[:, 0], path[:, 1], "-", color="#1a1a19", linewidth=2, label="solution path")
    s, g = cfg["Scenario"]["start"], cfg["Scenario"]["goal"]
    ax.plot(s[0], s[1], "o", color="#008300", markersize=8, label="start")
    ax.plot(g[0], g[1], "s", color="#c8102e", markersize=8, label="goal")
    ax.set_title(f"{cfg['Scenario']['name']}: {cfg['PlannerParameters']['type']} / {cfg['SamplerParameters']['type']}"
                 f" / {cfg['OptObjParameters']['type']}")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=6, frameon=False, fontsize=8, markerscale=3)
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    matplotlib.use("Agg")
    main()
