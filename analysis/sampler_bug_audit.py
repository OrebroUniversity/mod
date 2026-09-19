"""Quantify two sampler bugs in mod by re-implementing the exact C++ logic in numpy.

Bug A: IntensityMapSampler uniform-valid branch draws sampled_value from [0, sum(1-q))
       but walks nonq_map (weight 1 each) -> only a prefix of valid cells is reachable.
Bug B: DijkstraSampler heading uses path_.begin() instead of the neighbour cell.
"""
import base64, io, json, math, re, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

MAPS = "/home/ksatyaki/ws/phd/bench-mr/maps"
OUT = "."
rng = np.random.default_rng(1)

# palette (validated separately): ink, muted, sampled, never, obstacle, buggy, intended
C = dict(sampled="#9bb5a6", never="#b5402b", obstacle="#2b2f33", free="#f4f4f1",
         buggy="#b5402b", intended="#0f7fc0", path="#6b6b66")

def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    # P5 header with optional comments
    tokens, pos = [], 0
    while len(tokens) < 4:
        m = re.compile(rb"\s*(#[^\n]*\n)*\s*(\S+)").match(data, pos)
        tokens.append(m.group(2)); pos = m.end()
    w, h, maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])
    img = np.frombuffer(data[pos + 1:pos + 1 + w * h], dtype=np.uint8).reshape(h, w)
    return img

def read_yaml(path):
    d = {}
    for line in open(path):
        if ":" in line:
            k, v = line.split(":", 1); d[k.strip()] = v.strip()
    origin = json.loads(d["origin"])
    return float(d["resolution"]), origin, float(d["occupied_thresh"])

def read_intensity(path):
    txt = open(path).read()
    p = lambda k: float(re.search(rf"<{k}>([^<]+)</{k}>", txt).group(1))
    cs, xmin, ymin, xmax, ymax = p("cell_size"), p("x_min"), p("y_min"), p("x_max"), p("y_max")
    rows = int((ymax - ymin) / cs) + 1; cols = int((xmax - xmin) / cs) + 1
    vals = np.zeros(rows * cols)
    for r, c, v in re.findall(r"<row>(\d+)</row>\s*<col>(\d+)</col>\s*<value>([^<]+)</value>", txt):
        i = int(r) * cols + int(c)
        if i < rows * cols: vals[i] = float(v)
    return dict(cs=cs, xmin=xmin, ymin=ymin, xmax=xmax, ymax=ymax, rows=rows, cols=cols, vals=vals)

class Occ:
    """Point validity check equivalent to MRPTGridMap::collides(x,y) for a point robot."""
    def __init__(self, name):
        """name is the yaml stem; the image file comes from the yaml's image: field."""
        self.res, self.origin, thr = read_yaml(f"{MAPS}/{name}.yaml")
        image = [l.split(":", 1)[1].strip() for l in open(f"{MAPS}/{name}.yaml") if l.startswith("image:")][0]
        img = read_pgm(f"{MAPS}/{image}")
        self.h, self.w = img.shape
        # ROS map convention: row 0 of the image is the top (max y); occupied where (255-p)/255 > thresh
        self.occ = (255 - img.astype(float)) / 255.0 > thr
        self.xmin, self.ymin = self.origin[0], self.origin[1]
        self.xmax, self.ymax = self.xmin + self.w * self.res, self.ymin + self.h * self.res
    def valid(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        col = np.floor((x - self.xmin) / self.res).astype(int)
        row = (self.h - 1 - np.floor((y - self.ymin) / self.res)).astype(int)
        inside = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        out = np.zeros_like(x, dtype=bool)
        out[inside] = ~self.occ[row[inside], col[inside]]
        return out

def b64png(fig):
    buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=130, bbox_inches="tight"); plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()

# ---------------------------------------------------------------- Bug A
def bug_a(mapname, intname, n_draws=2_000_000):
    occ = Occ(mapname); im = read_intensity(f"{MAPS}/{intname}")
    idx = np.arange(im["rows"] * im["cols"])
    # getXYatIndex: cell corner, not centre (as in the C++)
    xs = (idx % im["cols"]) * im["cs"] + im["xmin"]; ys = (idx // im["cols"]) * im["cs"] + im["ymin"]
    valid = occ.valid(xs, ys)
    q = im["vals"][valid]; vx, vy = xs[valid], ys[valid]
    N = valid.sum()
    # C++: q_map sorted ascending by (1-q); nonq_map stays in row-major order with weight 1
    w_q = 1.0 - q; order = np.argsort(w_q, kind="stable"); w_q_sorted = w_q[order]
    value_sum = w_q_sorted.sum()
    # exact re-implementation of the selection loop, vectorised:
    # result = first index k with sampled_value < prefix(k) where prefix(k)=sum w[0..k-1]; else 0
    def select(weights, v):
        prefix = np.concatenate([[0.0], np.cumsum(weights)])  # prefix[k]
        k = np.searchsorted(prefix, v, side="right")            # smallest k with prefix[k] > v
        k[k > len(weights)] = 0                                 # loop ran off the end -> stays 0
        return k
    v = rng.uniform(0, value_sum, n_draws)
    hit_nonq = np.bincount(select(np.ones(N), v), minlength=N)
    hit_q = np.bincount(select(w_q_sorted, v), minlength=N)
    never_nonq = hit_nonq == 0
    reach_theory = min(N, int(math.floor(value_sum)) + 2)
    # what it should have been: uniform over all N valid cells
    stats = dict(map=mapname, valid_cells=int(N), total_cells=int(idx.size), value_sum=float(value_sum),
                 reachable_uniform_branch=int((~never_nonq).sum()), reachable_theory=reach_theory,
                 never_sampled_uniform_branch=int(never_nonq.sum()),
                 pct_never_uniform=100.0 * never_nonq.sum() / N,
                 never_sampled_intensity_branch=int((hit_q == 0).sum()),
                 mean_q=float(q.mean()))
    # --- figure: map with cells shaded
    fig, ax = plt.subplots(figsize=(7.2, 7.2 * (occ.ymax - occ.ymin) / (occ.xmax - occ.xmin)))
    ax.imshow(np.where(occ.occ, 1, 0), cmap=ListedColormap([C["free"], C["obstacle"]]),
              extent=[occ.xmin, occ.xmax, occ.ymin, occ.ymax], origin="upper", interpolation="nearest")
    cs = im["cs"]
    for k in range(N):  # cells reachable vs never
        col = C["never"] if never_nonq[k] else C["sampled"]
        ax.add_patch(plt.Rectangle((vx[k] - cs / 2, vy[k] - cs / 2), cs, cs, facecolor=col, edgecolor="none", alpha=0.85))
    ax.set_xlim(occ.xmin, occ.xmax); ax.set_ylim(occ.ymin, occ.ymax); ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    for s in ax.spines.values(): s.set_visible(False)
    ax.tick_params(colors="#666", labelsize=8)
    return stats, b64png(fig), dict(N=int(N), hit=hit_nonq.tolist()[:0])

# ---------------------------------------------------------------- Bug B
def bug_b(mapname, intname, start, goal, cell=0.5, w_d=1.0, w_c=0.2, seg_len=0.2, n_draws=100_000):
    occ = Occ(mapname); im = read_intensity(f"{MAPS}/{intname}")
    xmin, xmax, ymin, ymax = occ.xmin, occ.xmax, occ.ymin, occ.ymax  # bench-mr uses the map bounds
    cols = int((xmax - xmin) / cell) + 1; rows = int((ymax - ymin) / cell) + 1
    colToX = lambda c: c * cell + xmin; rowToY = lambda r: r * cell + ymin
    R, Cc = np.meshgrid(np.arange(rows), np.arange(cols), indexing="ij")
    node_valid = occ.valid(colToX(Cc.ravel()), rowToY(R.ravel())).reshape(rows, cols)
    def qat(x, y):  # IntensityMap::operator()
        r = np.floor((y - im["ymin"]) / im["cs"]).astype(int); c = np.floor((x - im["xmin"]) / im["cs"]).astype(int)
        i = np.clip(r * im["cols"] + c, 0, im["vals"].size - 1)
        ok = (r >= 0) & (r < im["rows"]) & (c >= 0) & (c < im["cols"])
        return np.where(ok, im["vals"][i], 0.0)
    src, dst, wgt = [], [], []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0: continue
            r0, c0 = R.ravel(), Cc.ravel(); r1, c1 = r0 + dr, c0 + dc
            ok = (r1 >= 0) & (r1 < rows) & (c1 >= 0) & (c1 < cols)
            r0, c0, r1, c1 = r0[ok], c0[ok], r1[ok], c1[ok]
            ok = node_valid[r0, c0] & node_valid[r1, c1]   # checkValidity: both endpoints
            r0, c0, r1, c1 = r0[ok], c0[ok], r1[ok], c1[ok]
            x0, y0, x1, y1 = colToX(c0), rowToY(r0), colToX(c1), rowToY(r1)
            d = np.hypot(x1 - x0, y1 - y0)
            nseg = int(math.ceil(d.max() / seg_len))          # objective interpolates; MoD cost at each sub-segment end
            cost = w_d * d
            for k in range(1, nseg + 1):
                t = k / nseg; cost = cost + w_c * qat(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
            src.append(r0 * cols + c0); dst.append(r1 * cols + c1); wgt.append(cost)
    src, dst, wgt = map(np.concatenate, (src, dst, wgt))
    G = coo_matrix((wgt, (src, dst)), shape=(rows * cols, rows * cols)).tocsr()
    s = int((start[1] - ymin) / cell) * cols + int((start[0] - xmin) / cell)
    g = int((goal[1] - ymin) / cell) * cols + int((goal[0] - xmin) / cell)
    dist, pred = dijkstra(G, directed=True, indices=s, return_predecessors=True)
    assert np.isfinite(dist[g]), "no path"
    path = []; cur = g
    while cur != s: path.append(cur); cur = pred[cur]
    path.append(s); path = path[::-1]
    P = np.array(path); px, py = colToX(P % cols), rowToY(P // cols); L = len(path)
    # intended heading per path index (paper: implicit direction to next cell; last cell uses previous)
    th_int = np.empty(L); th_bug = np.empty(L)
    for i in range(L):
        if i == L - 1:
            th_int[i] = math.atan2(py[i] - py[i - 1], px[i] - px[i - 1])
            th_bug[i] = math.atan2(py[i] - py[0], px[i] - px[0])       # prev_iter never advanced -> begin()
        else:
            th_int[i] = math.atan2(py[i + 1] - py[i], px[i + 1] - px[i])
            th_bug[i] = math.atan2(py[0] - py[i], px[0] - px[i])       # next_iter never advanced -> begin()
    idx = rng.integers(0, L, n_draws); jit = rng.uniform(-math.pi / 8, math.pi / 8, n_draws)
    samp_int = np.arctan2(np.sin(th_int[idx] + jit), np.cos(th_int[idx] + jit))
    samp_bug = np.arctan2(np.sin(th_bug[idx] + jit), np.cos(th_bug[idx] + jit))
    diff = np.abs(np.arctan2(np.sin(th_bug - th_int), np.cos(th_bug - th_int)))
    stats = dict(map=mapname, start=start, goal=goal, cell=cell, path_cells=L, grid=[rows, cols],
                 edges=int(len(wgt)), path_cost=float(dist[g]),
                 mean_abs_heading_error_deg=float(np.degrees(diff.mean())),
                 median_abs_heading_error_deg=float(np.degrees(np.median(diff))),
                 pct_cells_error_over_45deg=float(100 * (diff > math.pi / 4).mean()),
                 pct_cells_error_over_90deg=float(100 * (diff > math.pi / 2).mean()))
    # --- figure 1: polar histograms
    fig = plt.figure(figsize=(5.2, 5.2)); ax = fig.add_subplot(projection="polar")
    bins = np.linspace(-math.pi, math.pi, 49)
    for arr, col, lab in ((samp_int, C["intended"], "intended (paper)"), (samp_bug, C["buggy"], "current code")):
        h, _ = np.histogram(arr, bins=bins, density=True)
        ax.bar((bins[:-1] + bins[1:]) / 2, h, width=np.diff(bins), color=col, alpha=0.55, edgecolor=col, linewidth=0.6, label=lab)
    ax.set_theta_zero_location("E"); ax.set_yticklabels([]); ax.tick_params(labelsize=8, colors="#555")
    ax.legend(loc="lower left", bbox_to_anchor=(-0.15, -0.12), frameon=False, fontsize=9)
    polar = b64png(fig)
    # --- figure 2: path on map with arrows
    fig, ax = plt.subplots(figsize=(7.2, 7.2 * (ymax - ymin) / (xmax - xmin)))
    ax.imshow(np.where(occ.occ, 1, 0), cmap=ListedColormap([C["free"], C["obstacle"]]),
              extent=[xmin, xmax, ymin, ymax], origin="upper", interpolation="nearest")
    ax.plot(px, py, color=C["path"], lw=1.6, solid_capstyle="round", zorder=2)
    step = max(1, L // 28); ii = np.arange(0, L, step)
    ax.quiver(px[ii], py[ii], np.cos(th_bug[ii]), np.sin(th_bug[ii]), color=C["buggy"], scale=0.76 * (xmax - xmin), width=0.005, zorder=4, label="current code")
    ax.quiver(px[ii], py[ii], np.cos(th_int[ii]), np.sin(th_int[ii]), color=C["intended"], scale=0.76 * (xmax - xmin), width=0.005, zorder=5, label="intended")
    ax.scatter([start[0], goal[0]], [start[1], goal[1]], s=[70, 70], c=["#ffffff", C["path"]], edgecolors=C["path"], linewidths=2, zorder=6)
    ax.annotate("S", (start[0], start[1]), xytext=(6, 6), textcoords="offset points", fontsize=9, color="#333")
    ax.annotate("G", (goal[0], goal[1]), xytext=(6, 6), textcoords="offset points", fontsize=9, color="#333")
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax); ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    for s_ in ax.spines.values(): s_.set_visible(False)
    ax.tick_params(colors="#666", labelsize=8)
    return stats, polar, b64png(fig)

if __name__ == "__main__" and not (len(sys.argv) > 1 and sys.argv[1] == "atc"):
    out = {}
    out["A_office"], out["A_office_png"], _ = bug_a("office_cubicles", "office_cubicles_intensitymap.xml")
    out["A_warehouse"], out["A_warehouse_png"], _ = bug_a("pedsim_warehouse", "pedsim_warehouse_intensity1m.xml")
    out["B_office"], out["B_office_polar"], out["B_office_map"] = bug_b(
        "office_cubicles", "office_cubicles_intensitymap.xml", (-5.0, -5.0, 0.785), (19.0, 19.0, 0.785))
    out["B_warehouse"], out["B_warehouse_polar"], out["B_warehouse_map"] = bug_b(
        "pedsim_warehouse", "pedsim_warehouse_intensity1m.xml", (1.0, 1.0, 0.0), (35.0, 29.5, 0.0))
    json.dump(out, open(f"{OUT}/results.json", "w"))
    for k in ("A_office", "A_warehouse", "B_office", "B_warehouse"): print(k, json.dumps(out[k]))

# ---------------------------------------------------------------- ATC scenarios (Paper IV)
ATC_SG = [  # from bench-mr python/sg-pairs-atc.yaml
    ("atc-scenario1",     (47.690, -18.848), (-19.575, 12.390)),
    ("atc-scenario2",     (21.471, -17.647), (-3.85, -6.82)),
    ("atc-scenario3",     (-0.133, -8.555),  (31.093, -20.553)),
    ("atc-scenario4",     (11.5, -5.0),      (-25.00, 3.00)),
    ("atc-scenario1-rev", (-19.575, 12.390), (47.690, -18.848)),
    ("atc-scenario2-rev", (-3.85, -6.82),    (21.471, -17.647)),
]

def run_atc():
    out = {}
    out["A_atc"], out["A_atc_png"], _ = bug_a("atc", "atc_intensity1m.xml", n_draws=4_000_000)
    for name, s, g in ATC_SG:
        st, polar, mp = bug_b("atc", "atc_intensity1m.xml", (*s, 0.0), (*g, 0.0), n_draws=100_000)
        out["B_" + name] = st; out["B_" + name + "_polar"] = polar; out["B_" + name + "_map"] = mp
        print(name, json.dumps(st))
    print("A_atc", json.dumps(out["A_atc"]))
    json.dump(out, open(f"{OUT}/results_atc.json", "w"))

if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "atc":
    run_atc(); sys.exit(0)
