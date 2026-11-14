"""Chạy mô phỏng thuật toán Algo/ trên PC.

Các chế độ:
  python tools/sim/run_sim.py                     # 1 lượt không nhiễu + 100 lượt có nhiễu, vẽ hình
  python tools/sim/run_sim.py --runs 300          # Monte Carlo 300 lượt
  python tools/sim/run_sim.py --obstacle 95,81    # thêm vật cản ngẫu nhiên tại (95, 81)
  python tools/sim/run_sim.py --sweep             # đặt vật cản lần lượt dọc quỹ đạo, mỗi chỗ 1 lượt
  python tools/sim/run_sim.py --mode teach        # Lượt 1 lái tay (người giả lập) rồi Lượt 2 phát lại
  python tools/sim/run_sim.py --gif               # xuất ảnh động một lượt chạy
  python tools/sim/run_sim.py --set v_max=40      # đổi thông số NavParams, không cần sửa code
  python tools/sim/run_sim.py --watch             # tự build + chạy lại mỗi khi lưu file trong Algo/

Kết quả nằm trong tools/sim/out/.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = HERE / "out"
EXE = OUT / ("sim.exe" if sys.platform == "win32" else "sim")
ALGO = ROOT / "Algo"
PATH_H = ALGO / "planning" / "path.h"


def algo_dirs() -> list[Path]:
    """Algo/ và mọi thư mục con có file .h (include phẳng: #include "geometry.h")."""
    return sorted({p.parent for p in ALGO.rglob("*.h")})
CLEAR_TARGET = 1.5  # cm: thân xe phải luôn cách vật cản ít nhất mức này

sys.path.insert(0, str(ROOT / "tools" / "path_planner"))
import plan_path  # noqa: E402  (dùng lại hàm vẽ sân)


# ---------------------------------------------------------------- Build & chạy
def build() -> bool:
    gcc = shutil.which("gcc")
    if not gcc:
        print("Không tìm thấy gcc trên PATH.")
        return False
    OUT.mkdir(parents=True, exist_ok=True)
    cmd = [gcc, "-std=c11", "-O2", "-Wall", "-Wextra", *[f"-I{d}" for d in algo_dirs()],
           *map(str, sorted(ALGO.rglob("*.c"))), str(HERE / "sim_main.c"), "-lm", "-o", str(EXE)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or r.stderr.strip():
        print(r.stderr)
    return r.returncode == 0


def run_one(seed: int, noise: float, mode: str, obstacles: list[str], sets: list[str],
            csv: Path | None = None) -> dict:
    cmd = [str(EXE), "--seed", str(seed), "--noise", str(noise), "--mode", mode]
    for o in obstacles:
        cmd += ["--obstacle", o]
    for s in sets:
        cmd += ["--set", s]
    if csv:
        cmd += ["--out", str(csv)]
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0:
        raise SystemExit(f"sim lỗi: {r.stderr.strip()}")
    res = {"seed": seed, "obstacles": obstacles, "events": [], "teach": None}
    for line in r.stdout.splitlines():
        if line.startswith("RESULT"):
            res.update(dict(kv.split("=", 1) for kv in line.split()[1:]))
        elif line.startswith("TEACH"):
            res["teach"] = dict(kv.split("=", 1) for kv in line.split()[1:])
        elif line.startswith("EVENT"):
            res["events"].append(line[6:])
    for k in ("t", "min_clear", "end_err"):
        res[k] = float(res.get(k, "nan"))
    return res


def load_csv(path: Path) -> dict[str, np.ndarray]:
    import csv as _csv
    with open(path, encoding="utf-8") as f:
        rows = list(_csv.DictReader(f))
    return {k: np.array([r[k] for r in rows], dtype=(str if k == "state" else float)) for k in rows[0]}


# ---------------------------------------------------------------- Vẽ
def draw_obstacles(ax, obstacles: list[str]) -> None:
    from matplotlib.patches import Rectangle
    for o in obstacles:
        x, y = map(float, o.split(","))
        ax.add_patch(Rectangle((x - 5, y - 5), 10, 10, fc="#f4a", ec="k", lw=1.5, zorder=3))
        ax.text(x, y, "?", ha="center", va="center", fontsize=9, weight="bold", zorder=4)


def plot_run(csv: Path, png: Path, title: str, obstacles: list[str]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d = load_csv(csv)
    fig, ax = plt.subplots(figsize=(12, 8.5))
    plan_path.draw_field(ax)
    draw_obstacles(ax, obstacles)
    path = parse_path_points()
    ax.plot(path[:, 0], path[:, 1], color="gray", lw=1, ls="--", label="Quỹ đạo A*")
    rec = Path(str(csv) + ".rec.csv")
    if rec.exists():
        r = np.loadtxt(rec, delimiter=",", skiprows=1, ndmin=2)
        ax.plot(r[:, 0], r[:, 1], "m.-", lw=1, ms=4, label="Quỹ đạo ghi ở Lượt 1")
    ax.plot(d["x"], d["y"], color="tab:blue", lw=2, label="Xe đi THẬT")
    ax.plot(d["ex"], d["ey"], color="tab:orange", lw=1, ls="--", label="Xe TỰ ƯỚC LƯỢNG")
    i = int(np.argmin(d["clear"]))
    ax.plot(d["x"][i], d["y"][i], "rx", ms=12, mew=2, label=f"Sát vật cản nhất: {d['clear'][i]:.1f} cm")
    draw_body(ax, d["x"][i], d["y"][i], d["th"][i], color="red")
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    plt.close(fig)


def draw_body(ax, x, y, th, color="tab:blue", alpha=1.0):
    from matplotlib.patches import Polygon
    c, s = np.cos(th), np.sin(th)
    pts = [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in ((5, 4.5), (5, -4.5), (-5, -4.5), (-5, 4.5))]
    patch = Polygon(pts, closed=True, fill=False, ec=color, lw=1.5, alpha=alpha, zorder=5)
    ax.add_patch(patch)
    return patch


def plot_monte_carlo(results: list[dict], csvs: list[Path], png: Path, title: str, obstacles: list[str]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(17, 7), gridspec_kw={"width_ratios": [2.2, 1]})
    plan_path.draw_field(ax)
    draw_obstacles(ax, obstacles)
    for res, csv in zip(results, csvs):
        if not csv.exists():
            continue
        d = load_csv(csv)
        bad = res.get("status") != "DONE" or res["min_clear"] < CLEAR_TARGET
        ax.plot(d["x"], d["y"], color="red" if bad else "tab:blue", lw=1.2 if bad else 0.5,
                alpha=0.9 if bad else 0.25)
    ax.set_title(title)
    clears = np.array([r["min_clear"] for r in results])
    ax2.hist(clears, bins=30, color="tab:blue")
    ax2.axvline(CLEAR_TARGET, color="red", ls="--", label=f"Mục tiêu {CLEAR_TARGET} cm")
    ax2.set_xlabel("Khoảng cách nhỏ nhất thân xe - vật cản (cm)")
    ax2.set_ylabel("Số lượt")
    ax2.legend()
    fig.tight_layout()
    fig.savefig(png, dpi=100)
    plt.close(fig)


def make_gif(csv: Path, gif: Path, obstacles: list[str], speed: float = 2.0) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    d = load_csv(csv)
    fps = 20
    step = max(1, int(round(speed / fps / 0.02)))  # CSV ghi mỗi 20 ms
    idx = list(range(0, len(d["t"]), step)) + [len(d["t"]) - 1]
    fig, ax = plt.subplots(figsize=(9, 6.4))
    plan_path.draw_field(ax)
    draw_obstacles(ax, obstacles)
    path = parse_path_points()
    ax.plot(path[:, 0], path[:, 1], color="gray", lw=1, ls="--")
    (trail,) = ax.plot([], [], color="tab:blue", lw=2)
    (est,) = ax.plot([], [], color="tab:orange", lw=1, ls="--")
    (tgt,) = ax.plot([], [], "g+", ms=10, mew=2)
    body = [draw_body(ax, d["x"][0], d["y"][0], d["th"][0])]
    txt = ax.text(2, 123, "", fontsize=9)

    def update(k):
        i = idx[k]
        trail.set_data(d["x"][: i + 1], d["y"][: i + 1])
        est.set_data(d["ex"][: i + 1], d["ey"][: i + 1])
        tgt.set_data([d["tx"][i]], [d["ty"][i]])
        body[0].remove()
        body[0] = draw_body(ax, d["x"][i], d["y"][i], d["th"][i], color="tab:red")
        txt.set_text(f"t = {d['t'][i]:5.2f} s   v = {d['v'][i]:5.1f} cm/s   {d['state'][i]}   "
                     f"cách vật cản {d['clear'][i]:.1f} cm")
        return trail, est, tgt, body[0], txt

    anim = FuncAnimation(fig, update, frames=len(idx), blit=False)
    anim.save(gif, writer=PillowWriter(fps=fps))
    plt.close(fig)


def parse_path_points() -> np.ndarray:
    text = PATH_H.read_text(encoding="utf-8")
    block = text[text.index("PATH_POINTS"):text.index("};")]
    pts = re.findall(r"\{\s*([-\d.]+)f,\s*([-\d.]+)f\}", block)
    return np.array(pts, dtype=float)


# ---------------------------------------------------------------- Báo cáo
def summarize(results: list[dict], label: str) -> None:
    n = len(results)
    statuses: dict[str, int] = {}
    for r in results:
        statuses[r.get("status", "?")] = statuses.get(r.get("status", "?"), 0) + 1
    clears = np.array([r["min_clear"] for r in results])
    done = [r for r in results if r.get("status") == "DONE"]
    clean = [r for r in done if r["min_clear"] >= CLEAR_TARGET]
    times = np.array([r["t"] for r in done]) if done else np.array([np.nan])
    print(f"\n=== {label}: {n} lượt ===")
    print("Kết quả      : " + ", ".join(f"{k} {v}" for k, v in sorted(statuses.items())))
    print(f"Về đích sạch : {len(clean)}/{n} ({100 * len(clean) / n:.1f}%)  "
          f"[DONE và thân xe luôn cách vật cản >= {CLEAR_TARGET} cm]")
    print(f"Cách vật cản : nhỏ nhất {clears.min():.2f} cm, trung vị {np.median(clears):.2f} cm")
    print(f"Thời gian    : trung bình {np.nanmean(times):.2f} s, chậm nhất {np.nanmax(times):.2f} s")
    worst = sorted(results, key=lambda r: (r.get("status") == "DONE", r["min_clear"]))[:3]
    for r in worst:
        if r.get("status") != "DONE" or r["min_clear"] < CLEAR_TARGET:
            where = f" vật cản {r['obstacles']}" if r["obstacles"] else ""
            print(f"  - seed {r['seed']}{where}: {r.get('status')} cách {r['min_clear']:.2f} cm")


# ---------------------------------------------------------------- Các chế độ
def mode_default(args) -> None:
    obstacles = args.obstacle or []
    csv0 = OUT / "run_noise0.csv"
    r0 = run_one(0, 0.0, args.mode, obstacles, args.set, csv0)
    print(f"Không nhiễu: {r0['status']} sau {r0['t']:.2f} s, cách vật cản nhỏ nhất {r0['min_clear']:.2f} cm")
    for e in r0["events"]:
        if "state=RUN" not in e:
            print("   ", e)
    plot_run(csv0, OUT / "run_noise0.png", f"Không nhiễu: {r0['status']}, {r0['t']:.1f} s", obstacles)

    if args.runs > 0:
        csvs = [OUT / "mc" / f"seed{s}.csv" for s in range(1, args.runs + 1)]
        (OUT / "mc").mkdir(exist_ok=True)
        with ThreadPoolExecutor() as ex:
            results = list(ex.map(lambda s: run_one(s, args.noise, args.mode, obstacles, args.set,
                                                     csvs[s - 1]), range(1, args.runs + 1)))
        summarize(results, f"Monte Carlo, nhiễu x{args.noise}")
        plot_monte_carlo(results, csvs, OUT / "monte_carlo.png",
                         f"{args.runs} lượt có nhiễu (đỏ = va chạm / dừng / sát dưới {CLEAR_TARGET} cm)", obstacles)
    if args.gif:
        make_gif(csv0, OUT / "run.gif", obstacles)
        print(f"Đã ghi {OUT / 'run.gif'}")
    print(f"Hình: {OUT / 'run_noise0.png'}" + (f", {OUT / 'monte_carlo.png'}" if args.runs > 0 else ""))


def mode_sweep(args) -> None:
    """Đặt vật cản ngẫu nhiên lần lượt tại các điểm dọc quỹ đạo (cách nhau ~10 cm)."""
    path = parse_path_points()
    seg = np.r_[0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
    spots = []
    for s in np.arange(25.0, seg[-1] - 15.0, args.sweep_step):
        x, y = np.interp(s, seg, path[:, 0]), np.interp(s, seg, path[:, 1])
        if y > 6:
            spots.append(f"{x:.1f},{y:.1f}")
    jobs = [(i + 1, o) for i, o in enumerate(spots)]
    csvs = {o: OUT / "sweep" / f"obs_{i}.csv" for i, o in jobs}
    (OUT / "sweep").mkdir(exist_ok=True)
    with ThreadPoolExecutor() as ex:
        results = list(ex.map(lambda j: run_one(j[0], args.noise, "astar", [j[1]], args.set, csvs[j[1]]), jobs))
    summarize(results, "Vật cản ngẫu nhiên dọc quỹ đạo")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(12, 8.5))
    plan_path.draw_field(ax)
    ax.plot(path[:, 0], path[:, 1], color="gray", lw=1, ls="--")
    colors = {"DONE": "tab:green", "STOPPED": "orange"}
    for r in results:
        x, y = map(float, r["obstacles"][0].split(","))
        st = r.get("status")
        c = colors.get(st, "red") if not (st == "DONE" and r["min_clear"] < CLEAR_TARGET) else "gold"
        ax.plot(x, y, "s", ms=9, color=c, mec="k")
    ax.set_title("Vị trí vật cản lạ: xanh = về đích sạch, vàng = về đích nhưng sát < 1.5 cm, "
                 "cam = dừng an toàn, đỏ = va chạm", fontsize=9)
    fig.tight_layout()
    fig.savefig(OUT / "sweep.png", dpi=110)
    plt.close(fig)
    print(f"Hình: {OUT / 'sweep.png'}")


def mode_watch(args) -> None:
    def snapshot():
        return {p: p.stat().st_mtime for p in [*ALGO.rglob("*.[ch]"), HERE / "sim_main.c"]}

    print("Đang theo dõi Algo/ và tools/sim/sim_main.c ... (Ctrl+C để thoát)")
    last = None
    while True:
        snap = snapshot()
        if snap != last:
            last = snap
            print(f"\n[{time.strftime('%H:%M:%S')}] Build lại...")
            if build():
                quick = argparse.Namespace(**{**vars(args), "runs": min(args.runs, 30), "gif": False})
                mode_default(quick)
        time.sleep(0.5)


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=100, help="số lượt Monte Carlo có nhiễu")
    ap.add_argument("--noise", type=float, default=1.0, help="hệ số nhân mọi nguồn nhiễu")
    ap.add_argument("--mode", choices=["astar", "teach"], default="astar")
    ap.add_argument("--obstacle", action="append", help="vật cản lạ 10x10 tại X,Y (có thể lặp)")
    ap.add_argument("--set", action="append", default=[], help="đổi thông số, ví dụ v_max=40")
    ap.add_argument("--sweep", action="store_true", help="đặt vật cản lần lượt dọc quỹ đạo")
    ap.add_argument("--sweep-step", type=float, default=10.0)
    ap.add_argument("--gif", action="store_true")
    ap.add_argument("--watch", action="store_true")
    args = ap.parse_args()

    if args.watch:
        mode_watch(args)
        return
    if not build():
        raise SystemExit(1)
    if args.sweep:
        mode_sweep(args)
    else:
        mode_default(args)


if __name__ == "__main__":
    main()
