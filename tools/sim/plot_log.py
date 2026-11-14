"""Vẽ log chạy thật của xe lên sân (so với quỹ đạo A* và mô phỏng).

Trên xe: gõ "$log 1" qua Bluetooth trước khi nhấn SW1, lưu toàn bộ chữ nhận được từ terminal vào file.
Dòng log:  L,<ms>,<x mm>,<y mm>,<theta mrad>,<v mm/s>,<state>,<số vật lạ>
           E,<ms>,<tên trạng thái>      (mỗi lần đổi trạng thái)

Dùng:  python tools/sim/plot_log.py log.txt [--sim tools/sim/out/run_noise0.csv]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[0] / "path_planner"))
import plan_path  # noqa: E402
from run_sim import draw_body, load_csv, parse_path_points  # noqa: E402

STATES = ["IDLE", "RUN", "BRAKE", "AVOID", "TEACH", "DONE", "STOPPED", "EDGE"]


def parse_log(path: Path):
    pose, events = [], []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = raw.strip().split(",")
        try:
            if parts[0] == "L" and len(parts) >= 8:
                pose.append([float(parts[1]) / 1000.0, float(parts[2]) / 10.0, float(parts[3]) / 10.0,
                             float(parts[4]) / 1000.0, float(parts[5]) / 10.0, int(parts[6]), int(parts[7])])
            elif parts[0] == "E" and len(parts) >= 3:
                events.append((float(parts[1]) / 1000.0, parts[2]))
        except ValueError:
            continue  # dòng bị lỗi truyền
    return np.array(pose), events


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path)
    ap.add_argument("--sim", type=Path, help="CSV mô phỏng để vẽ chồng (tools/sim/out/*.csv)")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    pose, events = parse_log(args.log)
    if len(pose) == 0:
        raise SystemExit("Không có dòng 'L,...' nào trong log (đã gõ $log 1 chưa?)")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 8.5))
    plan_path.draw_field(ax)
    path = parse_path_points()
    ax.plot(path[:, 0], path[:, 1], color="gray", lw=1, ls="--", label="Quỹ đạo A*")
    if args.sim:
        d = load_csv(args.sim)
        ax.plot(d["ex"], d["ey"], color="tab:green", lw=1, label="Mô phỏng (ước lượng)")
    sc = ax.scatter(pose[:, 1], pose[:, 2], c=pose[:, 4], cmap="viridis", s=8, label="Xe thật (ước lượng)")
    fig.colorbar(sc, ax=ax, label="v (cm/s)")
    draw_body(ax, pose[-1, 1], pose[-1, 2], pose[-1, 3], color="red")
    t0 = pose[0, 0]
    for t, name in events:
        i = int(np.argmin(np.abs(pose[:, 0] - t)))
        ax.annotate(f"{name} {t - t0:.1f}s", (pose[i, 1], pose[i, 2]), fontsize=7, color="purple")
    ax.set_title(f"Log xe thật: {len(pose)} điểm, {pose[-1, 0] - t0:.1f} s, "
                 f"trạng thái cuối {STATES[int(pose[-1, 5])] if 0 <= pose[-1, 5] < len(STATES) else '?'}")
    ax.legend(loc="upper right", fontsize=8)
    out = args.out or args.log.with_suffix(".png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"Đã ghi {out}")


if __name__ == "__main__":
    main()
