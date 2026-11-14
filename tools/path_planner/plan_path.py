"""Lập quỹ đạo offline cho xe micromouse team 15.

Map sân (17 vật cản tĩnh + vùng cấm) -> A* trên lưới 1 cm có phạt ô sát vật cản
-> làm mượt -> lấy mẫu waypoint -> xuất Algo/planning/path.h và ảnh kiểm tra.

Hệ toạ độ: gốc (0,0) ở góc dưới-trái sân, x sang phải (0..180), y lên trên (0..120), đơn vị cm.

Chạy:  python tools/path_planner/plan_path.py
"""

from __future__ import annotations

import argparse
import heapq
import math
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt

# ---------------------------------------------------------------- Sân & vật cản
FIELD_W = 180
FIELD_H = 120
BLOCK = 10  # cạnh khối vật cản (cm)

# Tâm 17 vật cản tĩnh (đọc từ sơ đồ luật thi, sai số ~1-2 cm).
BLOCK_CENTERS = {
    "T1": (5, 15), "T2": (55, 25), "T3": (30, 35), "T4": (55, 45),
    "T5": (5, 55), "T6": (5, 75), "T7": (35, 90), "T8": (45, 115),
    "T9": (65, 75), "T10": (75, 65), "T11": (85, 100), "T12": (95, 65),
    "T13": (115, 115), "T14": (125, 85), "T15": (145, 45), "T16": (145, 15),
    "T17": (175, 25),
}

# Vùng cấm giữa đáy chữ U (tường bao trái/trên/phải): coi cả khối là vật cản.
FORBIDDEN = (60, 0, 120, 60)  # x_min, y_min, x_max, y_max

START = (30.0, 6.0)          # tâm xe khi đặt ở ô xuất phát
START_HEADING_DEG = 90.0     # hướng lên (+y)
FINISH_X = (120.0, 180.0)    # vạch đích nằm ở cạnh dưới Ô5 (y = 0)
GOAL_Y = 6.0                 # A* coi là tới đích khi tâm xe xuống tới y <= GOAL_Y
FINISH_OVERSHOOT = 10.0      # chạy thẳng tiếp qua vạch để cả thân xe (dài 10 cm) qua đích

# ---------------------------------------------------------------- Xe & tham số
ROBOT_L = 10.0
ROBOT_W = 9.0
ROBOT_RADIUS = math.hypot(ROBOT_L / 2, ROBOT_W / 2)  # ~6.7 cm, xoay tại chỗ không quệt
R_HARD = 7.5     # tâm xe phải cách mặt vật cản/tường ít nhất R_HARD
R_SOFT = 15.0    # dưới R_SOFT bắt đầu bị phạt chi phí
W_CLEAR = 4.0    # trọng số phạt sát vật cản
R_SMOOTH = 9.5   # đoạn làm mượt phải giữ khoảng cách này (hoặc không tệ hơn đường gốc)
WAYPOINT_STEP = 5.0  # khoảng cách giữa các waypoint (cm)


def block_rects() -> dict[str, tuple[float, float, float, float]]:
    h = BLOCK / 2
    return {k: (x - h, y - h, x + h, y + h) for k, (x, y) in BLOCK_CENTERS.items()}


# ---------------------------------------------------------------- Lưới & khoảng cách
def build_clearance() -> np.ndarray:
    """Khoảng cách (cm) từ tâm mỗi ô 1x1 cm tới vật cản/tường gần nhất. Mảng [y, x]."""
    occ = np.zeros((FIELD_H, FIELD_W), dtype=bool)
    for x0, y0, x1, y1 in list(block_rects().values()) + [FORBIDDEN]:
        occ[int(y0):int(y1), int(x0):int(x1)] = True

    # Tường trái, phải, trên là vật cản; cạnh dưới để mở (vạch xuất phát / vạch đích).
    padded = np.ones((FIELD_H + 1, FIELD_W + 2), dtype=bool)
    padded[:FIELD_H, 1:FIELD_W + 1] = occ
    dist = distance_transform_edt(~padded)[:FIELD_H, 1:FIELD_W + 1]
    # Khoảng cách tâm-ô tới tâm ô vật cản -> trừ 0.5 để ra khoảng cách tới mặt vật cản.
    return np.where(dist > 0, dist - 0.5, 0.0)


def clearance_at(clr: np.ndarray, x: float, y: float) -> float:
    ix = min(max(int(x), 0), FIELD_W - 1)
    if y < 0:  # ngoài cạnh dưới: chỉ còn tường hai bên
        return float(clr[0, ix])
    iy = min(int(y), FIELD_H - 1)
    return float(clr[iy, ix])


# ---------------------------------------------------------------- A*
def astar(clr: np.ndarray) -> list[tuple[int, int]]:
    def cell_cost(iy: int, ix: int) -> float:
        d = clr[iy, ix]
        if d >= R_SOFT:
            return 1.0
        p = (R_SOFT - d) / (R_SOFT - R_HARD)
        return 1.0 + W_CLEAR * p * p

    def h(ix: int, iy: int) -> float:
        cx, cy = ix + 0.5, iy + 0.5
        dx = max(FINISH_X[0] - cx, 0.0, cx - FINISH_X[1])
        dy = max(cy - GOAL_Y, 0.0)
        return math.hypot(dx, dy)

    def is_goal(ix: int, iy: int) -> bool:
        return iy + 0.5 <= GOAL_Y and FINISH_X[0] <= ix + 0.5 <= FINISH_X[1]

    sx, sy = int(START[0]), int(START[1])
    if clr[sy, sx] < R_HARD:
        raise SystemExit(f"Điểm xuất phát {START} quá sát vật cản ({clr[sy, sx]:.1f} cm)")

    moves = [(dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
    g = {(sx, sy): 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    heap = [(h(sx, sy), 0.0, sx, sy)]
    closed = set()
    while heap:
        _, gc, x, y = heapq.heappop(heap)
        if (x, y) in closed:
            continue
        closed.add((x, y))
        if is_goal(x, y):
            path = [(x, y)]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            return path[::-1]
        for dx, dy, step in moves:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < FIELD_W and 0 <= ny < FIELD_H) or clr[ny, nx] < R_HARD:
                continue
            ng = gc + step * 0.5 * (cell_cost(y, x) + cell_cost(ny, nx))
            if ng < g.get((nx, ny), math.inf):
                g[(nx, ny)] = ng
                parent[(nx, ny)] = (x, y)
                heapq.heappush(heap, (ng + h(nx, ny), ng, nx, ny))
    raise SystemExit("A* không tìm được đường: thử giảm R_HARD")


# ---------------------------------------------------------------- Làm mượt & lấy mẫu
def segment_clearance(clr: np.ndarray, a, b) -> float:
    n = max(int(math.dist(a, b) / 0.5), 1)
    return min(clearance_at(clr, a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n)
               for t in range(n + 1))


def smooth(clr: np.ndarray, pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Nối tắt tham lam: bỏ điểm giữa nếu đoạn thẳng vẫn đủ xa vật cản."""
    raw_clr = [clearance_at(clr, *p) for p in pts]
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1:
            need = min(R_SMOOTH, min(raw_clr[i:j + 1]))
            if segment_clearance(clr, pts[i], pts[j]) >= need:
                break
            j -= 1
        out.append(pts[j])
        i = j
    return out


def resample(poly: list[tuple[float, float]], step: float) -> list[tuple[float, float]]:
    out = [poly[0]]
    for a, b in zip(poly, poly[1:]):
        seg = math.dist(a, b)
        n = max(int(math.ceil(seg / step)), 1)
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
    return out


def path_length(pts) -> float:
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


# ---------------------------------------------------------------- Xuất kết quả
def write_header(path: Path, wps: list[tuple[float, float]], stats: dict) -> None:
    rects = list(block_rects().items()) + [("FORBIDDEN", FORBIDDEN)]
    lines = [
        "/* Tự sinh bởi tools/path_planner/plan_path.py - KHÔNG sửa tay, chạy lại script. */",
        "/* Toạ độ cm, gốc ở góc dưới-trái sân, x sang phải, y lên trên. */",
        "#ifndef PATH_H",
        "#define PATH_H",
        "",
        "typedef struct {",
        "    float x_cm;",
        "    float y_cm;",
        "} PathPoint;",
        "",
        "typedef struct {",
        "    float x_min;",
        "    float y_min;",
        "    float x_max;",
        "    float y_max;",
        "} MapRect;",
        "",
        f"#define MAP_FIELD_W_CM          {FIELD_W:.1f}f",
        f"#define MAP_FIELD_H_CM          {FIELD_H:.1f}f",
        f"#define PATH_START_HEADING_DEG  {START_HEADING_DEG:.1f}f",
        f"#define PATH_CLEARANCE_MIN_CM   {stats['min_clearance']:.1f}f",
        f"#define PATH_LENGTH_CM          {stats['length']:.1f}f",
        f"#define PATH_POINT_COUNT        {len(wps)}U",
        f"#define MAP_OBSTACLE_COUNT      {len(rects)}U",
        f"#define MAP_BLOCK_COUNT         {len(BLOCK_CENTERS)}U  /* MAP_OBSTACLES[0..n) là khối thật, sau đó là vùng cấm */",
        "",
        "/* Chỉ include file này từ MỘT file .c (mảng static const). */",
        "static const PathPoint PATH_POINTS[PATH_POINT_COUNT] = {",
    ]
    lines += [f"    {{{x:7.2f}f, {y:7.2f}f}}," for x, y in wps]
    lines += ["};", "", "static const MapRect MAP_OBSTACLES[MAP_OBSTACLE_COUNT] = {"]
    lines += [f"    {{{r[0]:6.1f}f, {r[1]:6.1f}f, {r[2]:6.1f}f, {r[3]:6.1f}f}}, /* {name} */"
              for name, r in rects]
    lines += ["};", "", "#endif /* PATH_H */", ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def draw_field(ax) -> None:
    """Vẽ sân, 17 khối, vùng cấm, vạch xuất phát/đích (dùng chung với tools/sim/run_sim.py)."""
    from matplotlib.patches import Rectangle

    x0, y0, x1, y1 = FORBIDDEN
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, hatch="//", fill=False, ec="k", lw=2))
    for name, (bx0, by0, bx1, by1) in block_rects().items():
        ax.add_patch(Rectangle((bx0, by0), bx1 - bx0, by1 - by0, fc="#bbb", ec="k"))
        ax.text((bx0 + bx1) / 2, (by0 + by1) / 2, name, ha="center", va="center", fontsize=7)
    ax.add_patch(Rectangle((0, 0), FIELD_W, FIELD_H, fill=False, ec="k", lw=3))
    ax.plot([0, 60], [0, 0], color="green", lw=4, label="Vạch xuất phát")
    ax.plot(FINISH_X, [0, 0], color="k", lw=4, ls=":", label="Vạch đích")
    ax.set_xlim(-5, FIELD_W + 5)
    ax.set_ylim(-FINISH_OVERSHOOT - 5, FIELD_H + 5)
    ax.set_aspect("equal")
    ax.set_xticks(range(0, FIELD_W + 1, 10))
    ax.set_yticks(range(0, FIELD_H + 1, 10))
    ax.grid(alpha=0.3)


def plot(png: Path, clr: np.ndarray, raw, smooth_pts, wps) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 8.5))
    ax.imshow(clr < R_HARD, origin="lower", extent=(0, FIELD_W, 0, FIELD_H),
              cmap="Reds", alpha=0.25, vmin=0, vmax=1.5)
    ax.contour(np.arange(FIELD_W) + 0.5, np.arange(FIELD_H) + 0.5, clr,
               levels=[R_SOFT], colors="orange", linewidths=0.6, linestyles="--")
    draw_field(ax)

    rx, ry = zip(*raw)
    ax.plot(rx, ry, color="gray", lw=0.8, label="A* gốc")
    sx, sy = zip(*smooth_pts)
    ax.plot(sx, sy, color="blue", lw=2, label="Sau làm mượt")
    wx, wy = zip(*wps)
    ax.plot(wx, wy, "o", color="blue", ms=2.5, label=f"Waypoint ({len(wps)})")
    ax.plot(*START, "g^", ms=10)
    ax.set_title(f"Quỹ đạo A* - R_HARD={R_HARD} cm (vùng đỏ: tâm xe không được vào), "
                 f"R_SOFT={R_SOFT} cm (nét cam)")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(png, dpi=130)
    plt.close(fig)


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    root = Path(__file__).resolve().parents[2]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--header", type=Path, default=root / "Algo" / "planning" / "path.h")
    ap.add_argument("--png", type=Path, default=Path(__file__).resolve().parent / "out" / "path.png")
    args = ap.parse_args()

    clr = build_clearance()
    cells = astar(clr)
    raw = [START] + [(x + 0.5, y + 0.5) for x, y in cells[1:]]
    smooth_pts = smooth(clr, raw)
    # Kéo thẳng xuống qua vạch đích để cả thân xe qua vạch.
    end = smooth_pts[-1]
    smooth_pts.append((end[0], -FINISH_OVERSHOOT))
    wps = resample(smooth_pts, WAYPOINT_STEP)

    in_field = [p for p in wps if p[1] >= 0]
    stats = {
        "length": path_length(wps),
        "min_clearance": min(segment_clearance(clr, a, b) for a, b in zip(in_field, in_field[1:])),
    }
    if stats["min_clearance"] < R_HARD - 0.5:
        raise SystemExit(f"Quỹ đạo quá sát vật cản: {stats['min_clearance']:.1f} cm < {R_HARD} cm")

    args.header.parent.mkdir(parents=True, exist_ok=True)
    args.png.parent.mkdir(parents=True, exist_ok=True)
    write_header(args.header, wps, stats)
    plot(args.png, clr, raw, smooth_pts, wps)

    print(f"Bán kính bao xe          : {ROBOT_RADIUS:.1f} cm (R_HARD = {R_HARD} cm)")
    print(f"Độ dài quỹ đạo           : {stats['length']:.1f} cm")
    print(f"Khoảng cách tối thiểu    : {stats['min_clearance']:.1f} cm (tâm xe -> mặt vật cản)")
    print(f"Số đoạn thẳng / waypoint : {len(smooth_pts) - 1} / {len(wps)}")
    print("Các góc quay của quỹ đạo:")
    for a, b in zip(smooth_pts, smooth_pts[1:]):
        print(f"  ({a[0]:6.1f},{a[1]:6.1f}) -> ({b[0]:6.1f},{b[1]:6.1f})  "
              f"dài {math.dist(a, b):5.1f} cm, hướng {math.degrees(math.atan2(b[1]-a[1], b[0]-a[0])):6.1f}°")
    print(f"Đã ghi: {args.header}")
    print(f"Đã ghi: {args.png}")


if __name__ == "__main__":
    main()
