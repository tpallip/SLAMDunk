"""Printable course for the path reproduction test.

Start pad -> line -> end pad, drawn at 1:1. Writes a full-size PDF, the same
course tiled over A4 pages, a PNG preview and the line centreline as CSV.

    python3 tools/path_course.py [a1|a4x6] [out_dir]
"""
import csv
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Polygon, Arc

MM_PER_IN = 25.4

# Line and robot
LINE_W = 18.5          # same as the course tape
ROBOT_R = 48.5         # 97 mm body

# Line sensors from the axle centre (Pololu drawing, item 4973), x fwd, y left
SENSORS = [(28.0, 30.0), (40.0, 10.0), (41.5, 0.0), (40.0, -10.0), (28.0, -30.0)]

# Pad: band of an arc round the axle. All five sensors sit at 41-41.5 mm,
# so any heading puts them on the band as long as the axle is near the centre.
PAD_R_IN, PAD_R_OUT = 33.0, 50.0
PAD_HALF_ANGLE = 60.0  # deg each side of the heading

# A4 portrait page with a 10 mm margin leaves a 190 x 277 mm tile
A4 = (210.0, 297.0)
A4_MARGIN = 10.0
TILE_W, TILE_H = A4[0] - 2 * A4_MARGIN, A4[1] - 2 * A4_MARGIN

# Each course: sheet size, start axle pose and the line as straights
# ("s", length) and arcs ("a", radius, turn in deg, + is left)
COURSES = {
    # landscape A1, for a plotter (or 8 A4 tiles)
    "a1": {
        "sheet": (841.0, 594.0),
        "start": (110.0, 110.0, 0.0),
        "segments": [
            ("s", 120),
            ("a", 180, 90),
            ("s", 40),
            ("a", 150, -180),
            ("s", 60),
            ("a", 120, -90),
            ("s", 80),
        ],
    },
    # exactly 3 x 2 A4 tiles; the end pad sits inside one tile, clear of seams
    "a4x6": {
        "sheet": (3 * TILE_W, 2 * TILE_H),
        "start": (95.0, 100.0, 0.0),
        "segments": [
            ("s", 60),
            ("a", 110, 90),
            ("a", 95, -90),
            ("a", 90, 180),
            ("s", 270),
        ],
        "print_blank_tiles": True,
    },
}


def centreline(start, segments, step=1.0):
    """Sample the line centre as (x, y, heading) every `step` mm."""
    x, y, th = start
    pts = [(x, y, th)]
    for seg in segments:
        if seg[0] == "s":
            n = max(1, round(seg[1] / step))
            ds = seg[1] / n
            for _ in range(n):
                x += ds * math.cos(th)
                y += ds * math.sin(th)
                pts.append((x, y, th))
        else:
            r, turn = seg[1], math.radians(seg[2])
            n = max(1, round(r * abs(turn) / step))
            dth = turn / n
            sign = 1 if turn > 0 else -1
            cx = x - sign * r * math.sin(th)
            cy = y + sign * r * math.cos(th)
            for _ in range(n):
                th += dth
                x = cx + sign * r * math.sin(th)
                y = cy - sign * r * math.cos(th)
                pts.append((x, y, th))
    return np.array(pts)


def strip(pts, width):
    """Polygon for a band of `width` around a sampled curve."""
    nx, ny = -np.sin(pts[:, 2]), np.cos(pts[:, 2])
    h = width / 2
    left = np.c_[pts[:, 0] + h * nx, pts[:, 1] + h * ny]
    right = np.c_[pts[:, 0] - h * nx, pts[:, 1] - h * ny]
    return np.r_[left, right[::-1]]


def pad_polygon(cx, cy, th):
    a = np.linspace(-PAD_HALF_ANGLE, PAD_HALF_ANGLE, 61) * math.pi / 180 + th
    outer = np.c_[cx + PAD_R_OUT * np.cos(a), cy + PAD_R_OUT * np.sin(a)]
    inner = np.c_[cx + PAD_R_IN * np.cos(a), cy + PAD_R_IN * np.sin(a)]
    return np.r_[outer, inner[::-1]]


def build(course):
    sx, sy, sth = course["start"]
    segments = course["segments"]
    # line runs from the start pad's outer edge to the end pad's inner edge
    lead = (sx + PAD_R_OUT * math.cos(sth), sy + PAD_R_OUT * math.sin(sth), sth)
    line = centreline(lead, segments)
    ex, ey, eth = line[-1]
    end = (ex - PAD_R_IN * math.cos(eth), ey - PAD_R_IN * math.sin(eth), eth)
    # tuck both ends 2 mm into the pads so there is no white gap
    line = centreline((lead[0] - 2 * math.cos(sth), lead[1] - 2 * math.sin(sth), sth),
                      [("s", 2)] + segments + [("s", 2)])
    return line, course["start"], end


def check(line, start, end, sheet):
    """Robot footprint stays on the sheet and the course does not touch itself."""
    m = ROBOT_R + 10
    SHEET_W, SHEET_H = sheet
    xs, ys = line[:, 0], line[:, 1]
    for name, (x, y, _) in (("start", start), ("end", end)):
        assert m <= x <= SHEET_W - m and m <= y <= SHEET_H - m, f"{name} pad off sheet"
    assert xs.min() >= m and xs.max() <= SHEET_W - m, "line too close to sheet edge (x)"
    assert ys.min() >= m and ys.max() <= SHEET_H - m, "line too close to sheet edge (y)"
    # points far apart along the line must stay far apart on the sheet
    p = line[::5, :2]
    d = np.hypot(p[:, None, 0] - p[None, :, 0], p[:, None, 1] - p[None, :, 1])
    idx = np.arange(len(p)) * 5
    along = np.abs(idx[:, None] - idx[None, :])
    close = (along > 4 * ROBOT_R) & (d < 2 * ROBOT_R + 20)
    assert not close.any(), "course passes too close to itself"


def draw_course(ax, line, start, end, labels=True):
    ax.add_patch(Polygon(strip(line, LINE_W), closed=True, fc="black", ec="none"))
    for name, (x, y, th) in (("START", start), ("END", end)):
        ax.add_patch(Polygon(pad_polygon(x, y, th), closed=True, fc="black", ec="none"))
        # back half of the footprint and an axle cross, thin and light so the
        # sensors never cross them
        deg = math.degrees(th)
        ax.add_patch(Arc((x, y), 2 * ROBOT_R, 2 * ROBOT_R, theta1=deg + 95, theta2=deg + 265,
                         lw=0.6, ec="0.6", ls="--"))
        c, s = math.cos(th), math.sin(th)
        for dx, dy in ((c, s), (-s, c)):
            ax.plot([x - 6 * dx, x + 6 * dx], [y - 6 * dy, y + 6 * dy], c="0.5", lw=0.6)
        if labels:
            # START label behind the robot, END to its left (the line comes from behind)
            if name == "START":
                lx, ly = x - (ROBOT_R + 14) * c, y - (ROBOT_R + 14) * s
            else:
                lx, ly = x - (ROBOT_R + 14) * s, y + (ROBOT_R + 14) * c
            ax.text(lx, ly, name, ha="center", va="center", fontsize=14, color="0.5")


def sheet_axes(fig, x0, y0, w, h):
    """Axes in mm filling the given region of the figure at 1:1."""
    fw, fh = fig.get_size_inches() * MM_PER_IN
    ax = fig.add_axes([x0 / fw, y0 / fh, w / fw, h / fh])
    ax.set_aspect("equal")
    ax.axis("off")
    return ax


def scale_bar(ax, x, y, text=True):
    ax.plot([x, x + 100], [y, y], c="0.3", lw=1)
    for t in (0, 50, 100):
        ax.plot([x + t, x + t], [y - 2, y + 2], c="0.3", lw=1)
    if text:
        ax.text(x + 104, y, "100 mm: check before taping", va="center", fontsize=7, color="0.3")


def full_sheet(line, start, end, path, sheet, title):
    SHEET_W, SHEET_H = sheet
    fig = plt.figure(figsize=(SHEET_W / MM_PER_IN, SHEET_H / MM_PER_IN))
    ax = sheet_axes(fig, 0, 0, SHEET_W, SHEET_H)
    ax.set_xlim(0, SHEET_W)
    ax.set_ylim(0, SHEET_H)
    draw_course(ax, line, start, end)
    scale_bar(ax, 20, 20)
    ax.text(SHEET_W - 20, 20, f"Path reproduction course, {title} at 1:1, line {LINE_W} mm",
            ha="right", fontsize=8, color="0.4")
    fig.savefig(path)
    return fig


def tiles(line, start, end, path, sheet, all_tiles=False, page=A4, margin=A4_MARGIN):
    """Tile the sheet over A4 portrait pages. Trim one margin and butt the
    pages together on the printed frame."""
    SHEET_W, SHEET_H = sheet
    cw, ch = page[0] - 2 * margin, page[1] - 2 * margin
    cols = math.ceil(SHEET_W / cw)
    rows = math.ceil(SHEET_H / ch)
    pts = strip(line, LINE_W)
    pads = [pad_polygon(*start), pad_polygon(*end)]
    ink = np.vstack([pts] + pads)

    used = []
    for r in range(rows):
        for c in range(cols):
            x0, y0 = c * cw, r * ch
            inside = ((ink[:, 0] > x0 - 2) & (ink[:, 0] < x0 + cw + 2)
                      & (ink[:, 1] > y0 - 2) & (ink[:, 1] < y0 + ch + 2))
            if inside.any() or all_tiles:
                used.append((r, c))

    with PdfPages(path) as pdf:
        # overview page
        fig = plt.figure(figsize=(page[0] / MM_PER_IN, page[1] / MM_PER_IN))
        # fit the overview between the notes (bottom 110 mm) and the title
        gw, gh = cols * cw, rows * ch
        k = min((page[0] - 20) / gw, (page[1] - 140) / gh)
        ax = sheet_axes(fig, (page[0] - gw * k) / 2, 115, gw * k, gh * k)
        ax.set_xlim(0, SHEET_W)
        ax.set_ylim(0, SHEET_H)
        draw_course(ax, line, start, end, labels=False)
        for r in range(rows):
            for c in range(cols):
                on = (r, c) in used
                ax.add_patch(Polygon([(c * cw, r * ch), ((c + 1) * cw, r * ch),
                                      ((c + 1) * cw, (r + 1) * ch), (c * cw, (r + 1) * ch)],
                                     fc="none", ec="tab:blue" if on else "0.8", lw=0.6,
                                     clip_on=False))
                ax.text((c + 0.5) * cw, (r + 0.5) * ch, f"R{r + 1}C{c + 1}" if on else "",
                        ha="center", va="center", fontsize=6, color="tab:blue")
        ax.set_xlim(0, cols * cw)
        ax.set_ylim(0, rows * ch)
        fig.text(0.5, 0.93, "Path reproduction course: A4 tiles", ha="center", fontsize=13)
        fig.text(0.1, 0.35,
                 "Print at 100 % (no fit to page) and check the 100 mm bar on each page.\n"
                 f"Print the {len(used)} blue tiles; grey ones stay plain white.\n"
                 "Trim the margin on the left and bottom of each tile, line it up on\n"
                 "the thin grey frame of its neighbour and tape on the back.\n\n"
                 "Row 1 is the bottom row, column 1 the left column.",
                 fontsize=8, va="top")
        pdf.savefig(fig)
        plt.close(fig)

        for r, c in used:
            fig = plt.figure(figsize=(page[0] / MM_PER_IN, page[1] / MM_PER_IN))
            ax = sheet_axes(fig, margin, margin, cw, ch)
            ax.set_xlim(c * cw, (c + 1) * cw)
            ax.set_ylim(r * ch, (r + 1) * ch)
            draw_course(ax, line, start, end)
            ax.add_patch(Polygon([(c * cw, r * ch), ((c + 1) * cw, r * ch),
                                  ((c + 1) * cw, (r + 1) * ch), (c * cw, (r + 1) * ch)],
                                 fc="none", ec="0.75", lw=0.3, clip_on=False))
            # page furniture lives in the margin
            frame = sheet_axes(fig, 0, 0, page[0], page[1])
            frame.set_xlim(0, page[0])
            frame.set_ylim(0, page[1])
            scale_bar(frame, margin, margin / 2)
            frame.text(page[0] - margin, page[1] - margin / 2, f"R{r + 1}C{c + 1}",
                       ha="right", va="center", fontsize=9, color="0.3")
            pdf.savefig(fig)
            plt.close(fig)
    return len(used), rows * cols


def write_csv(line, start, path):
    """Line centreline in the start frame: axle on the start pad at the origin,
    x along the start heading. Matches odometry zeroed on the start pad."""
    sx, sy, sth = start
    c, s = math.cos(sth), math.sin(sth)
    dist = np.r_[0, np.cumsum(np.hypot(np.diff(line[:, 0]), np.diff(line[:, 1])))]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["s_mm", "x_mm", "y_mm", "heading_rad"])
        for d, (x, y, th) in zip(dist, line):
            dx, dy = x - sx, y - sy
            w.writerow([f"{d:.1f}", f"{c * dx + s * dy:.2f}", f"{-s * dx + c * dy:.2f}",
                        f"{th - sth:.5f}"])


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "a1"
    course = COURSES[name]
    sheet = course["sheet"]
    out = Path(sys.argv[2] if len(sys.argv) > 2 else f"bench/path_reproduction/course_{name}")
    out.mkdir(parents=True, exist_ok=True)
    line, start, end = build(course)
    check(line, start, end, sheet)

    fig = full_sheet(line, start, end, out / "course_full.pdf", sheet,
                     "A1" if name == "a1" else f"{sheet[0]:.0f} x {sheet[1]:.0f} mm")
    fig.savefig(out / "course_preview.png", dpi=40)
    plt.close(fig)
    n, total = tiles(line, start, end, out / "course_A4_tiles.pdf", sheet,
                     course.get("print_blank_tiles", False))
    write_csv(line, start, out / "course_centreline.csv")

    length = np.hypot(np.diff(line[:, 0]), np.diff(line[:, 1])).sum()
    ex, ey, eth = end
    print(f"line {length:.0f} mm, end axle at ({ex:.0f}, {ey:.0f}) mm, "
          f"heading {math.degrees(eth):.0f} deg")
    print(f"A4 tiles: printing {n} of {total}")
    print(f"written to {out}/")


if __name__ == "__main__":
    main()
