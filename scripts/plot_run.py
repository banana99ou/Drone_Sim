#!/usr/bin/env python3
"""Plot a logged run as a standalone SVG. No dependencies.

    python3 scripts/plot_run.py logs/circle.csv

Writes logs/circle.svg -- open it in any browser or image viewer. Deliberately
dependency-free: matplotlib is not installed on this host, and an SVG written
into the repo is viewable from Windows without any display or X server.

Three panels: top view (planned vs flown), altitude vs time, error vs time.
"""
import csv
import pathlib
import sys

W, H = 980, 1000
PAD = 60


def read_csv(path):
    rows = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append({k: float(v) for k, v in r.items() if v != ""})
            except (ValueError, TypeError):
                continue
    if not rows:
        raise SystemExit(f"{path}: no usable rows")
    return rows


def scaler(lo, hi, out_lo, out_hi):
    if hi - lo < 1e-9:
        lo, hi = lo - 1.0, hi + 1.0
    return lambda v: out_lo + (v - lo) / (hi - lo) * (out_hi - out_lo)


def polyline(pts, colour, width=2.0, dash=None):
    if not pts:
        return ""
    d = f' stroke-dasharray="{dash}"' if dash else ""
    s = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
    return (f'<polyline points="{s}" fill="none" stroke="{colour}" '
            f'stroke-width="{width}"{d} stroke-linejoin="round"/>')


def axes(x0, y0, x1, y1, title, xlabel, ylabel):
    return f"""
  <rect x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" fill="#fbfbfd" stroke="#c8ccd4"/>
  <text x="{x0}" y="{y0-12}" font-family="sans-serif" font-size="15" font-weight="600"
        fill="#1a1a2e">{title}</text>
  <text x="{(x0+x1)/2}" y="{y1+38}" font-family="sans-serif" font-size="12"
        fill="#555" text-anchor="middle">{xlabel}</text>
  <text x="{x0-42}" y="{(y0+y1)/2}" font-family="sans-serif" font-size="12" fill="#555"
        text-anchor="middle" transform="rotate(-90 {x0-42} {(y0+y1)/2})">{ylabel}</text>"""


def ticks(x0, y0, x1, y1, lo, hi, axis, fmt="{:.1f}"):
    out = []
    for i in range(5):
        frac = i / 4.0
        val = lo + frac * (hi - lo)
        if axis == "x":
            px = x0 + frac * (x1 - x0)
            out.append(f'<line x1="{px:.1f}" y1="{y1}" x2="{px:.1f}" y2="{y1+5}" stroke="#888"/>')
            out.append(f'<text x="{px:.1f}" y="{y1+20}" font-family="sans-serif" font-size="11" '
                       f'fill="#666" text-anchor="middle">{fmt.format(val)}</text>')
        else:
            py = y1 - frac * (y1 - y0)
            out.append(f'<line x1="{x0-5}" y1="{py:.1f}" x2="{x0}" y2="{py:.1f}" stroke="#888"/>')
            out.append(f'<text x="{x0-9}" y="{py+4:.1f}" font-family="sans-serif" font-size="11" '
                       f'fill="#666" text-anchor="end">{fmt.format(val)}</text>')
    return "\n  ".join(out)


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    src = pathlib.Path(sys.argv[1])
    rows = read_csv(src)

    t = [r["t"] for r in rows]
    have_ref = "ref_x" in rows[0]

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
             f'viewBox="0 0 {W} {H}"><rect width="{W}" height="{H}" fill="#ffffff"/>']

    rmse = rows[-1].get("rmse", 0.0)
    collided = any(r.get("collided", 0) > 0.5 for r in rows)
    parts.append(
        f'<text x="{PAD}" y="34" font-family="sans-serif" font-size="19" font-weight="700" '
        f'fill="#12121e">{src.stem}</text>'
        f'<text x="{PAD}" y="54" font-family="sans-serif" font-size="13" fill="#555">'
        f'{t[-1]-t[0]:.1f} s flown &#183; final RMSE {rmse*100:.1f} cm &#183; '
        f'{"COLLIDED" if collided else "no collision"}</text>')

    # ---------------- panel 1: top view -------------------------------------
    x0, y0, x1, y1 = PAD + 40, 90, W - PAD, 470
    xs = [r["x"] for r in rows] + ([r["ref_x"] for r in rows] if have_ref else [])
    ys = [r["y"] for r in rows] + ([r["ref_y"] for r in rows] if have_ref else [])
    lo_x, hi_x, lo_y, hi_y = min(xs), max(xs), min(ys), max(ys)
    # keep the aspect ratio square so a circle looks like a circle
    span = max(hi_x - lo_x, hi_y - lo_y) * 1.1 or 1.0
    cx, cy = (lo_x + hi_x) / 2, (lo_y + hi_y) / 2
    lo_x, hi_x = cx - span / 2, cx + span / 2
    lo_y, hi_y = cy - span / 2, cy + span / 2
    side = min(x1 - x0, y1 - y0)
    sx = scaler(lo_x, hi_x, x0, x0 + side)
    sy = scaler(lo_y, hi_y, y1, y1 - side)

    parts.append(axes(x0, y0, x0 + side, y1, "Top view: planned vs flown",
                      "x (m)", "y (m)"))
    parts.append(ticks(x0, y0, x0 + side, y1, lo_x, hi_x, "x"))
    parts.append(ticks(x0, y0, x0 + side, y1, lo_y, hi_y, "y"))
    if have_ref:
        parts.append(polyline([(sx(r["ref_x"]), sy(r["ref_y"])) for r in rows],
                              "#c62828", 2.4, dash="7,5"))
    parts.append(polyline([(sx(r["x"]), sy(r["y"])) for r in rows], "#1565c0", 2.0))
    parts.append(
        f'<text x="{x0+side-160}" y="{y0+22}" font-family="sans-serif" font-size="12" '
        f'fill="#c62828">--- planned</text>'
        f'<text x="{x0+side-160}" y="{y0+40}" font-family="sans-serif" font-size="12" '
        f'fill="#1565c0">&#8212; flown</text>')

    # ---------------- panel 2: altitude -------------------------------------
    x0, y0, x1, y1 = PAD + 40, 540, W - PAD, 700
    zs = [r["z"] for r in rows] + ([r["ref_z"] for r in rows] if have_ref else [])
    st = scaler(t[0], t[-1], x0, x1)
    sz = scaler(min(zs), max(zs), y1, y0)
    parts.append(axes(x0, y0, x1, y1, "Altitude", "t (s)", "z (m)"))
    parts.append(ticks(x0, y0, x1, y1, t[0], t[-1], "x"))
    parts.append(ticks(x0, y0, x1, y1, min(zs), max(zs), "y", "{:.2f}"))
    if have_ref:
        parts.append(polyline([(st(r["t"]), sz(r["ref_z"])) for r in rows],
                              "#c62828", 2.2, dash="7,5"))
    parts.append(polyline([(st(r["t"]), sz(r["z"])) for r in rows], "#1565c0", 2.0))

    # ---------------- panel 3: tracking error --------------------------------
    x0, y0, x1, y1 = PAD + 40, 780, W - PAD, 940
    es = [r.get("err", 0.0) for r in rows]
    se = scaler(0.0, max(max(es), 1e-3), y1, y0)
    parts.append(axes(x0, y0, x1, y1, "Tracking error |p - p_ref|", "t (s)", "error (m)"))
    parts.append(ticks(x0, y0, x1, y1, t[0], t[-1], "x"))
    parts.append(ticks(x0, y0, x1, y1, 0.0, max(max(es), 1e-3), "y", "{:.3f}"))
    parts.append(polyline([(st(r["t"]), se(r.get("err", 0.0))) for r in rows], "#00695c", 1.8))

    parts.append("</svg>")
    out = src.with_suffix(".svg")
    out.write_text("\n  ".join(parts))
    print(f"wrote {out}  ({len(rows)} samples, {t[-1]-t[0]:.1f} s)")
    print(f"  final RMSE   {rmse*100:.2f} cm")
    print(f"  max error    {max(es)*100:.2f} cm")
    print(f"  collided     {collided}")


if __name__ == "__main__":
    main()
