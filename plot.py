#!/usr/bin/env python3
"""
3D path plotter for .dat files with blocks like:

ObjectName: NAME
Mass: FLOATMASSINKG
Path:
x_pos, y_pos, z_pos
... (more values)
...
...
EndObject

Usage examples:
  python plot3d_paths.py output.dat
  python plot3d_paths.py output.dat --center-on Sun --step 10 --save fig.png
  python plot3d_paths.py output.dat --unit auto   # auto-choose AU/Gm/Mm/km/m
"""

import argparse
import math
import re
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter


# ----------------------------- Parsing ---------------------------------

@dataclass
class SpaceObject:
    name: str
    mass_kg: Optional[float]
    path: np.ndarray  # shape (N, 3)


OBJ_RE = re.compile(r"^\s*ObjectName:\s*(.+?)\s*$")
MASS_RE = re.compile(r"^\s*Mass:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*$")
PATH_RE = re.compile(r"^\s*Path:\s*$")
END_RE = re.compile(r"^\s*EndObject\s*$")


def _try_parse_triplet(line: str) -> Optional[Tuple[float, float, float]]:
    # Accept "x, y, z" or "x y z" with optional commas/spaces
    if not line.strip():
        return None
    # Replace commas with spaces, split, take first three numbers
    clean = line.replace(",", " ")
    parts = [p for p in clean.split() if p]
    if len(parts) < 3:
        return None
    try:
        x, y, z = (float(parts[0]), float(parts[1]), float(parts[2]))
        return x, y, z
    except ValueError:
        return None


def parse_dat(filepath: str) -> List[SpaceObject]:
    objs: List[SpaceObject] = []
    with open(filepath, "r", encoding="utf-8") as f:
        lines = f.readlines()

    i = 0
    n = len(lines)
    while i < n:
        m_obj = OBJ_RE.match(lines[i])
        if not m_obj:
            i += 1
            continue

        name = m_obj.group(1).strip()
        i += 1

        # optional Mass line
        mass = None
        if i < n and MASS_RE.match(lines[i]):
            mass = float(MASS_RE.match(lines[i]).group(1))
            i += 1

        # require Path:
        if i < n and PATH_RE.match(lines[i]):
            i += 1
        else:
            # Skip until next object block if malformed
            while i < n and not END_RE.match(lines[i]):
                i += 1
            i += 1
            continue

        coords = []
        # Collect until EndObject
        while i < n and not END_RE.match(lines[i]):
            triplet = _try_parse_triplet(lines[i])
            if triplet is not None:
                coords.append(triplet)
            i += 1

        # consume EndObject if present
        if i < n and END_RE.match(lines[i]):
            i += 1

        if coords:
            arr = np.array(coords, dtype=np.float64)
        else:
            arr = np.zeros((0, 3), dtype=np.float64)

        objs.append(SpaceObject(name=name, mass_kg=mass, path=arr))

    if not objs:
        raise ValueError("No objects parsed. Check the file format.")
    return objs


# ----------------------------- Units -----------------------------------

UNIT_TABLE = {
    "m": ("m", 1.0),
    "km": ("km", 1e3),
    "Mm": ("Mm", 1e6),
    "Gm": ("Gm", 1e9),
    "AU": ("AU", 1.495978707e11),  # IAU 2012 conventional AU
}


def auto_pick_unit(extent_m: float) -> str:
    """
    Choose a readable unit based on overall span of data (max range on any axis).
    """
    # very large → AU; then Gm/Mm/km/m
    if extent_m >= 5e11:
        return "AU"
    if extent_m >= 5e8:
        return "Gm"
    if extent_m >= 5e5:
        return "Mm"
    if extent_m >= 5e3:
        return "km"
    return "m"


def format_with_unit(unit: str):
    scale = UNIT_TABLE[unit][1]
    def _fmt(x, _pos=None):
        return f"{x/scale:.3g}"
    return _fmt


# ----------------------------- Plotting --------------------------------

def compute_extent(all_points: np.ndarray) -> float:
    """Max axis range in meters (before scaling)."""
    mins = np.nanmin(all_points, axis=0)
    maxs = np.nanmax(all_points, axis=0)
    ranges = maxs - mins
    return float(np.nanmax(ranges)) if np.isfinite(ranges).all() else 1.0


def set_equal_aspect(ax, points_scaled: np.ndarray):
    mins = np.nanmin(points_scaled, axis=0)
    maxs = np.nanmax(points_scaled, axis=0)
    centers = (mins + maxs) / 2.0
    spans = (maxs - mins)
    max_span = float(np.nanmax(spans))
    if not np.isfinite(max_span) or max_span == 0:
        max_span = 1.0
    # Create a cube box around the center
    half = max_span / 2.0
    ax.set_xlim(centers[0] - half, centers[0] + half)
    ax.set_ylim(centers[1] - half, centers[1] + half)
    ax.set_zlim(centers[2] - half, centers[2] + half)
    # For modern matplotlib, this enforces equal aspect in 3D
    try:
        ax.set_box_aspect([1, 1, 1])
    except Exception:
        pass


def plot_objects(objs: List[SpaceObject],
                 unit: str,
                 center_on: Optional[str],
                 step: int,
                 save_path: Optional[str],
                 show_start: bool,
                 show_end: bool,
                 linewidth: float,
                 use_mass_linewidth: bool):
    # Build a single array of all points (in meters) for extent
    all_points = np.concatenate([o.path for o in objs if o.path.size > 0], axis=0)
    extent_m = compute_extent(all_points)
    if unit == "auto":
        unit = auto_pick_unit(extent_m)
    unit_lbl, unit_scale = UNIT_TABLE[unit]

    # Optional centering
    center_vec = np.zeros(3)
    if center_on:
        ref = next((o for o in objs if o.name == center_on), None)
        if ref is None or ref.path.size == 0:
            print(f"[Info] center-on '{center_on}' not found or has no path; skipping centering.")
        else:
            center_vec = ref.path[0].copy()

    # Prepare plot
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Linewidth scaling by mass (optional)
    def mass_to_lw(m: Optional[float]) -> float:
        if not use_mass_linewidth or m is None or m <= 0:
            return linewidth
        # log scale: map log10(mass) to a reasonable width range
        lw = 0.5 + (math.log10(m) - 20) * 0.25  # tweakable
        return float(max(0.5, min(5.0, lw)))

    # Plot each object
    for idx, o in enumerate(objs):
        if o.path.size == 0:
            continue
        P = (o.path - center_vec) / unit_scale
        P = P[::max(1, step)]  # downsample if requested

        line, = ax.plot(P[:, 0], P[:, 1], P[:, 2],
                        label=o.name,
                        linewidth=mass_to_lw(o.mass_kg))

        # Optional start/end markers
        if show_start:
            ax.scatter(P[0, 0], P[0, 1], P[0, 2], s=20, marker="o")
        if show_end:
            ax.scatter(P[-1, 0], P[-1, 1], P[-1, 2], s=20, marker="^")

        # Label near the last point for readability
        ax.text(P[-1, 0], P[-1, 1], P[-1, 2], f"  {o.name}", fontsize=9)

    # Axes/labels
    ax.set_xlabel(f"X [{unit_lbl}]")
    ax.set_ylabel(f"Y [{unit_lbl}]")
    ax.set_title("3D Trajectories")

    # Ticks formatter to avoid scientific notation spam if unit ≠ m
    if unit != "m":
        fmt = FuncFormatter(lambda x, pos: f"{x:.3g}")
        ax.xaxis.set_major_formatter(fmt)
        ax.yaxis.set_major_formatter(fmt)
        ax.zaxis.set_major_formatter(fmt)

    # Equal aspect
    if all_points.size > 0:
        set_equal_aspect(ax, (all_points - center_vec) / unit_scale)

    ax.grid(True, which="both", linewidth=0.3)
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1), borderaxespad=0.)
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"[Info] Figure saved to: {save_path}")
    else:
        plt.show()


# ----------------------------- CLI -------------------------------------

def main():
    ap = argparse.ArgumentParser(description="3D plotter for multi-object .dat trajectory files.")
    ap.add_argument("file", help="Path to the .dat file")
    ap.add_argument("--unit", default="auto",
                    choices=list(UNIT_TABLE.keys()) + ["auto"],
                    help="Distance unit for axes (default: auto)")
    ap.add_argument("--center-on", default=None,
                    help="Object name to center the scene on (subtract its initial position)")
    ap.add_argument("--step", type=int, default=1,
                    help="Downsample factor for plotting (plot every Nth point)")
    ap.add_argument("--save", dest="save_path", default=None,
                    help="If provided, save the figure to this path instead of showing interactively")
    ap.add_argument("--hide-start", action="store_true", help="Hide start markers")
    ap.add_argument("--hide-end", action="store_true", help="Hide end markers")
    ap.add_argument("--lw", type=float, default=1.5, help="Base line width")
    ap.add_argument("--lw-by-mass", action="store_true",
                    help="Scale line width by object mass (log scale) when Mass is present")
    args = ap.parse_args()

    objs = parse_dat(args.file)
    print(f"[Info] Parsed {len(objs)} object(s): " + ", ".join(o.name for o in objs))

    # Sanity ping about chosen unit
    # Compute extent to report auto selection
    all_points = np.concatenate([o.path for o in objs if o.path.size > 0], axis=0)
    extent_m = compute_extent(all_points)
    chosen_unit = args.unit if args.unit != "auto" else auto_pick_unit(extent_m)
    print(f"[Info] Plot units: scaled from meters → {chosen_unit} (1 {chosen_unit} = {UNIT_TABLE[chosen_unit][1]:.3g} m)")

    plot_objects(
        objs,
        unit=args.unit,
        center_on=args.center_on,
        step=max(1, args.step),
        save_path=args.save_path,
        show_start=not args.hide_start,
        show_end=not args.hide_end,
        linewidth=args.lw,
        use_mass_linewidth=args.lw_by_mass,
    )


if __name__ == "__main__":
    main()
