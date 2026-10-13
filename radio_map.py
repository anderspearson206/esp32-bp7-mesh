"""
radio_map.py  --  Reproduce the radio coverage map from saved coverage_*.csv files.

Usage:
    python radio_map.py bs_reports/coverage_*.csv
    python radio_map.py bs_reports/coverage_20260617_150217.csv --track
    python radio_map.py bs_reports/          (loads every coverage_*.csv in dir)
    python radio_map.py bs_reports/*.csv --out radio_map.png --title "Run 1"
    python radio_map.py bs_reports/*.csv --interp

Options:
    --out FILE      Save figure to FILE instead of (or in addition to) showing it.
                    Format is inferred from extension (png, pdf, svg, …).
    --title TEXT    Override figure title.
    --track         Draw the rover's path as a thin line connecting samples in time order.
    --interp        Overlay a nearest-neighbour interpolated RSSI heatmap behind the scatter.
    --vmin / --vmax RSSI colour-scale limits (default -90 / -30 dBm).
    --half-m FLOAT  Half-width of the plot area in metres (default: auto-fit + 10 % margin).
    --no-show       Skip the interactive window (useful when --out is specified in a script).
"""

import argparse
import csv
import glob
import os
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np


_NODE_COLORS  = ['dodgerblue', 'tomato', 'limegreen', 'darkorchid', 'gold', 'deepskyblue']
_NODE_MARKERS = ['o', 's', '^', 'D', 'P', 'X']


def load_csvs(paths: list[str]) -> dict[str, list[tuple]]:
    """Return {node_id: [(x, y, rssi, timestamp), ...]} from the given CSV files."""
    samples: dict[str, list] = defaultdict(list)
    loaded = skipped = 0

    for path in paths:
        try:
            with open(path, newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        rssi_raw = row['rssi_to_bs_dBm'].strip()
                        if not rssi_raw:
                            continue
                        x    = float(row['lat'])
                        y    = float(row['lon'])
                        rssi = int(rssi_raw)
                        nid  = row['node_id'].strip()
                        ts   = row['timestamp'].strip()
                        samples[nid].append((x, y, rssi, ts))
                        loaded += 1
                    except (ValueError, KeyError):
                        skipped += 1
        except OSError as e:
            print(f"[WARN] Could not open {path}: {e}")

    print(f"Loaded {loaded} samples across {len(samples)} node(s)"
          + (f", skipped {skipped} malformed rows" if skipped else "") + ".")
    return dict(samples)


def _expand_paths(raw_paths: list[str]) -> list[str]:
    """Expand directories, globs, and plain file paths into a flat list of CSV paths."""
    out = []
    for p in raw_paths:
        if os.path.isdir(p):
            out.extend(sorted(glob.glob(os.path.join(p, 'coverage_*.csv'))))
        else:
            expanded = glob.glob(p)
            if expanded:
                out.extend(sorted(expanded))
            elif os.path.exists(p):
                out.append(p)
            else:
                print(f"[WARN] No match for '{p}'")
    return out


def plot_radio_map(samples: dict[str, list[tuple]],
                   title: str = "Radio Coverage Map",
                   vmin: float = -100, vmax: float = -30,
                   half_m: float | None = None,
                   show_track: bool = False,
                   show_interp: bool = False,
                   out_path: str | None = None,
                   show_window: bool = True) -> None:

    if not samples:
        print("No samples to plot.")
        return

    all_x = [pt[0] for pts in samples.values() for pt in pts]
    all_y = [pt[1] for pts in samples.values() for pt in pts]

    if half_m is None:
        span = max(
            abs(max(all_x, default=0)),
            abs(min(all_x, default=0)),
            abs(max(all_y, default=0)),
            abs(min(all_y, default=0)),
            1.0,
        )
        half_m = span * 1.1

    fig, ax = plt.subplots(figsize=(9, 8))
    ax.set_xlim(-half_m, half_m)
    ax.set_ylim(-half_m, half_m)
    ax.set_aspect('equal')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='gray', linewidth=0.6)
    ax.axvline(0, color='gray', linewidth=0.6)

    # Base Station marker at origin
    ax.scatter([0], [0], s=300, marker='*', color='gold', edgecolors='black',
               linewidths=1.0, zorder=7, label='Base Station')
    ax.annotate('BS', (0, 0), textcoords='offset points', xytext=(6, 4),
                fontsize=8, fontweight='bold')

    last_scatter = None
    sorted_nodes = sorted(samples.keys())

    for idx, nid in enumerate(sorted_nodes):
        pts = samples[nid]
        xs    = np.array([p[0] for p in pts])
        ys    = np.array([p[1] for p in pts])
        rssis = np.array([p[2] for p in pts], dtype=float)

        color  = _NODE_COLORS[idx % len(_NODE_COLORS)]
        marker = _NODE_MARKERS[idx % len(_NODE_MARKERS)]

        # optional nearest-neighbour interpolated heatmap
        if show_interp and len(pts) >= 4:
            try:
                grid_n = 200
                gx = np.linspace(-half_m, half_m, grid_n)
                gy = np.linspace(-half_m, half_m, grid_n)
                gxx, gyy = np.meshgrid(gx, gy)
                triang = tri.Triangulation(xs, ys)
                interp = tri.LinearTriInterpolator(triang, rssis)
                zgrid  = interp(gxx, gyy)
                ax.contourf(gxx, gyy, zgrid, levels=20,
                            cmap='RdYlGn', alpha=0.35,
                            vmin=vmin, vmax=vmax, zorder=1)
            except Exception as e:
                print(f"[WARN] Interpolation failed for node {nid}: {e}")

        # track line
        if show_track:
            ax.plot(xs, ys, color=color, linewidth=0.7, alpha=0.5, zorder=2)

        # scatter coloured by RSSI
        sc = ax.scatter(xs, ys, c=rssis, cmap='RdYlGn',
                        vmin=vmin, vmax=vmax,
                        s=60, alpha=0.85,
                        marker=marker, linewidths=0, zorder=3,
                        label=f"Node {nid} ({len(pts)} pts)")
        last_scatter = sc

        # mark last known position with a larger outlined symbol
        ax.scatter([xs[-1]], [ys[-1]], s=180,
                   marker=marker, color='white', edgecolors=color,
                   linewidths=1.5, zorder=5)
        ax.annotate(nid, (xs[-1], ys[-1]),
                    textcoords='offset points', xytext=(6, 4),
                    fontsize=7, fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.6, lw=0))

    if last_scatter is not None:
        cb = plt.colorbar(last_scatter, ax=ax, fraction=0.046, pad=0.04)
        cb.set_label("RSSI to BS (dBm)")
        cb.ax.tick_params(labelsize=8)

    total = sum(len(v) for v in samples.values())
    ax.set_title(f"{title}  ({total} samples, {len(sorted_nodes)} node(s))",
                 fontsize=12, fontweight='bold')
    ax.legend(fontsize=8, loc='upper right')

    plt.tight_layout()

    if out_path:
        fig.savefig(out_path, dpi=150, bbox_inches='tight')
        print(f"Saved to {out_path}")

    if show_window:
        plt.show()
    else:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description='Reproduce radio coverage map from coverage_*.csv files.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('inputs', nargs='+',
                        help='Coverage CSV file(s) or directory containing them')
    parser.add_argument('--out',     default=None,  help='Save figure to this path')
    parser.add_argument('--title',   default='Radio Coverage Map', help='Figure title')
    parser.add_argument('--track',   action='store_true', help='Draw rover path lines')
    parser.add_argument('--interp',  action='store_true',
                        help='Overlay interpolated RSSI heatmap')
    parser.add_argument('--vmin',    type=float, default=-100, help='RSSI colour min (dBm)')
    parser.add_argument('--vmax',    type=float, default=-30, help='RSSI colour max (dBm)')
    parser.add_argument('--half-m',  type=float, default=None, dest='half_m',
                        help='Plot half-width in metres (default: auto)')
    parser.add_argument('--no-show', action='store_true',
                        help='Do not open interactive window')
    args = parser.parse_args()

    csv_paths = _expand_paths(args.inputs)
    if not csv_paths:
        print("No CSV files found. Pass coverage_*.csv paths or a directory.")
        sys.exit(1)

    print(f"Reading {len(csv_paths)} file(s)...")
    samples = load_csvs(csv_paths)
    if not samples:
        print("No valid samples found.")
        sys.exit(1)

    plot_radio_map(
        samples,
        title=args.title,
        vmin=args.vmin,
        vmax=args.vmax,
        half_m=args.half_m,
        show_track=args.track,
        show_interp=args.interp,
        out_path=args.out,
        show_window=not args.no_show,
    )


if __name__ == '__main__':
    main()
