"""
relay_replay.py - run a recorded explorer track through relay_brain.py and plot what the relay does.

The explorer's map positions come from a session's coverage.csv (bs_core.py writes it). The relay
starts at the BS (or --relay-start), and every RELAY_TICK_S the same RelayDriver the rover runs
decides; when it sends a route, the simulated relay drives it at --speed. The plot shows the
explorer track, the relay's path and targets, and over time the predicted link from the explorer
to the BS: direct, and through the relay (the weaker of its two links), using the same link model.

    python relay_replay.py bs_reports/session_20261021_153126 --explorer 57936 --site fountain
    python relay_replay.py <session> --explorer 57936 --site fountain --linktab ../terrain/fountain_linktab_nn.npz
    python relay_replay.py <session> --explorer 57936 --site fountain --dropout 120:200   # test ferrying
"""

import argparse
import csv
import json
import math
import os
from datetime import datetime

import numpy as np

import relay_brain
import site_map

DT = 0.25                 # simulation step [s]


def load_track(path, node):
    """[(epoch, map x, map y)] of one rover from coverage.csv, time-ordered, (0, 0) odometry skipped."""
    out = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            if row['node_id'].strip() != node:
                continue
            try:
                if float(row['lat']) == 0.0 and float(row['lon']) == 0.0:
                    continue
                t = datetime.strptime(row['timestamp'], '%Y-%m-%d %H:%M:%S.%f').timestamp()
                x, y = float(row['map_x']), float(row['map_y'])
            except (ValueError, KeyError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                out.append((t, x, y))
    out.sort()
    return out


def follow(route, pos, dist):
    """Move dist metres along the route polyline from pos (assumed on/near its start segment).
    Returns (new pos, remaining route) - the remaining route starts at the new pos."""
    pts = [tuple(pos)] + [tuple(p) for p in route[1:]]
    while len(pts) > 1 and dist > 0:
        (x0, y0), (x1, y1) = pts[0], pts[1]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg <= dist:
            dist -= seg
            pts.pop(0)
        else:
            f = dist / seg
            pts[0] = (x0 + f * (x1 - x0), y0 + f * (y1 - y0))
            dist = 0
    return pts[0], pts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', help='session folder or its coverage.csv')
    ap.add_argument('--explorer', required=True, help='node id whose track is the explorer')
    ap.add_argument('--site', required=True)
    ap.add_argument('--terrain-dir')
    ap.add_argument('--linktab', help='<site>_linktab_<model>.npz (default: distance model)')
    ap.add_argument('--bs', help='BS map position x,y (default: the session bs_position.json, else the site tx)')
    ap.add_argument('--relay-start', help='relay start x,y (default: at the BS)')
    ap.add_argument('--speed', type=float, default=0.5, help='relay speed [m/s]')
    ap.add_argument('--ferry-s', type=float, default=relay_brain.DEFAULTS['ferry_s'])
    ap.add_argument('--dropout', action='append', default=[], metavar='A:B',
                    help='explorer silent from A to B seconds after the start (repeatable)')
    ap.add_argument('--link-dbm', type=float, default=-85.0, help='"connected" threshold for the summary')
    ap.add_argument('--out', help='save the figure here (png/pdf/...)')
    ap.add_argument('--no-show', action='store_true')
    args = ap.parse_args()

    cov = args.session if args.session.endswith('.csv') else os.path.join(args.session, 'coverage.csv')
    track = load_track(cov, args.explorer)
    if len(track) < 2:
        raise SystemExit(f"{cov}: fewer than 2 positions for node {args.explorer}")
    site = site_map.Site(site_map.find_terrain_dir(args.terrain_dir), args.site)
    xy = lambda s: tuple(float(v) for v in s.split(','))   # noqa: E731
    if args.bs:
        bs = xy(args.bs)
    else:
        bs = site.tx_xy
        bp = os.path.join(os.path.dirname(cov), 'bs_position.json')
        if os.path.exists(bp):
            with open(bp) as f:
                p = json.load(f)
            bs = (float(p['x']), float(p['y']))
    model = relay_brain.load_model(site, args.linktab)
    driver = relay_brain.RelayDriver(site, model)          # the table (or distance) is its default
    driver.configure({'ex': args.explorer, 'speed': args.speed, 'ferry_s': args.ferry_s})
    drops = [tuple(float(v) for v in d.split(':')) for d in args.dropout]

    t0, t_end = track[0][0], track[-1][0]
    relay = tuple(model.cand_xy[model.nearest_cand(xy(args.relay_start) if args.relay_start else bs)])
    to_bs = model.link_to(bs)
    route, k, next_tick = None, 0, t0
    log = []            # (t - t0, relay x, relay y, state, direct dBm, via relay dBm)
    path, targets, driven, n_routes = [relay], [], 0.0, 0
    t = t0
    while t <= t_end:
        while k + 1 < len(track) and track[k + 1][0] <= t:
            k += 1
        ex_xy = track[k][1:]
        silent = any(a <= t - t0 < b for a, b in drops)
        if t >= next_tick:
            next_tick += relay_brain.RELAY_TICK_S
            nav_state = 'following' if route is not None and len(route) > 1 else 'done'
            fix = None if silent else (track[k][0], ex_xy)
            pts = driver.tick(t, relay, fix, bs, nav_state)
            if pts is not None:
                route = [tuple(p) for p in pts]
                n_routes += 1
                targets.append(driver.brain.target)
            # predicted link with the explorer where it really is
            i_r = model.nearest_cand(relay)
            to_ex = model.link_to(ex_xy)
            direct = float(to_bs[model.nearest_cand(ex_xy)])
            via = min(float(to_bs[i_r]), float(to_ex[i_r]))
            log.append((t - t0, relay[0], relay[1], driver.status['rl'], direct, via, silent))
        if route is not None and len(route) > 1:
            new, route = follow(route, relay, args.speed * DT)
            driven += math.hypot(new[0] - relay[0], new[1] - relay[1])
            relay = new
            path.append(relay)
        t += DT

    L = np.array([(r[0], r[4], r[5]) for r in log])
    best = np.maximum(L[:, 1], L[:, 2])
    states = [r[3] for r in log]
    print(f"model {model.name}; explorer {args.explorer}: {len(track)} fixes over {(t_end - t0) / 60:.1f} min; "
          f"BS at ({bs[0]:.1f}, {bs[1]:.1f})")
    print(f"relay: {n_routes} routes, drove {driven:.0f} m; states " +
          ", ".join(f"{s} {100 * states.count(s) / len(states):.0f}%" for s in sorted(set(states))))
    print(f"explorer link >= {args.link_dbm:g} dBm: direct {100 * (L[:, 1] >= args.link_dbm).mean():.0f}% "
          f"of the time, with the relay {100 * (best >= args.link_dbm).mean():.0f}%; "
          f"median {np.median(L[:, 1]):.0f} -> {np.median(best):.0f} dBm")

    import matplotlib.pyplot as plt
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 6.8), gridspec_kw={'width_ratios': [1, 1.25]})
    h = site.half
    if site.drivable is not None:
        ax.imshow(~site.drivable, cmap='gray', vmin=-1, vmax=1.6, extent=(-h, h, -h, h))
    tr = np.array(track)
    ax.plot(tr[:, 1], tr[:, 2], '-', color='tab:blue', lw=1.5, label=f'explorer {args.explorer}')
    pa = np.array(path)
    ax.plot(pa[:, 0], pa[:, 1], '-', color='tab:red', lw=2.2, label='relay (simulated)')
    if targets:
        tg = np.array(targets)
        ax.plot(tg[:, 0], tg[:, 1], 'D', color='tab:red', mec='k', ms=7, label='relay targets')
    ax.plot(*bs, '*', color='gold', mec='k', ms=18, label='BS')
    ax.plot(pa[0, 0], pa[0, 1], 'o', color='tab:red', mec='k', ms=8)
    allx = np.concatenate([tr[:, 1], pa[:, 0], [bs[0]]])
    ally = np.concatenate([tr[:, 2], pa[:, 1], [bs[1]]])
    pad = 10
    ax.set_xlim(allx.min() - pad, allx.max() + pad)
    ax.set_ylim(ally.min() - pad, ally.max() + pad)
    ax.set_aspect('equal')
    ax.set_xlabel('x / east [m]')
    ax.set_ylabel('y / north [m]')
    ax.legend(loc='best', fontsize=8)
    ax.set_title(f'{site.name}: relay replay ({model.name})', fontsize=10)

    ax2.plot(L[:, 0], L[:, 1], color='tab:blue', lw=1.2, label='explorer direct to BS')
    ax2.plot(L[:, 0], L[:, 2], color='tab:red', lw=1.2, label='through relay (weaker link)')
    ax2.axhline(args.link_dbm, color='k', ls=':', lw=1, label=f'{args.link_dbm:g} dBm')
    shade = {'ferry': '#bbdefb', 'lost_hold': '#ffe0b2'}
    for s, col in shade.items():
        on = np.array([st == s for st in states])
        if on.any():
            ax2.fill_between(L[:, 0], 0, 1, where=on, color=col, alpha=0.6, transform=ax2.get_xaxis_transform(),
                             label=s.replace('_', ' '), step='mid')
    for a, b in drops:
        ax2.axvspan(a, b, ymin=0, ymax=0.04, color='k', alpha=0.5)
    ax2.set_xlabel('time [s]')
    ax2.set_ylabel('predicted RSS [dBm]')
    ax2.legend(loc='lower left', fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=130)
        print(f"saved {args.out}")
    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
