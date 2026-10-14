"""
fix_track.py  --  Remove odometry drift from a session's coverage.csv using returns to the BS.

Wheel odometry gets heading from the steering command and wheel speed, so it goes wrong
when the wheels slip (tight turns, reversing, wheel spin) or the steering trim is off.
When each run starts and ends parked at the BS ("home"), every leg is a closed loop and
the drift can be taken out afterwards.

  1. Legs are found from the odometry: a leg is a stretch of driving (>= --min-leg m)
     between stops.  Everything between legs (parked, small shuffles at the BS) is home.
  2. Each leg is corrected with one of two drift models:
       turn  (default)  out-and-back legs.  The heading error is put at the turnaround
                        (farthest point from home): the way back is turned by delta and
                        stretched by scale so it ends exactly at home.
       bend             a steady steering bias: heading bent by k deg per metre driven,
                        leftover closure error spread along the leg.
       pf               map-matching: a particle filter follows the rover through the
                        drivable path network.  Each particle carries a map position and
                        an odometry -> map rotation; odometry steps move it (distance noise
                        ~7%, heading noise that grows with how sharply the odometry turned
                        and with wheel-spin speed spikes), particles off the drivable mask
                        lose weight, and every return to the BS is a strong position fix.
                        Heading is re-opened (+-40 deg) at each leg start, since the rover
                        may be turned by hand at the BS.  The output is the surviving
                        particle lineage (a smoothed track).  Assumes the rover stays on
                        mapped paths; off-path stretches fall back to plain odometry.
                        Writes *_pf files and, if coverage_fixed.csv exists, compares to it.
  3. Start heading of each leg: leg 1 uses the alignment logged in the session (where you
     placed the rover on the map); every later leg carries on from the corrected heading
     at the end of the leg before.  With --search N the start heading may move up to
     +-N deg if that keeps the track on the drivable mask noticeably better.
     --leg-heading 2:90 says leg 2 set off due north (map heading of the first ~5 m of
     driving, deg: 0 = east, 90 = north, 180 = west, -90 = south); later legs carry on
     from there.  Use it when the rover was turned or carried by hand at the BS between
     legs: odometry can't see that.  fix_track_candidates.png shows, for each leg, the
     start directions that keep it on the drivable paths, to help pick.

Writes next to the input:  coverage_fixed.csv  (same columns with map_* / terrain /
predictions recomputed, plus raw_map_x/y, leg, leg_turn_deg, leg_scale)  and  fix_track.png.

Usage:
    python fix_track.py bs_reports/session_..._exp_140251 --site fountain
    python fix_track.py <session> --site fountain --model pf
    python fix_track.py <session> --site fountain --leg-heading 2:90 --leg-heading 3:-45
"""

import argparse
import csv
import json
import math
import os
import sys
from datetime import datetime

import numpy as np

import site_map

STOP_STEP_M = 0.05        # odometry step below this between samples = not moving


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load(path, node):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    nodes = sorted({r['node_id'] for r in rows})
    if node is None:
        if len(nodes) != 1:
            sys.exit(f"several rovers in {path}: {nodes}; pick one with --node")
        node = nodes[0]
    rows = [r for r in rows if r['node_id'] == node]
    if not rows:
        sys.exit(f"no rows for node {node} (have {nodes})")
    return node, rows


def num(rows, key):
    return np.array([float(r[key]) if r.get(key) not in (None, '') else np.nan for r in rows])


def find_legs(x, y, min_leg):
    """[(a, b)] index pairs: a = last parked sample before driving, b = first parked after."""
    step = np.r_[0.0, np.hypot(np.diff(x), np.diff(y))]
    moving = step > STOP_STEP_M
    legs, i, n = [], 1, len(x)
    while i < n:
        if not moving[i]:
            i += 1
            continue
        j = i
        while j < n and moving[j]:
            j += 1
        a, b = i - 1, min(j, n - 1)
        if step[i:j].sum() >= min_leg:
            legs.append((a, b))
        i = j
    return legs


def find_turnaround(x, y, yaw, a, b, w=4, spike_factor=2.5):
    """Where the heading error goes in: the end of a wheel-spin stretch (odometry steps far
    above the leg's typical step) in the middle of the leg if there is one, else the end of
    the sharpest heading change in the middle half (by distance). Not simply the farthest
    point: with bad drift the logged end can be farther out."""
    step = np.hypot(np.diff(x[a:b + 1]), np.diff(y[a:b + 1]))
    s = np.r_[0.0, np.cumsum(step)]
    moving = step[step > STOP_STEP_M]
    if len(moving) >= 5:
        typ = float(np.median(moving))
        spikes = np.where((step > spike_factor * typ) & (s[1:] > 0.2 * s[-1]) & (s[1:] < 0.8 * s[-1]))[0]
        if len(spikes):
            # the spike cluster nearest the middle of the leg; turnaround = just after it
            gaps = np.split(spikes, np.where(np.diff(spikes) > 3)[0] + 1)
            mid_i = int(np.searchsorted(s, s[-1] / 2))
            cl = min(gaps, key=lambda g: abs((g[0] + g[-1]) / 2 - mid_i))
            return min(a + int(cl[-1]) + 2, b - 1)
    mid = np.where((s > 0.25 * s[-1]) & (s < 0.75 * s[-1]))[0] + a
    if np.isnan(yaw[a:b + 1]).any() or len(mid) == 0:
        return a + int(np.searchsorted(s, s[-1] / 2))
    u = np.degrees(np.unwrap(np.radians(yaw)))
    lo, hi = np.clip(mid - w, a, b), np.clip(mid + w, a, b)
    i = int(np.argmax(np.abs(u[hi] - u[lo])))
    return int(hi[i])


def fit_turn(x, y, yaw, a, b, k_max_deg=8.0):
    """Out-and-back: keep the way out; the way back is turned by delta at the turnaround,
    bent by a steady k deg/m (a heading bias that kept running after a bad turnaround) and
    stretched by scale so it ends exactly where it started. k is kept as small as gives a
    sensible stretch (k = 0 when a plain turn closes it with a stretch near 1).
    Returns per-increment rotation [rad] and scale."""
    d = np.hypot(x[a:b + 1] - x[a], y[a:b + 1] - y[a])
    m = find_turnaround(x, y, yaw, a, b)
    want = np.array([x[a] - x[m], y[a] - y[m]])           # turnaround -> start
    bdx, bdy = np.diff(x[m:b + 1]), np.diff(y[m:b + 1])    # way back increments
    sb = np.cumsum(np.hypot(bdx, bdy))
    ks = np.radians(np.linspace(-k_max_deg, k_max_deg, 641))
    th = ks[:, None] * sb
    gx = np.sum(np.cos(th) * bdx - np.sin(th) * bdy, axis=1)
    gy = np.sum(np.sin(th) * bdx + np.cos(th) * bdy, axis=1)
    stretch = np.hypot(*want) / np.maximum(np.hypot(gx, gy), 1e-6)
    # no bend while a plain turn needs < 25 % stretch; beyond that the smallest bend that gets there
    cost = 10 * np.maximum(0.0, np.abs(np.log(stretch)) - math.log(1.25)) + 0.03 * np.abs(np.degrees(ks))
    i = int(np.argmin(cost))
    k = float(ks[i])
    delta = wrap(math.atan2(want[1], want[0]) - math.atan2(gy[i], gx[i]))
    scale = float(stretch[i])
    idx = np.arange(a + 1, b + 1)
    back = idx > m
    s_back = np.zeros(len(idx))
    s_back[back] = sb
    rot = np.where(back, delta + k * s_back, 0.0)
    sc = np.where(back, scale, 1.0)
    return dict(rot=rot, scale=sc, turn_idx=m, delta=delta, scale_back=scale, k_back=k,
                out_m=float(d[m - a]), end_rot=float(rot[-1]) if len(rot) else 0.0, leftover=0.0)


def fit_bend(x, y, a, b, k_max_deg):
    """Steady steering bias: heading bent by k rad per metre, leftover spread along the leg."""
    dx, dy = np.diff(x[a:b + 1]), np.diff(y[a:b + 1])
    s = np.cumsum(np.hypot(dx, dy))
    ks = np.radians(np.linspace(-k_max_deg, k_max_deg, 2001))
    th = ks[:, None] * s
    ex = np.sum(np.cos(th) * dx - np.sin(th) * dy, axis=1)
    ey = np.sum(np.sin(th) * dx + np.cos(th) * dy, axis=1)
    k = ks[int(np.argmin(np.hypot(ex, ey)))]
    return dict(rot=k * s, scale=np.ones_like(s), k=k, s=s, end_rot=k * s[-1],
                leftover=float(np.min(np.hypot(ex, ey))))


def setoff_dir(x, y, a, b, dist=5.0):
    """Odometry direction [rad] of the first `dist` metres of a leg."""
    d = np.hypot(x[a:b + 1] - x[a], y[a:b + 1] - y[a])
    j = a + int(np.argmax(d >= dist)) if (d >= dist).any() else b
    return math.atan2(y[j] - y[a], x[j] - x[a])


def candidates(site, x, y, a, b, fit, home, n=4, sep=25):
    """Best few set-off headings [deg] for a leg by share on the drivable mask."""
    base = setoff_dir(x, y, a, b)
    hd = np.arange(-180, 180, 2.0)
    score = np.array([site.on_drivable(*place_leg(x, y, a, b, fit, math.radians(h) - base, home), 1.0).mean()
                      for h in hd])
    out = []
    for i in np.argsort(-score):
        if all(abs((hd[i] - h + 180) % 360 - 180) >= sep for h, _ in out):
            out.append((float(hd[i]), float(score[i])))
        if len(out) == n:
            break
    return base, out


# map-matching particle filter (--model pf)
PF = dict(
    n=3000,               # particles
    seed=1,
    dist_noise=0.07,      # odometry distance error (fraction)
    head_per_m=0.6,       # heading random walk [deg / sqrt(m)] while driving
    head_per_turn=0.12,   # extra heading noise per unit of odometry turn in one step
    spike_factor=2.5,     # step > this x the leg's typical step = wheel spin
    spike_head=12.0,      # extra heading noise on a spike step [deg]
    sigma=1.5,            # drivable likelihood width [m] (distance outside the mask)
    floor=0.03,           # likelihood floor so off-path stretches don't kill the filter
    home_sigma=2.5,       # position fix when parked at the BS [m]
    leg_spread=40.0,      # heading re-opened at each leg start [+- deg]
    follow_sigma=3.0,     # --follow: how close to the other leg's track [m]
)


def dist_to_polyline(px, py, line):
    """Distance from each point to a polyline (k x 2 array)."""
    a, b = line[:-1], line[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(axis=1), 1e-9)
    P = np.stack([px, py], axis=1)[:, None, :]
    t = np.clip(((P - a) * ab).sum(axis=2) / L2, 0, 1)
    q = a + t[..., None] * ab
    return np.sqrt(((P - q) ** 2).sum(axis=2)).min(axis=1)


def follow_guides(x, y, yaw, legs, fx, fy, pairs):
    """--follow L:M  (leg L drove leg M's route, maybe further): for leg L's samples
    within leg M's out distance of home (on the way out and on the way back), a polyline
    of leg M's way out from a previous pass.  Returns {sample index: polyline}."""
    guides = {}
    for L, M in pairs:
        (a, b), (am, bm) = legs[L - 1], legs[M - 1]
        tm = find_turnaround(x, y, yaw, am, bm)
        sm = np.hypot(np.diff(x[am:tm + 1]), np.diff(y[am:tm + 1])).sum()
        line = np.stack([fx[am:tm + 1], fy[am:tm + 1]], axis=1)
        t = find_turnaround(x, y, yaw, a, b)
        s = np.r_[0.0, np.cumsum(np.hypot(np.diff(x[a:b + 1]), np.diff(y[a:b + 1])))]
        for k, i in enumerate(range(a + 1, b)):
            out_dist = s[k + 1] if i <= t else s[-1] - s[k + 1]
            if out_dist <= sm:
                guides[i] = line
    return guides


def run_pf(site, x, y, yaw, legs, home, theta0, P=PF, guides=None):
    """Particle filter over the whole session.  Returns map x, y, rotation per sample
    (from the final surviving lineage) and the effective sample size per sample."""
    from scipy.ndimage import distance_transform_edt
    rng = np.random.default_rng(P['seed'])
    N, n = P['n'], len(x)
    dmap = distance_transform_edt(~site.drivable) * site.cell if site.drivable is not None else None

    in_leg = np.zeros(n, dtype=bool)
    starts = set()
    step = np.r_[0.0, np.hypot(np.diff(x), np.diff(y))]
    typical = np.zeros(n)
    for a, b in legs:
        in_leg[a + 1:b] = True
        starts.add(a)
        mv = step[a + 1:b + 1]
        typical[a + 1:b + 1] = np.median(mv[mv > STOP_STEP_M]) if (mv > STOP_STEP_M).any() else 1.0
    yaw_r = np.radians(np.nan_to_num(yaw))

    px = np.full(N, home[0])
    py = np.full(N, home[1])
    th = theta0 + np.radians(rng.uniform(-P['leg_spread'], P['leg_spread'], N))
    w = np.full(N, 1.0 / N)
    hist = np.zeros((n, 3, N), dtype=np.float32)
    parent = np.zeros((n, N), dtype=np.int32)
    ident = np.arange(N, dtype=np.int32)
    ess = np.full(n, float(N))
    hist[0] = (px, py, th)
    parent[0] = ident
    for i in range(1, n):
        if i - 1 in starts:                               # leaving the BS: re-open heading
            px[:], py[:] = home
            if i - 1 != legs[0][0]:
                th = th + np.radians(rng.uniform(-P['leg_spread'], P['leg_spread'], N))
        dx, dy = x[i] - x[i - 1], y[i] - y[i - 1]
        d = step[i]
        if d > 1e-6:
            dyaw = abs(wrap(yaw_r[i] - yaw_r[i - 1]))
            sig = math.radians(P['head_per_m']) * math.sqrt(d) + P['head_per_turn'] * dyaw
            scale = 1 + rng.normal(0, P['dist_noise'], N)
            if typical[i] > 0 and d > P['spike_factor'] * typical[i]:
                sig += math.radians(P['spike_head'])
                scale = rng.uniform(0.15, 1.0, N)          # wheels spun: moved less than logged
            th = th + rng.normal(0, sig, N)
            c, sn = np.cos(th), np.sin(th)
            px = px + scale * (c * dx - sn * dy)
            py = py + scale * (sn * dx + c * dy)
        if in_leg[i]:
            if dmap is not None:
                dist = np.nan_to_num(site.sample(dmap, px, py), nan=50.0)
                w = w * (P['floor'] + np.exp(-(dist / P['sigma']) ** 2))
            if guides and i in guides:
                dg = dist_to_polyline(px, py, guides[i])
                w = w * (0.01 + np.exp(-(dg / P['follow_sigma']) ** 2))
        else:                                             # parked at / shuffling near the BS
            lw = -((px - home[0]) ** 2 + (py - home[1]) ** 2) / (2 * P['home_sigma'] ** 2)
            w = w * np.exp(lw - lw.max())                 # relative, so far-off clouds still get pulled
        tot = w.sum()
        w = w / tot if tot > 0 and np.isfinite(tot) else np.full(N, 1.0 / N)
        ess[i] = 1.0 / np.sum(w ** 2)
        idx = ident
        if ess[i] < N / 2:                                # systematic resampling
            cs = np.cumsum(w)
            cs[-1] = 1.0
            idx = np.searchsorted(cs, (rng.random() + np.arange(N)) / N).astype(np.int32)
            px, py, th = px[idx], py[idx], th[idx]
            w = np.full(N, 1.0 / N)
        parent[i] = idx
        hist[i] = (px, py, th)
    # follow the best final particle back through its ancestors
    k = int(np.argmax(w))
    fx, fy, rot = np.zeros(n), np.zeros(n), np.zeros(n)
    for i in range(n - 1, -1, -1):
        fx[i], fy[i], rot[i] = hist[i, 0, k], hist[i, 1, k], hist[i, 2, k]
        k = parent[i, k]
    return fx, fy, rot, ess


def place_leg(x, y, a, b, fit, theta0, home):
    """Map track of a leg from home with start heading theta0 and the fitted correction."""
    dx, dy = np.diff(x[a:b + 1]) * fit['scale'], np.diff(y[a:b + 1]) * fit['scale']
    th = theta0 + fit['rot']
    px = home[0] + np.cumsum(np.cos(th) * dx - np.sin(th) * dy)
    py = home[1] + np.cumsum(np.sin(th) * dx + np.cos(th) * dy)
    if fit['leftover'] > 0:                               # bend model: close the remainder
        s = fit['s'] / fit['s'][-1]
        px, py = px - (px[-1] - home[0]) * s, py - (py[-1] - home[1]) * s
    return px, py


def place_bs(site, x, y):
    """Move the site's BS to (x, y) for the predictions; generate missing maps (rm_generate.py,
    lunar-mesh-env venv) and wait for them."""
    import subprocess
    import rm_generate
    missing = site.set_bs(x, y)[2]
    if missing:
        py = rm_generate.env_python(rm_generate.DEFAULT_ENV)
        if not py:
            print(f"no maps for the BS at ({x:+.1f}, {y:+.1f}) and no lunar-mesh-env venv; predictions blank")
        else:
            print(f"generating radio maps for the BS at ({site.tx_xy[0]:+.1f}, {site.tx_xy[1]:+.1f}) (~30 s)...")
            subprocess.run([py, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rm_generate.py'),
                            '--site', site.name, '--terrain-dir', site.dir, '--bs', f"{x},{y}"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            missing = site.reload_bs_maps()
    print(f"BS for the predictions at ({site.tx_xy[0]:+.1f}, {site.tx_xy[1]:+.1f})"
          + (f"; still missing {', '.join(missing)}" if missing else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', help='session folder or its coverage.csv')
    ap.add_argument('--site', required=True)
    ap.add_argument('--terrain-dir')
    ap.add_argument('--node')
    ap.add_argument('--model', choices=['turn', 'bend', 'pf'], default='turn')
    ap.add_argument('--bs', help='x,y: BS position for the predictions (overrides bs_position.json); '
                                 'maps for a new spot are generated with rm_generate.py')
    ap.add_argument('--bs-at-start', action='store_true',
                    help='put the BS where the rover started (home) for the predictions')
    ap.add_argument('--home', help='map x,y where the rover parks at the BS '
                                   '(default: first sample under the logged alignment)')
    ap.add_argument('--heading', type=float,
                    help='map heading of the rover at the first sample [deg, 0 = east, 90 = north] '
                         '(default: from the logged alignment)')
    ap.add_argument('--rotate', type=float, default=0.0,
                    help='turn the whole fixed track about home [deg, + = CCW, - = clockwise]')
    ap.add_argument('--leg-heading', action='append', default=[],
                    help='LEG:DEG  map heading the rover set off in on that leg (0 = east, 90 = north)')
    ap.add_argument('--search', type=float, default=0.0,
                    help='let each start heading move up to +-DEG to stay on the drivable mask')
    ap.add_argument('--min-leg', type=float, default=20.0, help='shortest drive counted as a leg [m]')
    ap.add_argument('--k-max', type=float, default=2.0, help='bend model: largest bend tried [deg/m]')
    ap.add_argument('--follow', action='append', default=[],
                    help='pf: L:M  leg L drove the same route as leg M (it may go further)')
    ap.add_argument('--compare', help='pf: coverage CSV to compare against '
                                      '(default: coverage_fixed.csv next to the input, if present)')
    ap.add_argument('--no-plot', action='store_true')
    args = ap.parse_args()

    path = args.session if args.session.endswith('.csv') else os.path.join(args.session, 'coverage.csv')
    out_dir = os.path.dirname(os.path.abspath(path))
    tdir = site_map.find_terrain_dir(args.terrain_dir)
    if tdir is None:
        sys.exit('terrain directory not found (use --terrain-dir)')
    site = site_map.Site(tdir, args.site)
    bs_file = os.path.join(out_dir, 'bs_position.json')
    if os.path.exists(bs_file):                       # BS moved for this session (dashboard "Move BS")
        with open(bs_file) as f:
            pos = json.load(f)
        missing = site.set_bs(float(pos['x']), float(pos['y']))[2]
        if site.bs_moved:
            print(f"BS at ({site.tx_xy[0]:+.1f}, {site.tx_xy[1]:+.1f}) for this session (bs_position.json)"
                  + (f"; no generated maps for {', '.join(missing)} (run rm_generate.py)" if missing else ""))
    node, rows = load(path, args.node)

    x, y, yaw = num(rows, 'lat'), num(rows, 'lon'), num(rows, 'yaw_deg')
    raw_mx, raw_my, raw_mh = num(rows, 'map_x'), num(rows, 'map_y'), num(rows, 'map_heading_deg')
    home = tuple(float(v) for v in args.home.split(',')) if args.home else (raw_mx[0], raw_my[0])
    if args.bs or args.bs_at_start:
        bx, by = home if args.bs_at_start else (float(v) for v in args.bs.split(','))
        place_bs(site, bx, by)
    # odometry -> map rotation at the start (what the logged alignment used)
    if args.heading is not None:
        theta = math.radians(args.heading - (yaw[0] if not np.isnan(yaw[0]) else 0.0))
    elif not np.isnan(raw_mh[0]) and not np.isnan(yaw[0]):
        theta = math.radians(raw_mh[0] - yaw[0])
    else:
        theta = 0.0
    theta += math.radians(args.rotate)
    set_heading = {int(k): math.radians(float(v)) for k, v in (s.split(':') for s in args.leg_heading)}
    legs = find_legs(x, y, args.min_leg)
    if not legs:
        sys.exit('no legs found (try a smaller --min-leg)')

    n = len(rows)
    fx, fy = np.full(n, np.nan), np.full(n, np.nan)
    rot = np.full(n, np.nan)                  # odometry -> map rotation at each sample [rad]
    leg_id = np.zeros(n, dtype=int)           # 0 = at home
    info = []
    print(f"node {node}, {n} samples, {args.model} model, home ({home[0]:.1f}, {home[1]:.1f}), "
          f"start rotation {math.degrees(theta):+.1f} deg | {site.describe()}")
    suffix = '_pf' if args.model == 'pf' else ''
    if args.model == 'pf':
        fx, fy, rot, ess = run_pf(site, x, y, yaw, legs, home, theta)
        if args.follow:
            pairs = [tuple(int(v) for v in f.split(':')) for f in args.follow]
            guides = follow_guides(x, y, yaw, legs, fx, fy, pairs)
            fx, fy, rot, ess = run_pf(site, x, y, yaw, legs, home, theta, guides=guides)
        for li, (a, b) in enumerate(legs, 1):
            leg_id[a + 1:b] = li
            seg = slice(a + 1, b)
            print(f"leg {li}: {rows[a]['timestamp'][11:19]}-{rows[b]['timestamp'][11:19]}, "
                  f"{np.hypot(np.diff(x[a:b + 1]), np.diff(y[a:b + 1])).sum():.0f} m driven, "
                  f"odometry ended {math.hypot(x[b] - x[a], y[b] - y[a]):.1f} m from where it started | "
                  f"pf ends {math.hypot(fx[b] - home[0], fy[b] - home[1]):.1f} m from home, "
                  f"set off heading {math.degrees(wrap(rot[a + 1] + setoff_dir(x, y, a, b))):+.0f} deg, "
                  f"rotation change over leg {math.degrees(wrap(rot[b] - rot[a + 1])):+.0f} deg, "
                  f"min ESS {ess[seg].min():.0f} | on drivable (1 m) "
                  f"{100 * site.on_drivable(fx[seg], fy[seg], 1.0).mean():.0f}%")
    for li, (a, b) in enumerate(legs if args.model != 'pf' else [], 1):
        fit = fit_turn(x, y, yaw, a, b) if args.model == 'turn' else fit_bend(x, y, a, b, args.k_max)
        if li in set_heading:
            theta = set_heading[li] - setoff_dir(x, y, a, b)
        cont = theta
        if args.search > 0:
            best = None
            for dth in np.radians(np.arange(-args.search, args.search + 0.5, 1.0)):
                px, py = place_leg(x, y, a, b, fit, cont + dth, home)
                score = site.on_drivable(px, py, 1.0).mean() - 0.002 * abs(math.degrees(dth))
                if best is None or score > best[0]:
                    best = (score, dth)
            theta = cont + best[1]
        px, py = place_leg(x, y, a, b, fit, theta, home)
        fx[a], fy[a], rot[a] = home[0], home[1], theta
        fx[a + 1:b + 1], fy[a + 1:b + 1] = px, py
        rot[a + 1:b + 1] = theta + fit['rot']
        leg_id[a + 1:b] = li
        drv = site.on_drivable(px, py, 1.0).mean()
        raw_close = math.hypot(x[b] - x[a], y[b] - y[a])
        line = (f"leg {li}: {rows[a]['timestamp'][11:19]}-{rows[b]['timestamp'][11:19]}, "
                f"{np.hypot(np.diff(x[a:b + 1]), np.diff(y[a:b + 1])).sum():.0f} m driven, "
                f"odometry ended {raw_close:.1f} m from where it started | ")
        if args.model == 'turn':
            line += (f"turnaround {fit['out_m']:.0f} m out at {rows[fit['turn_idx']]['timestamp'][11:19]}: "
                     f"way back turned {math.degrees(fit['delta']):+.0f} deg"
                     + (f", bent {math.degrees(fit['k_back']):+.2f} deg/m" if fit['k_back'] else "")
                     + f", stretched x{fit['scale_back']:.2f}")
        else:
            line += (f"bend {math.degrees(fit['k']):+.3f} deg/m ({math.degrees(fit['end_rot']):+.0f} deg), "
                     f"leftover {fit['leftover']:.1f} m spread")
        line += (f" | set off heading {math.degrees(wrap(theta + setoff_dir(x, y, a, b))):+.0f} deg"
                 f"{' (given)' if li in set_heading else ''}"
                 f"{f' ({math.degrees(theta - cont):+.0f} from search)' if theta != cont else ''}")
        line += f" | on drivable (1 m) {100 * drv:.0f}%"
        print(line)
        info.append((a, b, fit))
        theta = theta + fit['end_rot']

    # parked / shuffling at the BS: home plus the small odometry moves, with the rotation
    # carried on from the leg before (or the first leg's start before any driving)
    i = 0
    while i < n:
        if not np.isnan(fx[i]):
            i += 1
            continue
        j = i
        while j < n and np.isnan(fx[j]):
            j += 1
        ref = i - 1 if i > 0 else j                       # neighbouring sample that is home
        th = rot[ref]
        c, s = math.cos(th), math.sin(th)
        ox, oy = x[i:j] - x[ref], y[i:j] - y[ref]
        fx[i:j], fy[i:j] = home[0] + c * ox - s * oy, home[1] + s * ox + c * oy
        rot[i:j] = th
        i = j

    heading = (yaw + np.degrees(rot) + 180.0) % 360.0 - 180.0
    drv = site.on_drivable(fx, fy)
    elev = site.sample(site.terrain, fx, fy)
    preds = {m: site.sample(arr, fx, fy) for m, arr in site.radio_maps.items()}

    out_csv = os.path.join(out_dir, f'coverage_fixed{suffix}.csv')
    keep = [k for k in rows[0].keys() if k not in ('map_x', 'map_y', 'map_heading_deg', 'on_drivable', 'elev_m')
            and not k.startswith('pred_')]
    header = keep + ['map_x', 'map_y', 'map_heading_deg', 'on_drivable', 'elev_m'] + \
        [f"pred_{m}_dB" for m in preds] + ['raw_map_x', 'raw_map_y', 'leg']
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        for i, r in enumerate(rows):
            w.writerow([r[k] for k in keep] +
                       [f"{fx[i]:.2f}", f"{fy[i]:.2f}",
                        f"{heading[i]:.1f}" if not np.isnan(heading[i]) else '',
                        int(drv[i]), f"{elev[i]:.2f}"] +
                       [f"{preds[m][i]:.1f}" for m in preds] +
                       [r['map_x'], r['map_y'], leg_id[i]])
    rs = num(rows, 'rssi_to_bs_dBm')
    print(f"on drivable: raw {100 * site.on_drivable(raw_mx, raw_my).mean():.0f}% -> fixed {100 * drv.mean():.0f}%")
    for m, p in preds.items():
        ok = ~np.isnan(p) & ~np.isnan(rs)
        if ok.sum() > 5:
            print(f"RSSI vs pred_{m}: corr {np.corrcoef(rs[ok], p[ok])[0, 1]:+.2f}, "
                  f"mean error {np.mean(rs[ok] - p[ok]):+.1f} dB, rms {np.sqrt(np.mean((rs[ok] - p[ok]) ** 2)):.1f} dB")
    print(f"wrote {out_csv}")
    ref = None
    if args.model == 'pf':
        cmp_path = args.compare or os.path.join(out_dir, 'coverage_fixed.csv')
        if os.path.exists(cmp_path):
            with open(cmp_path, newline='') as f:
                crow = [r for r in csv.DictReader(f) if r['node_id'] == node]
            if len(crow) == n:
                ref = (num(crow, 'map_x'), num(crow, 'map_y'), os.path.basename(cmp_path))
                for li, (a, b) in enumerate(legs, 1):
                    dd = np.hypot(fx[a:b + 1] - ref[0][a:b + 1], fy[a:b + 1] - ref[1][a:b + 1])
                    print(f"leg {li} vs {ref[2]}: mean {dd.mean():.1f} m, max {dd.max():.1f} m apart")
    if not args.no_plot:
        plot(site, out_dir, rows, raw_mx, raw_my, fx, fy, rs, preds, leg_id, home, node, args.model, suffix)
        plot_rm(site, out_dir, rows, raw_mx, raw_my, fx, fy, rs, leg_id, node, args.model, suffix)
        if args.model != 'pf':
            plot_candidates(site, out_dir, x, y, info, home)
        elif ref is not None:
            plot_compare(site, out_dir, fx, fy, ref, leg_id, home)


def save_fig(fig, out, dpi):
    """Save; if the file is locked (open in an image viewer on Windows) use a new name."""
    try:
        fig.savefig(out, dpi=dpi)
    except OSError:
        out = out[:-4] + datetime.now().strftime('_%H%M%S') + '.png'
        fig.savefig(out, dpi=dpi)
    print(f"wrote {out}")


def plot(site, out_dir, rows, raw_mx, raw_my, fx, fy, rs, preds, leg_id, home, node, model, suffix=''):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    t = np.array([datetime.strptime(r['timestamp'], '%Y-%m-%d %H:%M:%S.%f').timestamp() for r in rows])
    t -= t[0]
    h = site.half
    ext = [-h, h, -h, h]
    allx, ally = np.r_[raw_mx, fx], np.r_[raw_my, fy]
    span = max(np.nanmax(allx) - np.nanmin(allx), np.nanmax(ally) - np.nanmin(ally)) / 2 + 15
    cx, cy = (np.nanmin(allx) + np.nanmax(allx)) / 2, (np.nanmin(ally) + np.nanmax(ally)) / 2

    fig, ax = plt.subplots(2, 2, figsize=(16, 14))
    for k, (a, mx, my, title) in enumerate([(ax[0, 0], raw_mx, raw_my, 'raw odometry (as logged)'),
                                            (ax[0, 1], fx, fy, f'fixed ({model} model, closed at home)')]):
        if site.terrain is not None:
            a.imshow(site.terrain, extent=ext, cmap='gray')
        if site.drivable is not None:
            a.imshow(np.ma.masked_where(~site.drivable, site.drivable), extent=ext,
                     cmap='Blues', alpha=0.35, vmin=0, vmax=1.5)
        if k == 1:
            for li in range(1, leg_id.max() + 1):
                idx = np.where(leg_id == li)[0]
                a.plot(mx[idx], my[idx], '-', color=f'C{li}', lw=2.5, alpha=0.6, label=f'leg {li}')
        sc = a.scatter(mx, my, c=rs, s=8, cmap='viridis', vmin=-90, vmax=-40, zorder=3)
        if site.tx_xy:
            a.plot(*site.tx_xy, 'r*', ms=16, zorder=4, label='BS (tx)')
        a.plot(*home, 'o', mfc='none', mec='r', ms=12, mew=2, zorder=4, label='home')
        a.set_xlim(cx - span, cx + span)
        a.set_ylim(cy - span, cy + span)
        a.set_title(title)
        a.legend(loc='lower left', fontsize=8)
        fig.colorbar(sc, ax=a, label='RSSI to BS [dBm]', shrink=0.8)

    a = ax[1, 0]
    a.plot(t, rs, '.', ms=4, color='k', label='measured RSSI')
    for m, p in preds.items():
        a.plot(t, p, '.', ms=2, label=f'pred {m} (fixed track)')
    for li in range(1, leg_id.max() + 1):
        idx = np.where(leg_id == li)[0]
        a.axvspan(t[idx[0]], t[idx[-1]], color=f'C{li}', alpha=0.08)
        a.text(t[idx[0]], a.get_ylim()[1], f' leg {li}', va='top')
    a.set_xlabel('time [s]')
    a.set_ylabel('dB')
    a.legend(fontsize=8, loc='lower right')
    a.set_title('RSSI vs predicted along the fixed track')

    a = ax[1, 1]
    ref = site.tx_xy or home
    a.scatter(np.hypot(raw_mx - ref[0], raw_my - ref[1]), rs, s=5, color='0.75', label='raw track')
    for li in range(0, leg_id.max() + 1):
        idx = leg_id == li
        a.scatter(np.hypot(fx[idx] - ref[0], fy[idx] - ref[1]), rs[idx], s=7,
                  color='0.3' if li == 0 else f'C{li}', label='fixed, at home' if li == 0 else f'fixed, leg {li}')
    a.set_xlabel('distance to BS [m]')
    a.set_ylabel('RSSI [dBm]')
    a.legend(fontsize=8)
    a.set_title('RSSI vs distance to the BS')
    fig.suptitle(f'{os.path.basename(out_dir)}  node {node}')
    fig.tight_layout()
    out = os.path.join(out_dir, f'fix_track{suffix}.png')
    save_fig(fig, out, 80)


def plot_rm(site, out_dir, rows, raw_mx, raw_my, fx, fy, rs, leg_id, node, model, suffix=''):
    """The predicted radio maps against the drive: each map with the fixed track on it (same
    colour scale, so a point that blends in agrees), measured - predicted along the track,
    a profile over distance driven (raw vs fixed track), and measured vs predicted."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import TwoSlopeNorm

    maps = {m: a for m, a in site.radio_maps.items() if a is not None}
    if not maps:
        return
    h = site.half
    ext = [-h, h, -h, h]
    drive = leg_id > 0
    span = max(np.nanmax(fx) - np.nanmin(fx), np.nanmax(fy) - np.nanmin(fy)) / 2 + 25
    cx, cy = (np.nanmin(fx) + np.nanmax(fx)) / 2, (np.nanmin(fy) + np.nanmax(fy)) / 2
    vmin, vmax = -95, -25
    names = list(maps)
    main = 'nn_pmnet' if 'nn_pmnet' in maps else names[0]
    pred = {m: site.sample(a, fx, fy) for m, a in maps.items()}
    pred_raw = {m: site.sample(a, raw_mx, raw_my) for m, a in maps.items()}
    ok = ~np.isnan(rs) & ~np.isnan(pred[main])
    bias = float(np.mean(rs[ok] - pred[main][ok])) if ok.any() else 0.0

    ncol = max(2, len(names))
    fig = plt.figure(figsize=(6.2 * ncol, 17))
    gs = fig.add_gridspec(3, ncol, height_ratios=[1.15, 1.15, 0.9])

    def base(ax):
        ax.set_xlim(cx - span, cx + span)
        ax.set_ylim(cy - span, cy + span)
        if site.tx_xy:
            ax.plot(*site.tx_xy, '*', color='r', mec='k', ms=18, zorder=5)

    # row 1: each predicted map with the track coloured by measured RSSI (same scale)
    for k, m in enumerate(names):
        ax = fig.add_subplot(gs[0, k])
        im = ax.imshow(maps[m], extent=ext, cmap='viridis', vmin=vmin, vmax=vmax)
        ax.plot(fx, fy, '-', color='w', lw=0.8, alpha=0.8)
        ax.scatter(fx, fy, c=rs, cmap='viridis', vmin=vmin, vmax=vmax, s=22, edgecolors='k', linewidths=0.4, zorder=4)
        base(ax)
        ax.set_title(f"predicted {m} (background) vs measured RSSI (dots)")
        fig.colorbar(im, ax=ax, shrink=0.8, label='dBm')

    # row 2 left: residual along the track on the main map
    ax = fig.add_subplot(gs[1, 0])
    if site.terrain is not None:
        ax.imshow(site.terrain, extent=ext, cmap='gray')
    if site.drivable is not None:
        ax.imshow(np.ma.masked_where(~site.drivable, site.drivable), extent=ext, cmap='Blues', alpha=0.3, vmin=0, vmax=1.5)
    res = rs - pred[main] - bias
    lim = max(5.0, float(np.nanpercentile(np.abs(res), 95)))
    sc = ax.scatter(fx, fy, c=res, cmap='RdBu', norm=TwoSlopeNorm(0, -lim, lim), s=24, edgecolors='k',
                    linewidths=0.3, zorder=4)
    base(ax)
    ax.set_title(f"measured - {main} - ({bias:+.1f} dB mean offset)\nred = weaker than predicted, blue = stronger")
    fig.colorbar(sc, ax=ax, shrink=0.8, label='dB')

    # row 2 right: measured vs predicted scatter
    ax = fig.add_subplot(gs[1, 1])
    lo, hi = -100, -20
    for k, m in enumerate(names):
        okm = ~np.isnan(rs) & ~np.isnan(pred[m]) & drive
        if okm.sum() < 3:
            continue
        r = np.corrcoef(rs[okm], pred[m][okm])[0, 1]
        b = float(np.mean(rs[okm] - pred[m][okm]))
        rms = float(np.sqrt(np.mean((rs[okm] - pred[m][okm] - b) ** 2)))
        ax.scatter(pred[m][okm], rs[okm], s=10, color=f'C{k}', alpha=0.7,
                   label=f"{m}: r {r:+.2f}, offset {b:+.1f} dB, rms after offset {rms:.1f} dB")
    ax.plot([lo, hi], [lo, hi], 'k--', lw=1, label='1:1')
    ax.plot([lo, hi], [lo + bias, hi + bias], 'k:', lw=1, label=f'1:1 {bias:+.1f} dB')
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.set_xlabel('predicted [dB]')
    ax.set_ylabel('measured RSSI [dBm]')
    ax.legend(fontsize=8, loc='upper left')
    ax.set_title('measured vs predicted (while driving)')
    if ncol > 2:
        for k in range(2, ncol):
            fig.add_subplot(gs[1, k]).axis('off')

    # row 3: profile over distance driven
    ax = fig.add_subplot(gs[2, :])
    dist = np.r_[0.0, np.cumsum(np.nan_to_num(np.hypot(np.diff(fx), np.diff(fy))))]
    ax.plot(dist, rs, 'k.', ms=5, label='measured RSSI')
    for k, m in enumerate(names):
        ax.plot(dist, pred[m], '-', color=f'C{k}', lw=1.5, label=f'pred {m}, fixed track')
        ax.plot(dist, pred_raw[m], ':', color=f'C{k}', lw=1.2, label=f'pred {m}, raw track')
    ax.plot(dist, pred[main] + bias, '-', color='0.5', lw=1, label=f'pred {main} {bias:+.1f} dB')
    for li in range(1, leg_id.max() + 1):
        idx = np.where(leg_id == li)[0]
        ax.axvspan(dist[idx[0]], dist[idx[-1]], color=f'C{li}', alpha=0.06)
    ax.set_xlabel('distance driven [m] (fixed track)')
    ax.set_ylabel('dB')
    ax.legend(fontsize=8, ncol=3, loc='lower right')
    ax.grid(alpha=0.3)
    ax.set_title('along the drive: measured vs predicted on the fixed track (solid) and on the raw track (dotted)')
    fig.suptitle(f"{os.path.basename(out_dir)}  node {node}  -  predicted radio maps on the {model}-fixed track"
                 + (f"  (BS moved to {site.tx_xy[0]:+.1f}, {site.tx_xy[1]:+.1f})" if site.bs_moved else ""))
    fig.tight_layout()
    save_fig(fig, os.path.join(out_dir, f'fix_track_rm{suffix}.png'), 70)


def plot_candidates(site, out_dir, x, y, info, home):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    h = site.half
    fig, ax = plt.subplots(len(info), 4, figsize=(16, 4.2 * len(info)), squeeze=False)
    for li, (a, b, fit) in enumerate(info, 1):
        base, cands = candidates(site, x, y, a, b, fit, home)
        for k, (hd, sc) in enumerate(cands):
            A = ax[li - 1, k]
            px, py = place_leg(x, y, a, b, fit, math.radians(hd) - base, home)
            if site.terrain is not None:
                A.imshow(site.terrain, extent=[-h, h, -h, h], cmap='gray')
            if site.drivable is not None:
                A.imshow(np.ma.masked_where(~site.drivable, site.drivable), extent=[-h, h, -h, h],
                         cmap='Blues', alpha=0.35, vmin=0, vmax=1.5)
            A.plot(np.r_[home[0], px], np.r_[home[1], py], '-', color=f'C{li}', lw=2)
            if site.tx_xy:
                A.plot(*site.tx_xy, 'r*', ms=12)
            A.set_xlim(home[0] - 75, home[0] + 75)
            A.set_ylim(home[1] - 75, home[1] + 75)
            A.set_title(f'leg {li}: --leg-heading {li}:{hd:.0f}  ({100 * sc:.0f}% on paths)', fontsize=10)
    fig.tight_layout()
    out = os.path.join(out_dir, 'fix_track_candidates.png')
    save_fig(fig, out, 70)


def plot_compare(site, out_dir, fx, fy, ref, leg_id, home):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    h = site.half
    nl = leg_id.max()
    fig, ax = plt.subplots(1, nl, figsize=(6 * nl, 6.4), squeeze=False)
    for li in range(1, nl + 1):
        A = ax[0, li - 1]
        idx = np.where(leg_id == li)[0]
        idx = np.r_[idx[0] - 1, idx, idx[-1] + 1]
        if site.terrain is not None:
            A.imshow(site.terrain, extent=[-h, h, -h, h], cmap='gray')
        if site.drivable is not None:
            A.imshow(np.ma.masked_where(~site.drivable, site.drivable), extent=[-h, h, -h, h],
                     cmap='Blues', alpha=0.35, vmin=0, vmax=1.5)
        A.plot(ref[0][idx], ref[1][idx], '-', color='orange', lw=2, label=ref[2])
        A.plot(fx[idx], fy[idx], '-', color='magenta', lw=2, label='pf')
        if site.tx_xy:
            A.plot(*site.tx_xy, 'r*', ms=12)
        A.set_xlim(home[0] - 70, home[0] + 70)
        A.set_ylim(home[1] - 85, home[1] + 55)
        A.set_title(f'leg {li}')
        A.legend(loc='lower left', fontsize=8)
    fig.tight_layout()
    out = os.path.join(out_dir, 'fix_track_pf_compare.png')
    save_fig(fig, out, 75)


if __name__ == '__main__':
    main()
