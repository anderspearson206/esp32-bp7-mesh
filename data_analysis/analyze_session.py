"""
analyze_session.py  --  Offline analysis of one BS dashboard session (bs_reports/session_*).

For each session it writes into data_analysis/<name>/:
    map_rm.png         predicted radio maps (nn_pmnet, nn) with the driven path and planned routes
    map_comms.png      measured RSSI along the path, and where each data bundle was generated
                       coloured by its delivery latency
    timeseries.png     RSSI (measured vs predicted), distance to BS, packet rates, latency
    link_quality.png   first-try delivery and beacon reception against RSSI
    pred_vs_true.png   predicted vs measured RSSI (scatter per model + profile along the path)
    summary.md         the numbers behind the figures

Usage (from the repo root, with the ..\\.venv Python):
    python data_analysis/analyze_session.py bs_reports/session_20261015_094354_shadow1 --name shadow1
    python data_analysis/analyze_session.py bs_reports/session_*_shadow2
    python data_analysis/analyze_session.py bs_reports/session_20261015_091320 --name preshadow \
        --window 09:14:20-09:35:41          (only part of a session)

Data sources and their timing:
  * Measured RSSI = the BS's own RSSI of the rover's beacons (@NET lines). These are fresh
    (one per received beacon), unlike the rssi field of @LOCATION, which is the rover's last
    BS-beacon RSSI and goes stale while the link is down.
  * Rover position = coverage.csv map_x/map_y. Each row is one @LOCATION bundle; its true
    time is its BS arrival minus its (clock-corrected) latency from deliveries.csv, so
    positions carried across a disconnection land at the right time.
  * Predictions are path gain [dB] from the site radio maps for the BS cell in use. The BS
    cell is found by matching the pred_nn_pmnet_dB column against the generated maps
    (bs_position.json is overwritten when the BS is moved after a session).
    Measured RSSI = path gain + an unknown constant (TX power + antenna gains), so the
    comparison fits that constant (median of measured - predicted) and reports the
    error after it, alongside the raw bias.
"""

import argparse
import csv
import glob
import json
import os
import re
import sys
import warnings
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from matplotlib.lines import Line2D

HERE = os.path.dirname(os.path.abspath(__file__))
TERRAIN_DIR = os.path.join(HERE, 'terrain')     # archived copy of the site + radio maps the runs used
sys.path.insert(0, os.path.dirname(HERE))
import site_map as sm  # noqa: E402

MODELS = ('nn_pmnet', 'nn', 'avg')      # avg = mean of the nn_pmnet and nn maps in dB
# reference categorical palette (dataviz skill), fixed order
C_MEAS, C_PMNET, C_NN, C_AVG = '#2a78d6', '#eb6834', '#1baf7a', '#6250d6'
C_TEL, C_GEN = '#e87ba4', '#52514e'
C_DOWN = '#e34948'
MODEL_COLOR = {'nn_pmnet': C_PMNET, 'nn': C_NN, 'avg': C_AVG}
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'
RSSI_CMAP = 'viridis'
RATE_BIN_S = 5.0
warnings.filterwarnings('ignore', 'invalid value encountered in cast')  # NaN positions -> off-grid

plt.rcParams.update({
    'font.size': 9, 'axes.edgecolor': INK2, 'axes.labelcolor': INK, 'xtick.color': INK2,
    'ytick.color': INK2, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
    'axes.spines.top': False, 'axes.spines.right': False, 'figure.dpi': 110,
    'savefig.dpi': 150, 'savefig.bbox': 'tight', 'legend.frameon': False,
})


# ----------------------------------------------------------------------------- loading

def ts(s: str) -> float:
    return datetime.strptime(s.strip(), '%Y-%m-%d %H:%M:%S.%f').timestamp()


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


class Session:
    def __init__(self, path: str, node: str = None):
        self.path = path
        self.name = os.path.basename(path.rstrip('/\\'))
        self.cov = read_csv(os.path.join(path, 'coverage.csv'))
        self.deliv = read_csv(os.path.join(path, 'deliveries.csv'))
        self.routes = read_csv(os.path.join(path, 'routes.csv'))
        nodes = sorted({r['node_id'] for r in self.cov})
        self.node = node or (nodes[0] if nodes else None)
        self.cov = [r for r in self.cov if r['node_id'] == self.node]
        self.deliv = [r for r in self.deliv if r['src'] == self.node]
        self.routes = [r for r in self.routes if r['node'] == self.node]
        self._parse_serial()

    def _parse_serial(self):
        self.net = []          # (t, rssi) BS-measured, rover beacons
        self.contacts = []     # (t_up, t_down or None)
        self.bs_moves = []     # t of #BS lines
        self.status = []       # (t, dict)
        self.markers = []
        up = None
        with open(os.path.join(self.path, 'serial.log'), encoding='utf-8', errors='replace') as f:
            for line in f:
                if '\t' not in line:
                    continue
                tstr, msg = line.rstrip('\n').split('\t', 1)
                try:
                    t = ts(tstr)
                except ValueError:
                    continue
                if msg.startswith('@NET:'):
                    p = msg.split(':')
                    if len(p) >= 4 and p[1] == self.node:
                        self.net.append((t, fnum(p[3])))
                elif msg.startswith('#CONTACT_UP:') and msg.split(':')[1] == self.node:
                    up = t
                elif msg.startswith('#CONTACT_DOWN:') and msg.split(':')[1] == self.node:
                    if up is not None:
                        self.contacts.append((up, t))
                    up = None
                elif msg.startswith('#BS:'):
                    self.bs_moves.append(t)
                elif msg.startswith('#MARKER:'):
                    self.markers.append((t, msg[8:]))
                elif msg.startswith('@ROVER_STATUS:'):
                    p = msg.split(':', 2)
                    if len(p) == 3 and p[1] == self.node:
                        d = dict(kv.split('=', 1) for kv in p[2].split(',') if '=' in kv)
                        self.status.append((t, d))
                last_t = t
        if up is not None:
            self.contacts.append((up, None))
        self.t_end = last_t
        self.t_start = min([ts(r['timestamp']) for r in self.cov[:1]] + [t for t, _ in self.net[:1]]
                           + [t for t, _ in self.markers[:1]])
        self.contacts = [(a, b if b is not None else self.t_end) for a, b in self.contacts]


# ----------------------------------------------------------------------------- derived data

def position_track(s: Session):
    """(t_true, x, y, t_arrival) per @LOCATION sample, sorted by true time."""
    loc = [r for r in s.deliv if r['kind'] == 'tel' and r['label'] == '@LOCATION']
    arr = np.array([ts(r['timestamp']) for r in s.cov])
    x = np.array([fnum(r['map_x']) for r in s.cov])
    y = np.array([fnum(r['map_y']) for r in s.cov])
    if len(loc) == len(s.cov):
        t_true = np.array([fnum(r['epoch']) - fnum(r['latency_ms']) / 1000.0 for r in loc])
    else:
        # fall back: nearest @LOCATION delivery in arrival time
        le = np.array([fnum(r['epoch']) for r in loc])
        ll = np.array([fnum(r['latency_ms']) for r in loc])
        idx = np.clip(np.searchsorted(le, arr), 0, len(le) - 1)
        t_true = arr - np.where(np.abs(le[idx] - arr) < 0.5, ll[idx], 0) / 1000.0
    o = np.argsort(t_true)
    return t_true[o], x[o], y[o], arr[o]


def interp_pos(track, t):
    tt, x, y, _ = track
    ok = np.isfinite(x) & np.isfinite(y)
    tt, x, y = tt[ok], x[ok], y[ok]
    t = np.asarray(t, dtype=float)
    px, py = np.interp(t, tt, x), np.interp(t, tt, y)
    out = (t < tt[0] - 2) | (t > tt[-1] + 2)
    px[out] = np.nan
    py[out] = np.nan
    return px, py


def find_bs_cell(s: Session, site: sm.Site):
    """BS (col, row) whose generated nn_pmnet map matches the logged predictions best."""
    x = np.array([fnum(r['map_x']) for r in s.cov])
    y = np.array([fnum(r['map_y']) for r in s.cov])
    logged = np.array([fnum(r.get('pred_nn_pmnet_dB')) for r in s.cov])
    cands = [(None, site.radio_maps.get('nn_pmnet'))]
    for p in glob.glob(os.path.join(site.dir, f'{site.name}_rm58_nn_pmnet@*.npy')):
        m = re.search(r'@(\d+)_(\d+)\.npy$', p)
        cands.append(((int(m.group(1)), int(m.group(2))), np.load(p)))
    best, best_frac = None, -1
    for cell, arr in cands:
        if arr is None:
            continue
        d = np.abs(site.sample(arr, x, y) - logged)
        frac = np.nanmean(d < 0.5)
        if frac > best_frac:
            best, best_frac = cell, frac
    if best is None:
        best = (site.base_rc[1], site.base_rc[0])
    c, r = best
    maps = {}
    for m in MODELS:
        p = site.bs_map_path(m, c, r)
        if (r, c) == site.base_rc:
            maps[m] = site.base_maps.get(m)
        elif os.path.exists(p):
            maps[m] = np.load(p).astype(np.float32)
    if maps.get('nn_pmnet') is not None and maps.get('nn') is not None:
        maps['avg'] = (maps['nn_pmnet'] + maps['nn']) / 2.0
    return (c, r), site.rc_to_xy(r, c), maps, best_frac


def contact_state(s: Session, t):
    t = np.asarray(t, dtype=float)
    up = np.zeros(t.shape, dtype=bool)
    for a, b in s.contacts:
        up |= (t >= a) & (t <= b)
    return up


def rate_series(times, t0, t1, bin_s=RATE_BIN_S):
    edges = np.arange(t0, t1 + bin_s, bin_s)
    h, _ = np.histogram(times, bins=edges)
    return edges[:-1] + bin_s / 2, h / bin_s


def fit_offset(meas, pred):
    ok = np.isfinite(meas) & np.isfinite(pred)
    m, p = meas[ok], pred[ok]
    if len(m) < 5:
        return None
    off = float(np.median(m - p))
    res = m - (p + off)
    slope, icpt = np.polyfit(p, m, 1)
    return {
        'n': int(len(m)),
        'offset_db': off,
        'raw_bias_db': float(np.mean(m - p)),
        'rmse_db': float(np.sqrt(np.mean(res ** 2))),
        'mae_db': float(np.mean(np.abs(res))),
        'r': float(np.corrcoef(p, m)[0, 1]),
        'slope': float(slope), 'intercept': float(icpt),
    }


# ----------------------------------------------------------------------------- figures

def map_extent(site, xs, ys, bs_xy, pad=12.0):
    xs = np.concatenate([np.asarray(xs)[np.isfinite(xs)], [bs_xy[0]]])
    ys = np.concatenate([np.asarray(ys)[np.isfinite(ys)], [bs_xy[1]]])
    x0, x1 = xs.min() - pad, xs.max() + pad
    y0, y1 = ys.min() - pad, ys.max() + pad
    # square-ish
    w, h = x1 - x0, y1 - y0
    if w > h:
        y0, y1 = y0 - (w - h) / 2, y1 + (w - h) / 2
    else:
        x0, x1 = x0 - (h - w) / 2, x1 + (h - w) / 2
    return x0, x1, y0, y1


def grid_extent(site):
    return (-site.half, site.half, -site.half, site.half)


def draw_base(ax, site, ext, bs_xy, background=None, cmap=None, vmin=None, vmax=None):
    im = None
    if background is not None:
        im = ax.imshow(background, extent=grid_extent(site), origin='upper', cmap=cmap,
                       vmin=vmin, vmax=vmax, interpolation='nearest')
    elif site.drivable is not None:
        ax.imshow(np.where(site.drivable, 0.86, 0.97), extent=grid_extent(site), origin='upper',
                  cmap='gray', vmin=0, vmax=1, interpolation='nearest')
    if site.drivable is not None:
        xx = np.arange(site.size) + 0.5 - site.half
        yy = site.half - np.arange(site.size) - 0.5
        ax.contour(xx, yy, site.drivable.astype(float), levels=[0.5],
                   colors=['white' if background is not None else '#9a9993'], linewidths=0.6)
    ax.plot(*bs_xy, marker='*', ms=16, color='#eda100', mec=INK, mew=0.8, zorder=10)
    ax.annotate('BS', bs_xy, xytext=(7, 6), textcoords='offset points', fontsize=9,
                fontweight='bold', color=INK, zorder=10,
                bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.8))
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_aspect('equal')
    ax.set_xlabel('east [m]')
    ax.set_ylabel('north [m]')
    ax.grid(False)
    return im


def draw_routes(ax, s, color='#ffffff'):
    for i, r in enumerate(s.routes):
        if r['action'] != 'route':
            continue
        pts = np.array(json.loads(r['map_xy']))
        ax.plot(pts[:, 0], pts[:, 1], ls=(0, (4, 3)), lw=1.4, color=color, zorder=4,
                label='planned route' if i == 0 else None)


def fig_map_rm(s, site, track, bs_xy, maps, out):
    _, x, y, _ = track
    ext = map_extent(site, x, y, bs_xy)
    present = [m for m in MODELS if maps.get(m) is not None]
    fig, axes = plt.subplots(1, len(present), figsize=(6.6 * len(present), 6.0), squeeze=False,
                             layout='constrained')
    # colour range from the part of the map that is shown, not the whole site
    (r0, r1), (c0, c1) = site.xy_to_rc([ext[0], ext[1]], [ext[3], ext[2]])
    r0, c0 = max(r0, 0), max(c0, 0)
    r1 = r1 if r1 >= 0 else site.size - 1
    c1 = c1 if c1 >= 0 else site.size - 1
    vals = np.concatenate([maps[m][r0:r1 + 1, c0:c1 + 1].ravel() for m in present])
    vmin, vmax = np.nanpercentile(vals, 2), np.nanpercentile(vals, 99.5)
    for ax, m in zip(axes[0], present):
        im = draw_base(ax, site, ext, bs_xy, maps[m], 'cividis', vmin, vmax)
        draw_routes(ax, s, '#ffffff')
        ax.plot(x, y, color='#e34948', lw=1.6, zorder=5, label='driven path')
        ax.plot(x[0], y[0], 'o', ms=8, color='white', mec=INK, zorder=6, label='start')
        ax.set_title(f'Predicted path gain: {m}', loc='left', fontsize=11, color=INK)
        cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03)
        cb.set_label('path gain [dB]')
        ax.legend(loc='lower left', fontsize=8, frameon=True, facecolor='white', framealpha=0.85)
    fig.suptitle(f'{s.label} - radio maps with driven path', x=0.01, ha='left', fontsize=13)
    fig.savefig(out)
    plt.close(fig)


def fig_map_comms(s, site, track, bs_xy, net_xy, data, out):
    _, x, y, _ = track
    ext = map_extent(site, x, y, bs_xy)
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 6.0), layout='constrained')

    # (a) measured RSSI along the path
    ax = axes[0]
    draw_base(ax, site, ext, bs_xy)
    ax.plot(x, y, color='#9a9993', lw=0.8, zorder=3)
    nt, nx, ny, nr = net_xy
    sc = ax.scatter(nx, ny, c=nr, cmap=RSSI_CMAP, vmin=-95, vmax=-35, s=14, lw=0.3,
                    edgecolors='white', zorder=5)
    # where the link was down: path samples (true time) outside any contact
    tt, px, py, _ = track
    down = ~contact_state(s, tt)
    if down.any():
        ax.scatter(px[down], py[down], marker='x', s=22, color=C_DOWN, lw=1.2, zorder=6,
                   label=f'link down ({down.sum()} position samples)')
        ax.legend(loc='lower left', fontsize=8, frameon=True, facecolor='white', framealpha=0.85)
    cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.03)
    cb.set_label('measured RSSI at BS [dBm]')
    ax.set_title('Measured RSSI (BS side, rover beacons)', loc='left', fontsize=11)

    # (b) data bundles at their generation position, coloured by latency
    ax = axes[1]
    draw_base(ax, site, ext, bs_xy)
    ax.plot(x, y, color='#9a9993', lw=0.8, zorder=3)
    gx, gy, lat = data['gx'], data['gy'], data['lat_ms']
    ok = np.isfinite(gx)
    if not ok.any():
        ax.text(0.5, 0.5, 'no data bundles in this run', transform=ax.transAxes, ha='center',
                fontsize=11, color=INK2, bbox=dict(fc='white', ec=GRID))
        ax.set_title('Data bundles at generation point', loc='left', fontsize=11)
        fig.suptitle(f'{s.label} - communication over the path', x=0.01, ha='left', fontsize=13)
        fig.savefig(out)
        plt.close(fig)
        return
    sc = ax.scatter(gx[ok], gy[ok], c=np.clip(lat[ok], 100, None), cmap='plasma_r',
                    norm=LogNorm(vmin=100, vmax=max(2000, np.nanmax(lat))), s=16, lw=0.3,
                    edgecolors='white', zorder=5)
    cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.03)
    cb.set_label('delivery latency [ms] (log)')
    ax.set_title(f'Data bundles at generation point ({ok.sum()} delivered)', loc='left',
                 fontsize=11)
    fig.suptitle(f'{s.label} - communication over the path', x=0.01, ha='left', fontsize=13)
    fig.savefig(out)
    plt.close(fig)


def shade_down(ax, s, t0):
    """Shade intervals with no contact."""
    t = s.t_start
    for a, b in s.contacts:
        if a - t > 0.5:
            ax.axvspan(t - t0, a - t0, color=C_DOWN, alpha=0.12, lw=0)
        t = b
    if s.t_end - t > 0.5:
        ax.axvspan(t - t0, s.t_end - t0, color=C_DOWN, alpha=0.12, lw=0)


def fig_timeseries(s, track, bs_xy, net_xy, pred_net, fits, data, tel_t, out):
    t0 = s.t_start
    nt, nx, ny, nr = net_xy
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True,
                             gridspec_kw={'height_ratios': [3, 1.4, 2, 2]})

    ax = axes[0]
    for m in MODELS:
        if m in pred_net and fits.get(m):
            ax.plot(nt - t0, pred_net[m] + fits[m]['offset_db'], color=MODEL_COLOR[m], lw=1.4,
                    label=f'{m} {fits[m]["offset_db"]:+.1f} dB')
    ax.plot(nt - t0, nr, '.', ms=3.5, color=C_MEAS, label='measured (BS, per beacon)')
    ax.set_ylabel('RSSI [dBm]')
    ax.legend(loc='lower left', bbox_to_anchor=(0, 1.02), fontsize=8, ncol=4)
    ax.set_title('RSSI: measured vs predicted (per-run fitted offset)', pad=22, loc='left', fontsize=11)

    ax = axes[1]
    tt, px, py, _ = track
    ax.plot(tt - t0, np.hypot(px - bs_xy[0], py - bs_xy[1]), color=INK2, lw=1.4)
    ax.set_ylabel('dist to BS [m]')

    ax = axes[2]
    t1 = s.t_end
    c, r_del = rate_series(data['t_arr'], t0, t1)
    _, r_gen = rate_series(data['t_gen'], t0, t1)
    _, r_tel = rate_series(tel_t, t0, t1)
    ax.step(c - t0, r_gen, where='mid', color=C_GEN, lw=1.2, ls='--', label='data generated (at rover)')
    ax.step(c - t0, r_del, where='mid', color=C_MEAS, lw=1.6, label='data delivered at BS')
    ax.step(c - t0, r_tel, where='mid', color=C_TEL, lw=1.2, label='telemetry delivered')
    ax.set_ylabel(f'bundles/s ({RATE_BIN_S:.0f} s bins)')
    ax.legend(loc='upper left', fontsize=8, ncol=3)
    ax.set_title('Packet rates', loc='left', fontsize=11)

    ax = axes[3]
    ax.scatter(data['t_arr'] - t0, np.clip(data['lat_ms'], 50, None), s=6, color=C_MEAS, lw=0)
    if len(data['t_arr']):
        ax.set_yscale('log')
    else:
        ax.text(0.5, 0.5, 'no data bundles in this run', transform=ax.transAxes, ha='center',
                color=INK2)
    ax.set_ylabel('data latency [ms]')
    ax.set_xlabel(f'time since session start [s]   (red bands: link to BS down)')
    ax.set_title('Delivery latency (by arrival time)', loc='left', fontsize=11)

    for ax in axes:
        shade_down(ax, s, t0)
        ax.set_xlim(0, s.t_end - t0)
    fig.suptitle(f'{s.label} - time series', x=0.01, ha='left', fontsize=13)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def fig_pred_vs_true(s, track, net_xy, pred_net, fits, out):
    nt, nx, ny, nr = net_xy
    present = [m for m in MODELS if fits.get(m)]
    fig, axes = plt.subplots(1, len(present) + 1, figsize=(5.2 * (len(present) + 1), 4.8),
                             gridspec_kw={'width_ratios': [1] * len(present) + [1.6]})
    lo, hi = np.nanmin(nr) - 5, np.nanmax(nr) + 5
    for ax, m in zip(axes, present):
        f = fits[m]
        p = pred_net[m] + f['offset_db']
        ax.scatter(p, nr, s=7, color=MODEL_COLOR[m], alpha=0.55, lw=0)
        ax.plot([lo, hi], [lo, hi], color=INK2, lw=1, ls='--', label='y = x')
        ax.set_xlim(lo, hi)
        ax.set_ylim(lo, hi)
        ax.set_aspect('equal')
        ax.set_xlabel(f'predicted {m} {f["offset_db"]:+.1f} dB [dBm]')
        ax.set_ylabel('measured RSSI [dBm]')
        ax.set_title(m, loc='left', fontsize=11)
        ax.text(0.03, 0.97,
                f"n = {f['n']}\nr = {f['r']:.2f}\nRMSE = {f['rmse_db']:.1f} dB\n"
                f"MAE = {f['mae_db']:.1f} dB\nslope = {f['slope']:.2f}",
                transform=ax.transAxes, va='top', fontsize=8.5, color=INK,
                bbox=dict(boxstyle='round,pad=0.3', fc='white', ec=GRID))
        ax.legend(loc='lower right', fontsize=8)

    # profile along the driven path
    ax = axes[-1]
    tt, px, py, _ = track
    ok = np.isfinite(px) & np.isfinite(py)
    seg = np.r_[0, np.cumsum(np.hypot(np.diff(px[ok]), np.diff(py[ok])))]
    dist = np.interp(nt, tt[ok], seg)
    for m in present:
        o = np.argsort(dist)
        ax.plot(dist[o], (pred_net[m] + fits[m]['offset_db'])[o], color=MODEL_COLOR[m], lw=1.4,
                label=m)
    ax.plot(dist, nr, '.', ms=3.5, color=C_MEAS, label='measured')
    ax.set_xlabel('distance driven [m]')
    ax.set_ylabel('RSSI [dBm]')
    ax.set_title('Along the path', loc='left', fontsize=11)
    ax.legend(loc='lower left', fontsize=8)
    fig.suptitle(f'{s.label} - predicted vs measured RSSI (constant offset fitted per model)',
                 x=0.01, ha='left', fontsize=13)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def binned_frac(x, ok, edges):
    """Per bin: centre, fraction of ok, count (bins with < 3 samples dropped)."""
    idx = np.digitize(x, edges) - 1
    c, f, n = [], [], []
    for i in range(len(edges) - 1):
        sel = idx == i
        if sel.sum() >= 3:
            c.append((edges[i] + edges[i + 1]) / 2)
            f.append(ok[sel].mean())
            n.append(int(sel.sum()))
    return np.array(c), np.array(f), np.array(n)


def fig_link_quality(s, net_xy, data, maps, fits, site, out):
    """First-try delivery and beacon reception against RSSI (measured, and each model's prediction)."""
    nt, _, _, nr = net_xy
    first = data['lat_ms'] < 2000          # retransmits come 5 s apart, so >= 2 s = not first try
    # measured RSSI at generation: nearest beacon within 3 s
    i = np.clip(np.searchsorted(nt, data['t_gen']), 1, len(nt) - 1)
    j = np.where(np.abs(nt[i - 1] - data['t_gen']) < np.abs(nt[i] - data['t_gen']), i - 1, i)
    meas_gen = np.where(np.abs(nt[j] - data['t_gen']) < 3, nr[j], np.nan)
    models = [m for m in MODELS if fits.get(m) and maps.get(m) is not None]
    marker = {'nn_pmnet': 's', 'nn': 'D', 'avg': '^'}
    edges = np.arange(-115, -25, 5.0)

    def lbl(m):
        return f'vs predicted {m} {fits[m]["offset_db"]:+.1f} dB'

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6), layout='constrained')
    ax = axes[0]
    ok = np.isfinite(meas_gen)
    c, f, n = binned_frac(meas_gen[ok], first[ok], edges)
    ax.plot(c, f * 100, '-o', color=C_MEAS, lw=2.2, ms=6, label='vs measured RSSI', zorder=5)
    for x_, y_, k in zip(c, f * 100, n):
        ax.annotate(str(k), (x_, y_), xytext=(0, -13), textcoords='offset points', ha='center',
                    fontsize=7, color=C_MEAS)
    for m in models:
        pg = site.sample(maps[m], data['gx'], data['gy']) + fits[m]['offset_db']
        okp = np.isfinite(pg)
        c2, f2, _ = binned_frac(pg[okp], first[okp], edges)
        ax.plot(c2, f2 * 100, '-' + marker.get(m, 's'), color=MODEL_COLOR[m], lw=1.6, ms=6, label=lbl(m))
    ax.set_ylim(-5, 105)
    ax.set_xlabel('RSSI at generation [dBm] (5 dB bins; numbers = bundles per measured bin)')
    ax.set_ylabel('delivered on first try [%]')
    ax.set_title('Data bundles delivered without retransmission (latency < 2 s)', loc='left', fontsize=11)
    ax.legend(loc='lower right', fontsize=8)

    ax = axes[1]
    cen, heard = rate_series(nt, s.t_start, s.t_end)   # beacons heard per second; rover sends 1/s
    hb_all = np.clip(heard, 0, 1)
    bx, by = interp_pos(position_track(s), cen)
    for m in models:
        pb = site.sample(maps[m], bx, by) + fits[m]['offset_db']
        okb = np.isfinite(pb)
        idx = np.digitize(pb[okb], edges) - 1
        hb = hb_all[okb]
        cc, ff = [], []
        for k in range(len(edges) - 1):
            sel = idx == k
            if sel.sum() >= 2:
                cc.append((edges[k] + edges[k + 1]) / 2)
                ff.append(hb[sel].mean())
        ax.plot(cc, np.array(ff) * 100, '-' + marker.get(m, 's'), color=MODEL_COLOR[m], lw=2, ms=6,
                label=lbl(m))
    ax.set_ylim(-5, 105)
    ax.set_xlabel(f'predicted RSSI [dBm] (5 dB bins of {RATE_BIN_S:.0f} s windows)')
    ax.set_ylabel('rover beacons heard at BS [%]')
    ax.set_title('Beacon reception (rover sends 1 per second)', loc='left', fontsize=11)
    ax.legend(loc='lower right', fontsize=8)
    fig.suptitle(f'{s.label} - link quality vs RSSI', x=0.01, ha='left', fontsize=13)
    fig.savefig(out)
    plt.close(fig)


# ----------------------------------------------------------------------------- main

def parse_window(s: Session, window):
    """'HH:MM:SS-HH:MM:SS' (either side may be empty) on the session's date -> (t0, t1) epochs."""
    if not window:
        return None, None
    day = datetime.fromtimestamp(s.t_start).strftime('%Y-%m-%d')
    a, b = window.split('-')
    return (ts(f'{day} {a}.0') if a else None, ts(f'{day} {b}.0') if b else None)


def clip_session(s: Session, t0=None, t1=None):
    """Keep only [t0, t1). Bundles are kept by BS arrival time."""
    lo, hi = (t0 if t0 is not None else -np.inf), (t1 if t1 is not None else np.inf)
    s.cov = [r for r in s.cov if lo <= ts(r['timestamp']) < hi]
    s.deliv = [r for r in s.deliv if lo <= fnum(r['epoch']) < hi]
    s.net = [(t, v) for t, v in s.net if lo <= t < hi]
    s.status = [(t, d) for t, d in s.status if lo <= t < hi]
    s.contacts = [(max(a, lo), min(b, hi)) for a, b in s.contacts if a < hi and b > lo]
    s.t_start = max(s.t_start, lo)
    s.t_end = min(s.t_end, hi)


def prepare(path, name, site_name, window=None):
    """Load a session and line up beacons (measured RSSI) with positions and predictions.
    window: 'HH:MM:SS-HH:MM:SS' to analyse only part of the session."""
    s = Session(path)
    s.label = name
    s.window = window
    if window:
        clip_session(s, *parse_window(s, window))
    terrain = sm.find_terrain_dir(TERRAIN_DIR if os.path.isdir(TERRAIN_DIR) else None)
    site = sm.Site(terrain, site_name)
    (bc, br), bs_xy, maps, match = find_bs_cell(s, site)

    # drop anything after the BS was moved mid-session (the predictions no longer apply)
    moves = [t for t in s.bs_moves if t > s.t_start + 5]
    cut = min(moves) if moves else None
    if cut and cut < s.t_end:
        clip_session(s, None, cut)
    else:
        cut = None

    track = position_track(s)
    net_t = np.array([t for t, _ in s.net])
    net_r = np.array([v for _, v in s.net])
    nx, ny = interp_pos(track, net_t)
    ok = np.isfinite(nx)
    net_xy = (net_t[ok], nx[ok], ny[ok], net_r[ok])
    pred_net = {m: site.sample(maps[m], net_xy[1], net_xy[2]) for m in MODELS if maps.get(m) is not None}
    fits = {m: fit_offset(net_xy[3], p) for m, p in pred_net.items()}
    return {'s': s, 'site': site, 'bs_cell': (bc, br), 'bs_xy': bs_xy, 'maps': maps,
            'match': match, 'cut': cut, 'track': track, 'net_xy': net_xy,
            'pred_net': pred_net, 'fits': fits}


def analyze(path, name, site_name, out_root, window=None):
    P = prepare(path, name, site_name, window)
    s, site, (bc, br), bs_xy, maps, match, cut = (P[k] for k in ('s', 'site', 'bs_cell', 'bs_xy',
                                                                  'maps', 'match', 'cut'))
    track, net_xy, pred_net, fits = P['track'], P['net_xy'], P['pred_net'], P['fits']
    net_r = net_xy[3]

    d = [r for r in s.deliv if r['kind'] == 'data']
    t_arr = np.array([fnum(r['epoch']) for r in d])
    lat = np.array([fnum(r['latency_ms']) for r in d])
    t_gen = t_arr - lat / 1000.0
    gx, gy = interp_pos(track, t_gen)
    data = {'t_arr': t_arr, 'lat_ms': lat, 't_gen': t_gen, 'gx': gx, 'gy': gy}
    tel_t = np.array([fnum(r['epoch']) for r in s.deliv if r['kind'] == 'tel'])

    out = os.path.join(out_root, name)
    os.makedirs(out, exist_ok=True)
    fig_map_rm(s, site, track, bs_xy, maps, os.path.join(out, 'map_rm.png'))
    fig_map_comms(s, site, track, bs_xy, net_xy, data, os.path.join(out, 'map_comms.png'))
    fig_timeseries(s, track, bs_xy, net_xy, pred_net, fits, data, tel_t,
                   os.path.join(out, 'timeseries.png'))
    fig_pred_vs_true(s, track, net_xy, pred_net, fits, os.path.join(out, 'pred_vs_true.png'))
    fig_link_quality(s, net_xy, data, maps, fits, site, os.path.join(out, 'link_quality.png'))

    # numbers
    dur = s.t_end - s.t_start
    tt, px, py, _ = track
    okp = np.isfinite(px)
    driven = float(np.sum(np.hypot(np.diff(px[okp]), np.diff(py[okp]))))
    dist_bs = np.hypot(px[okp] - bs_xy[0], py[okp] - bs_xy[1])
    up_s = sum(b - a for a, b in s.contacts)
    last = s.status[-1][1] if s.status else {}
    direct = lat < 2000
    gen_up = contact_state(s, t_gen)
    summary = {
        'session': s.name, 'node': s.node, 'duration_s': round(dur, 1),
        'bs_cell': [bc, br], 'bs_xy': [round(bs_xy[0], 1), round(bs_xy[1], 1)],
        'bs_cell_match_frac': round(float(match), 3), 'cut_at_bs_move': bool(cut),
        'driven_m': round(driven, 1),
        'dist_to_bs_m': {'min': round(float(dist_bs.min()), 1), 'max': round(float(dist_bs.max()), 1)},
        'contacts': len(s.contacts), 'uptime_frac': round(up_s / dur, 3),
        'beacons_heard': len(s.net), 'beacon_rate_hz': round(len(s.net) / dur, 2),
        'rssi_measured': {'min': float(np.min(net_r)), 'median': float(np.median(net_r)),
                          'max': float(np.max(net_r))},
        'data_delivered': len(d), 'tel_delivered': int(len(tel_t)),
        'rover_gen': int(fnum(last.get('gen', 'nan'))) if last else None,
        'rover_tx': int(fnum(last.get('tx', 'nan'))) if last else None,
        'rover_retx': int(fnum(last.get('retx', 'nan'))) if last else None,
        'data_rate_per_s': round(len(d) / dur, 3),
        'latency_ms': ({'mean': round(float(np.mean(lat)), 0), 'p50': round(float(np.median(lat)), 0),
                        'p95': round(float(np.percentile(lat, 95)), 0), 'max': round(float(np.max(lat)), 0)}
                       if len(lat) else None),
        'window': s.window,
        'data_direct_lt2s': int(direct.sum()), 'data_delayed_ge2s': int((~direct).sum()),
        'data_generated_while_link_down': int((~gen_up).sum()),
        'fits': fits,
    }
    with open(os.path.join(out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    write_md(summary, os.path.join(out, 'summary.md'), name)
    return summary


def write_md(sm_, path, name):
    f = sm_
    lines = [
        f"# {name}  (`{f['session']}`)", '',
        *([f"- Analysed window: {f['window']} only", ''] if f.get('window') else []),
        f"- Rover {f['node']}, {f['duration_s']:.0f} s, drove {f['driven_m']:.0f} m, "
        f"{f['dist_to_bs_m']['min']}-{f['dist_to_bs_m']['max']} m from the BS",
        f"- BS at map ({f['bs_xy'][0]:+.1f}, {f['bs_xy'][1]:+.1f}) m, cell col {f['bs_cell'][0]} row {f['bs_cell'][1]}"
        f" (matches {f['bs_cell_match_frac']:.0%} of the logged predictions)"
        + ("; data after a mid-session BS move dropped" if f['cut_at_bs_move'] else ''),
        f"- Link up {f['uptime_frac']:.0%} of the time in {f['contacts']} contact(s); "
        f"{f['beacons_heard']} rover beacons heard at the BS ({f['beacon_rate_hz']}/s)",
        f"- Measured RSSI {f['rssi_measured']['min']:.0f} / {f['rssi_measured']['median']:.0f} / "
        f"{f['rssi_measured']['max']:.0f} dBm (min / median / max)", '',
        '## Delivery', '',
        '| | |', '|---|---|',
        f"| Data bundles delivered | {f['data_delivered']} (rover generated {f['rover_gen']}) |",
        f"| Telemetry delivered | {f['tel_delivered']} |",
        f"| Rover tx / retx | {f['rover_tx']} / {f['rover_retx']} |",
        f"| Data rate | {f['data_rate_per_s']:.2f} bundles/s |",
        (f"| Latency mean / p50 / p95 / max | {f['latency_ms']['mean']:.0f} / {f['latency_ms']['p50']:.0f} / "
         f"{f['latency_ms']['p95']:.0f} / {f['latency_ms']['max']:.0f} ms |" if f['latency_ms']
         else "| Latency | no data bundles |"),
        f"| Delivered < 2 s / >= 2 s | {f['data_direct_lt2s']} / {f['data_delayed_ge2s']} |",
        f"| Generated while link down | {f['data_generated_while_link_down']} |", '',
        '## Predicted vs measured RSSI', '',
        'Prediction = map path gain + a constant offset (median of measured - predicted).', '',
        '| Model | n | offset [dB] | r | RMSE [dB] | MAE [dB] | slope |',
        '|---|---|---|---|---|---|---|',
    ]
    for m, v in f['fits'].items():
        if v:
            lines.append(f"| {m} | {v['n']} | {v['offset_db']:.1f} | {v['r']:.2f} | {v['rmse_db']:.1f} | "
                         f"{v['mae_db']:.1f} | {v['slope']:.2f} |")
    lines += ['', '![radio maps](map_rm.png)', '', '![comms](map_comms.png)', '',
              '![time series](timeseries.png)', '', '![pred vs true](pred_vs_true.png)', '',
              '![link quality](link_quality.png)', '']
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sessions', nargs='+', help='session folders (globs ok)')
    ap.add_argument('--name', help='output name (default: experiment suffix of the folder)')
    ap.add_argument('--site', default='fountain')
    ap.add_argument('--out', default=HERE)
    ap.add_argument('--window', help="only this part, 'HH:MM:SS-HH:MM:SS' (single session only)")
    a = ap.parse_args()
    paths = [p for raw in a.sessions for p in sorted(glob.glob(raw))]
    for p in paths:
        name = a.name if (a.name and len(paths) == 1) else os.path.basename(p.rstrip('/\\')).split('_', 3)[-1]
        sm_ = analyze(p, name, a.site, a.out, a.window if len(paths) == 1 else None)
        print(f"{name}: {sm_['data_delivered']} data, uptime {sm_['uptime_frac']:.0%}, "
              + ', '.join(f"{m} RMSE {v['rmse_db']:.1f} dB r={v['r']:.2f}" for m, v in sm_['fits'].items() if v))


if __name__ == '__main__':
    main()
