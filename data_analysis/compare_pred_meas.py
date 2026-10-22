"""
compare_pred_meas.py  --  Predicted vs measured RSSI, side by side on the map, and the constant
offset to apply to the predictions.

Writes:
    data_analysis/<name>/side_by_side.png   measured | nn_pmnet | nn | avg along the path on one
                                            RSSI scale (offset applied), and the residual maps
    data_analysis/offset/offset_analysis.png  offset estimates, RMSE vs offset, residual vs RSSI
    data_analysis/offset/offset.md / .json    the numbers

Usage (repo root, ..\\.venv Python). A run is folder:name, optionally @HH:MM:SS-HH:MM:SS to use
only part of a session and #<node> to pick one rover of a multi-rover session (it is then named
<name>/<node>). --pool names the runs whose cells form the pooled offset; runs left out
(e.g. a different BS setup) get their own offset in their side-by-side figure:
    python data_analysis/compare_pred_meas.py bs_reports/session_20261015_094354_shadow1:shadow1 \\
        bs_reports/session_20261015_095910_shadow2:shadow2 \\
        bs_reports/session_20261015_091320:preshadow@09:14:20-09:35:41 --pool shadow1,shadow2

Models: nn_pmnet, nn, and avg = the mean of the two maps in dB (built in analyze_session).

Offset estimator. measured RSSI = predicted path gain + offset, so offset = median(meas - pred).
Beacons arrive once per second, so a parked rover adds hundreds of samples at one spot
(shadow1 sits 6 m from the BS for ~200 s). To weight places, not seconds, the residuals are first
reduced to one median per 1 m map cell, then the offset is the median over cells. The 95%
interval is a block bootstrap over runs of 10 consecutive cells, since neighbouring cells are
correlated (same shadow edge, same alignment error).
"""

import argparse
import itertools
import json
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_session as A  # noqa: E402

RSSI_LIM = (-95, -35)
RES_LIM = 20
BLOCK = 10
BOOT_SEED = 0          # fixed per estimate, so results do not depend on run order


def cell_residuals(P, m):
    """One row per 1 m cell visited, in order of first visit: (x, y, meas, pred, n)."""
    t, x, y, r = P['net_xy']
    p = P['pred_net'][m]
    ok = np.isfinite(p)
    t, x, y, r, p = t[ok], x[ok], y[ok], r[ok], p[ok]
    key = np.floor(x).astype(int) * 10000 + np.floor(y).astype(int)
    _, first, inv = np.unique(key, return_index=True, return_inverse=True)
    rows = []
    for k in np.argsort(first):
        sel = inv == k
        rows.append((np.mean(x[sel]), np.mean(y[sel]), np.median(r[sel]), np.median(p[sel]), sel.sum()))
    return np.array(rows)


def block_boot(d, stat=np.median, n=4000):
    if len(d) <= BLOCK:
        return (np.nan, np.nan)
    k = int(np.ceil(len(d) / BLOCK))
    st = np.random.default_rng(BOOT_SEED).integers(0, len(d) - BLOCK + 1, size=(n, k))
    idx = (st[:, :, None] + np.arange(BLOCK)).reshape(n, -1)[:, :len(d)]
    vals = stat(d[idx], axis=1)
    return tuple(np.percentile(vals, [2.5, 97.5]))


def offset_stats(cells):
    meas, pred = cells[:, 2], cells[:, 3]
    d = meas - pred
    off = float(np.median(d))
    res = d - off
    lo, hi = block_boot(d)
    slope, icpt = np.polyfit(pred, meas, 1)
    return {'cells': int(len(d)), 'offset_db': off, 'ci95': [float(lo), float(hi)],
            'mean_db': float(np.mean(d)), 'raw_rmse_db': float(np.sqrt(np.mean(d ** 2))),
            'rmse_db': float(np.sqrt(np.mean(res ** 2))), 'mae_db': float(np.mean(np.abs(res))),
            'r': float(np.corrcoef(pred, meas)[0, 1]), 'slope': float(slope)}


# ----------------------------------------------------------------------------- figures

def fig_side_by_side(P, name, offsets, out, offset_note='pooled offset applied'):
    s, site, bs_xy, maps = P['s'], P['site'], P['bs_xy'], P['maps']
    t, x, y, r = P['net_xy']
    _, tx, ty, _ = P['track']
    ext = A.map_extent(site, tx, ty, bs_xy, pad=8)
    models = [m for m in A.MODELS if m in P['pred_net']]
    nm = len(models)
    fig, axes = plt.subplots(2, nm + 1, figsize=(5.3 * (nm + 1), 11), layout='constrained')

    def path_panel(ax, vals, title, field=None):
        if field is not None:
            A.draw_base(ax, site, ext, bs_xy, field, A.RSSI_CMAP, *RSSI_LIM)
            # fade the field so the path reads as the subject
            ax.imshow(np.ones_like(field), extent=A.grid_extent(site), origin='upper', cmap='gray',
                      vmin=0, vmax=1, alpha=0.45, zorder=1)
        else:
            A.draw_base(ax, site, ext, bs_xy)
        o = np.argsort(t)
        return ax.scatter(x[o], y[o], c=vals[o], cmap=A.RSSI_CMAP, vmin=RSSI_LIM[0], vmax=RSSI_LIM[1],
                          s=20, lw=0.4, edgecolors='white', zorder=5), ax.set_title(title, loc='left',
                                                                                     fontsize=11)

    sc, _ = path_panel(axes[0, 0], r, 'Measured RSSI (BS side)')
    for ax, m in zip(axes[0, 1:], models):
        off = offsets[m]
        path_panel(ax, P['pred_net'][m] + off, f'Predicted: {m} {off:+.1f} dB', maps[m] + off)
    cb = fig.colorbar(sc, ax=axes[0, :], fraction=0.025, pad=0.01)
    cb.set_label('RSSI [dBm]  (same scale in all panels)')

    norm = TwoSlopeNorm(vcenter=0, vmin=-RES_LIM, vmax=RES_LIM)
    for ax, m in zip(axes[1, :nm], models):
        A.draw_base(ax, site, ext, bs_xy)
        res = r - (P['pred_net'][m] + offsets[m])
        o = np.argsort(np.abs(res))          # largest errors drawn on top
        rs = ax.scatter(x[o], y[o], c=np.clip(res[o], -RES_LIM, RES_LIM), cmap='RdBu', norm=norm,
                        s=20, lw=0.4, edgecolors='#52514e', zorder=5)
        ax.set_title(f'Measured - predicted, {m} {offsets[m]:+.1f} dB', loc='left', fontsize=11)
    cb = fig.colorbar(rs, ax=axes[1, :nm], fraction=0.025, pad=0.01)
    cb.set_label('residual [dB]   blue: better than predicted, red: worse')

    ax = axes[1, nm]
    bins = np.arange(-40, 41, 2.5)
    for m in models:
        res = r - (P['pred_net'][m] + offsets[m])
        ax.hist(res, bins=bins, histtype='step', lw=2, color=A.MODEL_COLOR[m],
                label=f'{m}: RMSE {np.sqrt(np.mean(res ** 2)):.1f} dB')
    ax.axvline(0, color=A.INK2, lw=1, ls='--')
    ax.set_xlabel('measured - predicted [dB]')
    ax.set_ylabel('beacons')
    ax.set_title('Residual distribution', loc='left', fontsize=11)
    ax.legend(loc='upper left', fontsize=8)
    fig.suptitle(f'{name}  ({A.span_text(s.when)}) - predicted vs measured RSSI along the path '
                 f'({offset_note})', x=0.01, ha='left', fontsize=13)
    fig.savefig(out)
    plt.close(fig)


def fig_offset(runs, stats, pool, out):
    models = list(A.MODELS)
    names = [n for n, _ in runs]
    pooled_label = f"pooled ({'+'.join(pool)})"
    others = [n for n in names if n not in pool]
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.4), layout='constrained')

    # (a) estimates with CIs
    ax = axes[0]
    labels = names + ['pooled']
    for k, m in enumerate(models):
        for i, lab in enumerate(labels):
            st = stats[m][lab]
            yv = i + (k - (len(models) - 1) / 2) * 0.22
            ax.errorbar(st['offset_db'], yv, xerr=[[st['offset_db'] - st['ci95'][0]],
                                                    [st['ci95'][1] - st['offset_db']]],
                        fmt='o', color=A.MODEL_COLOR[m], ms=8, lw=2, capsize=4,
                        label=m if i == 0 else None)
            ax.annotate(f"{st['offset_db']:+.1f}", (st['offset_db'], yv), xytext=(0, 7),
                        textcoords='offset points', ha='center', fontsize=8, color=A.INK)
    ax.set_yticks(range(len(labels)), names + [pooled_label])
    ax.set_ylim(len(labels) - 0.5, -0.6)
    ax.set_xlabel('offset = median(measured - predicted) [dB], 95% block-bootstrap CI')
    ax.set_title('Offset estimate per run', loc='left', fontsize=11)
    ax.legend(loc='lower left', fontsize=8)

    # (b) RMSE vs applied offset: pooled runs (solid), runs kept out of the pool (dashed)
    ax = axes[1]
    offs = np.linspace(-45, 5, 201)
    groups = [(pooled_label, pool, '-', 'pooled')] + [(n, [n], '--', n) for n in others]
    for m in models:
        for lab, members, ls, key in groups:
            d = np.concatenate([c[m][:, 2] - c[m][:, 3] for n, c in runs if n in members])
            rm = [np.sqrt(np.mean((d - o) ** 2)) for o in offs]
            ax.plot(offs, rm, color=A.MODEL_COLOR[m], lw=2 if ls == '-' else 1.4, ls=ls,
                    label=f'{m}, {lab}')
            o = stats[m][key]['offset_db']
            e = np.sqrt(np.mean((d - o) ** 2))
            ax.plot(o, e, 'o', ms=7, color=A.MODEL_COLOR[m], mec='white', mew=1.5)
            if m == 'nn_pmnet':
                ax.annotate(f"{o:+.1f} dB", (o, e), xytext=(0, -16), textcoords='offset points',
                            ha='center', fontsize=8)
    ax.axvline(0, color=A.INK2, lw=1, ls='--')
    ax.text(0.5, 0.97, 'no offset', transform=ax.get_xaxis_transform(), fontsize=8, color=A.INK2)
    ax.set_xlabel('offset applied to prediction [dB]')
    ax.set_ylabel('RMSE over map cells [dB]')
    ax.set_ylim(top=ax.get_ylim()[1] * 1.3)
    ax.set_title('Error vs offset (1 value per 1 m cell)', loc='left', fontsize=11)
    ax.legend(loc='upper center', fontsize=7.5, ncol=2)

    # (c) is it constant? residual vs predicted level
    ax = axes[2]
    edges = np.arange(-115, -25, 7.5)
    for m in models:
        st = stats[m]['pooled']
        for (n, c), ls in zip(runs, itertools.cycle(['-', '--', ':', '-.', (0, (5, 1, 1, 1, 1, 1))])):
            cc = c[m]
            d = cc[:, 2] - cc[:, 3]
            pred = cc[:, 3] + stats[m]['pooled' if n in pool else n]['offset_db']
            idx = np.digitize(pred, edges) - 1
            xs, ys = [], []
            for b in range(len(edges) - 1):
                sel = idx == b
                if sel.sum() >= 4:
                    xs.append((edges[b] + edges[b + 1]) / 2)
                    ys.append(np.median(d[sel]))
            ax.plot(xs, ys, ls=ls, marker='o', ms=5, color=A.MODEL_COLOR[m], lw=1.8,
                    label=f'{m}, {n}')
        ax.axhline(st['offset_db'], color=A.MODEL_COLOR[m], lw=1, ls=':', alpha=0.9)
    ax.set_xlabel('predicted RSSI with offset [dBm] (pooled; own offset for runs outside the pool)')
    ax.set_ylabel('median(measured - predicted) [dB]')
    ax.set_title('Offset by signal level (flat = constant offset fits)', loc='left', fontsize=11)
    ax.legend(loc='lower left', fontsize=7.5, ncol=3)
    fig.suptitle('Constant offset between predicted path gain and measured RSSI',
                 x=0.01, ha='left', fontsize=13)
    fig.savefig(out)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='session_folder:name[@HH:MM:SS-HH:MM:SS]')
    ap.add_argument('--pool', help='comma-separated run names that form the pooled offset (default: all)')
    ap.add_argument('--site', default='fountain')
    a = ap.parse_args()

    runs, preps = [], {}
    for spec in a.runs:
        spec, _, node = spec.partition('#')
        spec, _, window = spec.partition('@')
        path, name = spec.rsplit(':', 1) if ':' in spec[2:] else (spec, os.path.basename(spec))
        P = A.prepare(path, name, a.site, window or None, node or None)
        if node:
            name = f'{name}/{node}'          # one rover of a multi-rover run: data_analysis/<name>/<node>/
        preps[name] = P
        runs.append((name, {m: cell_residuals(P, m) for m in A.MODELS if m in P['pred_net']}))

    pool = a.pool.split(',') if a.pool else [n for n, _ in runs]
    stats = {}
    for m in A.MODELS:
        stats[m] = {n: offset_stats(c[m]) for n, c in runs}
        stats[m]['pooled'] = offset_stats(np.concatenate([c[m] for n, c in runs if n in pool]))
    pooled_label = f"pooled ({'+'.join(pool)})"

    for name, P in preps.items():
        os.makedirs(os.path.join(HERE, name), exist_ok=True)
        key, note = (('pooled', f'{pooled_label} offset applied') if name in pool
                     else (name, "this run's own offset applied, not in the pool"))
        fig_side_by_side(P, name, {m: stats[m][key]['offset_db'] for m in A.MODELS},
                         os.path.join(HERE, name, 'side_by_side.png'), note)
    od = os.path.join(HERE, 'offset')
    os.makedirs(od, exist_ok=True)
    fig_offset(runs, stats, pool, os.path.join(od, 'offset_analysis.png'))
    with open(os.path.join(od, 'offset.json'), 'w') as f:
        json.dump(stats, f, indent=1)

    lines = ['# Prediction offset', '',
             'offset = median over 1 m cells of median(measured - predicted) per cell; '
             '95% CI = block bootstrap (10 consecutive cells).', '',
             '| Model | Run | cells | offset [dB] | 95% CI | RMSE raw | RMSE w/ offset | MAE | r | slope |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for m in A.MODELS:
        for n, st in stats[m].items():
            n = pooled_label if n == 'pooled' else n
            lines.append(f"| {m} | {n} | {st['cells']} | {st['offset_db']:+.1f} | "
                         f"{st['ci95'][0]:+.1f} .. {st['ci95'][1]:+.1f} | {st['raw_rmse_db']:.1f} | "
                         f"{st['rmse_db']:.1f} | {st['mae_db']:.1f} | {st['r']:.2f} | {st['slope']:.2f} |")
    lines += ['', '![offset analysis](offset_analysis.png)', '']
    with open(os.path.join(od, 'offset.md'), 'w') as f:
        f.write('\n'.join(lines))
    print('\n'.join(lines[4:-2]))


if __name__ == '__main__':
    main()
