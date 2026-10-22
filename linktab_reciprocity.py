"""
linktab_reciprocity.py - how two-way consistent are the predicted link tables?

A radio link loses the same in both directions (reciprocity), but RadioLunaDiff predicts a map
per transmitter, so rss[a -> b] and rss[b -> a] come from two different model runs and can
disagree. This compares that disagreement for the link tables of rm_linktab_colab.ipynb
(nn_pmnet = PMNet stage only, nn = + diffusion residual) and writes figures + metrics:

    asym_hist.png      distribution of |rss(a->b) - rss(b->a)| over all drivable pairs
    asym_binned.png    the same vs distance and vs link strength
    asym_spatial.png   per cell: mean disagreement with all other cells
    asym_examples.png  example transmitters: forward map, reverse map, difference
    peak_check.png     where each map's strongest cell is relative to its transmitter
    metrics.json       all numbers, incl. how much the asymmetry changes relay choices

    python linktab_reciprocity.py --site fountain --out bs_reports/reciprocity
"""

import argparse
import json
import math
import os

import numpy as np

import relay_brain
import site_map

MODELS = ('nn_pmnet', 'nn')
LABEL = {'nn_pmnet': 'PMNet (nn_pmnet)', 'nn': 'PMNet + diffusion (nn)'}
COLOR = {'nn_pmnet': '#1f77b4', 'nn': '#d62728'}
DIST_BINS = [0, 5, 10, 20, 40, 80, 160, 400]
LEVEL_BINS = [-130, -100, -90, -80, -70, -60, -40, 0]


def load(site, model):
    z = np.load(os.path.join(site.dir, f"{site.name}_linktab_{model}.npz"))
    tx, rx = z['tx_rc'], z['rx_rc']
    if tx.shape != rx.shape or not (tx == rx).all():
        raise SystemExit(f"{model}: transmitter and receiver cells differ (need TX_SPACING_M = 0)")
    return z['rss'], rx.astype(int)


def stats(absd):
    a = absd.astype(np.float32)
    return {'mean': float(a.mean()), 'median': float(np.median(a)), 'rms': float(np.sqrt((a * a).mean())),
            'p90': float(np.percentile(a, 90)), 'p99': float(np.percentile(a, 99)), 'max': float(a.max()),
            'share_gt3': float((a > 3).mean()), 'share_gt6': float((a > 6).mean()),
            'share_gt10': float((a > 10).mean()), 'share_gt20': float((a > 20).mean())}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--site', default='fountain')
    ap.add_argument('--terrain-dir')
    ap.add_argument('--bs', help='BS map position x,y for the relay test (default: bs_reports/bs_position_<site>.json, else the site tx)')
    ap.add_argument('--out', default=os.path.join('bs_reports', 'reciprocity'))
    ap.add_argument('--pairs', type=int, default=4_000_000, help='random pairs for the binned plots')
    ap.add_argument('--scenarios', type=int, default=3000, help='random explorer cells for the relay test')
    args = ap.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    site = site_map.Site(site_map.find_terrain_dir(args.terrain_dir), args.site)
    os.makedirs(args.out, exist_ok=True)
    tables = {m: load(site, m) for m in MODELS}
    rc = tables[MODELS[0]][1]
    n = len(rc)
    xy = np.stack([(rc[:, 1] + 0.5) * site.cell - site.half, site.half - (rc[:, 0] + 0.5) * site.cell], 1)
    iu = np.triu_indices(n, 1)                 # each unordered pair once
    metrics = {'site': site.name, 'cells': n, 'pairs': int(len(iu[0])), 'models': {}}
    print(f"{site.name}: {n} drivable cells, {len(iu[0]):,} pairs")

    # 1. all pairs
    asym = {}
    for m in MODELS:
        r = tables[m][0].astype(np.int16)
        d = (r - r.T)[iu]                     # rss(a->b) - rss(b->a), a < b
        asym[m] = d
        s = stats(np.abs(d))
        s['signed_mean'] = float(d.mean())
        s['corr_fwd_rev'] = float(np.corrcoef(r[iu].astype(np.float32), r.T[iu].astype(np.float32))[0, 1])
        metrics['models'][m] = {'all_pairs': s}
        print(f"{m}: |a->b - b->a| mean {s['mean']:.2f}, median {s['median']:.0f}, p90 {s['p90']:.0f}, "
              f"p99 {s['p99']:.0f} dB; >6 dB {100 * s['share_gt6']:.1f}%; corr {s['corr_fwd_rev']:.4f}")
        del r
    # between models (same direction): how much the diffusion stage changes a link
    dm = tables['nn'][0].astype(np.int16) - tables['nn_pmnet'][0].astype(np.int16)
    metrics['nn_minus_pmnet_same_direction'] = {**stats(np.abs(dm.ravel()[::7])), 'signed_mean': float(dm.mean())}
    del dm

    # 2. vs distance and link strength (random pairs)
    rng = np.random.default_rng(0)
    k = rng.choice(len(iu[0]), size=min(args.pairs, len(iu[0])), replace=False)
    a, b = iu[0][k], iu[1][k]
    dist = np.hypot(*(xy[a] - xy[b]).T)
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.3))
    for m in MODELS:
        r = tables[m][0]
        level = 0.5 * (r[a, b].astype(np.float32) + r[b, a])
        ad = np.abs(asym[m][k]).astype(np.float32)
        rows = []
        for ax, val, bins, name in ((axs[0], dist, DIST_BINS, 'distance_m'), (axs[1], level, LEVEL_BINS, 'level_dbm')):
            idx = np.digitize(val, bins) - 1
            cen, med, p90, mean, cnt = [], [], [], [], []
            for i in range(len(bins) - 1):
                sel = idx == i
                if sel.sum() < 100:
                    continue
                cen.append(f"{bins[i]}–{bins[i + 1]}")
                med.append(float(np.median(ad[sel])))
                mean.append(float(ad[sel].mean()))
                p90.append(float(np.percentile(ad[sel], 90)))
                cnt.append(int(sel.sum()))
            x = np.arange(len(cen)) + (-0.18 if m == 'nn_pmnet' else 0.18)
            ax.bar(x, mean, width=0.36, color=COLOR[m], alpha=0.85, label=f"{LABEL[m]} mean")
            ax.plot(x, p90, 'v', color=COLOR[m], mec='k', ms=7, label=f"{LABEL[m]} 90th pct")
            ax.set_xticks(np.arange(len(cen)))
            ax.set_xticklabels(cen, rotation=0, fontsize=8)
            rows.append({name: cen, 'mean': mean, 'median': med, 'p90': p90, 'count': cnt})
        metrics['models'][m]['by_distance'], metrics['models'][m]['by_level'] = rows
    axs[0].set_xlabel('distance between the two cells [m]')
    axs[1].set_xlabel('link strength, mean of both directions [dBm]')
    for ax in axs:
        ax.set_ylabel('|rss(a→b) − rss(b→a)| [dB]')
        ax.grid(axis='y', alpha=0.3)
    axs[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'asym_binned.png'), dpi=130)
    plt.close(fig)

    # 3. histogram / CDF
    fig, axs = plt.subplots(1, 2, figsize=(12, 4))
    for m in MODELS:
        ad = np.abs(asym[m][k])
        h = np.bincount(ad, minlength=60)[:60] / len(ad)
        axs[0].step(np.arange(60), h, where='mid', color=COLOR[m], label=LABEL[m])
        srt = np.sort(ad)
        axs[1].plot(srt, np.arange(1, len(srt) + 1) / len(srt), color=COLOR[m], label=LABEL[m])
    axs[0].set_yscale('log')
    axs[0].set_xlabel('|rss(a→b) − rss(b→a)| [dB]')
    axs[0].set_ylabel('share of pairs')
    axs[1].set_xlabel('|rss(a→b) − rss(b→a)| [dB]')
    axs[1].set_ylabel('cumulative share of pairs')
    axs[1].set_xlim(0, 40)
    for ax in axs:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'asym_hist.png'), dpi=130)
    plt.close(fig)

    # 4. per-cell mean disagreement on the map
    def grid(vals):
        g = np.full(site.drivable.shape, np.nan, np.float32)
        g[rc[:, 0], rc[:, 1]] = vals
        return g

    rows, cols = np.nonzero(site.drivable)
    pad = 4
    crop = (max(rows.min() - pad, 0), rows.max() + pad + 1, max(cols.min() - pad, 0), cols.max() + pad + 1)
    ext = ((crop[2]) * site.cell - site.half, crop[3] * site.cell - site.half,
           site.half - crop[1] * site.cell, site.half - crop[0] * site.cell)

    def show(ax, g, **kw):
        bg = np.where(site.drivable, np.nan, 1.0)[crop[0]:crop[1], crop[2]:crop[3]]
        ax.imshow(bg, cmap='Greys', vmin=0, vmax=4, extent=ext, interpolation='nearest')
        return ax.imshow(g[crop[0]:crop[1], crop[2]:crop[3]], extent=ext, interpolation='nearest', **kw)

    fig, axs = plt.subplots(1, 2, figsize=(13, 6))
    per_cell = {}
    for ax, m in zip(axs, MODELS):
        r = tables[m][0].astype(np.int16)
        per_cell[m] = np.abs(r - r.T).mean(1)
        im = show(ax, grid(per_cell[m]), cmap='magma', vmin=0, vmax=20)
        ax.set_title(f"{LABEL[m]}: mean |a→b − b→a| per cell", fontsize=10)
        plt.colorbar(im, ax=ax, fraction=0.046, label='dB')
        del r
    for m in MODELS:
        metrics['models'][m]['per_cell_mean'] = {'median': float(np.median(per_cell[m])),
                                                 'p90': float(np.percentile(per_cell[m], 90)),
                                                 'max': float(per_cell[m].max())}
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'asym_spatial.png'), dpi=120)
    plt.close(fig)

    # 4b. edge effect: disagreement vs the cell's distance to the map border
    border = np.minimum.reduce([rc[:, 0], rc[:, 1], site.size - 1 - rc[:, 0], site.size - 1 - rc[:, 1]]) * site.cell
    edge_bins = [0, 5, 10, 20, 40, 80, 200]
    fig, ax = plt.subplots(figsize=(7, 4))
    inner = border >= 10
    ii = np.nonzero(inner)[0]
    for j, m in enumerate(MODELS):
        idx = np.digitize(border, edge_bins) - 1
        lab, val, cnt = [], [], []
        for i in range(len(edge_bins) - 1):
            sel = idx == i
            if sel.any():
                lab.append(f"{edge_bins[i]}–{edge_bins[i + 1]}")
                val.append(float(per_cell[m][sel].mean()))
                cnt.append(int(sel.sum()))
        x = np.arange(len(lab)) + (-0.18 if j == 0 else 0.18)
        ax.bar(x, val, width=0.36, color=COLOR[m], label=LABEL[m])
        ax.set_xticks(np.arange(len(lab)))
        ax.set_xticklabels(lab)
        r = tables[m][0].astype(np.int16)[np.ix_(ii, ii)]
        d_in = np.abs(r - r.T)[np.triu_indices(len(ii), 1)]
        metrics['models'][m]['by_border_distance'] = {'border_m': lab, 'per_cell_mean': val, 'cells': cnt}
        metrics['models'][m]['pairs_both_10m_from_border'] = stats(d_in)
        del r, d_in
    metrics['cells_within_10m_of_border'] = int((~inner).sum())
    ax.set_xlabel("cell's distance to the edge of the 256 m map [m]")
    ax.set_ylabel('mean |a→b − b→a| over its pairs [dB]')
    ax.grid(axis='y', alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'asym_border.png'), dpi=130)
    plt.close(fig)

    # 5. example transmitters: forward, reverse, difference (both models)
    picks = [int(np.argmin(per_cell['nn_pmnet']))]                      # most consistent cell
    picks.append(int(np.argsort(per_cell['nn_pmnet'])[len(rc) // 2]))   # typical cell
    picks.append(int(np.argmax(per_cell['nn_pmnet'])))                  # least consistent (PMNet)
    picks.append(int(np.argmax(per_cell['nn'])))                        # least consistent (nn)
    fig, axs = plt.subplots(len(picks), 6, figsize=(24, 4.1 * len(picks)))
    for row, i in enumerate(picks):
        for j, m in enumerate(MODELS):
            r = tables[m][0]
            fwd, rev = r[i].astype(np.float32), r[:, i].astype(np.float32)
            for c, (vals, title, kw) in enumerate((
                    (fwd, f"{m}: tx at ★ (a→b)", dict(cmap='viridis', vmin=-110, vmax=-20)),
                    (rev, f"{m}: tx at each cell, rx at ★ (b→a)", dict(cmap='viridis', vmin=-110, vmax=-20)),
                    (fwd - rev, f"{m}: difference (a→b − b→a)", dict(cmap='RdBu_r', vmin=-25, vmax=25)))):
                ax = axs[row, 3 * j + c]
                im = show(ax, grid(vals), **kw)
                ax.plot(*xy[i], '*', color='w', mec='k', ms=15)
                if c == 2:
                    ad = np.abs(fwd - rev)
                    title += f"\nmean {ad.mean():.1f} dB, 90% {np.percentile(ad, 90):.0f} dB"
                ax.set_title(title, fontsize=9)
                ax.set_xticks([])
                ax.set_yticks([])
                plt.colorbar(im, ax=ax, fraction=0.046)
        axs[row, 0].set_ylabel(['most consistent cell', 'typical cell', 'least consistent (PMNet)',
                                'least consistent (nn)'][row] + f"\n({xy[i][0]:+.0f}, {xy[i][1]:+.0f}) m",
                               fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'asym_examples.png'), dpi=85)
    plt.close(fig)
    metrics['example_cells'] = [{'xy': [float(v) for v in xy[i]]} for i in picks]

    # 6. transmitter placement check: each map's strongest cell should be at its transmitter
    fig, ax = plt.subplots(figsize=(6, 4))
    for m in MODELS:
        r = tables[m][0]
        off = np.hypot(*(xy[r.argmax(1)] - xy).T)
        metrics['models'][m]['peak_offset_m'] = {'at_tx': float((off < 0.5).mean()),
                                                 'within_2m': float((off <= 2.0 * site.cell + 1e-6).mean()),
                                                 'p99': float(np.percentile(off, 99))}
        ax.hist(np.minimum(off, 10), bins=np.arange(0, 10.5, 0.5), color=COLOR[m], alpha=0.6, label=LABEL[m])
    ax.set_xlabel("distance from the transmitter to its map's strongest cell [m] (capped at 10)")
    ax.set_ylabel('transmitters')
    ax.set_yscale('log')
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'peak_check.png'), dpi=120)
    plt.close(fig)

    # 7. does it change what the relay picks? BS fixed, random explorer cells
    if args.bs:
        bs = tuple(float(v) for v in args.bs.split(','))
    else:
        bs = site.tx_xy
        p = os.path.join('bs_reports', f"bs_position_{site.name}.json")
        if os.path.exists(p):
            with open(p) as f:
                q = json.load(f)
            bs = (float(q['x']), float(q['y']))
    i_bs = int(np.argmin(((xy - np.asarray(bs)) ** 2).sum(1)))
    ex = rng.choice(n, size=min(args.scenarios, n), replace=False)
    relay = {}
    for m in MODELS:
        r = tables[m][0]
        bs_fwd, bs_rev = r[i_bs].astype(np.float32), r[:, i_bs].astype(np.float32)   # BS->p, p->BS
        bs_avg = 0.5 * (bs_fwd + bs_rev)
        pick = {'fwd': [], 'rev': [], 'avg': []}
        regret = {'fwd': [], 'rev': []}
        for e in ex:
            e_fwd, e_rev = r[:, e].astype(np.float32), r[e].astype(np.float32)        # p->ex, ex->p
            avg = np.minimum(bs_avg, 0.5 * (e_fwd + e_rev))
            choice = {'fwd': int(np.argmax(np.minimum(bs_fwd, e_fwd))),        # downlink only
                      'rev': int(np.argmax(np.minimum(bs_rev, e_rev))),        # uplink only
                      'avg': int(np.argmax(avg))}                              # what the brain uses
            for kk in pick:
                pick[kk].append(choice[kk])
            for kk in regret:
                regret[kk].append(float(avg[choice['avg']] - avg[choice[kk]]))
        sep = np.hypot(*(xy[pick['fwd']] - xy[pick['rev']]).T)
        relay[m] = pick
        metrics['models'][m]['relay_choice'] = {
            'fwd_vs_rev_spot_m_median': float(np.median(sep)), 'fwd_vs_rev_spot_m_p90': float(np.percentile(sep, 90)),
            'fwd_vs_rev_same_within_3m': float((sep <= 3).mean()),
            'regret_db_if_one_direction_median': float(np.median(regret['fwd'] + regret['rev'])),
            'regret_db_if_one_direction_p90': float(np.percentile(regret['fwd'] + regret['rev'], 90))}
    sep_models = np.hypot(*(xy[relay['nn']['avg']] - xy[relay['nn_pmnet']['avg']]).T)
    metrics['relay_choice_nn_vs_pmnet'] = {'spot_m_median': float(np.median(sep_models)),
                                           'spot_m_p90': float(np.percentile(sep_models, 90)),
                                           'same_within_3m': float((sep_models <= 3).mean())}
    metrics['relay_test'] = {'bs_xy': [float(v) for v in xy[i_bs]], 'explorer_cells': int(len(ex))}
    # 8. which direction matches measurements? coverage.csv rows: measured RSSI at the BS from a
    # rover at a known map position, BS position from the session's bs_position.json
    import csv
    import glob
    samples = []                               # (i_bs, i_cell, rssi)
    sessions = 0
    meas_sessions = []
    for cov in sorted(glob.glob(os.path.join('bs_reports', 'session_*', 'coverage.csv'))):
        bp = os.path.join(os.path.dirname(cov), 'bs_position.json')
        if not os.path.exists(bp):
            continue
        with open(bp) as f:
            q = json.load(f)
        if q.get('site') != site.name:
            continue
        i_b = int(np.argmin(((xy - np.array([q['x'], q['y']])) ** 2).sum(1)))
        before = len(samples)
        rows_all = []
        with open(cov, newline='') as f:
            for row in csv.DictReader(f):
                try:
                    rssi, mx, my = int(row['rssi_to_bs_dBm']), float(row['map_x']), float(row['map_y'])
                except (ValueError, KeyError, TypeError):
                    continue
                if not (math.isfinite(mx) and math.isfinite(my)) or (float(row['lat']) == 0 and float(row['lon']) == 0):
                    continue
                d2 = ((xy - np.array([mx, my])) ** 2).sum(1)
                j = int(np.argmin(d2))
                rows_all.append((j, rssi, d2[j] <= 2.0 ** 2, math.hypot(mx - q['x'], my - q['y'])))
        # keep a session only if it looks like a real outdoor run with a sane alignment, judged
        # without the radio predictions: rover got >= 20 m from the BS, RSSI spans >= 40 dB,
        # >= 85 % of positions within 2 m of the drivable area
        if not rows_all:
            continue
        on = np.mean([r[2] for r in rows_all])
        span = max(r[1] for r in rows_all) - min(r[1] for r in rows_all)
        far = max(r[3] for r in rows_all)
        keep = far >= 20 and span >= 40 and on >= 0.85
        meas_sessions.append({'session': os.path.basename(os.path.dirname(cov)), 'rows': len(rows_all),
                              'max_dist_m': round(far, 1), 'rssi_span_db': int(span),
                              'on_drivable': round(float(on), 2), 'used': bool(keep)})
        if keep:
            samples += [(i_b, j, rssi) for j, rssi, ok, _ in rows_all if ok]
        sessions += len(samples) > before
    meas = {'samples': len(samples), 'sessions': sessions, 'session_filter': meas_sessions}
    if len(samples) >= 50:
        S = np.array(samples)
        y = S[:, 2].astype(np.float32)
        fig, axs = plt.subplots(1, 2, figsize=(12, 4.6))
        for ax, m in zip(axs, MODELS):
            r = tables[m][0]
            down = r[S[:, 0], S[:, 1]].astype(np.float32)          # BS -> rover cell
            up = r[S[:, 1], S[:, 0]].astype(np.float32)            # rover cell -> BS
            est = {'BS→rover (a→b)': down, 'rover→BS (b→a)': up, 'mean of both': 0.5 * (down + up),
                   'max of both': np.maximum(down, up), 'min of both': np.minimum(down, up)}
            res = {}
            for name, v in est.items():
                off = float(np.median(y - v))                       # antenna/cable gains: one constant
                err = y - (v + off)
                res[name] = {'pearson_r': float(np.corrcoef(v, y)[0, 1]), 'offset_db': off,
                             'mae_after_offset': float(np.abs(err).mean()),
                             'rmse_after_offset': float(np.sqrt((err * err).mean()))}
            meas[m] = res
            names = list(res)
            ax.bar(np.arange(len(names)) - 0.2, [res[k]['mae_after_offset'] for k in names], width=0.4,
                   color=COLOR[m], label='MAE after a constant offset [dB]')
            ax2 = ax.twinx()
            ax2.plot(np.arange(len(names)) + 0.2, [res[k]['pearson_r'] for k in names], 'ko', label='Pearson r')
            ax2.set_ylim(0, 1)
            ax2.set_ylabel('Pearson r with measured RSSI')
            ax.set_xticks(np.arange(len(names)))
            ax.set_xticklabels(names, rotation=20, fontsize=8)
            ax.set_ylabel('MAE [dB]')
            ax.set_title(f"{LABEL[m]} vs measured RSSI ({len(y)} samples, {sessions} sessions)", fontsize=9)
            ax.grid(axis='y', alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, 'asym_vs_measured.png'), dpi=130)
        plt.close(fig)
    metrics['measured'] = meas

    with open(os.path.join(args.out, 'metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=1)
    print(json.dumps({m: metrics['models'][m]['relay_choice'] for m in MODELS}, indent=1))
    print('nn vs pmnet relay spot:', metrics['relay_choice_nn_vs_pmnet'])
    print(f"wrote {args.out}")


if __name__ == '__main__':
    main()
