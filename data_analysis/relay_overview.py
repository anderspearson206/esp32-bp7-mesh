"""
relay_overview.py  --  Relay-run figure and summary for a two-rover session (explorer + relay).

Writes into data_analysis/<name>/:
    relay.png    map of both paths with the relay's targets; RSSI of both rovers at the BS; the
                 relay's measured links (BS / explorer); explorer bundles delivered direct vs via the
                 relay; and which BS control version each rover had adopted (control loss shows here)
    relay.md     the numbers behind it

Usage (repo root, ..\\.venv Python):
    python data_analysis/relay_overview.py data_analysis/raw/session_20261023_135640_double_ferry3 \
        --name double_ferry3 --explorer 54272 --relay 57936

Rover status comes from the @ROVER_STATUS lines in serial.log (rover_status_*.csv can have a stale
header). Measured RSSI at the BS is from @NET (one per received beacon), as in analyze_session.py.
"""

import argparse
import json
import os
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

from analyze_session import (HERE, TERRAIN_DIR, INK, INK2, C_MEAS, C_PMNET, C_NN, C_DOWN, sm, ts,
                             clock, span_info, span_text, read_csv, map_extent, draw_base)

C_EX, C_RL, C_HOP = C_MEAS, C_PMNET, C_NN
CONTROL_LAG_S = 10.0        # a rover still on an older control version this long after a new one = lost


def read_log(path):
    """(net, status, controls) from serial.log: net {node: [(t, rssi)]}, status {node: [(t, fields)]},
    controls [(t, version, air bytes estimate)]."""
    net, status, controls = {}, {}, []
    with open(os.path.join(path, 'serial.log'), encoding='utf-8', errors='replace') as f:
        for line in f:
            stamp, _, rest = line.rstrip('\n').partition('\t')
            if rest.startswith('@NET:'):
                p = rest.split(':')
                if len(p) >= 4 and p[2] != '0':
                    net.setdefault(p[1], []).append((ts(stamp), float(p[3])))
            elif rest.startswith('@ROVER_STATUS:'):
                _, node, kv = rest.split(':', 2)
                fields = dict(x.split('=', 1) for x in kv.split(',') if '=' in x)
                status.setdefault(node, []).append((ts(stamp), fields))
            elif rest.startswith('#CONTROL:v'):
                v, js = rest[len('#CONTROL:v'):].split(':', 1)
                controls.append((ts(stamp), int(v), len(js)))
    return net, status, controls


def fval(fields, key):
    try:
        return float(fields[key])
    except (KeyError, ValueError):
        return np.nan


def control_loss(status, controls, t_end):
    """Per rover: spans where the BS had a newer control version than the rover reported for more
    than CONTROL_LAG_S, as [(t0, t1, version)]."""
    out = {}
    for node, rows in status.items():
        spans = []
        for i, (t, v, _) in enumerate(controls):
            t_next = controls[i + 1][0] if i + 1 < len(controls) else t_end
            got = next((ts_ for ts_, f in rows if t <= ts_ <= t_next and int(fval(f, 'cv') or 0) >= v), None)
            if got is None and t_next - t > CONTROL_LAG_S:
                spans.append((t, t_next, v))
            elif got is not None and got - t > CONTROL_LAG_S:
                spans.append((t, got, v))
        out[node] = spans
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('--name', required=True)
    ap.add_argument('--explorer', required=True)
    ap.add_argument('--relay', required=True)
    ap.add_argument('--site', default='fountain')
    ap.add_argument('--out', default=HERE)
    a = ap.parse_args()
    ex, rl = a.explorer, a.relay
    out_dir = os.path.join(a.out, a.name)
    os.makedirs(out_dir, exist_ok=True)

    site = sm.Site(TERRAIN_DIR, a.site)
    with open(os.path.join(a.session, 'bs_position.json')) as f:
        b = json.load(f)
    bs_xy = (b['x'], b['y'])
    cov = read_csv(os.path.join(a.session, 'coverage.csv'))
    deliv = read_csv(os.path.join(a.session, 'deliveries.csv'))
    net, status, controls = read_log(a.session)
    t0 = min(ts(r['time']) for r in deliv)
    t1 = max(ts(r['time']) for r in deliv)
    span = span_info(t0, t1)
    loss = control_loss(status, controls, t1)

    tracks = {n: np.array([(float(r['map_x']), float(r['map_y'])) for r in cov if r['node_id'] == n])
              for n in (ex, rl)}
    targets = np.array([(fval(f, 'rtx'), fval(f, 'rty')) for _, f in status.get(rl, []) if 'rtx' in f])

    fig = plt.figure(figsize=(15, 10))
    gs = fig.add_gridspec(4, 2, width_ratios=[1, 1.5], height_ratios=[1, 1, 1, 0.55],
                          hspace=0.45, wspace=0.15)
    ax = fig.add_subplot(gs[:, 0])
    allp = np.vstack([p for p in tracks.values() if len(p)])
    draw_base(ax, site, map_extent(site, allp[:, 0], allp[:, 1], bs_xy, pad=6), bs_xy)
    for n, c, lab in ((ex, C_EX, f'explorer {ex}'), (rl, C_RL, f'relay {rl}')):
        p = tracks[n]
        if len(p):
            ax.plot(p[:, 0], p[:, 1], color=c, lw=2, label=lab, zorder=5)
            ax.plot(*p[0], 'o', color=c, ms=8, mec='white', mew=2, zorder=6)
    if len(targets):
        ax.plot(targets[:, 0], targets[:, 1], 'D', color=C_RL, ms=8, mfc='none', mew=2,
                label='relay targets', zorder=7)
    ax.legend(loc='upper right', fontsize=8, frameon=True, facecolor='white', framealpha=0.85)
    ax.set_title('Paths (start = dot)', loc='left', fontsize=11, color=INK)

    def tline(i, title):
        axx = fig.add_subplot(gs[i, 1], sharex=axes[0] if axes else None)
        axx.set_title(title, loc='left', fontsize=11, color=INK)
        axes.append(axx)
        return axx

    axes = []
    dt = datetime.fromtimestamp
    ax1 = tline(0, 'RSSI at the BS (dBm, per received beacon)')
    for n, c in ((ex, C_EX), (rl, C_RL)):
        rr = net.get(n, [])
        if rr:
            ax1.plot([dt(t) for t, _ in rr], [r for _, r in rr], '.', ms=3, color=c, label=f'{n}')
    ax1.set_ylim(-100, -30)

    ax2 = tline(1, 'Relay-measured links (dBm, 8 s median; -95 = nothing heard)')
    for key, c, lab in (('rmb', INK, 'relay hears BS'), ('rme', C_EX, 'relay hears explorer')):
        rr = [(dt(t), fval(f, key)) for t, f in status.get(rl, []) if key in f]
        if rr:
            ax2.plot(*zip(*rr), '-', lw=2, color=c, label=lab)
    ax2.set_ylim(-100, -30)
    if not any(key in f for _, f in status.get(rl, []) for key in ('rmb', 'rme')):
        ax2.text(0.5, 0.5, f'relay {rl} never ran (no relay status)', transform=ax2.transAxes,
                 ha='center', va='center', color=INK2, fontsize=11)

    ax3 = tline(2, f'Explorer {ex} bundles delivered per 10 s')
    bins = {}
    for r in deliv:
        if r['src'] == ex:
            k = int((ts(r['time']) - t0) // 10)
            bins.setdefault(k, [0, 0])[0 if r['hops'] == '1' else 1] += 1
    ks = sorted(bins)
    xs = [dt(t0 + 10 * k + 5) for k in ks]
    d1, d2 = [bins[k][0] for k in ks], [bins[k][1] for k in ks]
    ax3.bar(xs, d1, width=8 / 86400, color=C_EX, label='direct')
    ax3.bar(xs, d2, width=8 / 86400, bottom=d1, color=C_HOP, label='via relay (2 hops)',
            edgecolor='white', linewidth=1)

    ax4 = tline(3, 'BS control not adopted (red = rover still on an older version)')
    for i, n in enumerate((ex, rl)):
        for s0, s1, _ in loss.get(n, []):
            ax4.barh(i, (s1 - s0) / 86400, left=dt(s0), height=0.6, color=C_DOWN, alpha=0.8)
    ax4.set_yticks([0, 1])
    ax4.set_yticklabels([f'{ex} explorer', f'{rl} relay'])
    ax4.set_ylim(-0.6, 1.6)
    ax4.grid(axis='y', visible=False)
    for t, v, _ in controls:
        ax4.axvline(dt(t), color=INK2, lw=0.8, ls=':')
    for axx in axes:
        axx.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        axx.set_xlim(dt(t0), dt(t1))
        if axx is not ax4 and axx.get_legend_handles_labels()[0]:
            axx.legend(loc='lower left', fontsize=8, ncol=3, markerscale=4)
    fig.suptitle(f'{a.name}: relay {rl} for explorer {ex}  ({span_text(span)})', x=0.01, ha='left',
                 fontsize=13, color=INK)
    fig.savefig(os.path.join(out_dir, 'relay.png'))
    plt.close(fig)

    # summary
    exd = [r for r in deliv if r['src'] == ex]
    lines = [f'# {a.name}: relay {rl} for explorer {ex}', '',
             f'{span_text(span)} (UTC{span["utc_offset"]}), session `{os.path.basename(a.session.rstrip("/"))}`, '
             f'BS at ({bs_xy[0]:+.1f}, {bs_xy[1]:+.1f}).', '',
             '| | explorer | relay |', '|---|---|---|']
    for kind in ('data', 'tel'):
        row = []
        for n in (ex, rl):
            rr = [r for r in deliv if r['src'] == n and r['kind'] == kind]
            hop2 = sum(1 for r in rr if r['hops'] != '1')
            row.append(f'{len(rr)} ({hop2} via 2 hops)' if rr else '0')
        lines.append(f'| {kind} bundles delivered | {row[0]} | {row[1]} |')
    row = []
    for n in (ex, rl):
        lat = sorted(float(r['latency_ms']) for r in deliv if r['src'] == n)
        row.append(f'{lat[len(lat) // 2]:.0f} / {lat[int(0.95 * (len(lat) - 1))]:.0f} / {lat[-1]:.0f} ms'
                   if lat else '-')
    lines.append(f'| latency p50 / p95 / max | {row[0]} | {row[1]} |')
    hop2 = [ts(r['time']) for r in exd if r['hops'] != '1']
    if hop2:
        lines += ['', f'Explorer bundles via the relay arrived between {clock(min(hop2))} and {clock(max(hop2))}.']
    rl_states = [f.get('rl') for _, f in status.get(rl, []) if f.get('rl')]
    lines += ['', f'Relay states seen: {", ".join(sorted(set(rl_states))) or "none (relay never ran)"}.']
    nav = [(t, f.get('nav')) for t, f in status.get(rl, [])]
    stops = [clock(t) for (t, s), (_, p) in zip(nav[1:], nav) if s == 'no_radio' and p != 'no_radio']
    if stops:
        lines.append(f'Relay stopped by the radio deadman (nav=no_radio) at {", ".join(stops)}.')
    lines += ['', '## BS control versions not adopted', '',
              f'A rover still reporting an older control version (cv) more than {CONTROL_LAG_S:.0f} s after '
              'the BS published a new one. Air bytes are the logged JSON without the live fleet fields, '
              't and hop (~200 bytes more on air).', '',
              '| version | published | JSON bytes | not adopted by |', '|---|---|---|---|']
    for t, v, nb in controls:
        who = [n for n in (ex, rl) if any(sv == v for _, _, sv in loss.get(n, []))]
        lines.append(f'| {v} | {clock(t)} | {nb} | {", ".join(who) or "-"} |')
    with open(os.path.join(out_dir, 'relay.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'{a.name}: relay.png, relay.md -> {out_dir}')


if __name__ == '__main__':
    main()
