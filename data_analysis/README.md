# data_analysis

Real experiment runs and their analysis. Bench and test sessions stay in `bs_reports/` (git-ignored);
a session is copied here only when it is a run worth keeping.

## Layout

```
raw/<session folder>/      copy of the BS dashboard session folder from bs_reports/ (contents unmodified;
                           the folder may be renamed to the run name, e.g. ..._reconnect -> ..._double_rover1)
terrain/                   the site files and radio maps the runs used (copy of ../terrain, only what is needed)
<run name>/                figures + summary.md / summary.json for one run
<run name>/<node>/         the same per rover for a multi-rover run; <run name>/ then holds overview.png
                           (both paths, RSSI and link/driving state by clock time) and a summary.md comparing them
offset/                    predicted vs measured offset across runs (offset.md, offset.json, offset_analysis.png)
analyze_session.py         per-run figures and summary
compare_pred_meas.py       side-by-side maps and the offset analysis
run_all.py                 the list of runs; regenerates everything above from raw/
```

Regenerate everything (repo root, `..\.venv` Python):

```
..\.venv\Scripts\python data_analysis\run_all.py
```

## Runs

All at the `fountain` site with the fused pose (wheel distance + T265 heading) and routes driven by
`rover_nav.py`. The 2026-10-15 runs used rover B only (node 57936, MuSHR rover 1 `.21`) at 0.5 m/s;
double_rover1 added rover C (node 54272, MuSHR rover 2 `.22`). Every summary.md starts with the date
and local clock time of the analysed span, and the figure titles carry it too.

| Run | Session folder | BS at (map m, cell col/row) | Window used | Data bundles | Notes |
|---|---|---|---|---|---|
| preshadow | `session_20261015_091320` | (+34.5, +9.5), 162/118 | 09:14:20-09:35:41 | none (RSSI only) | Short test routes, then a 107 m loop. After 09:35 the BS was moved and the rover was carried with ROS stopped, so the rest of the session is unusable. Offset about 15 dB lower than shadow1/2, probably a different BS setup, so it is kept out of the pooled offset. |
| shadow1 | `session_20261015_094354_shadow1` | (+54.5, -20.5), 182/148 | whole session up to the BS move at 09:54:36 | 613 | 129 m route into the shadow south of the BS, then parked 6 m from the BS for ~200 s. `bs_position.json` in the folder is wrong (overwritten when the BS was moved for shadow2); the scripts find the BS cell from the logged predictions. |
| shadow2 | `session_20261015_095910_shadow2` | (+42.5, -58.5), 170/186 | whole session | 229 | 98 m route out and back. Latency never drops below ~1000 ms, probably a stale clock offset at the BS, so its absolute latency is not trustworthy. |
| double_rover1 | `session_20261021_171010_double_rover1` (was `..._reconnect` in bs_reports) | (+35.5, +9.5), 163/118 | 17:15:45-17:26:26 on 2026-10-21 (from when both rovers were placed on the map) | none (RSSI + telemetry only) | Both rovers on the same ~108 m loop: 57936 sent 17:16:47 at 0.5 m/s, done 17:22:20; 54272 sent 17:17:45 at 0.75 m/s, reached 105 of 109 m and never reported done. Both rovers lost the link at the far side of the loop (~17:20, down to -93 dBm). The session started at a BS serial reconnect, and rover 54272's daemon restarted twice at 17:11, before the window. Not in the pooled offset: the offset is ~-33 dB with slope ~1.7-2.1, close to preshadow (BS in about the same place) and far from shadow1/2. `rover_status_54272.csv` has a stale header (columns shifted), so the scripts read the rover status from `serial.log`. |

## Adding a run

1. Copy the session folder from `bs_reports/` into `raw/`.
2. Copy any radio maps it used into `terrain/` (`<site>_rm58_nn@<col>_<row>.npy` and
   `<site>_rm58_nn_pmnet@<col>_<row>.npy` for the BS cell, from `../terrain`).
3. Add a line to `RUNS` in `run_all.py` (name, folder, optional `HH:MM:SS-HH:MM:SS` window, whether
   it belongs in the pooled offset, and the rover node ids for a multi-rover run) and a row to the
   table above.
4. Run `run_all.py`.

## Method notes

- Measured RSSI is the BS's RSSI of the rover's beacons (`@NET`), one per received beacon. The RSSI
  in `coverage.csv` is the rover's last reading of the BS and stays frozen while the link is down,
  so it is not used.
- Rover positions come from `coverage.csv` (map x/y with the live alignment). They are placed at the
  time the rover generated them (BS arrival minus clock-corrected latency), not when they arrived.
- Predictions are the radio maps for the run's BS cell. `avg` is the mean of `nn_pmnet` and `nn` in dB.
- The maps are not in dBm, so each model gets a constant offset: the median of measured minus
  predicted. In `offset/` each 1 m cell counts once, so parked periods don't dominate. The 95%
  interval is a block bootstrap over runs of 10 consecutive cells.
