"""
run_all.py  --  Regenerate every figure and summary in data_analysis/ from the archived raw data.

RUNS is the list of real experiments. To add one: copy its session folder from bs_reports/ into
raw/, copy any radio maps it used (<site>_rm58_*@<col>_<row>.npy) into terrain/, and add a line.

    python data_analysis/run_all.py          (repo root, ..\\.venv Python)
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# name, session folder in raw/, time window ('HH:MM:SS-HH:MM:SS' or None), in the pooled offset,
# rover node ids (None = the session's only rover; several = one folder per rover + an overview)
RUNS = [
    ('preshadow', 'session_20261015_091320', '09:14:20-09:35:41', False, None),   # BS at (34.5, 9.5); no data bundles
    ('shadow1', 'session_20261015_094354_shadow1', None, True, None),             # BS at (54.5, -20.5)
    ('shadow2', 'session_20261015_095910_shadow2', None, True, None),             # BS at (42.5, -58.5)
    ('double_rover1', 'session_20261021_171010_double_rover1', '17:15:45-', False,
     ['57936', '54272']),                                                         # BS at (35.5, 9.5); no data bundles
]


def run(args):
    print('>', ' '.join(os.path.basename(a) if a.endswith('.py') else a for a in args))
    subprocess.run([sys.executable] + args, check=True)


def main():
    for name, folder, window, _, nodes in RUNS:
        args = [os.path.join(HERE, 'analyze_session.py'), os.path.join(HERE, 'raw', folder), '--name', name]
        run(args + (['--window', window] if window else []) + (['--node', ','.join(nodes)] if nodes else []))
    specs, pool = [], []
    for name, folder, window, pooled, nodes in RUNS:
        spec = os.path.join(HERE, 'raw', folder) + ':' + name + (f'@{window}' if window else '')
        for n in nodes or [None]:
            specs.append(spec + (f'#{n}' if n else ''))
            if pooled:
                pool.append(f'{name}/{n}' if n else name)
    pool = ','.join(pool)
    run([os.path.join(HERE, 'compare_pred_meas.py')] + specs + ['--pool', pool])


if __name__ == '__main__':
    main()
