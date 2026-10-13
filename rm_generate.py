"""
rm_generate.py  --  Predicted 5.8 GHz radio maps for a BS at any cell of a terrain site.

Runs the RadioLunaDiff pipeline from lunar-mesh-env (K2-UNet -> PMNet -> diffusion residual)
on <site>_terrain.npy with the transmitter at the given map position, and writes

    <terrain>/<site>_rm58_nn_pmnet@<col>_<row>.npy    PMNet stage (deterministic, ~1 s on CPU)
    <terrain>/<site>_rm58_nn@<col>_<row>.npy          full pipeline (~15 s on CPU)

next to the site's own maps (site_map.Site.set_bs picks them up). Existing files are reused
unless --force. Needs the lunar-mesh-env Python (torch, diffusers), so run it with that venv:

    ~/Documents/lunar-mesh-env/.venv/Scripts/python.exe rm_generate.py --site fountain --bs 10,-20

bs_core starts it in the background when the BS is moved on the dashboard. Progress lines
on stdout:  "READY <model> <path>"  per finished map, "ERROR <text>" on failure.
"""

import argparse
import os
import sys
import time

import numpy as np

import site_map

DEFAULT_ENV = os.path.expanduser('~/Documents/lunar-mesh-env')
MODEL_SUBDIR = 'RadioLunaDiff/pretrained_models_network'


def env_python(rm_env: str) -> str:
    """Python of the lunar-mesh-env venv (Windows or Linux layout)."""
    for p in (os.path.join(rm_env, '.venv', 'Scripts', 'python.exe'),
              os.path.join(rm_env, '.venv', 'bin', 'python')):
        if os.path.exists(p):
            return p
    return ''


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--site', required=True)
    ap.add_argument('--terrain-dir')
    ap.add_argument('--bs', required=True, help='BS map position x,y [m] (east, north from the site centre)')
    ap.add_argument('--rm-env', default=DEFAULT_ENV, help=f'lunar-mesh-env checkout (default {DEFAULT_ENV})')
    ap.add_argument('--freq', default='5.8')
    ap.add_argument('--seed', type=int, default=0, help='diffusion noise seed')
    ap.add_argument('--force', action='store_true', help='regenerate even if the files exist')
    args = ap.parse_args()

    try:
        tdir = site_map.find_terrain_dir(args.terrain_dir)
        if tdir is None:
            raise RuntimeError('terrain directory not found (use --terrain-dir)')
        site = site_map.Site(tdir, args.site)
        x, y = (float(v) for v in args.bs.split(','))
        r, c = (int(v) for v in site.xy_to_rc(x, y))
        if r < 0:
            raise RuntimeError(f'({x}, {y}) is outside the {site.size * site.cell:.0f} m site')
        out = {m: site.bs_map_path(m, c, r) for m in ('nn_pmnet', 'nn')}
        todo = [m for m in out if args.force or not os.path.exists(out[m])]
        for m in out:
            if m not in todo:
                print(f"READY {m} {out[m]}", flush=True)
        if not todo:
            return

        sys.path.insert(0, args.rm_env)
        import torch
        from lunar_mesh_env.radio_model_nn import RadioMapModelNN
        md = os.path.join(args.rm_env, MODEL_SUBDIR)
        paths = {'k2_model': os.path.join(md, 'k2unet', 'best_k2_model.pth'),
                 'pmnet_model': os.path.join(md, 'pmnet', 'best_pm_model.pt'),
                 'diffusion_model': os.path.join(md, 'diffusion')}
        t0 = time.time()
        hm = np.load(site._path('terrain.npy'))
        n = hm.shape[0] - 1
        # the model puts the transmitter at pixel (x / width * (size - 1)): width = size - 1 => pixel = (col, row)
        model = RadioMapModelNN(paths, hm, float(n), float(n), device='cpu')
        print(f"LOADED {time.time() - t0:.1f}s", flush=True)
        tx = (float(c), float(r))
        torch.manual_seed(args.seed)

        def save(m, arr):
            tmp = out[m] + '.tmp.npy'
            np.save(tmp, arr.astype(np.float32))
            os.replace(tmp, out[m])
            print(f"READY {m} {out[m]}", flush=True)

        with torch.no_grad():
            if 'nn_pmnet' in todo:
                p = model.processor
                x_in = torch.stack([model.static_hm_tensor, p.create_tx_map(tx, n, n, model.device),
                                    p.create_freq_map(args.freq, model.device),
                                    model.static_fhm_tensor]).unsqueeze(0)
                k2 = torch.sigmoid(model.k2_model(x_in.clone()))
                pm = model.pmnet(torch.cat([x_in.clone(), k2], 1))
                pm01 = (model.normalize_transform_single(pm).clamp(-1, 1) + 1.0) / 2.0
                save('nn_pmnet', pm01.squeeze().cpu().numpy() * 210.0 - 200.0)
            if 'nn' in todo:
                save('nn', model.generate_map(tx, args.freq))
        print(f"DONE {time.time() - t0:.1f}s", flush=True)
    except Exception as e:                                   # reported to the dashboard
        print(f"ERROR {type(e).__name__}: {e}", flush=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
