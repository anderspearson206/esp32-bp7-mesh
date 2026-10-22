"""
rm_colab_bundle.py  --  Pack what rm_linktab_colab.ipynb needs for one site into a zip.

The notebook generates a predicted 5.8 GHz radio map with the transmitter at every drivable
cell of a site (on a Colab GPU) and keeps the RSS at every drivable cell: the relay link
table. It needs the RadioLunaDiff model code from lunar-mesh-env and the site's terrain
files, which this script puts in one zip:

    code/lunar_mesh_env/{radio_model_nn,inference_dataloader}.py   (+ an empty __init__.py,
                                                 so the gym/pettingzoo env isn't imported)
    code/RadioLunaDiff/k2net/diff_modules.py
    code/RadioLunaDiff/pmnet/models/pmnet_v3.py
    terrain/<site>_{meta.json,terrain.npy,drivable.npy,tx.npy,rm58_nn_pmnet.npy,rm58_nn.npy}

Upload the zip plus lunar-mesh-env's RadioLunaDiff/pretrained_models_network.zip (the
weights, same for every site) to one Google Drive folder, MyDrive/rm_linktab by default.

    python rm_colab_bundle.py --site fountain          -> rm_colab_fountain.zip
"""

import argparse
import os
import sys
import zipfile

import site_map

DEFAULT_ENV = os.path.expanduser('~/Documents/lunar-mesh-env')
CODE = ['lunar_mesh_env/radio_model_nn.py', 'lunar_mesh_env/inference_dataloader.py',
        'RadioLunaDiff/k2net/diff_modules.py', 'RadioLunaDiff/pmnet/models/pmnet_v3.py']
REQUIRED = ['meta.json', 'terrain.npy', 'drivable.npy']
OPTIONAL = ['tx.npy', 'rm58_nn_pmnet.npy', 'rm58_nn.npy']   # tx + its maps: the notebook's check


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--site', required=True)
    ap.add_argument('--terrain-dir')
    ap.add_argument('--rm-env', default=DEFAULT_ENV, help=f'lunar-mesh-env checkout (default {DEFAULT_ENV})')
    ap.add_argument('--out', help='zip to write (default rm_colab_<site>.zip)')
    args = ap.parse_args()

    tdir = site_map.find_terrain_dir(args.terrain_dir)
    if tdir is None:
        sys.exit('terrain directory not found (use --terrain-dir)')
    out = args.out or f'rm_colab_{args.site}.zip'
    with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED) as z:
        for rel in CODE:
            src = os.path.join(args.rm_env, rel)
            if not os.path.exists(src):
                sys.exit(f'missing {src} (use --rm-env)')
            z.write(src, 'code/' + rel)
        z.writestr('code/lunar_mesh_env/__init__.py', '')
        for suffix in REQUIRED + OPTIONAL:
            src = os.path.join(tdir, f'{args.site}_{suffix}')
            if os.path.exists(src):
                z.write(src, f'terrain/{args.site}_{suffix}')
            elif suffix in REQUIRED:
                sys.exit(f'missing {src}')
            else:
                print(f'note: no {src}, skipped')
    print(f'wrote {out} ({os.path.getsize(out) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
