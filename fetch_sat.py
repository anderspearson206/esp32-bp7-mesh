"""
Download a north-up aerial image for a terrain site, for the dashboard's "Satellite" background.

Writes <terrain>/<site>_sat.png covering exactly the site's square (from <site>_meta.json,
UTM), which site_map.Site picks up as sat_path. Source: USGS NAIP orthoimagery
(imagery.nationalmap.gov, public domain, ~0.6 m/pixel, US only).

    python fetch_sat.py fountain range
    python fetch_sat.py fountain --px 2048 --force
"""

import argparse
import json
import os
import re
import sys
import urllib.parse
import urllib.request

from site_map import find_terrain_dir

URL = 'https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPImagery/ImageServer/exportImage'


def fetch(terrain_dir, site, px, force):
    meta_path = os.path.join(terrain_dir, f'{site}_meta.json')
    out = os.path.join(terrain_dir, f'{site}_sat.png')
    if os.path.exists(out) and not force:
        print(f'{out} exists (use --force to replace)')
        return
    with open(meta_path) as f:
        meta = json.load(f)
    epsg = re.search(r'EPSG:(\d+)', meta['crs']).group(1)
    span = meta['size_px'] * meta['cell_m']
    w, n = meta['west_E'], meta['north_N']
    q = urllib.parse.urlencode({
        'bbox': f'{w},{n - span},{w + span},{n}', 'bboxSR': epsg, 'imageSR': epsg,
        'size': f'{px},{px}', 'format': 'png', 'interpolation': 'RSP_BilinearInterpolation',
        'f': 'image'})
    with urllib.request.urlopen(f'{URL}?{q}', timeout=120) as r:
        data, ctype = r.read(), r.headers.get('Content-Type', '')
    if not ctype.startswith('image/'):
        sys.exit(f'{site}: server returned {ctype}: {data[:200]!r}')
    with open(out, 'wb') as f:
        f.write(data)
    print(f'{out}: {px}x{px} px, {span / px:.2f} m/px, {len(data) // 1024} KB')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sites', nargs='+')
    ap.add_argument('--terrain', help='terrain directory (default: ./terrain or ../terrain)')
    ap.add_argument('--px', type=int, default=1024, help='image size in pixels (default 1024)')
    ap.add_argument('--force', action='store_true', help='replace an existing image')
    a = ap.parse_args()
    terrain_dir = find_terrain_dir(a.terrain)
    if not terrain_dir:
        sys.exit('no terrain directory found')
    for s in a.sites:
        fetch(terrain_dir, s, a.px, a.force)


if __name__ == '__main__':
    main()
