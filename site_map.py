"""
Terrain site maps (the ../terrain directory) and the rover odometry -> map alignment.

A site is a square grid (fountain, range, ...) described by <site>_meta.json:
    256 x 256 cells of 1 m, row 0 = north, col 0 = west.
Map coordinates used everywhere here: x = east, y = north, metres from the grid centre,
so pixel (row, col) has its centre at x = col + 0.5 - half, y = half - row - 0.5.

Files per site (all optional except meta):
    <site>_terrain.npy      elevation relative to the site minimum [m]
    <site>_drivable.npy     bool drivable mask
    <site>_tx.npy           1 at the transmitter (BS) cell
    <site>_rm58_<model>.npy predicted radio map [dB]  (nn, nn_pmnet, sionna_quick, ...)
                            for the BS at the tx cell
    <site>_rm58_<model>@<col>_<row>.npy   the same for a BS moved to another cell
                            (written by rm_generate.py; see Site.set_bs)
    <site>_sat.png/.jpg     north-up image covering exactly the same square (optional)

Alignment: each rover's odometry frame starts wherever the T265 started, so per rover
    map = R(theta) * odom + (x, y),   map_heading = odom_yaw + theta
"""

import glob
import json
import math
import os
from typing import Dict, Optional, Tuple

import numpy as np


GENERATED_MODELS = ('nn_pmnet', 'nn')     # radio maps rm_generate.py can make for any BS cell


def find_terrain_dir(explicit: Optional[str] = None) -> Optional[str]:
    here = os.path.dirname(os.path.abspath(__file__))
    for d in ([explicit] if explicit else []) + [os.path.join(here, 'terrain'),
                                                 os.path.join(here, '..', 'terrain')]:
        if d and os.path.isdir(d):
            return os.path.normpath(d)
    return None


def list_sites(terrain_dir: str):
    return sorted(os.path.basename(p)[:-len('_meta.json')]
                  for p in glob.glob(os.path.join(terrain_dir, '*_meta.json')))


class Site:
    def __init__(self, terrain_dir: str, name: str):
        self.name = name
        self.dir = terrain_dir
        with open(self._path('meta.json')) as f:
            self.meta = json.load(f)
        self.size = int(self.meta.get('size_px', 256))
        self.cell = float(self.meta.get('cell_m', 1.0))
        self.half = self.size * self.cell / 2.0
        self.terrain = self._load('terrain.npy')
        drv = self._load('drivable.npy')
        self.drivable = drv.astype(bool) if drv is not None else None
        tx = self._load('tx.npy')
        self.tx_xy = None
        self.base_rc = None                   # (row, col) of the BS the site's own maps are for
        if tx is not None and tx.any():
            r, c = np.argwhere(tx)[0]
            self.base_rc = (int(r), int(c))
            self.tx_xy = self.rc_to_xy(int(r), int(c))
        # radio_maps: model -> map for the current BS position, None if there is none (yet).
        # The keys never change, so CSV columns stay fixed when the BS moves.
        self.radio_maps: Dict[str, Optional[np.ndarray]] = {}
        prefix = f"{name}_rm58_"
        for p in sorted(glob.glob(os.path.join(terrain_dir, prefix + '*.npy'))):
            if '@' in os.path.basename(p):
                continue
            arr = np.load(p).astype(np.float32)
            if arr.shape == (self.size, self.size):
                self.radio_maps[os.path.basename(p)[len(prefix):-4]] = arr
        for m in GENERATED_MODELS:
            self.radio_maps.setdefault(m, None)
        self.base_maps = dict(self.radio_maps)
        self.bs_rc = self.base_rc
        self.sat_path = next((p for ext in ('png', 'jpg', 'jpeg', 'tif')
                              for p in [self._path(f'sat.{ext}')] if os.path.exists(p)), None)

    # BS position and the radio maps that go with it
    @property
    def bs_moved(self) -> bool:
        return self.bs_rc != self.base_rc

    def bs_map_path(self, model: str, c: int, r: int) -> str:
        return self._path(f"rm58_{model}@{c}_{r}.npy")

    def set_bs(self, x: float, y: float):
        """Move the BS to the cell containing map point (x, y). The site's own maps are used
        at its tx cell; elsewhere the generated nn / nn_pmnet maps if they exist (others None).
        Returns (row, col, [models still missing]); row -1 if outside the site."""
        r, c = (int(v) for v in self.xy_to_rc(x, y))
        if r < 0:
            return -1, -1, []
        self.bs_rc = (r, c)
        self.tx_xy = self.rc_to_xy(r, c)
        if not self.bs_moved:
            self.radio_maps = dict(self.base_maps)
            return r, c, []
        self.radio_maps = {m: None for m in self.base_maps}
        return (r, c, self.reload_bs_maps())

    def reload_bs_maps(self):
        """Load generated maps for the current BS cell that have appeared; returns missing models."""
        if not self.bs_moved:
            return []
        r, c = self.bs_rc
        missing = []
        for m in GENERATED_MODELS:
            p = self.bs_map_path(m, c, r)
            if os.path.exists(p):
                if self.radio_maps.get(m) is None:
                    arr = np.load(p).astype(np.float32)
                    if arr.shape == (self.size, self.size):
                        self.radio_maps[m] = arr
            else:
                missing.append(m)
        return missing

    def _path(self, suffix: str) -> str:
        return os.path.join(self.dir, f"{self.name}_{suffix}")

    def _load(self, suffix: str):
        p = self._path(suffix)
        if not os.path.exists(p):
            return None
        a = np.load(p)
        return a if a.shape == (self.size, self.size) else None

    def rc_to_xy(self, r: int, c: int) -> Tuple[float, float]:
        return ((c + 0.5) * self.cell - self.half, self.half - (r + 0.5) * self.cell)

    def xy_to_rc(self, x, y):
        """Arrays in, arrays out; cells outside the grid get -1."""
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        c = np.floor((x + self.half) / self.cell).astype(int)
        r = np.floor((self.half - y) / self.cell).astype(int)
        bad = (r < 0) | (r >= self.size) | (c < 0) | (c >= self.size)
        return np.where(bad, -1, r), np.where(bad, -1, c)

    def sample(self, arr: Optional[np.ndarray], x, y):
        """Value of a grid at map points (NaN outside the grid or without that layer)."""
        x = np.asarray(x, dtype=float)
        if arr is None:
            return np.full(x.shape, np.nan)
        r, c = self.xy_to_rc(x, y)
        ok = r >= 0
        out = np.full(x.shape, np.nan)
        out[ok] = arr[r[ok], c[ok]]
        return out

    def on_drivable(self, x, y, buffer_m: float = 0.0):
        """True where a map point is on (or within buffer_m of) the drivable mask."""
        if self.drivable is None:
            return np.full(np.asarray(x).shape, False)
        if buffer_m <= 0:
            v = self.sample(self.drivable, x, y)
            return np.nan_to_num(v, nan=0) > 0.5
        k = int(math.ceil(buffer_m / self.cell))
        hit = np.zeros(np.asarray(x).shape, dtype=bool)
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                if dr * dr + dc * dc <= k * k:
                    v = self.sample(self.drivable, np.asarray(x) + dc * self.cell,
                                    np.asarray(y) + dr * self.cell)
                    hit |= np.nan_to_num(v, nan=0) > 0.5
        return hit

    def describe(self) -> str:
        parts = [f"{self.name}: {self.size * self.cell:.0f} m square"]
        if self.meta.get('crs'):
            parts.append(f"{self.meta['crs']} NW corner E{self.meta.get('west_E')} N{self.meta.get('north_N')}")
        if self.tx_xy:
            parts.append(f"BS/tx at ({self.tx_xy[0]:+.1f}, {self.tx_xy[1]:+.1f}) m"
                         + (" (moved)" if self.bs_moved else ""))
        return ", ".join(parts)


# alignment (odometry -> map), per rover

IDENTITY = {'x': 0.0, 'y': 0.0, 'theta': 0.0}


def to_map(al: dict, ox, oy, oyaw=None):
    """Apply one rover's alignment. Works on scalars or numpy arrays; yaw in degrees."""
    th = math.radians(al['theta'])
    c, s = math.cos(th), math.sin(th)
    ox, oy = np.asarray(ox, dtype=float), np.asarray(oy, dtype=float)
    mx = al['x'] + c * ox - s * oy
    my = al['y'] + s * ox + c * oy
    myaw = None if oyaw is None else (np.asarray(oyaw, dtype=float) + al['theta'] + 180.0) % 360.0 - 180.0
    return mx, my, myaw


def solve_alignment(ox: float, oy: float, oyaw: Optional[float],
                    mx: float, my: float, mheading: Optional[float], keep_theta: float) -> dict:
    """Alignment that puts odometry pose (ox, oy, oyaw) at map pose (mx, my, mheading).
    Without a heading (plain click, or no yaw from the rover) the current rotation is kept."""
    theta = keep_theta
    if mheading is not None and oyaw is not None:
        theta = (mheading - oyaw + 180.0) % 360.0 - 180.0
    th = math.radians(theta)
    c, s = math.cos(th), math.sin(th)
    return {'x': mx - (c * ox - s * oy), 'y': my - (s * ox + c * oy), 'theta': theta}


class AlignmentStore:
    """Per-site, per-rover alignments kept in <dir>/alignment_<site>.json."""

    def __init__(self, directory: str, site: str):
        self.path = os.path.join(directory, f"alignment_{site or 'nosite'}.json")
        self.data: Dict[str, dict] = {}
        try:
            with open(self.path) as f:
                self.data = {k: dict(IDENTITY, **v) for k, v in json.load(f).items()}
        except (OSError, ValueError):
            pass

    def get(self, node: str) -> dict:
        return dict(self.data.get(node, IDENTITY))

    def set(self, node: str, al: dict) -> None:
        self.data[node] = {k: float(al.get(k, IDENTITY[k])) for k in IDENTITY}
        if 'anchor' in al:
            self.data[node]['anchor'] = [float(v) for v in al['anchor']]
        try:
            os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
            with open(self.path, 'w') as f:
                json.dump(self.data, f, indent=1)
        except OSError:
            pass
