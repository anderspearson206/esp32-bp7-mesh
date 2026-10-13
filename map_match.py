"""
Live map-matching of rover odometry against a site's drivable paths (particle filter).

Wheel odometry drifts in heading (tight turns, reversing, wheel spin), but the rovers drive
on the paths of the site map.  Each LiveMatcher keeps a cloud of particles; every particle is
a guess of the rover's map position plus the rotation between its odometry frame and the map.
Each @LOCATION sample moves them by the odometry step (with noise that grows on sharp turns
and on implausible jumps = wheel spin) and weights them by how far they are off the drivable
mask.  While the rover is parked near the BS ("home") it is pulled back to home, and the
heading is opened up again when it sets off (it may have been turned by hand).

Seeded from the dashboard alignment ("Place on map"), so the user's placement is the prior.
The current estimate is causal (what is known now); the trail behind it is re-drawn from the
best particle's ancestry over the last LAG samples, so junction mix-ups fix themselves on the
map a few metres later.  fix_track.py --model pf does the same offline over a whole session.
"""

import math
from typing import Optional

import numpy as np

STOP_STEP_M = 0.05        # odometry step below this = not moving

PARAMS = dict(
    n=2000,               # particles
    dist_noise=0.07,      # odometry distance error (fraction)
    head_per_m=0.6,       # heading random walk [deg / sqrt(m)] while driving
    head_per_turn=0.12,   # extra heading noise per unit of odometry turn in one step
    spike_factor=2.5,     # step > this x the typical step = wheel spin
    spike_head=12.0,      # extra heading noise on a spike step [deg]
    sigma=1.5,            # drivable likelihood width [m] (distance outside the mask)
    floor=0.03,           # likelihood floor so off-path stretches don't kill the filter
    home_radius=8.0,      # parked within this of home counts as "at the BS" [m]
    home_sigma=2.5,       # position fix while parked at the BS [m]
    park_samples=3,       # samples without moving before it counts as parked
    depart_spread=20.0,   # heading re-opened once per stop at the BS, on setting off [+- deg]
    reset_jump=15.0,      # odometry step this big in one sample = odometry restarted [m]
    lag=180,              # samples of trail re-drawn from the best ancestry (~a leg)
)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def drivable_distance(site) -> Optional[np.ndarray]:
    """Distance [m] from each cell to the nearest drivable cell (0 on the mask), cached on
    the site.  scipy if available, else a chamfer transform (within a few %)."""
    if getattr(site, '_drv_dist', None) is not None or site.drivable is None:
        return getattr(site, '_drv_dist', None)
    try:
        from scipy.ndimage import distance_transform_edt
        d = distance_transform_edt(~site.drivable)
    except ImportError:
        # relax d = min(d, neighbour + step) over the 8 neighbours until nothing changes
        d = np.where(site.drivable, 0.0, np.inf)
        steps = [(dr, dc, math.hypot(dr, dc)) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
        while True:
            prev = d.copy()
            for dr, dc, cost in steps:
                sh = np.full_like(d, np.inf)
                sh[max(dr, 0):d.shape[0] + min(dr, 0), max(dc, 0):d.shape[1] + min(dc, 0)] =                     d[max(-dr, 0):d.shape[0] + min(-dr, 0), max(-dc, 0):d.shape[1] + min(-dc, 0)]
                np.minimum(d, sh + cost, out=d)
            if np.array_equal(d, prev):
                break
    site._drv_dist = (d * site.cell).astype(np.float32)
    return site._drv_dist


class LiveMatcher:
    def __init__(self, site, home, seed: Optional[int] = None, params: Optional[dict] = None):
        self.site = site
        self.P = dict(PARAMS, **(params or {}))
        self.home = tuple(home)
        self.dmap = drivable_distance(site)
        self.rng = np.random.default_rng(seed)
        self.N = int(self.P['n'])
        self.px = self.py = self.th = self.w = None
        self.last = None                  # last odometry (x, y, yaw_rad or None)
        self.steps = []                   # recent moving step lengths (typical speed)
        self.parked = 0
        self.at_home = False
        self.reopened = True              # heading already re-opened for this stop at the BS
        self.trail = []                   # [t, x, y, heading_deg, spread_m, rssi]
        self.hist = []                    # last LAG (px, py, parent) for the trail
        self.est = None

    @property
    def seeded(self) -> bool:
        return self.px is not None

    def seed(self, mx, my, rot, spread_deg, odom, pos_spread=0.5) -> None:
        """Rover is at map (mx, my) with odometry -> map rotation `rot` [rad] (+- spread)."""
        N = self.N
        self.px = mx + self.rng.normal(0, pos_spread, N)
        self.py = my + self.rng.normal(0, pos_spread, N)
        self.th = rot + np.radians(self.rng.uniform(-spread_deg, spread_deg, N))
        self.w = np.full(N, 1.0 / N)
        self.last = odom
        self.hist = []
        self.parked = 0

    def snap_home(self) -> None:
        """'Rover is parked at the BS now': position to home, headings kept."""
        if not self.seeded:
            return
        self.px = self.home[0] + self.rng.normal(0, 0.5, self.N)
        self.py = self.home[1] + self.rng.normal(0, 0.5, self.N)
        self.at_home = True
        self.parked = self.P['park_samples']
        self.hist = []

    def update(self, t, ox, oy, oyaw_deg, rssi=None) -> Optional[dict]:
        if not self.seeded:
            return None
        P, N, rng = self.P, self.N, self.rng
        yaw = math.radians(oyaw_deg) if oyaw_deg is not None else None
        lx, ly, lyaw = self.last
        dx, dy = ox - lx, oy - ly
        d = math.hypot(dx, dy)
        if d > P['reset_jump']:
            # odometry restarted (new frame, usually at 0,0 facing yaw 0): keep the map pose,
            # carry the heading over into the new frame
            if yaw is not None and lyaw is not None:
                self.th = self.th + (lyaw - yaw)
            dx = dy = d = 0.0
        self.last = (ox, oy, yaw)

        moving = d > STOP_STEP_M
        if moving:
            if self.at_home and not self.reopened:
                # setting off from the BS: it may have been turned by hand while parked
                self.th = self.th + np.radians(rng.uniform(-P['depart_spread'], P['depart_spread'], N))
                self.reopened = True
            self.parked = 0
            typical = float(np.median(self.steps)) if len(self.steps) >= 5 else d
            self.steps = (self.steps + [d])[-60:]
            dyaw = abs(wrap(yaw - lyaw)) if yaw is not None and lyaw is not None else 0.0
            sig = math.radians(P['head_per_m']) * math.sqrt(d) + P['head_per_turn'] * dyaw
            scale = 1 + rng.normal(0, P['dist_noise'], N)
            if d > P['spike_factor'] * typical:
                sig += math.radians(P['spike_head'])
                scale = rng.uniform(0.15, 1.0, N)
            self.th = self.th + rng.normal(0, sig, N)
            c, s = np.cos(self.th), np.sin(self.th)
            self.px = self.px + scale * (c * dx - s * dy)
            self.py = self.py + scale * (s * dx + c * dy)
            if self.dmap is not None:
                dist = np.nan_to_num(self.site.sample(self.dmap, self.px, self.py), nan=50.0)
                self.w = self.w * (P['floor'] + np.exp(-(dist / P['sigma']) ** 2))
        else:
            self.parked += 1
        if self.at_home and moving:
            mx, my = float(np.sum(self.w * self.px)), float(np.sum(self.w * self.py))
            if math.hypot(mx - self.home[0], my - self.home[1]) > P['home_radius']:
                self.at_home = False
        if self.parked >= P['park_samples']:
            mx, my = float(np.sum(self.w * self.px)), float(np.sum(self.w * self.py))
            if math.hypot(mx - self.home[0], my - self.home[1]) < P['home_radius']:
                if not self.at_home:
                    self.reopened = False
                self.at_home = True
            if self.at_home:
                self.w = self.w * np.exp(-((self.px - self.home[0]) ** 2 + (self.py - self.home[1]) ** 2)
                                         / (2 * P['home_sigma'] ** 2))

        tot = self.w.sum()
        self.w = self.w / tot if tot > 0 and np.isfinite(tot) else np.full(N, 1.0 / N)
        ess = 1.0 / float(np.sum(self.w ** 2))
        # estimate before resampling
        mx, my = float(np.sum(self.w * self.px)), float(np.sum(self.w * self.py))
        spread = math.sqrt(float(np.sum(self.w * ((self.px - mx) ** 2 + (self.py - my) ** 2))))
        heading = None
        if yaw is not None:
            h = self.th + yaw
            heading = math.degrees(math.atan2(float(np.sum(self.w * np.sin(h))), float(np.sum(self.w * np.cos(h)))))
        rot = math.atan2(float(np.sum(self.w * np.sin(self.th))), float(np.sum(self.w * np.cos(self.th))))
        parent = np.arange(N, dtype=np.int32)
        if ess < N / 2:                                    # systematic resampling
            cs = np.cumsum(self.w)
            cs[-1] = 1.0
            parent = np.searchsorted(cs, (rng.random() + np.arange(N)) / N).astype(np.int32)
            self.px, self.py, self.th = self.px[parent], self.py[parent], self.th[parent]
            self.w = np.full(N, 1.0 / N)
        k = int(np.argmin((self.px - mx) ** 2 + (self.py - my) ** 2))   # particle nearest the estimate
        self.est = {'x': mx, 'y': my, 'heading': heading, 'rot_deg': math.degrees(rot),
                    'spread': spread, 'ess': ess, 'at_home': self.at_home, 't': t}

        # trail: append now, then re-draw the last LAG points from particle k's ancestry
        self.trail.append([t, float(self.px[k]), float(self.py[k]), heading, spread, rssi])
        self.hist.append((self.px.astype(np.float32), self.py.astype(np.float32), parent))
        self.hist = self.hist[-P['lag']:]
        j = k
        for back, (hx, hy, par) in enumerate(reversed(self.hist)):
            idx = len(self.trail) - 1 - back
            if idx < 0:
                break
            self.trail[idx][1], self.trail[idx][2] = float(hx[j]), float(hy[j])
            j = int(par[j])
        if len(self.trail) > 20000:
            self.trail = self.trail[-20000:]
        return self.est
