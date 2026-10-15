"""
Route planning on a site's drivable mask, for sending a rover along waypoints (rover_nav.py).

A* over the 8-connected drivable cells. Steps close to the edge of the drivable area cost
more, so routes keep to the middle of paths instead of cutting corners over kerbs. The cell
path is then simplified (Douglas-Peucker) so it fits in a control message: the rover follows
straight segments between the points, so the tolerance is how far the simplified route may
leave the A* path.

    python route_plan.py --site fountain 34,12 -30,0 34,12      # plan and plot through points
"""

import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

EDGE_CLEAR_M = 3.0      # steps closer than this to the drivable edge cost extra
EDGE_PENALTY = 4.0      # extra cost per step right at the edge (scaled down to 0 at EDGE_CLEAR_M)
SMOOTH_CELLS = 5         # moving-average window over the A* cell path
SNAP_MAX_M = 6.0        # waypoints this far off the drivable area are moved onto it, else refused

_MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
          (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def clearance(drivable: np.ndarray, max_cells: int) -> np.ndarray:
    """Cells to the nearest non-drivable cell (8-neighbour erosion count, capped at max_cells)."""
    clr = np.zeros(drivable.shape, np.int32)
    cur = drivable.copy()
    for k in range(1, max_cells + 1):
        p = np.pad(cur, 1, constant_values=False)
        er = cur.copy()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                er &= p[1 + dr:1 + dr + cur.shape[0], 1 + dc:1 + dc + cur.shape[1]]
        clr[cur] = k
        cur = er
        if not cur.any():
            break
    return clr


class Planner:
    def __init__(self, site):
        if site is None or site.drivable is None:
            raise ValueError("route planning needs a site with a drivable mask")
        self.site = site
        self.drv = site.drivable
        cells = max(1, int(round(EDGE_CLEAR_M / site.cell)))
        clr = clearance(self.drv, cells)
        self.cost = 1.0 + EDGE_PENALTY * np.clip(1.0 - (clr - 1) / float(cells), 0.0, 1.0)

    def snap(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """Drivable cell nearest to map point (x, y), or None if none within SNAP_MAX_M."""
        r, c = (int(v) for v in self.site.xy_to_rc(x, y))
        k = int(math.ceil(SNAP_MAX_M / self.site.cell))
        n = self.drv.shape[0]
        if r < 0:
            r, c = (int(np.clip(v, 0, n - 1)) for v in (r, c))
        r0, r1, c0, c1 = max(0, r - k), min(n, r + k + 1), max(0, c - k), min(n, c + k + 1)
        rr, cc = np.nonzero(self.drv[r0:r1, c0:c1])
        if rr.size == 0:
            return None
        d = (rr + r0 - r) ** 2 + (cc + c0 - c) ** 2
        i = int(np.argmin(d))
        if d[i] * self.site.cell ** 2 > SNAP_MAX_M ** 2:
            return None
        return int(rr[i] + r0), int(cc[i] + c0)

    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        drv, cost = self.drv, self.cost
        n_r, n_c = drv.shape
        gr, gc = goal

        def h(r, c):
            return math.hypot(r - gr, c - gc)

        g = {start: 0.0}
        came = {}
        heap = [(h(*start), 0.0, start)]
        while heap:
            _, gs, node = heapq.heappop(heap)
            if node == goal:
                path = [node]
                while node in came:
                    node = came[node]
                    path.append(node)
                return path[::-1]
            if gs > g.get(node, math.inf):
                continue
            r, c = node
            for dr, dc, step in _MOVES:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < n_r and 0 <= nc < n_c) or not drv[nr, nc]:
                    continue
                if dr and dc and not (drv[r, nc] and drv[nr, c]):
                    continue                      # no diagonal squeeze past a corner
                ng = gs + step * 0.5 * (cost[r, c] + cost[nr, nc])
                if ng < g.get((nr, nc), math.inf):
                    g[(nr, nc)] = ng
                    came[(nr, nc)] = node
                    heapq.heappush(heap, (ng + h(nr, nc), ng, (nr, nc)))
        return None

    def plan(self, start_xy: Sequence[float], waypoints: Sequence[Sequence[float]]):
        """Route from start_xy through the waypoints in order (map metres).
        Returns (points Nx2 along the cell centres, error string or '')."""
        stops = [tuple(start_xy)] + [tuple(w) for w in waypoints]
        cells = []
        for i, (x, y) in enumerate(stops):
            rc = self.snap(x, y)
            if rc is None:
                what = "the rover's position" if i == 0 else f"waypoint {i}"
                return None, f"{what} is more than {SNAP_MAX_M:.0f} m from the drivable area"
            cells.append(rc)
        path = [cells[0]]
        for i in range(1, len(cells)):
            leg = self.astar(cells[i - 1], cells[i])
            if leg is None:
                return None, f"no drivable path to waypoint {i}"
            path += leg[1:]
        pts = np.array([self.site.rc_to_xy(r, c) for r, c in path], float)
        # start exactly at the rover, so the first segment leads from where it is
        pts = np.vstack([np.asarray(start_xy, float)[None, :], pts[1:]]) if len(pts) > 1 else pts
        return smooth(pts), ''


def smooth(pts: np.ndarray, k: int = SMOOTH_CELLS) -> np.ndarray:
    """Moving average over k cells to take out the 8-connected staircase; ends stay put."""
    if len(pts) <= k:
        return pts
    pad = np.vstack([np.repeat(pts[:1], k // 2, axis=0), pts, np.repeat(pts[-1:], k // 2, axis=0)])
    ker = np.ones(k) / k
    out = np.c_[np.convolve(pad[:, 0], ker, 'valid'), np.convolve(pad[:, 1], ker, 'valid')]
    out[0], out[-1] = pts[0], pts[-1]
    return out


def simplify(pts: np.ndarray, tol: float) -> np.ndarray:
    """Douglas-Peucker: drop points while the polyline stays within tol of the original."""
    if len(pts) < 3:
        return pts
    keep = np.zeros(len(pts), bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        if b <= a + 1:
            continue
        p, q = pts[a], pts[b]
        seg = q - p
        L = float(np.hypot(*seg))
        mid = pts[a + 1:b]
        if L < 1e-9:
            d = np.hypot(*(mid - p).T)
        else:
            t = np.clip(((mid - p) @ seg) / (L * L), 0.0, 1.0)
            d = np.hypot(*(mid - (p + t[:, None] * seg)).T)
        i = int(np.argmax(d))
        if d[i] > tol:
            k = a + 1 + i
            keep[k] = True
            stack += [(a, k), (k, b)]
    return pts[keep]


def to_odom(al: dict, pts: np.ndarray) -> np.ndarray:
    """Map points -> a rover's odometry frame (inverse of site_map.to_map)."""
    th = math.radians(al['theta'])
    c, s = math.cos(th), math.sin(th)
    dx, dy = pts[:, 0] - al['x'], pts[:, 1] - al['y']
    return np.c_[c * dx + s * dy, -s * dx + c * dy]


def encode(pts_odom: np.ndarray) -> List[int]:
    """Flat [x0, y0, x1, y1, ...] in decimetres: compact JSON for the control message."""
    return [int(round(v * 10)) for v in pts_odom.ravel()]


def length(pts: np.ndarray) -> float:
    return float(np.sum(np.hypot(*np.diff(pts, axis=0).T))) if len(pts) > 1 else 0.0


def main():
    import argparse
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import site_map

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--site', required=True)
    ap.add_argument('--terrain-dir')
    ap.add_argument('--tol', type=float, default=0.3, help='simplification tolerance [m]')
    ap.add_argument('--out', default='route_plan.png')
    ap.add_argument('points', nargs='+', help='x,y map points: start, then waypoints')
    args = ap.parse_args()
    site = site_map.Site(site_map.find_terrain_dir(args.terrain_dir), args.site)
    xy = [tuple(float(v) for v in p.split(',')) for p in args.points]
    pl = Planner(site)
    pts, err = pl.plan(xy[0], xy[1:])
    if pts is None:
        raise SystemExit(err)
    simple = simplify(pts, args.tol)
    print(f"route {length(pts):.0f} m, {len(pts)} cells -> {len(simple)} points at {args.tol} m tolerance")
    h = site.half
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(pl.cost, extent=[-h, h, -h, h], cmap='Blues_r', vmin=1, vmax=1 + EDGE_PENALTY)
    ax.imshow(np.ma.masked_where(site.drivable, ~site.drivable), extent=[-h, h, -h, h], cmap='gray', alpha=0.6)
    ax.plot(pts[:, 0], pts[:, 1], 'y-', lw=1)
    ax.plot(simple[:, 0], simple[:, 1], 'r.-', ms=6, lw=1.2)
    ax.plot(*np.array(xy).T, 'ko', mfc='none', ms=10)
    span = np.ptp(pts, axis=0).max() / 2 + 15
    cx, cy = pts.mean(axis=0)
    ax.set_xlim(cx - span, cx + span)
    ax.set_ylim(cy - span, cy + span)
    fig.savefig(args.out, dpi=80, bbox_inches='tight')
    print('wrote', args.out)


if __name__ == '__main__':
    main()
