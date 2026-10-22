"""
relay_brain.py - where the relay rover should be, between the BS and an explorer rover.

The relay rover (radio_daemon.py with --site, role set from the dashboard) keeps choosing a
spot on the drivable area that maximises the weaker of its two links, BS <-> relay and
relay <-> explorer, and drives there with rover_nav.py. Positions are map metres
(x east, y north from the site centre, see site_map.py); times are BS time in seconds.

Link models (LinkModel), chosen from the dashboard per relay:
    nn_pmnet  <site>_linktab_nn_pmnet.npz from rm_linktab_colab.ipynb: predicted RSS between
              every pair of drivable cells (both directions averaged). The default: PMNet is
              deterministic, so its two directions agree better than nn's
    nn        <site>_linktab_nn.npz, the same with the diffusion residual (noisier)
    distance  no table: log-distance path loss, so the best spot is roughly the midpoint,
              kept on the drivable area
The rover loads every <site>_linktab_*.npz in its terrain folder at startup (find_tables).

Behaviour (RelayBrain.step), every RELAY_TICK_S:
    The explorer's position is predicted PREDICT_S ahead from its recent fixes, so the relay
    starts moving before the link gets weak. With no target yet the relay stays where it is
    while its own spot is comfortable (weaker link >= comfort_dbm) and no spot is gain_db
    better. It retargets when the best spot beats the current target by gain_db, or by
    small_gain_db once the target has dropped below comfort_dbm. Retargets are at least
    min_shift_m apart and min_retarget_s apart, so it doesn't keep changing its mind.
    Explorer silent for stale_s: keep the current target. Silent for stale_s + ferry_s:
    drive to its last known position (ferry), until it is heard again.

    python relay_brain.py --site fountain --bs 44.5,-62.5 --explorer=-20,30   # one decision
"""

import glob
import math
import os
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

RELAY_TICK_S = 2.0
DEFAULT_MODEL = 'nn_pmnet'

DEFAULTS = {
    'comfort_dbm': -75.0,      # weaker link at or above this = fine where it is
    'gain_db': 6.0,            # a spot this much better is worth moving for anyway
    'small_gain_db': 2.0,      # enough to move once the current spot is below comfort
    'travel_db_per_m': 0.1,    # distance from the relay counts against a spot when choosing
    'min_shift_m': 3.0,        # don't retarget to a spot closer than this to the current target
    'min_retarget_s': 4.0,
    'arrive_m': 1.5,           # this close to the target = there
    'predict_s': 4.0,          # explorer position predicted this far ahead
    'max_ex_speed': 1.5,       # m/s cap on the explorer velocity estimate
    'stale_s': 10.0,           # explorer fix older than this = explorer lost
    'ferry_s': 15.0,           # lost this much longer = drive to its last known position
}

# fallback path loss: rss = P0 - 10 n log10(d)
DIST_P0_DBM = -40.0
DIST_N = 3.0


class LinkModel(object):
    """Predicted link quality between relay candidate cells and any map point.

    cand_xy (N, 2): candidate relay positions. link_to(xy) -> (N,) predicted RSS [dBm]
    between each candidate and point xy (snapped to the nearest drivable cell)."""

    def __init__(self, site, table_path: Optional[str] = None):
        self.site = site
        self.name = self.key = 'distance'
        self._cache = {}                       # snapped cell -> link vector (BS, mostly)
        if table_path:
            z = np.load(table_path)
            self.rss = z['rss']                                  # int8 [tx, rx]
            tx_rc, rx_rc = z['tx_rc'].astype(int), z['rx_rc'].astype(int)
            idx = z['idx']
            self.rx_of_tx = idx[tx_rc[:, 0], tx_rc[:, 1]].astype(int)
            if (self.rx_of_tx < 0).any():
                raise ValueError(f"{table_path}: transmitter cells must be drivable cells")
            self.cand_rc, self.rx_rc = tx_rc, rx_rc
            self.name = 'table:' + os.path.basename(table_path)
            self.key = table_key(table_path)
        else:
            if site.drivable is None:
                raise ValueError("the distance model needs a site with a drivable mask")
            self.rss = None
            self.cand_rc = self.rx_rc = np.argwhere(site.drivable)
        self.cand_xy = self._rc_xy(self.cand_rc)
        self.rx_xy = self.cand_xy if self.rx_rc is self.cand_rc else self._rc_xy(self.rx_rc)

    def _rc_xy(self, rc):
        s = self.site
        return np.stack([(rc[:, 1] + 0.5) * s.cell - s.half, s.half - (rc[:, 0] + 0.5) * s.cell], 1)

    def nearest_cand(self, xy) -> int:
        d = self.cand_xy - np.asarray(xy, float)
        return int(np.argmin((d * d).sum(1)))

    def nearest_rx(self, xy) -> int:
        d = self.rx_xy - np.asarray(xy, float)
        return int(np.argmin((d * d).sum(1)))

    def link_to(self, xy, cache: bool = False) -> np.ndarray:
        j = self.nearest_rx(xy)
        if cache and j in self._cache:
            return self._cache[j]
        if self.rss is None:
            d = np.hypot(*(self.cand_xy - self.rx_xy[j]).T)
            out = DIST_P0_DBM - 10.0 * DIST_N * np.log10(np.maximum(d, 1.0))
        else:
            # the model predicts each direction on its own: average them
            i = self.nearest_cand(self.rx_xy[j])
            out = 0.5 * (self.rss[:, j].astype(np.float32)
                         + self.rss[i, self.rx_of_tx].astype(np.float32))
        if cache:
            if len(self._cache) > 8:
                self._cache.clear()
            self._cache[j] = out
        return out

    def link(self, a_xy, b_xy) -> float:
        return float(self.link_to(b_xy)[self.nearest_cand(a_xy)])


class RelayBrain(object):
    """Pure decision logic (no I/O), shared by the rover (RelayDriver) and relay_replay.py."""

    def __init__(self, model: LinkModel, **params):
        self.model = model
        self.p = dict(DEFAULTS, **{k: v for k, v in params.items() if v is not None})
        self.reset()

    def reset(self):
        self.target = None                     # map (x, y) the relay is heading to / holding
        self.target_score = None
        self.state = 'idle'
        self._last_retarget = -1e9
        self._fixes = []                       # explorer [(t, x, y)], oldest first

    def set_params(self, **params):
        self.p.update({k: v for k, v in params.items() if v is not None})

    def add_fix(self, t: float, xy: Sequence[float]):
        """Explorer position at BS time t (repeats and older fixes are ignored)."""
        if self._fixes and t <= self._fixes[-1][0]:
            return
        self._fixes.append((float(t), float(xy[0]), float(xy[1])))
        while len(self._fixes) > 2 and self._fixes[0][0] < t - 15.0:
            self._fixes.pop(0)

    def explorer(self, now: float):
        """(last fix xy, age s, predicted xy) or None."""
        if not self._fixes:
            return None
        t, x, y = self._fixes[-1]
        px, py = x, y
        old = [f for f in self._fixes if t - f[0] >= 2.0]
        if old and now - t < self.p['stale_s']:
            t0, x0, y0 = old[-1]
            vx, vy = (x - x0) / (t - t0), (y - y0) / (t - t0)
            v = math.hypot(vx, vy)
            if v > self.p['max_ex_speed']:
                vx, vy = vx * self.p['max_ex_speed'] / v, vy * self.p['max_ex_speed'] / v
            ahead = self.p['predict_s'] + (now - t)
            px, py = x + vx * ahead, y + vy * ahead
        return (x, y), now - t, (px, py)

    def step(self, now: float, relay_xy, bs_xy) -> dict:
        """One decision. Returns {'state', 'target' (xy or None), 'new_target' (bool),
        'score_target', 'score_here', 'direct', 'ex_age', 'why'}."""
        p, m = self.p, self.model
        out = {'new_target': False, 'score_here': None, 'direct': None, 'ex_age': None, 'why': ''}

        def done(state, why=''):
            self.state = state
            out.update(state=state, target=self.target, score_target=self.target_score, why=why)
            return out

        if relay_xy is None:
            return done('no_pose')
        if bs_xy is None:
            return done('no_bs')
        ex = self.explorer(now)
        if ex is None:
            return done('wait_ex')
        last_xy, age, pred_xy = ex
        out['ex_age'] = age
        to_bs = m.link_to(bs_xy, cache=True)
        out['direct'] = float(to_bs[m.nearest_cand(last_xy)])

        if age > p['stale_s']:
            if age < p['stale_s'] + p['ferry_s']:
                return done('lost_hold', f"explorer silent {age:.0f} s")
            ferry = tuple(float(v) for v in m.cand_xy[m.nearest_cand(last_xy)])
            if self.target is None or self.state != 'ferry' or math.hypot(
                    ferry[0] - self.target[0], ferry[1] - self.target[1]) > p['min_shift_m']:
                self.target, self.target_score = ferry, None
                self._last_retarget = now
                out['new_target'] = True
            return done('ferry', f"explorer silent {age:.0f} s: to its last position")

        score = np.minimum(to_bs, m.link_to(pred_xy))
        i_here = m.nearest_cand(relay_xy)
        here = float(score[i_here])
        out['score_here'] = here
        dist = np.hypot(*(m.cand_xy - np.asarray(relay_xy, float)).T)
        best = int(np.argmax(score - p['travel_db_per_m'] * dist))
        best_xy, best_score = tuple(float(v) for v in m.cand_xy[best]), float(score[best])

        if self.state == 'ferry':
            self.target = None                 # explorer is back: decide afresh
        if self.target is not None:
            ref = float(score[m.nearest_cand(self.target)])
            self.target_score = ref
            why_ref = 'target'
        else:
            ref, why_ref = here, 'here'
        want = (best_score > ref + p['gain_db']
                or (ref < p['comfort_dbm'] and best_score > ref + p['small_gain_db']))
        far_enough = self.target is None or math.hypot(
            best_xy[0] - self.target[0], best_xy[1] - self.target[1]) >= p['min_shift_m']
        if self.target is None and math.hypot(best_xy[0] - relay_xy[0], best_xy[1] - relay_xy[1]) < p['arrive_m']:
            far_enough = False                 # already at the best spot
        if want and far_enough and now - self._last_retarget >= p['min_retarget_s']:
            self.target, self.target_score = best_xy, best_score
            self._last_retarget = now
            out['new_target'] = True
            return done('move', f"best {best_score:.0f} dBm vs {why_ref} {ref:.0f} dBm")
        if self.target is None:
            return done('hold', f"here {here:.0f} dBm, best {best_score:.0f} dBm")
        if math.hypot(self.target[0] - relay_xy[0], self.target[1] - relay_xy[1]) <= p['arrive_m']:
            return done('at_target', f"target {ref:.0f} dBm")
        return done('move', f"target {ref:.0f} dBm")


class RelayDriver(object):
    """RelayBrain + route planning for the rover. tick() returns a route (map points) to
    hand to rover_nav.py when one should be (re)sent, else None."""

    RESEND_S = 10.0           # resend the route this often while rover_nav.py isn't driving it

    def __init__(self, site, models):
        """models: {key: LinkModel} (see load_models) or a single LinkModel."""
        import route_plan
        if isinstance(models, LinkModel):
            models = {models.key: models}
        self.models = dict(models)
        self.default_model = (DEFAULT_MODEL if DEFAULT_MODEL in self.models
                              else next(k for k in self.models if k != 'distance')
                              if len(self.models) > 1 else next(iter(self.models)))
        self.brain = RelayBrain(self.models[self.default_model])
        self.planner = route_plan.Planner(site)
        self.site_name = site.name
        self.cfg = None                        # {'ex': node str, 'speed': m/s, ...} or None = off
        self.status = {'rl': 'off'}
        self.why = ''                          # reason for the last decision (console / log)
        self._sent_at = 0.0
        self._sent_target = None

    def configure(self, cfg: Optional[dict]) -> Optional[str]:
        """Apply the BS setting. Returns a note when something changed."""
        if cfg == self.cfg:
            return None
        old, self.cfg = self.cfg, cfg
        if cfg is None:
            self.brain.reset()
            self.status = {'rl': 'off'}
            self._sent_target = None
            return 'relay off'
        if old is None or old.get('ex') != cfg.get('ex'):
            self.brain.reset()
            self._sent_target = None
        self.brain.set_params(**{k: cfg.get(k) for k in ('ferry_s', 'comfort_dbm', 'gain_db', 'predict_s')})
        want = cfg.get('model') or self.default_model
        missing = ''
        if want not in self.models:
            missing = f" (no {want} table on this rover)"
            want = self.default_model
        self.brain.model = self.models[want]           # target kept; re-scored on the next tick
        p = self.brain.p
        return (f"relay for {cfg.get('ex')} at {cfg.get('speed')} m/s, model {want}{missing} (ferry after "
                f"{p['ferry_s']:g} s, comfort {p['comfort_dbm']:g} dBm, gain {p['gain_db']:g} dB, "
                f"look-ahead {p['predict_s']:g} s)")

    def tick(self, now: float, relay_xy, ex_fix, bs_xy, nav_state: str):
        """ex_fix = (bs_time, (x, y)) of the explorer or None. Returns map points or None."""
        if self.cfg is None:
            return None
        if ex_fix is not None:
            self.brain.add_fix(ex_fix[0], ex_fix[1])
        d = self.brain.step(now, relay_xy, bs_xy)
        self.why = d['why']
        st = {'rl': d['state'], 'rm': self.brain.model.key}
        if d['target'] is not None:
            st['rtx'], st['rty'] = round(d['target'][0], 1), round(d['target'][1], 1)
        for k, v in (('rsc', d['score_target']), ('rcu', d['score_here']), ('rdi', d['direct']),
                     ('rea', d['ex_age'])):
            if v is not None:
                st[k] = round(float(v), 1 if k != 'rea' else 0)
        route = None
        t = d['target']
        if (t is not None and relay_xy is not None and d['state'] in ('move', 'ferry')
                and math.hypot(t[0] - relay_xy[0], t[1] - relay_xy[1]) > self.brain.p['arrive_m']):
            resend = (nav_state not in ('following', 'hold_rb')
                      and now - self._sent_at >= self.RESEND_S)
            if d['new_target'] or t != self._sent_target or resend:
                pts, err = self.planner.plan(relay_xy, [t])
                if pts is None:
                    st['rl'] = 'no_route'
                    st['why'] = err
                else:
                    route, self._sent_target, self._sent_at = pts, t, now
        self.status = st
        return route


def table_key(path: str) -> str:
    """fountain_linktab_nn_pmnet.npz -> nn_pmnet"""
    base = os.path.basename(path)[:-len('.npz')] if path.endswith('.npz') else os.path.basename(path)
    return base.split('_linktab_', 1)[1] if '_linktab_' in base else base


def find_tables(terrain_dir: str, site: str) -> List[str]:
    return sorted(glob.glob(os.path.join(terrain_dir, f"{site}_linktab_*.npz")))


def load_models(site, table_paths: Sequence[str]) -> Dict[str, LinkModel]:
    """{key: LinkModel} for every readable table, plus 'distance' (printed notes on failures)."""
    out = {}
    for path in table_paths:
        try:
            t0 = time.time()
            m = LinkModel(site, path)
            out[m.key] = m
            print(f"[RELAY] link table {m.key}: {len(m.cand_xy)} cells, loaded in {time.time() - t0:.1f} s")
        except (OSError, ValueError, KeyError) as e:
            print(f"[RELAY] could not load link table {path}: {e}")
    out['distance'] = LinkModel(site, None)
    return out


def load_model(site, table_path: Optional[str]) -> LinkModel:
    """One table if given and readable, else the distance model."""
    return next(iter(load_models(site, [table_path] if table_path else []).values()))


def main():
    import argparse
    import site_map
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--site', required=True)
    ap.add_argument('--terrain-dir')
    ap.add_argument('--linktab', help='<site>_linktab_<model>.npz (default: distance model)')
    ap.add_argument('--bs', required=True, help='BS map position x,y')
    ap.add_argument('--explorer', required=True, help='explorer map position x,y')
    ap.add_argument('--relay', help='relay map position x,y (default: at the BS)')
    args = ap.parse_args()
    site = site_map.Site(site_map.find_terrain_dir(args.terrain_dir), args.site)
    xy = lambda s: tuple(float(v) for v in s.split(','))   # noqa: E731
    bs, ex = xy(args.bs), xy(args.explorer)
    relay = xy(args.relay) if args.relay else bs
    t0 = time.time()
    brain = RelayBrain(load_model(site, args.linktab))
    brain.add_fix(0.0, ex)
    d = brain.step(0.0, relay, bs)
    print(f"model {brain.model.name}, {len(brain.model.cand_xy)} candidates, {time.time() - t0:.2f} s")
    print(d)


if __name__ == '__main__':
    main()
