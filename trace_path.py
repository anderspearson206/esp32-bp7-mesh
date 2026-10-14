"""
trace_path.py  --  Draw the route a rover actually drove and put the session's samples on it.

When odometry is too far off to correct automatically (fix_track.py), you know where you
drove. Trace that route on the site map and the logged samples are slid along it:

  * Trace: left-click along the route in the order you drove it (out and back = go out and
    come back over the same path). "Snap to paths" puts each click on the nearest drivable
    cell. Backspace / "Undo point" removes the last point.
  * Placement: every sample sits on the route at a share of the route length equal to its
    share of the odometry distance driven (wheel-spin jumps capped), so parked periods stay
    put and speed changes are kept.
  * Pins: move the time slider (the sample shows on the route and in the RSSI plot), press
    "Pin…" and click where the rover really was at that moment (turnaround, landmark,
    back at the BS). Between pins samples are spread by odometry distance.
  * Save writes, next to the session's coverage.csv:
        trace.json             the route + pins (reloaded next time, so you can keep editing)
        coverage_traced.csv    coverage columns with map_x / map_y / heading / drivable /
                               elevation / predictions recomputed on the traced route
        fix_track_traced.png   raw vs traced track + RSSI checks   (fix_track.plot)
        fix_track_rm_traced.png  predicted radio maps on the route (fix_track.plot_rm)

    python trace_path.py bs_reports/session_..._exp_161720 --site fountain
"""

import argparse
import csv
import json
import math
import os
import sys
from datetime import datetime

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

import fix_track
import site_map
from bs_dashboard import shaded_relief, radio_rgba, rssi_color

SPIKE_FACTOR = 2.5          # odometry step > this x typical = wheel spin, capped


class TraceViewBox(pg.ViewBox):
    def __init__(self, win):
        super().__init__()
        self.win = win

    def mouseClickEvent(self, ev):
        if ev.button() == QtCore.Qt.LeftButton and self.win.mode() != 'pan':
            ev.accept()
            p = self.mapSceneToView(ev.scenePos())
            self.win.on_map_click(p.x(), p.y())
            return
        super().mouseClickEvent(ev)


class TraceWindow(QtWidgets.QMainWindow):
    def __init__(self, session_dir, site, rows, node):
        super().__init__()
        self.dir, self.site, self.rows, self.node = session_dir, site, rows, node
        self.trace_file = os.path.join(session_dir, 'trace.json')
        n = len(rows)
        num = lambda k: fix_track.num(rows, k)
        self.ox, self.oy, self.rssi = num('lat'), num('lon'), num('rssi_to_bs_dBm')
        self.raw_mx, self.raw_my = num('map_x'), num('map_y')
        self.t = np.array([datetime.strptime(r['timestamp'], '%Y-%m-%d %H:%M:%S.%f').timestamp() for r in rows])
        step = np.r_[0.0, np.hypot(np.diff(self.ox), np.diff(self.oy))]
        mv = step[step > fix_track.STOP_STEP_M]
        cap = SPIKE_FACTOR * (float(np.median(mv)) if len(mv) else 1.0)
        step = np.where(step > fix_track.STOP_STEP_M, np.minimum(step, cap), 0.0)
        self.D = np.cumsum(step)                      # odometry distance driven, wheel spin capped
        self.pts = []                                 # traced route [(x, y)]
        self.pins = {}                                # sample index -> route distance [m]
        self.cur = 0
        self.fx = self.fy = None
        self._load()

        pg.setConfigOptions(antialias=True, background='w', foreground='k', imageAxisOrder='row-major')
        self.setWindowTitle(f"Trace the route - {os.path.basename(session_dir)} (node {node})")
        self.resize(1600, 1000)
        central = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(central)
        self.setCentralWidget(central)

        left = QtWidgets.QVBoxLayout()
        self.vb = TraceViewBox(self)
        self.pmap = pg.PlotWidget(viewBox=self.vb)
        self.pmap.setAspectLocked(True)
        self.pmap.showGrid(x=True, y=True, alpha=0.2)
        h = site.half
        rect = QtCore.QRectF(-h, -h, 2 * h, 2 * h)
        self.layers = {}
        if site.terrain is not None:
            self.layers['Elevation'] = self._image(shaded_relief(site.terrain, site.cell), rect, -100)
        if site.sat_path:
            pix = QtGui.QPixmap(site.sat_path)
            item = QtWidgets.QGraphicsPixmapItem(pix)
            item.setTransform(QtGui.QTransform(2 * h / pix.width(), 0, 0, -2 * h / pix.height(), -h, h))
            item.setZValue(-100)
            self.pmap.addItem(item)
            self.layers['Satellite'] = item
        for m, arr in site.radio_maps.items():
            if arr is not None:
                self.layers[f'Predicted {m}'] = self._image(radio_rgba(arr), rect, -100)
        self.drv = None
        if site.drivable is not None:
            rgba = np.zeros(site.drivable.shape + (4,), np.uint8)
            rgba[site.drivable] = (30, 110, 255, 255)
            self.drv = self._image(rgba, rect, -90)
            self.drv.setOpacity(0.4)
        self.raw_line = self.pmap.plot(self.raw_mx, self.raw_my, pen=pg.mkPen((120, 120, 120, 150), width=1.5,
                                                                            style=QtCore.Qt.DashLine))
        self.route_line = self.pmap.plot([], [], pen=pg.mkPen('#d81b60', width=3))
        self.route_pts = pg.ScatterPlotItem(size=9, brush=pg.mkBrush('#d81b60'), pen=pg.mkPen('w', width=1))
        self.pmap.addItem(self.route_pts)
        self.samples = pg.ScatterPlotItem(size=7, pen=None)
        self.pmap.addItem(self.samples)
        self.pin_marks = pg.ScatterPlotItem(size=16, symbol='t', brush=pg.mkBrush('#ffeb3b'), pen=pg.mkPen('k', width=1.5))
        self.pmap.addItem(self.pin_marks)
        self.cur_mark = pg.ScatterPlotItem(size=20, symbol='o', brush=pg.mkBrush(None), pen=pg.mkPen('k', width=3))
        self.pmap.addItem(self.cur_mark)
        if site.tx_xy:
            self.pmap.addItem(pg.ScatterPlotItem([site.tx_xy[0]], [site.tx_xy[1]], size=24, symbol='star',
                                                 brush=pg.mkBrush('#ffd54f'), pen=pg.mkPen('k', width=1.5)))
        span = max(np.nanmax(self.raw_mx) - np.nanmin(self.raw_mx), np.nanmax(self.raw_my) - np.nanmin(self.raw_my)) / 2 + 30
        cx, cy = np.nanmean(self.raw_mx), np.nanmean(self.raw_my)
        if site.tx_xy:
            cx, cy = (cx + site.tx_xy[0]) / 2, (cy + site.tx_xy[1]) / 2
        self.pmap.setRange(xRange=(cx - span, cx + span), yRange=(cy - span, cy + span))
        left.addWidget(self.pmap, 3)

        self.prof = pg.PlotWidget()
        self.prof.setLabel('bottom', 'time', units='s')
        self.prof.setLabel('left', 'dB')
        tt = self.t - self.t[0]
        self.prof.plot(tt, self.rssi, pen=None, symbol='o', symbolSize=4, symbolBrush='k', name='measured')
        self.pred_curves = {}
        for k, m in enumerate(m for m, a in site.radio_maps.items() if a is not None):
            self.pred_curves[m] = self.prof.plot([], [], pen=pg.mkPen(fix_track_color(k), width=2))
        self.cur_line = pg.InfiniteLine(angle=90, pen=pg.mkPen('#d81b60', width=2))
        self.prof.addItem(self.cur_line)
        self.pin_lines = []
        left.addWidget(self.prof, 1)
        sl = QtWidgets.QHBoxLayout()
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, n - 1)
        self.slider.valueChanged.connect(self.on_slider)
        sl.addWidget(self.slider, 1)
        self.time_label = QtWidgets.QLabel()
        self.time_label.setMinimumWidth(330)
        sl.addWidget(self.time_label)
        left.addLayout(sl)
        lay.addLayout(left, 1)

        side = QtWidgets.QVBoxLayout()
        box = QtWidgets.QGroupBox("Mode (left click on the map)")
        bl = QtWidgets.QVBoxLayout(box)
        self.mode_btns = {}
        for key, text in [('trace', "Trace route (add point)"), ('pin', "Pin current sample here"),
                          ('pan', "Pan / zoom only")]:
            b = QtWidgets.QRadioButton(text)
            self.mode_btns[key] = b
            bl.addWidget(b)
        self.mode_btns['trace'].setChecked(True)
        self.snap = QtWidgets.QCheckBox("Snap clicks to paths (drivable)")
        self.snap.setChecked(site.drivable is not None)
        bl.addWidget(self.snap)
        side.addWidget(box)

        rb = QtWidgets.QGroupBox("Route")
        rl = QtWidgets.QVBoxLayout(rb)
        for text, fn in [("Undo point (Backspace)", self.undo_point), ("Start at the BS", self.start_at_bs),
                         ("End at the BS", self.end_at_bs), ("Clear route", self.clear_route)]:
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(fn)
            rl.addWidget(b)
        side.addWidget(rb)

        pb = QtWidgets.QGroupBox("Pins (sample -> place on route)")
        pl = QtWidgets.QVBoxLayout(pb)
        self.pin_list = QtWidgets.QListWidget()
        self.pin_list.itemClicked.connect(lambda it: self.slider.setValue(int(it.data(QtCore.Qt.UserRole))))
        pl.addWidget(self.pin_list)
        b = QtWidgets.QPushButton("Delete selected pin")
        b.clicked.connect(self.delete_pin)
        pl.addWidget(b)
        hint = QtWidgets.QLabel("Tip: pin the turnaround (lowest RSSI / when the rover turned) and "
                                "moments you remember. Without pins the samples are spread along the "
                                "route by odometry distance.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#555")
        pl.addWidget(hint)
        side.addWidget(pb, 1)

        lb = QtWidgets.QGroupBox("Show")
        ll = QtWidgets.QFormLayout(lb)
        self.bg = QtWidgets.QComboBox()
        self.bg.addItems(list(self.layers))
        self.bg.currentTextChanged.connect(self._layers)
        ll.addRow("Background", self.bg)
        self.drv_check = QtWidgets.QCheckBox("Drivable paths")
        self.drv_check.setChecked(True)
        self.drv_check.toggled.connect(self._layers)
        ll.addRow(self.drv_check)
        self.raw_check = QtWidgets.QCheckBox("Logged (raw) track")
        self.raw_check.setChecked(True)
        self.raw_check.toggled.connect(lambda on: self.raw_line.setVisible(on))
        ll.addRow(self.raw_check)
        side.addWidget(lb)

        self.stats = QtWidgets.QLabel()
        self.stats.setWordWrap(True)
        self.stats.setTextFormat(QtCore.Qt.RichText)
        side.addWidget(self.stats)
        save = QtWidgets.QPushButton("Save (CSV + figures)")
        save.setStyleSheet("font-weight:bold; padding:6px")
        save.clicked.connect(self.save)
        side.addWidget(save)
        sw = QtWidgets.QWidget()
        sw.setLayout(side)
        sw.setMaximumWidth(330)
        lay.addWidget(sw)
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Backspace), self, self.undo_point)
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Left), self, lambda: self.slider.setValue(self.cur - 1))
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Right), self, lambda: self.slider.setValue(self.cur + 1))
        self._layers()
        self.recompute()
        self.on_slider(0)

    # helpers
    def _image(self, arr, rect, z):
        item = pg.ImageItem(arr[::-1], levels=(0, 255) if arr.dtype == np.uint8 else None)
        item.setRect(rect)
        item.setZValue(z)
        self.pmap.addItem(item)
        return item

    def _layers(self, *_):
        for name, item in self.layers.items():
            item.setVisible(self.bg.currentText() == name)
        if self.drv is not None:
            self.drv.setVisible(self.drv_check.isChecked())

    def mode(self):
        return next(k for k, b in self.mode_btns.items() if b.isChecked())

    def _snap(self, x, y):
        if not self.snap.isChecked() or self.site.drivable is None or self.site.on_drivable(x, y):
            return x, y
        best = None
        for r in np.arange(0.5, 6.01, 0.5):
            for a in np.linspace(0, 2 * math.pi, int(8 * r) + 8, endpoint=False):
                px, py = x + r * math.cos(a), y + r * math.sin(a)
                if self.site.on_drivable(px, py):
                    best = (px, py)
                    break
            if best:
                break
        return best or (x, y)

    def _route_len(self):
        if len(self.pts) < 2:
            return np.zeros(len(self.pts))
        p = np.array(self.pts)
        return np.r_[0.0, np.cumsum(np.hypot(*np.diff(p, axis=0).T))]

    def _at(self, s):
        """Route point and direction (deg) at route distance s (arrays)."""
        p = np.array(self.pts)
        L = self._route_len()
        s = np.clip(s, 0, L[-1])
        i = np.clip(np.searchsorted(L, s, side='right') - 1, 0, len(p) - 2)
        seg = np.maximum(L[i + 1] - L[i], 1e-9)
        f = (s - L[i]) / seg
        x = p[i, 0] + f * (p[i + 1, 0] - p[i, 0])
        y = p[i, 1] + f * (p[i + 1, 1] - p[i, 1])
        hd = np.degrees(np.arctan2(p[i + 1, 1] - p[i, 1], p[i + 1, 0] - p[i, 0]))
        return x, y, hd

    def _project(self, x, y):
        """Route distance of the route point nearest (x, y)."""
        p = np.array(self.pts)
        L = self._route_len()
        a, b = p[:-1], p[1:]
        ab = b - a
        t = np.clip(((np.array([x, y]) - a) * ab).sum(1) / np.maximum((ab ** 2).sum(1), 1e-9), 0, 1)
        q = a + t[:, None] * ab
        k = int(np.argmin(np.hypot(*(q - [x, y]).T)))
        return float(L[k] + t[k] * (L[k + 1] - L[k]))

    # placement
    def recompute(self):
        n = len(self.rows)
        if len(self.pts) < 2:
            self.fx = self.fy = self.hd = None
        else:
            L = self._route_len()[-1]
            D = self.D
            knots = {0: 0.0, n - 1: L}
            knots.update({i: min(max(s, 0.0), L) for i, s in self.pins.items()})
            ki = sorted(knots)
            s = np.interp(D, [D[i] for i in ki], [knots[i] for i in ki]) if D[-1] > 0 else np.zeros(n)
            # identical odometry distance at two knots (parked between pins): interpolate in time
            for i0, i1 in zip(ki[:-1], ki[1:]):
                if D[i1] - D[i0] < 1e-6 and i1 > i0:
                    s[i0:i1 + 1] = np.linspace(knots[i0], knots[i1], i1 - i0 + 1)
            self.s = s
            self.fx, self.fy, self.hd = self._at(s)
        self.redraw()

    def redraw(self):
        p = np.array(self.pts) if self.pts else np.zeros((0, 2))
        self.route_line.setData(p[:, 0], p[:, 1])
        self.route_pts.setData(p[:, 0], p[:, 1])
        if self.fx is not None:
            self.samples.setData([{'pos': (x, y), 'brush': pg.mkBrush(rssi_color(r))}
                                  for x, y, r in zip(self.fx, self.fy, self.rssi) if not math.isnan(r)])
            pins = sorted(self.pins)
            self.pin_marks.setData(self.fx[pins], self.fy[pins]) if pins else self.pin_marks.setData([], [])
            tt = self.t - self.t[0]
            for m, c in self.pred_curves.items():
                c.setData(tt, self.site.sample(self.site.radio_maps[m], self.fx, self.fy))
            drv = self.site.on_drivable(self.fx, self.fy, 1.0).mean() if self.site.drivable is not None else float('nan')
            rs, pr = self.rssi, self.site.sample(self.site.radio_maps.get('nn_pmnet'), self.fx, self.fy)
            ok = ~np.isnan(rs) & ~np.isnan(pr)
            corr = np.corrcoef(rs[ok], pr[ok])[0, 1] if ok.sum() > 5 else float('nan')
            self.stats.setText(f"route {self._route_len()[-1]:.0f} m, odometry {self.D[-1]:.0f} m "
                               f"(wheel spin capped)<br>{len(self.pins)} pin(s), on paths (1 m) "
                               f"{100 * drv:.0f}%<br>RSSI vs nn_pmnet on this route: r {corr:+.2f}")
        else:
            self.samples.setData([])
            self.pin_marks.setData([], [])
            self.stats.setText("Trace the route: click along it from the BS, in the order driven.")
        for ln in self.pin_lines:
            self.prof.removeItem(ln)
        self.pin_lines = []
        self.pin_list.clear()
        for i in sorted(self.pins):
            ln = pg.InfiniteLine(self.t[i] - self.t[0], angle=90, pen=pg.mkPen('#fbc02d', width=2))
            self.prof.addItem(ln)
            self.pin_lines.append(ln)
            it = QtWidgets.QListWidgetItem(f"{self.rows[i]['timestamp'][11:19]}  RSSI {self.rssi[i]:.0f}  "
                                           f"-> {self.pins[i]:.0f} m along route")
            it.setData(QtCore.Qt.UserRole, i)
            self.pin_list.addItem(it)
        self.on_slider(self.cur)

    # interaction
    def on_map_click(self, x, y):
        if self.mode() == 'trace':
            self.pts.append(self._snap(x, y))
            self.recompute()
        elif self.mode() == 'pin':
            if len(self.pts) < 2:
                return
            self.pins[self.cur] = self._project(x, y)
            self.recompute()

    def on_slider(self, i):
        self.cur = int(max(0, min(i, len(self.rows) - 1)))
        if self.slider.value() != self.cur:
            self.slider.setValue(self.cur)
        self.cur_line.setValue(self.t[self.cur] - self.t[0])
        if self.fx is not None:
            self.cur_mark.setData([self.fx[self.cur]], [self.fy[self.cur]])
        else:
            self.cur_mark.setData([self.raw_mx[self.cur]], [self.raw_my[self.cur]])
        r = self.rows[self.cur]
        self.time_label.setText(f"{r['timestamp'][11:19]}  (+{self.t[self.cur] - self.t[0]:.0f} s)  "
                                f"RSSI {self.rssi[self.cur]:.0f}  odom {self.D[self.cur]:.0f} m driven")

    def undo_point(self):
        if self.pts:
            self.pts.pop()
            self.recompute()

    def start_at_bs(self):
        if self.site.tx_xy:
            self.pts.insert(0, tuple(self.site.tx_xy))
            self.recompute()

    def end_at_bs(self):
        if self.site.tx_xy:
            self.pts.append(tuple(self.site.tx_xy))
            self.recompute()

    def clear_route(self):
        if QtWidgets.QMessageBox.question(self, "Clear", "Remove the route and all pins?") == QtWidgets.QMessageBox.Yes:
            self.pts, self.pins = [], {}
            self.recompute()

    def delete_pin(self):
        it = self.pin_list.currentItem()
        if it is not None:
            self.pins.pop(int(it.data(QtCore.Qt.UserRole)), None)
            self.recompute()

    # files
    def _load(self):
        if not os.path.exists(self.trace_file):
            return
        with open(self.trace_file) as f:
            d = json.load(f)
        self.pts = [tuple(p) for p in d.get('route', [])]
        stamps = {r['timestamp']: i for i, r in enumerate(self.rows)}
        self.pins = {stamps[k]: float(v) for k, v in d.get('pins', {}).items() if k in stamps}

    def save(self):
        if self.fx is None:
            QtWidgets.QMessageBox.information(self, "Save", "Trace the route first (two or more points).")
            return
        with open(self.trace_file, 'w') as f:
            json.dump({'node': self.node, 'site': self.site.name, 'route': [list(map(float, p)) for p in self.pts],
                       'pins': {self.rows[i]['timestamp']: s for i, s in sorted(self.pins.items())}}, f, indent=1)
        site, rows = self.site, self.rows
        fx, fy = self.fx, self.fy
        drv = site.on_drivable(fx, fy)
        elev = site.sample(site.terrain, fx, fy)
        preds = {m: site.sample(a, fx, fy) for m, a in site.radio_maps.items()}
        keep = [k for k in rows[0] if k not in ('map_x', 'map_y', 'map_heading_deg', 'on_drivable', 'elev_m')
                and not k.startswith('pred_')]
        out = os.path.join(self.dir, 'coverage_traced.csv')
        with open(out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(keep + ['map_x', 'map_y', 'map_heading_deg', 'on_drivable', 'elev_m']
                       + [f"pred_{m}_dB" for m in preds] + ['raw_map_x', 'raw_map_y', 'route_m'])
            for i, r in enumerate(rows):
                w.writerow([r[k] for k in keep] + [f"{fx[i]:.2f}", f"{fy[i]:.2f}", f"{self.hd[i]:.1f}",
                                                   int(drv[i]), f"{elev[i]:.2f}"]
                           + [f"{preds[m][i]:.1f}" for m in preds]
                           + [r['map_x'], r['map_y'], f"{self.s[i]:.1f}"])
        legs = fix_track.find_legs(self.ox, self.oy, 20.0)
        leg_id = np.zeros(len(rows), dtype=int)
        for li, (a, b) in enumerate(legs, 1):
            leg_id[a + 1:b] = li
        if not legs:
            leg_id[:] = 1
        home = site.tx_xy or (fx[0], fy[0])
        fix_track.plot(site, self.dir, rows, self.raw_mx, self.raw_my, fx, fy, self.rssi, preds, leg_id,
                       home, self.node, 'traced', '_traced')
        fix_track.plot_rm(site, self.dir, rows, self.raw_mx, self.raw_my, fx, fy, self.rssi, leg_id,
                          self.node, 'traced', '_traced')
        self.statusBar().showMessage(f"saved {out}, trace.json, fix_track_traced.png, fix_track_rm_traced.png", 8000)


def fix_track_color(k):
    return ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'][k % 4]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', help='session folder (with coverage.csv)')
    ap.add_argument('--site', required=True)
    ap.add_argument('--terrain-dir')
    ap.add_argument('--node')
    args = ap.parse_args()
    sdir = args.session[:-len('coverage.csv')] if args.session.endswith('coverage.csv') else args.session
    tdir = site_map.find_terrain_dir(args.terrain_dir)
    if tdir is None:
        sys.exit('terrain directory not found (use --terrain-dir)')
    site = site_map.Site(tdir, args.site)
    bs_file = os.path.join(sdir, 'bs_position.json')
    if os.path.exists(bs_file):
        with open(bs_file) as f:
            pos = json.load(f)
        site.set_bs(float(pos['x']), float(pos['y']))
    node, rows = fix_track.load(os.path.join(sdir, 'coverage.csv'), args.node)
    app = QtWidgets.QApplication(sys.argv)
    win = TraceWindow(sdir, site, rows, node)
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
