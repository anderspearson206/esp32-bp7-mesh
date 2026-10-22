"""
Live BS dashboard (PySide6 + pyqtgraph) on top of bs_core.BSCore.

    pip install pyserial pyqtgraph PySide6
    python bs_dashboard.py --port COM5

Left: fleet control (start / live-change / standby experiments on every rover), rover table,
event log. Right: comms timeline (link-up lanes, data throughput, data latency, markers),
topology and radio map. The display only reads snapshots from the core, so a slow or closed
window never affects logging or control. Closing the window writes the session summary.
"""

import argparse
import math
import sys
import time
from datetime import datetime

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

import bs_core
import relay_brain
import route_plan
import site_map
from bs_core import BSCore, BS_NODE_ID

PALETTE = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd', '#ff7f0e', '#17becf', '#8c564b', '#e377c2']
TIMELINE_CHOICES = [(60, 1), (180, 5), (600, 10), (1800, 30), (3600, 60)]   # (window s, bin s)
EVENT_COLORS = {'data': '#2e7d32', 'ferry': '#ef6c00', 'contact': '#1565c0', 'control': '#6a1b9a',
                'marker': '#000000', 'warn': '#c62828', 'session': '#616161', 'map': '#ad1457'}


def rssi_color(rssi: float) -> QtGui.QColor:
    """Red (-90 dBm) -> yellow (-60) -> green (-30)."""
    f = min(1.0, max(0.0, (rssi + 90) / 60.0))
    if f < 0.5:
        r, g = 220, int(60 + 340 * f)
    else:
        r, g = int(220 - 360 * (f - 0.5)), 200
    return QtGui.QColor(r, g, 60)


def shaded_relief(elev: np.ndarray, cell: float) -> np.ndarray:
    """RGB hillshade (light from the north-west) tinted green -> tan -> white by height.
    Row 0 = north, like the terrain files."""
    gy, gx = np.gradient(elev.astype(float), cell)       # gy is per row, i.e. towards the south
    nx, ny, nz = -gx, gy, np.ones_like(gx)
    light = np.array([-1.0, 1.0, 1.4]) / np.linalg.norm([-1.0, 1.0, 1.4])
    shade = np.clip((nx * light[0] + ny * light[1] + nz * light[2]) / np.sqrt(nx ** 2 + ny ** 2 + 1), 0, 1)
    f = (elev - np.nanmin(elev)) / max(1e-6, float(np.nanmax(elev) - np.nanmin(elev)))
    stops = np.array([[90, 150, 90], [205, 190, 135], [245, 245, 240]], float)
    lo = np.clip(f * 2, 0, 1)[..., None]
    hi = np.clip(f * 2 - 1, 0, 1)[..., None]
    rgb = np.where(f[..., None] < 0.5, stops[0] + (stops[1] - stops[0]) * lo,
                   stops[1] + (stops[2] - stops[1]) * hi)
    return np.clip(rgb * (0.35 + 0.65 * shade[..., None]), 0, 255).astype(np.uint8)


def radio_rgba(arr: np.ndarray) -> np.ndarray:
    """Predicted radio map in dB -> RGBA (viridis, 2-98 % range, NaN transparent)."""
    ok = np.isfinite(arr)
    lo, hi = np.percentile(arr[ok], [2, 98]) if ok.any() else (0, 1)
    f = np.clip((np.nan_to_num(arr, nan=lo) - lo) / max(1e-6, hi - lo), 0, 1)
    rgba = pg.colormap.get('viridis').map(f.ravel(), mode='byte').reshape(arr.shape + (4,))
    rgba[..., 3] = np.where(ok, 255, 0)
    return rgba.astype(np.uint8)


class MapViewBox(pg.ViewBox):
    """Normal pan/zoom, except in "Place on map" mode where a left click or drag aligns the
    selected rover (see Dashboard.align_from_map), in "Move BS" mode where a left click
    puts the BS there (Dashboard.move_bs_to), and in "Add waypoints" mode where each left
    click adds a route waypoint (Dashboard.add_waypoint)."""

    def __init__(self, dash):
        super().__init__()
        self.dash = dash

    def _aligning(self, ev) -> bool:
        return self.dash.align_btn.isChecked() and ev.button() == QtCore.Qt.LeftButton

    def mouseDragEvent(self, ev, axis=None):
        if not self._aligning(ev):
            return super().mouseDragEvent(ev, axis)
        ev.accept()
        p0 = self.mapSceneToView(ev.buttonDownScenePos())
        p1 = self.mapSceneToView(ev.scenePos())
        self.dash.align_from_map(p0.x(), p0.y(), p1.x(), p1.y(), ev.isFinish())

    def mouseClickEvent(self, ev):
        if self.dash.wp_btn.isChecked() and ev.button() == QtCore.Qt.LeftButton:
            ev.accept()
            p = self.mapSceneToView(ev.scenePos())
            self.dash.add_waypoint(p.x(), p.y())
            return
        if self.dash.bs_btn.isChecked() and ev.button() == QtCore.Qt.LeftButton:
            ev.accept()
            p = self.mapSceneToView(ev.scenePos())
            self.dash.move_bs_to(p.x(), p.y())
            return
        if not self._aligning(ev):
            return super().mouseClickEvent(ev)
        ev.accept()
        p = self.mapSceneToView(ev.scenePos())
        self.dash.align_from_map(p.x(), p.y(), p.x(), p.y(), True)


def fmt_age(sec) -> str:
    if sec is None:
        return '-'
    sec = int(sec)
    m, s = divmod(sec, 60)
    h, m = divmod(m, 60)
    return f"{h}h{m:02d}m" if h else (f"{m}m{s:02d}s" if m else f"{s}s")


class Dashboard(QtWidgets.QMainWindow):
    TABLE_COLS = ['Node', 'Link', 'For', 'RSSI', 'State', 'Experiment', 'Ctrl', 'Clock', 'ROS', 'Radio window',
                  'Queue own/ferr', 'Gen', 'Tx', 'Retx', 'Ack', 'Data 60s', 'Data total',
                  'Avg lat', 'Status age']

    def __init__(self, core: BSCore, map_half: float):
        super().__init__()
        self.core = core
        self.site = core.site
        self.map_half = map_half
        self.colors = {}
        self.last_event_id = 0
        self.tp_curves = {}
        self.lat_scatters = {}
        self.link_bars = []
        self.marker_lines = []
        self.marker_key = None
        self.setWindowTitle(f"DTN Rover Mesh - BS dashboard ({core.port})")
        self.resize(1800, 1000)
        pg.setConfigOptions(antialias=True, background='w', foreground='k', imageAxisOrder='row-major')

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(self._build_left())
        splitter.addWidget(self._build_right())
        splitter.setSizes([780, 1020])
        self.setCentralWidget(splitter)
        self._build_toolbar()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)
        self.refresh()

    # layout
    def _build_toolbar(self) -> None:
        tb = self.addToolBar("main")
        tb.setMovable(False)
        for text, slot, key in (("Marker", self.on_marker, "Ctrl+M"),
                                ("Save summary", self.on_save, "Ctrl+S"),
                                ("New session", self.on_new_session, None)):
            act = QtGui.QAction(text, self)
            if key:
                act.setShortcut(QtGui.QKeySequence(key))
                act.setToolTip(f"{text} ({key})")
            act.triggered.connect(slot)
            tb.addAction(act)
        tb.addSeparator()
        self.marker_edit = QtWidgets.QLineEdit()
        self.marker_edit.setPlaceholderText("marker label (optional)")
        self.marker_edit.setMaximumWidth(200)
        self.marker_edit.returnPressed.connect(self.on_marker)
        tb.addWidget(self.marker_edit)
        tb.addSeparator()
        tb.addWidget(QtWidgets.QLabel(" Timeline: "))
        self.window_box = QtWidgets.QComboBox()
        for w, _b in TIMELINE_CHOICES:
            self.window_box.addItem(f"{w // 60} min" if w >= 60 else f"{w} s", w)
        self.window_box.setCurrentIndex(1)
        tb.addWidget(self.window_box)
        self.conn_label = QtWidgets.QLabel()
        self.session_label = QtWidgets.QLabel()
        self.statusBar().addWidget(self.conn_label)
        self.statusBar().addPermanentWidget(self.session_label)

    def _build_left(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(4, 4, 4, 4)

        box = QtWidgets.QGroupBox("Fleet control")
        g = QtWidgets.QGridLayout(box)
        self.exp_edit = QtWidgets.QLineEdit()
        self.exp_edit.setPlaceholderText("experiment name, e.g. route3_bw1 (blank = timestamp)")
        g.addWidget(QtWidgets.QLabel("Experiment"), 0, 0)
        g.addWidget(self.exp_edit, 0, 1, 1, 5)

        def dspin(lo, hi, val, step, suffix='', dec=2, special=None):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(dec)
            s.setSingleStep(step)
            s.setValue(val)
            s.setSuffix(suffix)
            if special:
                s.setSpecialValueText(special)
            return s

        self.cfg_widgets = {
            'interval':      dspin(0, 3600, 1.0, 0.1, ' s', special='off'),
            'bw_limit':      dspin(0, 1000, 0, 0.5, ' /s', special='unlimited'),
            'bw_burst':      dspin(1, 100, 1, 1, '', 0),
            'comms_on':      dspin(0, 3600, 0, 5, ' s', 0, special='always'),
            'comms_off':     dspin(0, 3600, 0, 5, ' s', 0, special='never'),
            'comms_phase':   dspin(-3600, 3600, 0, 1, ' s', 1),
            'loc_interval':  dspin(0.2, 60, 1.0, 0.5, ' s', 1),
        }
        self.cfg_widgets['payload_bytes'] = QtWidgets.QSpinBox()
        self.cfg_widgets['payload_bytes'].setRange(0, 1024)
        self.cfg_widgets['payload_bytes'].setSpecialValueText("natural")
        self.cfg_widgets['payload_bytes'].setSuffix(" B")
        self.cfg_widgets['bw_order'] = QtWidgets.QComboBox()
        self.cfg_widgets['bw_order'].addItems(['oldest', 'newest'])
        labels = [('interval', 'Data every'), ('bw_limit', 'BW limit'), ('bw_burst', 'Burst'),
                  ('bw_order', 'Order'), ('comms_on', 'Radio on'), ('comms_off', 'Radio off'),
                  ('comms_phase', 'Phase'), ('payload_bytes', 'Payload'), ('loc_interval', 'Location every')]
        tips = {'interval': 'seconds between generated data bundles',
                'bw_limit': 'data bundles per second over the air (beacons/acks are never limited)',
                'bw_burst': 'data bundles that may go back-to-back after idling',
                'bw_order': 'which queued bundles get the budget first',
                'comms_on': 'periodic mode: seconds the radio is on (needs Radio off > 0 too)',
                'comms_off': 'periodic mode: seconds the radio is fully off (no TX, no RX)',
                'comms_phase': "shift a rover's window schedule (fleet-wide here)",
                'payload_bytes': 'pad data bundle payloads to this size',
                'loc_interval': 'seconds between rover @LOCATION reports (also applies in standby)'}
        for i, (key, text) in enumerate(labels):
            r, c = 1 + i // 3, (i % 3) * 2
            lab = QtWidgets.QLabel(text)
            lab.setToolTip(tips[key])
            self.cfg_widgets[key].setToolTip(tips[key])
            g.addWidget(lab, r, c)
            g.addWidget(self.cfg_widgets[key], r, c + 1)

        btns = QtWidgets.QHBoxLayout()
        for text, slot, color in (("Start experiment", self.on_start, '#a5d6a7'),
                                  ("Apply live", self.on_apply, '#fff59d'),
                                  ("Standby", self.on_standby, '#ef9a9a')):
            b = QtWidgets.QPushButton(text)
            b.setStyleSheet(f"QPushButton {{ background: {color}; padding: 6px; font-weight: bold; }}")
            b.clicked.connect(slot)
            btns.addWidget(b)
        g.addLayout(btns, 4, 0, 1, 6)
        self.ctrl_label = QtWidgets.QLabel()
        self.ctrl_label.setWordWrap(True)
        g.addWidget(self.ctrl_label, 5, 0, 1, 6)
        self.fw_warning = QtWidgets.QLabel(
            "BS firmware does not forward control (unknown cmd 0x30). Close this window, then "
            "<code>idf.py build</code> and <code>idf.py -p &lt;BS port&gt; flash</code>.")
        self.fw_warning.setStyleSheet("background: #c62828; color: white; padding: 6px; font-weight: bold;")
        self.fw_warning.setWordWrap(True)
        self.fw_warning.hide()
        g.addWidget(self.fw_warning, 6, 0, 1, 6)
        lay.addWidget(box)

        self.table = QtWidgets.QTableWidget(0, len(self.TABLE_COLS))
        self.table.setHorizontalHeaderLabels(self.TABLE_COLS)
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        tbox = QtWidgets.QGroupBox("Rovers")
        tlay = QtWidgets.QVBoxLayout(tbox)
        tlay.addWidget(self.table)
        rbtns = QtWidgets.QHBoxLayout()
        rbtns.addWidget(QtWidgets.QLabel("ROS stack on selected rovers:"))
        for text, on in (("Start ROS", True), ("Stop ROS", False)):
            b = QtWidgets.QPushButton(text)
            b.setToolTip("rovers need radio_daemon.py --ros-control; driving still needs the "
                         "controller deadman")
            b.clicked.connect(lambda _=False, on=on: self.on_ros(on))
            rbtns.addWidget(b)
        rbtns.addStretch()
        tlay.addLayout(rbtns)
        lay.addWidget(tbox, 2)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        self.log.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        lbox = QtWidgets.QGroupBox("Events")
        QtWidgets.QVBoxLayout(lbox).addWidget(self.log)
        lay.addWidget(lbox, 3)
        return w

    def _build_right(self) -> QtWidgets.QWidget:
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_comms_tab(), "Comms")
        self.tabs.addTab(self._build_map_tab(), "Map")
        if self.site is not None:
            self.tabs.setCurrentIndex(1)
        return self.tabs

    def _build_comms_tab(self) -> QtWidgets.QWidget:
        split = QtWidgets.QSplitter(QtCore.Qt.Vertical)

        glw = pg.GraphicsLayoutWidget()
        self.p_link = glw.addPlot(row=0, col=0)
        self.p_tp = glw.addPlot(row=1, col=0)
        self.p_lat = glw.addPlot(row=2, col=0)
        glw.ci.layout.setRowStretchFactor(0, 1)
        glw.ci.layout.setRowStretchFactor(1, 3)
        glw.ci.layout.setRowStretchFactor(2, 2)
        self.p_link.setTitle("Link to BS (bar = contact)")
        self.p_tp.setTitle("Data throughput")
        self.p_lat.setTitle("Data latency")
        self.p_tp.setLabel('left', 'bundles/s')
        self.p_lat.setLabel('left', 'latency', units='s')
        self.p_lat.setLabel('bottom', 'seconds (0 = now)')
        for p in (self.p_link, self.p_tp, self.p_lat):
            p.showGrid(x=True, y=True, alpha=0.25)
            p.setMouseEnabled(x=False, y=True)
            p.getAxis('left').setWidth(60)
        self.p_tp.setXLink(self.p_link)
        self.p_lat.setXLink(self.p_link)
        self.p_link.hideAxis('bottom')
        self.p_tp.hideAxis('bottom')
        self.p_link.setMouseEnabled(x=False, y=False)
        self.tp_legend = self.p_tp.addLegend(offset=(5, 5))
        split.addWidget(glw)

        self.p_topo = pg.PlotWidget(title="Topology (orange = ferried, red = direct delivery)")
        self.p_topo.hideAxis('left')
        self.p_topo.hideAxis('bottom')
        self.p_topo.setAspectLocked(True)
        self.p_topo.setMouseEnabled(False, False)
        split.addWidget(self.p_topo)
        split.setSizes([650, 350])
        return split

    def _build_map_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(w)
        lay.setContentsMargins(2, 2, 2, 2)
        site = self.site
        half = site.half if site else self.map_half

        self.map_vb = MapViewBox(self)
        self.p_map = pg.PlotWidget(viewBox=self.map_vb)
        self.p_map.setAspectLocked(True)
        self.p_map.showGrid(x=True, y=True, alpha=0.2)
        self.p_map.setRange(xRange=(-half, half), yRange=(-half, half), padding=0.02)
        self.p_map.setLabel('bottom', 'x / east', units='m')
        self.p_map.setLabel('left', 'y / north', units='m')
        self.p_map.setTitle(site.describe() if site else
                            "No --site: raw rover odometry (track colour = RSSI to BS)")
        lay.addWidget(self.p_map, 1)

        # layers, bottom to top
        rect = QtCore.QRectF(-half, -half, 2 * half, 2 * half)
        self.layer_bg = {}
        if site is not None:
            if site.terrain is not None:
                self.layer_bg['Elevation (shaded)'] = self._image_item(shaded_relief(site.terrain, site.cell), rect)
            if site.sat_path:
                pix = QtGui.QPixmap(site.sat_path)
                item = QtWidgets.QGraphicsPixmapItem(pix)
                item.setTransform(QtGui.QTransform(2 * half / pix.width(), 0, 0, -2 * half / pix.height(),
                                                   -half, half))
                item.setZValue(-100)
                self.p_map.addItem(item)
                self.layer_bg['Satellite'] = item
        self.layer_drv = None
        if site is not None and site.drivable is not None:
            rgba = np.zeros(site.drivable.shape + (4,), np.uint8)
            rgba[site.drivable] = (30, 110, 255, 255)
            self.layer_drv = self._image_item(rgba, rect, z=-90)
        self.layer_rm = {}
        for name, arr in (site.radio_maps.items() if site else []):
            self.layer_rm[name] = self._image_item(
                radio_rgba(arr if arr is not None else np.full((site.size, site.size), np.nan)), rect, z=-80)
        self.rm_version = 0

        self.track_lines = {}
        self.track_scatter = pg.ScatterPlotItem(size=7, pen=None)
        self.p_map.addItem(self.track_scatter)
        self.pose_dots = pg.ScatterPlotItem(size=15, pen=pg.mkPen('k', width=2))
        self.p_map.addItem(self.pose_dots)
        self.pose_arrows = {}
        self.map_labels = {}
        self.anchor_marks = pg.ScatterPlotItem(size=14, symbol='x', pen=pg.mkPen('k', width=2))
        self.p_map.addItem(self.anchor_marks)
        self.align_preview = self.p_map.plot([], [], pen=pg.mkPen('#d81b60', width=3))
        self.route_lines = {}                 # node -> route the rover was sent
        # relay rovers' current targets (relay_brain.py, from their @ROVER_STATUS rtx/rty)
        self.relay_marks = pg.ScatterPlotItem(size=17, symbol='d', pen=pg.mkPen('k', width=1.5))
        self.relay_marks.setZValue(9)
        self.p_map.addItem(self.relay_marks)
        self.route_preview = self.p_map.plot([], [], pen=pg.mkPen('#ff6f00', width=3, style=QtCore.Qt.DashLine))
        self.wp_marks = pg.ScatterPlotItem(size=13, symbol='o', brush=pg.mkBrush('#ff6f00'),
                                           pen=pg.mkPen('k', width=1.2))
        self.wp_marks.setZValue(10)
        self.p_map.addItem(self.wp_marks)
        self.wp_labels = []
        self.waypoints = []                   # [[x, y], ...] map metres, in driving order
        self.route_plan = None                # Nx2 preview from the selected rover's position
        self.bs_star = self.bs_text = None
        if site is not None and site.tx_xy:
            self.bs_star = pg.ScatterPlotItem([site.tx_xy[0]], [site.tx_xy[1]], size=22, symbol='star',
                                              brush=pg.mkBrush('#ffd54f'), pen=pg.mkPen('k', width=1.5))
            self.p_map.addItem(self.bs_star)
            self.bs_text = pg.TextItem("BS", color='k', anchor=(-0.3, 1.0))
            self.bs_text.setPos(*site.tx_xy)
            self.p_map.addItem(self.bs_text)
        self.hover_label = QtWidgets.QLabel(" ")
        self.p_map.scene().sigMouseMoved.connect(self._on_map_hover)

        side = QtWidgets.QWidget()
        side.setMaximumWidth(330)
        sl = QtWidgets.QVBoxLayout(side)
        sl.setContentsMargins(4, 0, 0, 0)

        lb = QtWidgets.QGroupBox("Layers")
        lg = QtWidgets.QFormLayout(lb)
        self.bg_box = QtWidgets.QComboBox()
        self.bg_box.addItems(list(self.layer_bg) + ['None'])
        self.bg_box.currentTextChanged.connect(self._apply_layers)
        lg.addRow("Background", self.bg_box)
        self.drv_check = QtWidgets.QCheckBox("Drivable area (blue)")
        self.drv_check.setChecked(True)
        self.drv_check.setEnabled(self.layer_drv is not None)
        self.drv_check.toggled.connect(self._apply_layers)
        lg.addRow(self.drv_check)
        self.rm_box = QtWidgets.QComboBox()
        self.rm_box.addItems(['off'] + list(self.layer_rm))
        self.rm_box.currentTextChanged.connect(self._apply_layers)
        lg.addRow("Predicted radio", self.rm_box)
        self.opacity = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.opacity.setRange(10, 100)
        self.opacity.setValue(45)
        self.opacity.valueChanged.connect(self._apply_layers)
        lg.addRow("Overlay opacity", self.opacity)
        self.rssi_check = QtWidgets.QCheckBox("Colour track by RSSI to BS")
        self.rssi_check.setChecked(True)
        self.rssi_check.toggled.connect(lambda: setattr(self, 'track_key', None))
        lg.addRow(self.rssi_check)
        row = QtWidgets.QHBoxLayout()
        clear = QtWidgets.QPushButton("Clear trail")
        clear.setToolTip("Hide the track so far (stats restart too); logs are not affected")
        clear.clicked.connect(self.on_clear_trail)
        row.addWidget(clear)
        fit = QtWidgets.QPushButton("Whole map")
        fit.clicked.connect(lambda: self.p_map.setRange(xRange=(-half, half), yRange=(-half, half)))
        row.addWidget(fit)
        lg.addRow(row)
        sl.addWidget(lb)

        bb = QtWidgets.QGroupBox("Base station")
        bl = QtWidgets.QVBoxLayout(bb)
        brow = QtWidgets.QHBoxLayout()
        self.bs_btn = QtWidgets.QPushButton("Move BS…")
        self.bs_btn.setCheckable(True)
        self.bs_btn.setStyleSheet("QPushButton:checked { background: #fff59d; font-weight: bold; }")
        self.bs_btn.setToolTip("Then click where the BS is. New radio maps (PMNet, then the full "
                               "RadioLunaDiff map) are generated for that spot in the background.")
        self.bs_btn.toggled.connect(lambda on: self.map_vb.setCursor(
            QtCore.Qt.CrossCursor if on else QtCore.Qt.ArrowCursor))
        brow.addWidget(self.bs_btn)
        bs_reset = QtWidgets.QPushButton("Reset BS")
        bs_reset.setToolTip("Back to the surveyed BS cell and the site's own radio maps")
        bs_reset.clicked.connect(self.on_reset_bs)
        brow.addWidget(bs_reset)
        bl.addLayout(brow)
        self.bs_label = QtWidgets.QLabel()
        self.bs_label.setWordWrap(True)
        self.bs_label.setTextFormat(QtCore.Qt.RichText)
        bl.addWidget(self.bs_label)
        bb.setEnabled(site is not None and site.base_rc is not None)
        sl.addWidget(bb)

        ab = QtWidgets.QGroupBox("Align rover to map")
        al = QtWidgets.QVBoxLayout(ab)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Rover"))
        self.rover_box = QtWidgets.QComboBox()
        self.rover_box.setMinimumWidth(90)
        top.addWidget(self.rover_box, 1)
        self.follow_check = QtWidgets.QCheckBox("Follow")
        top.addWidget(self.follow_check)
        al.addLayout(top)
        self.camera_check = QtWidgets.QCheckBox("Camera-only pose (no wheel fusion)")
        self.camera_check.setToolTip(
            "rover_nav.py pose source for the selected rover. Off (default): distance from the wheels, "
            "heading from the T265. On: the T265's own position (under-reads distance outdoors). "
            "Switching doesn't move the rover on the map. Shows what the rover reports.")
        self.camera_check.clicked.connect(self.on_camera_toggled)
        al.addWidget(self.camera_check)
        self.align_btn = QtWidgets.QPushButton("Place on map…")
        self.align_btn.setCheckable(True)
        self.align_btn.setStyleSheet("QPushButton:checked { background: #f8bbd0; font-weight: bold; }")
        self.align_btn.setToolTip("Then click where the rover is (keeps rotation), or press there and drag "
                                  "the way it faces (sets rotation too)")
        self.align_btn.toggled.connect(self._on_align_toggled)
        al.addWidget(self.align_btn)
        hint = QtWidgets.QLabel("Click = position only. Press on the rover and drag the way it faces = "
                                "position + heading. Nudges move/rotate about the placed point (x).")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #555;")
        al.addWidget(hint)

        grid = QtWidgets.QGridLayout()
        self.step_m = QtWidgets.QComboBox()
        self.step_m.addItems(['0.2 m', '0.5 m', '1 m', '5 m'])
        self.step_m.setCurrentIndex(1)
        self.step_deg = QtWidgets.QComboBox()
        self.step_deg.addItems(['0.5°', '1°', '5°', '15°'])
        self.step_deg.setCurrentIndex(1)
        buttons = [("↑", 0, 1, (0, 1, 0), "move north"), ("←", 1, 0, (-1, 0, 0), "move west"),
                   ("→", 1, 2, (1, 0, 0), "move east"), ("↓", 2, 1, (0, -1, 0), "move south"),
                   ("↺", 0, 3, (0, 0, 1), "rotate counter-clockwise about the placed point"),
                   ("↻", 0, 4, (0, 0, -1), "rotate clockwise about the placed point")]
        for text, r, c, (dx, dy, dth), tip in buttons:
            b = QtWidgets.QPushButton(text)
            b.setToolTip(tip)
            b.setFixedWidth(36)
            b.setAutoRepeat(True)
            b.clicked.connect(lambda _=False, a=(dx, dy, dth): self.on_nudge(*a))
            grid.addWidget(b, r, c)
        grid.addWidget(self.step_m, 1, 3, 1, 2)
        grid.addWidget(self.step_deg, 2, 3, 1, 2)
        reset = QtWidgets.QPushButton("Reset")
        reset.setToolTip("Map position = raw odometry")
        reset.clicked.connect(self.on_reset_alignment)
        grid.addWidget(reset, 2, 0)
        al.addLayout(grid)
        self.align_label = QtWidgets.QLabel()
        self.align_label.setWordWrap(True)
        self.align_label.setTextFormat(QtCore.Qt.RichText)
        al.addWidget(self.align_label)
        sl.addWidget(ab)
        sl.addWidget(self._build_route_box())
        sl.addWidget(self._build_relay_box())
        sl.addWidget(self.hover_label)
        sl.addStretch(1)
        # scrolls instead of growing past short (laptop) screens as the wrapped labels change
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(side)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        sb = scroll.style().pixelMetric(QtWidgets.QStyle.PM_ScrollBarExtent)
        scroll.setFixedWidth(side.maximumWidth() + sb)
        lay.addWidget(scroll)

        self.track_key = None
        self.trail_since = 0.0
        self.last_snap = None
        self._apply_layers()
        return w

    def _build_route_box(self) -> QtWidgets.QGroupBox:
        rb = QtWidgets.QGroupBox("Drive route (rover selected above)")
        rl = QtWidgets.QVBoxLayout(rb)
        row = QtWidgets.QHBoxLayout()
        self.wp_btn = QtWidgets.QPushButton("Add waypoints…")
        self.wp_btn.setCheckable(True)
        self.wp_btn.setStyleSheet("QPushButton:checked { background: #ffe0b2; font-weight: bold; }")
        self.wp_btn.setToolTip("Then click waypoints on the map in driving order. The route is planned "
                               "(A* on the drivable area) from the rover's current position.")
        self.wp_btn.toggled.connect(self._on_wp_toggled)
        row.addWidget(self.wp_btn, 1)
        undo = QtWidgets.QPushButton("Undo")
        undo.clicked.connect(self.on_wp_undo)
        row.addWidget(undo)
        clear = QtWidgets.QPushButton("Clear")
        clear.clicked.connect(self.on_wp_clear)
        row.addWidget(clear)
        rl.addLayout(row)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Speed"))
        self.route_speed = QtWidgets.QDoubleSpinBox()
        self.route_speed.setRange(0.2, bs_core.ROUTE_SPEED_MAX)
        self.route_speed.setSingleStep(0.05)
        self.route_speed.setValue(0.5)
        self.route_speed.setSuffix(" m/s")
        row.addWidget(self.route_speed)
        send = QtWidgets.QPushButton("Send route")
        send.setStyleSheet("font-weight: bold;")
        send.setToolTip("Replans from the rover's current position and sends it. The rover only drives "
                        "while RB is held on its controller; LB (manual) always overrides.")
        send.clicked.connect(self.on_send_route)
        row.addWidget(send)
        stop = QtWidgets.QPushButton("Stop")
        stop.setToolTip("Cancel the rover's route (only reaches it while in radio range; "
                        "releasing RB stops it anywhere)")
        stop.clicked.connect(self.on_stop_route)
        row.addWidget(stop)
        rl.addLayout(row)
        stop_all = QtWidgets.QPushButton("STOP ALL rovers (routes + relays)")
        stop_all.setStyleSheet("background: #c62828; color: white; font-weight: bold;")
        stop_all.setToolTip("Cancel every rover's route and turn every relay off (only reaches rovers in "
                            "radio range; unplugging the motor battery always works)")
        stop_all.clicked.connect(self.on_stop_all)
        rl.addWidget(stop_all)
        self.rb_check = QtWidgets.QCheckBox("Drive without RB (LB still overrides)")
        self.rb_check.setToolTip(
            "Selected rover: rover_nav.py drives routes and the relay without RB held, on an extra mux "
            "input above teleop. Its controller must stay connected (it stops if the controller drops); "
            "holding LB takes over. Shows what the rover reports. Needs the Autonomy input in mux.yaml.")
        self.rb_check.clicked.connect(self.on_rb_toggled)
        rl.addWidget(self.rb_check)
        self.route_label = QtWidgets.QLabel()
        self.route_label.setWordWrap(True)
        self.route_label.setTextFormat(QtCore.Qt.RichText)
        rl.addWidget(self.route_label)
        rb.setEnabled(self.site is not None and self.site.drivable is not None)
        return rb

    def _build_relay_box(self) -> QtWidgets.QGroupBox:
        b = QtWidgets.QGroupBox("Relay (rover selected above)")
        lay = QtWidgets.QVBoxLayout(b)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Explorer"))
        self.relay_explorer = QtWidgets.QComboBox()
        self.relay_explorer.setToolTip("The rover to keep linked to the BS")
        row.addWidget(self.relay_explorer, 1)
        row.addWidget(QtWidgets.QLabel("Ferry after"))
        self.relay_ferry = QtWidgets.QSpinBox()
        self.relay_ferry.setRange(0, 600)
        self.relay_ferry.setValue(15)
        self.relay_ferry.setSuffix(" s")
        self.relay_ferry.setToolTip("When the explorer has been silent for 10 s plus this long, the relay "
                                    "drives to its last known position until it is heard again")
        row.addWidget(self.relay_ferry)
        lay.addLayout(row)
        grid = QtWidgets.QGridLayout()
        self.relay_comfort = QtWidgets.QDoubleSpinBox()
        self.relay_comfort.setRange(-100.0, -40.0)
        self.relay_comfort.setSingleStep(1.0)
        self.relay_comfort.setDecimals(0)
        self.relay_comfort.setValue(relay_brain.DEFAULTS['comfort_dbm'])
        self.relay_comfort.setSuffix(" dBm")
        self.relay_comfort.setToolTip("Weaker link at or above this = the relay is fine where it is. Below it, "
                                      "a spot just 2 dB better is worth moving for")
        self.relay_gain = QtWidgets.QDoubleSpinBox()
        self.relay_gain.setRange(1.0, 30.0)
        self.relay_gain.setDecimals(0)
        self.relay_gain.setValue(relay_brain.DEFAULTS['gain_db'])
        self.relay_gain.setSuffix(" dB")
        self.relay_gain.setToolTip("A spot this much better than the current one is worth moving for even "
                                   "when the link is comfortable. Higher = moves less often")
        self.relay_predict = QtWidgets.QDoubleSpinBox()
        self.relay_predict.setRange(0.0, 20.0)
        self.relay_predict.setDecimals(0)
        self.relay_predict.setValue(relay_brain.DEFAULTS['predict_s'])
        self.relay_predict.setSuffix(" s")
        self.relay_predict.setToolTip("The explorer's position is predicted this far ahead from its recent "
                                      "movement (0 = use where it is)")
        grid.addWidget(QtWidgets.QLabel("Comfort"), 0, 0)
        grid.addWidget(self.relay_comfort, 0, 1)
        grid.addWidget(QtWidgets.QLabel("Gain"), 0, 2)
        grid.addWidget(self.relay_gain, 0, 3)
        self.relay_model = QtWidgets.QComboBox()
        self.relay_model.addItems(['nn_pmnet', 'nn', 'distance'])
        self.relay_model.setToolTip("Link model on the relay rover: nn_pmnet = PMNet link table (default, "
                                    "smoother), nn = with the diffusion residual (noisier), distance = no "
                                    "table, roughly the midpoint. Tables must be in the rover's terrain folder")
        grid.addWidget(QtWidgets.QLabel("Look-ahead"), 1, 0)
        grid.addWidget(self.relay_predict, 1, 1)
        grid.addWidget(QtWidgets.QLabel("Model"), 1, 2)
        grid.addWidget(self.relay_model, 1, 3)
        lay.addLayout(grid)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Speed"))
        self.relay_speed = QtWidgets.QDoubleSpinBox()
        self.relay_speed.setRange(0.2, bs_core.ROUTE_SPEED_MAX)
        self.relay_speed.setSingleStep(0.05)
        self.relay_speed.setValue(0.5)
        self.relay_speed.setSuffix(" m/s")
        row.addWidget(self.relay_speed)
        start = QtWidgets.QPushButton("Start relay")
        start.setStyleSheet("font-weight: bold;")
        start.setToolTip("The selected rover keeps choosing a spot between the BS and the explorer and "
                         "drives there (radio_daemon.py needs --site). Press again to apply changed "
                         "settings (same explorer: the relay keeps its target).")
        start.clicked.connect(self.on_start_relay)
        row.addWidget(start)
        stop = QtWidgets.QPushButton("Stop relays")
        stop.setToolTip("Every relay off and its route stopped, plus the selected rover's route, whichever "
                        "rover is selected above")
        stop.clicked.connect(self.on_stop_relays)
        row.addWidget(stop)
        lay.addLayout(row)
        self.relay_label = QtWidgets.QLabel()
        self.relay_label.setWordWrap(True)
        self.relay_label.setTextFormat(QtCore.Qt.RichText)
        lay.addWidget(self.relay_label)
        b.setEnabled(self.site is not None and self.site.drivable is not None)
        return b

    def _image_item(self, arr, rect, z=-100):
        item = pg.ImageItem(arr[::-1], levels=(0, 255) if arr.dtype == np.uint8 else None)
        item.setRect(rect)
        item.setZValue(z)
        self.p_map.addItem(item)
        return item

    def _apply_layers(self, *_):
        for name, item in self.layer_bg.items():
            item.setVisible(self.bg_box.currentText() == name)
        op = self.opacity.value() / 100.0
        if self.layer_drv is not None:
            self.layer_drv.setVisible(self.drv_check.isChecked())
            self.layer_drv.setOpacity(op)
        for name, item in self.layer_rm.items():
            item.setVisible(self.rm_box.currentText() == name and self.site.radio_maps.get(name) is not None)
            item.setOpacity(min(1.0, op + 0.3))

    def color(self, nid: str) -> str:
        if nid not in self.colors:
            self.colors[nid] = PALETTE[len(self.colors) % len(PALETTE)]
        return self.colors[nid]

    # actions
    def _cfg(self) -> dict:
        cfg = {}
        for k, w in self.cfg_widgets.items():
            cfg[k] = w.currentText() if isinstance(w, QtWidgets.QComboBox) else w.value()
        cfg['payload_bytes'] = int(cfg['payload_bytes'])
        if (cfg['comms_on'] > 0) != (cfg['comms_off'] > 0):
            QtWidgets.QMessageBox.warning(self, "Periodic mode",
                                          "Periodic mode needs both Radio on and Radio off > 0.\n"
                                          "Sending with the radio always on.")
            cfg['comms_on'] = cfg['comms_off'] = 0.0
        return cfg

    def on_start(self) -> None:
        cfg = self._cfg()
        exp = self.core.start_experiment(self.exp_edit.text(), cfg)
        self.exp_edit.setText(exp)

    def on_apply(self) -> None:
        if not self.core.apply_live(self._cfg()):
            QtWidgets.QMessageBox.information(self, "Apply live", "No experiment is running. Use Start.")

    def on_standby(self) -> None:
        self.core.standby(self._cfg())

    def on_ros(self, on: bool) -> None:
        rows = sorted({ix.row() for ix in self.table.selectionModel().selectedRows()})
        nodes = [self.table.item(r, 0).text() for r in rows if self.table.item(r, 0)]
        verb = "Start" if on else "Stop"
        if not nodes:
            nodes = [self.table.item(r, 0).text() for r in range(self.table.rowCount())
                     if self.table.item(r, 0)]
            if not nodes:
                QtWidgets.QMessageBox.information(self, "ROS stack", "No rovers known yet.")
                return
            ans = QtWidgets.QMessageBox.question(
                self, "ROS stack", f"No rover selected. {verb} the ROS stack on all rovers "
                                   f"({', '.join(nodes)})?")
            if ans != QtWidgets.QMessageBox.Yes:
                return
        self.core.set_ros(nodes, on)

    def on_marker(self) -> None:
        self.core.add_marker(self.marker_edit.text())
        self.marker_edit.clear()

    def on_save(self) -> None:
        path = self.core.save_summary()
        if path:
            self.statusBar().showMessage(f"summary saved: {path}", 5000)

    def on_new_session(self) -> None:
        self.core.new_session('')

    # refresh
    def refresh(self) -> None:
        window = self.window_box.currentData()
        bin_s = dict(TIMELINE_CHOICES)[window]
        snap = self.core.snapshot(since_event=self.last_event_id, history_s=window)
        now = snap['now']
        self._refresh_status(snap)
        self._refresh_control(snap, now)
        self._refresh_table(snap)
        self._refresh_events(snap)
        self._refresh_timeline(snap, now, window, bin_s)
        self._refresh_topology(snap, now)
        self._refresh_map(snap, now)

    def _refresh_status(self, snap) -> None:
        age = snap['last_line_age']
        if snap['connected']:
            self.conn_label.setText(f"<b style='color:#2e7d32'>{snap['port']} connected</b>  "
                                    f"{snap['lines_total']} lines, last {fmt_age(age)} ago")
        else:
            self.conn_label.setText(f"<b style='color:#c62828'>{snap['port']} not connected</b> (retrying)")
        self.session_label.setText(snap['session_dir'])
        self.fw_warning.setVisible(snap['bs_rejects_control'])

    def _refresh_control(self, snap, now) -> None:
        c = snap['control']
        msg = c['msg']
        if msg is None:
            text = "No control sent yet: rovers keep their last state (they boot into standby)."
        else:
            state = msg.get('state', 'no fleet state set').upper()
            exp = f" <b>'{msg['exp']}'</b>" if msg.get('exp') else ""
            starts = ""
            if msg.get('state') == 'active' and msg.get('start_at', 0) > now:
                starts = f", starts in {msg['start_at'] - now:.0f} s"
            text = (f"Fleet state v{c['version']}: <b>{state}</b>{exp}{starts} "
                    f"(set {fmt_age(now - c['sent_at'])} ago, re-sent every {bs_core.CONTROL_PERIOD_S:g} s)")
            if msg.get('ros'):
                text += "<br>ROS requested: " + ', '.join(f"{n} {w}" for n, (w, _id) in
                                                          sorted(msg['ros'].items()))
            pending = [r['node'] for r in snap['rovers']
                       if int(r['status'].get('cv', 0) or 0) < c['version']]
            if pending:
                text += (f"<br><span style='color:#ef6c00'>waiting for: {', '.join(pending)}</span>"
                         " (out of range, in an off window, or not yet reported)")
            elif snap['rovers']:
                text += "<br><span style='color:#2e7d32'>all rovers have applied it</span>"
        self.ctrl_label.setText(text)

    def _cell(self, row, col, text, bg=None, fg=None) -> None:
        item = QtWidgets.QTableWidgetItem(str(text))
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        if bg:
            item.setBackground(QtGui.QColor(bg))
        if fg:
            item.setForeground(QtGui.QColor(fg))
        self.table.setItem(row, col, item)

    def _refresh_table(self, snap) -> None:
        rows = snap['rovers']
        version = snap['control']['version']
        self.table.setRowCount(len(rows))
        for i, r in enumerate(rows):
            st = r['status']
            num = lambda k: f"{st[k]:.0f}" if isinstance(st.get(k), float) else '-'
            cv = int(st.get('cv', 0) or 0)
            win = r['window']
            self._cell(i, 0, r['node'], fg=self.color(r['node']))
            self._cell(i, 1, "UP" if r['link_up'] else "DOWN", bg='#c8e6c9' if r['link_up'] else '#ffcdd2')
            self._cell(i, 2, fmt_age(r['link_for']))
            self._cell(i, 3, f"{r['rssi']} dBm" if r['rssi'] is not None else '-')
            state = st.get('st', '-')
            self._cell(i, 4, state, bg={'active': '#c8e6c9', 'pending': '#fff9c4'}.get(state))
            self._cell(i, 5, st.get('exp', '-'))
            self._cell(i, 6, f"v{cv}" if cv else '-',
                       bg=None if not version else ('#c8e6c9' if cv >= version else '#ffe0b2'))
            # rover clock vs BS: the rover corrects for it, this just shows how far off it was
            ck = st.get('ck')
            if isinstance(ck, float):
                self._cell(i, 7, f"{ck:+.1f} s", bg='#fff9c4' if abs(ck) > 2 else None)
            else:
                self._cell(i, 7, 'unsynced' if ck == 'na' else '-', bg='#ffe0b2' if ck == 'na' else None)
            ros = st.get('ros', '-')
            # loc (fused | wheel | wait) only from rovers running --ros-wheel-topic
            loc = st.get('loc')
            bg = {'on': '#c8e6c9', 'stopping': '#fff9c4'}.get(ros)
            if ros == 'on' and loc in ('wheel', 'wait'):
                bg = '#fff9c4'                    # stack up but no T265 heading (yet)
            self._cell(i, 8, f"{ros}, loc {loc}" if loc else ros, bg=bg)
            self._cell(i, 9, '-' if win is None else f"{'ON' if win[0] else 'OFF'} {win[1]:.0f}s left",
                       bg=None if win is None else ('#c8e6c9' if win[0] else '#eeeeee'))
            self._cell(i, 10, f"{num('q')}/{num('qf')}")
            for col, k in ((11, 'gen'), (12, 'tx'), (13, 'retx'), (14, 'ack')):
                self._cell(i, col, num(k))
            self._cell(i, 15, r['data_60s'])
            self._cell(i, 16, r['data_total'])
            self._cell(i, 17, f"{r['avg_lat']:.0f} ms" if r['avg_lat'] is not None else '-')
            age = r['status_age']
            self._cell(i, 18, fmt_age(age), fg='#c62828' if age is not None and age > 15 else None)

    def _refresh_events(self, snap) -> None:
        for eid, t, cat, text in snap['events']:
            self.last_event_id = eid
            color = EVENT_COLORS.get(cat, '#000000')
            self.log.appendHtml(f"<span style='color:#888'>{datetime.fromtimestamp(t):%H:%M:%S}</span> "
                                f"<span style='color:{color}'><b>[{cat}]</b> {text}</span>")

    def _refresh_timeline(self, snap, now, window, bin_s) -> None:
        nodes = sorted(set(snap['spans']) | {x[1] for x in snap['timeline']})
        for p in (self.p_link, self.p_tp, self.p_lat):
            p.setXRange(-window, 0, padding=0)

        # link lanes
        for item in self.link_bars:
            self.p_link.removeItem(item)
        self.link_bars = []
        for i, nid in enumerate(nodes):
            sp = [(max(a, now - window) - now, b - now) for a, b in snap['spans'].get(nid, [])]
            if sp:
                bar = pg.BarGraphItem(x0=[a for a, _b in sp], width=[max(0.3, b - a) for a, b in sp],
                                      y0=i + 0.1, height=0.8, brush=self.color(nid), pen=None)
                self.p_link.addItem(bar)
                self.link_bars.append(bar)
        self.p_link.setYRange(0, max(1, len(nodes)), padding=0.05)
        self.p_link.getAxis('left').setTicks([[(i + 0.5, n) for i, n in enumerate(nodes)]])

        # throughput (bins aligned to absolute time so they don't jitter)
        last_edge = (math.floor(now / bin_s) + 1) * bin_s
        n_bins = int(window // bin_s) + 1
        first_edge = last_edge - n_bins * bin_s
        edges = np.arange(n_bins + 1) * bin_s + first_edge - now
        for nid in nodes:
            ts = np.array([x[0] for x in snap['timeline'] if x[1] == nid and x[2] == 'data'])
            counts = np.histogram(ts, bins=n_bins, range=(first_edge, last_edge))[0] if len(ts) else np.zeros(n_bins)
            if nid not in self.tp_curves:
                self.tp_curves[nid] = self.p_tp.plot(stepMode='center', pen=pg.mkPen(self.color(nid), width=2),
                                                     fillLevel=0, brush=pg.mkBrush(self.color(nid) + '30'),
                                                     name=nid)
                self.lat_scatters[nid] = pg.ScatterPlotItem(size=6, pen=None,
                                                            brush=pg.mkBrush(self.color(nid)))
                self.p_lat.addItem(self.lat_scatters[nid])
            self.tp_curves[nid].setData(edges, counts / float(bin_s))
            pts = [(x[0] - now, x[3] / 1000.0) for x in snap['timeline'] if x[1] == nid and x[2] == 'data']
            self.lat_scatters[nid].setData([p[0] for p in pts], [p[1] for p in pts])
        self.p_tp.setTitle(f"Data throughput ({bin_s} s bins)")

        # markers (rebuilt only when they change)
        key = (len(snap['markers']), snap['markers'][-1][0] if snap['markers'] else 0)
        if key != self.marker_key:
            self.marker_key = key
            for p, line in self.marker_lines:
                p.removeItem(line)
            self.marker_lines = []
            for idx, (t, label) in enumerate(snap['markers']):
                for p in (self.p_link, self.p_tp, self.p_lat):
                    # stagger label heights so nearby markers stay readable
                    line = pg.InfiniteLine(angle=90, pen=pg.mkPen('k', style=QtCore.Qt.DashLine),
                                           label=label if p is self.p_tp else None,
                                           labelOpts={'position': 0.92 - 0.1 * (idx % 5), 'color': 'k',
                                                      'fill': (255, 255, 255, 200)})
                    line.marker_t = t
                    p.addItem(line)
                    self.marker_lines.append((p, line))
        for _p, line in self.marker_lines:
            line.setValue(line.marker_t - now)

    def _refresh_topology(self, snap, now) -> None:
        p = self.p_topo
        p.clear()
        nodes = sorted(snap['nodes'])
        rovers = [n for n in nodes if n != str(BS_NODE_ID)]
        pos = {str(BS_NODE_ID): (0.0, 0.0)}
        for i, n in enumerate(rovers):
            a = math.pi / 2 + 2 * math.pi * i / max(1, len(rovers))
            pos[n] = (math.cos(a), math.sin(a))
        for node, (parent, rssi) in snap['edges'].items():
            if node in pos and parent in pos:
                (x1, y1), (x2, y2) = pos[node], pos[parent]
                p.plot([x1, x2], [y1, y2], pen=pg.mkPen('#9e9e9e', width=2))
                t = pg.TextItem(f"{rssi} dBm", color='#1565c0', anchor=(0.5, 0.5))
                t.setPos((x1 + x2) / 2, (y1 + y2) / 2)
                p.addItem(t)
        for tr in snap['transfers']:
            a, b = (tr['via'], tr['dst']) if tr['ferry'] else (tr['src'], tr['dst'])
            if a in pos and b in pos and (tr['ferry'] or now - tr['time'] < 2.0):
                p.plot([pos[a][0], pos[b][0]], [pos[a][1], pos[b][1]],
                       pen=pg.mkPen('#ef6c00' if tr['ferry'] else '#d32f2f', width=4,
                                    style=QtCore.Qt.DashLine if tr['ferry'] else QtCore.Qt.SolidLine))
        if pos:
            xs = [pos[n][0] for n in pos]
            ys = [pos[n][1] for n in pos]
            brushes = [pg.mkBrush('#ffd54f' if n == str(BS_NODE_ID) else self.color(n)) for n in pos]
            p.addItem(pg.ScatterPlotItem(xs, ys, size=34, brush=brushes, pen=pg.mkPen('k', width=1)))
            for n, (x, y) in pos.items():
                t = pg.TextItem(("BS " if n == str(BS_NODE_ID) else "") + n, color='k', anchor=(0.5, -0.6))
                t.setPos(x, y)
                p.addItem(t)
        p.setXRange(-1.5, 1.5)
        p.setYRange(-1.5, 1.5)

    def _refresh_map(self, snap, now) -> None:
        self.last_snap = snap
        locs = {n: l for n, l in snap['locations'].items() if l['x'] != 0.0 or l['y'] != 0.0}
        aligns = snap['alignments']

        sel = self.rover_box.currentText()
        names = sorted(locs)
        if names != [self.rover_box.itemText(i) for i in range(self.rover_box.count())]:
            self.rover_box.blockSignals(True)
            self.rover_box.clear()
            self.rover_box.addItems(names)
            if sel in names:
                self.rover_box.setCurrentText(sel)
            self.rover_box.blockSignals(False)

        # tracks: rebuilt only when a sample arrives, the alignment or the trail cut changes
        key = (tuple(sorted(snap['track_total'].items())),
               tuple(sorted((n, tuple(sorted(a.items(), key=str))) for n, a in aligns.items())),
               self.trail_since, self.rssi_check.isChecked())
        if key != self.track_key:
            self.track_key = key
            spots = []
            for n, tr in self.core.get_tracks().items():
                tr = [p for p in tr if p[0] >= self.trail_since]
                if n not in self.track_lines:
                    self.track_lines[n] = self.p_map.plot([], [], pen=pg.mkPen(self.color(n), width=2))
                if not tr:
                    self.track_lines[n].setData([], [])
                    continue
                mx, my, _ = site_map.to_map(aligns.get(n, site_map.IDENTITY),
                                            [p[1] for p in tr], [p[2] for p in tr])
                self.track_lines[n].setData(mx, my)
                if self.rssi_check.isChecked():
                    spots += [{'pos': (x, y), 'brush': pg.mkBrush(rssi_color(p[4]))}
                              for x, y, p in zip(mx, my, tr) if p[4] is not None]
            self.track_scatter.setData(spots)

        # current poses
        xs, ys, brushes, anchors = [], [], [], []
        for n, l in locs.items():
            al = aligns.get(n, site_map.IDENTITY)
            mx, my, myaw = site_map.to_map(al, l['x'], l['y'], l['yaw'])
            mx, my = float(mx), float(my)
            stale = now - l['updated_at'] > 5.0
            xs.append(mx)
            ys.append(my)
            brushes.append(pg.mkBrush('#bdbdbd' if stale else self.color(n)))
            if 'anchor' in al:
                anchors.append(al['anchor'])
            if n not in self.map_labels:
                self.map_labels[n] = pg.TextItem(n, color='k', anchor=(-0.3, 1.3))
                self.p_map.addItem(self.map_labels[n])
                self.pose_arrows[n] = pg.ArrowItem(headLen=26, tailLen=0, headWidth=16,
                                                   brush=self.color(n), pen=pg.mkPen('k', width=1.5))
                self.p_map.addItem(self.pose_arrows[n])
            self.map_labels[n].setPos(mx, my)
            arrow = self.pose_arrows[n]
            arrow.setVisible(myaw is not None)
            if myaw is not None:
                # ArrowItem points along -x at angle 0 and rotates clockwise on screen
                arrow.setStyle(angle=180.0 - float(myaw))
                arrow.setPos(mx, my)
        self.pose_dots.setData(xs, ys, brush=brushes)
        self.anchor_marks.setData([a[0] for a in anchors], [a[1] for a in anchors])
        self._refresh_bs(snap.get('bs') or {}, now)
        sel = self.rover_box.currentText()
        if self.follow_check.isChecked() and sel in locs:
            i = list(locs).index(sel)
            r = self.map_vb.viewRect()
            self.map_vb.translateBy(x=xs[i] - r.center().x(), y=ys[i] - r.center().y())
        self._refresh_align_label(snap, locs, now)
        self._refresh_routes(snap)
        self._refresh_relay(snap, names)
        self._refresh_pose_mode(snap)
        self._refresh_rb(snap)

    def _refresh_bs(self, bs, now) -> None:
        """BS star, radio-map layers (rebuilt when the BS or its maps change) and status."""
        if not bs or self.site is None:
            self.bs_label.setText("<i>No BS cell for this site.</i>" if self.site else "")
            return
        if bs['rm_version'] != self.rm_version:
            self.rm_version = bs['rm_version']
            for name, item in self.layer_rm.items():
                arr = self.site.radio_maps.get(name)
                if arr is not None:
                    item.setImage(radio_rgba(arr)[::-1], levels=(0, 255))
            self.bs_star.setData([bs['x']], [bs['y']])
            self.bs_text.setPos(bs["x"], bs["y"])
            self.p_map.setTitle(self.site.describe())
            self._apply_layers()
        job = bs['rm_job']
        have = [m for m, a in self.site.radio_maps.items() if a is not None]
        lines = [f"<b>({bs['x']:+.1f}, {bs['y']:+.1f}) m</b>"
                 + (" - moved" if bs['moved'] else " - surveyed position")]
        if job['status'] == 'running':
            lines.append(f"<span style='color:#ef6c00'>generating {', '.join(job['missing'])}… "
                         f"{now - job['started']:.0f} s</span>")
        elif job['status'] == 'error':
            lines.append(f"<span style='color:#c62828'>radio maps failed: {job['error']}</span>")
        lines.append("maps: " + (", ".join(have) if have else "none"))
        self.bs_label.setText("<br>".join(lines))

    def move_bs_to(self, x, y) -> None:
        self.bs_btn.setChecked(False)
        if not self.core.move_bs(x, y):
            QtWidgets.QMessageBox.information(self, "Move BS", "That point is outside the site map.")

    def on_reset_bs(self) -> None:
        self.bs_btn.setChecked(False)
        self.core.reset_bs()

    def _refresh_align_label(self, snap, locs, now) -> None:
        n = self.rover_box.currentText()
        if not n or n not in locs:
            self.align_label.setText("<i>No rover location yet. Start radio_daemon.py with "
                                     "--ros-location-topic.</i>")
            return
        l, al = locs[n], snap['alignments'].get(n, site_map.IDENTITY)
        mx, my, myaw = site_map.to_map(al, l['x'], l['y'], l['yaw'])
        yaw = f"{l['yaw']:+.1f}°" if l['yaw'] is not None else "<span style='color:#c62828'>none</span>"
        head = f", heading {float(myaw):+.0f}°" if myaw is not None else ""
        age = now - l['updated_at']
        lines = [f"odom ({l['x']:+.2f}, {l['y']:+.2f}) yaw {yaw}",
                 f"<b>map ({float(mx):+.1f}, {float(my):+.1f}){head}</b>"
                 + (f" <span style='color:#c62828'>({age:.0f} s old)</span>" if age > 5 else ""),
                 f"alignment ({al['x']:+.2f}, {al['y']:+.2f}) m, rotation {al['theta']:+.1f}°"]
        if self.site is not None:
            on = self.site.on_drivable(mx, my)
            lines.append("on drivable area" if on else "<span style='color:#c62828'>off the drivable area</span>")
        st = self.core.track_stats(n, self.trail_since)
        if st:
            s = f"trail: {st['n']} pts, {st['length_m']:.0f} m"
            if 'on_drivable' in st:
                s += (f", <b>{100 * st['on_drivable']:.0f}%</b> on drivable "
                      f"({100 * st['on_drivable_1m']:.0f}% within 1 m)")
            lines.append(s)
            if 'to_anchor_m' in st:
                lines.append(f"distance to placed point (x): <b>{st['to_anchor_m']:.1f} m</b>"
                             " (drift when back on the start mark)")
        if l['yaw'] is None:
            lines.append("<span style='color:#c62828'>rover sends no heading: update radio_daemon.py; "
                         "only click-to-place works</span>")
        self.align_label.setText("<br>".join(lines))

    def _refresh_routes(self, snap) -> None:
        routes = snap.get('routes', {})
        for n in set(self.route_lines) | set(routes):
            if n not in self.route_lines:
                self.route_lines[n] = self.p_map.plot([], [], pen=pg.mkPen(self.color(n), width=4))
                self.route_lines[n].setOpacity(0.55)
            pts = routes.get(n, {}).get('map') or []
            self.route_lines[n].setData([p[0] for p in pts], [p[1] for p in pts])
        n = self.rover_box.currentText()
        st = next((r['status'] for r in snap['rovers'] if r['node'] == n), {})
        lines = []
        if self.route_plan is not None:
            lines.append(f"plan: <b>{route_plan.length(self.route_plan):.0f} m</b> through "
                         f"{len(self.waypoints)} waypoint(s)")
        elif self.waypoints:
            lines.append(f"{len(self.waypoints)} waypoint(s), no plan")
        nav = st.get('nav')
        if nav:
            nav = str(nav)
            prog = f" {st.get('ni', 0):.0f}/{st.get('np', 0):.0f} m" if st.get('np') else ''
            xte = f", {st['xte']:.1f} m off route" if isinstance(st.get('xte'), float) else ''
            color = ('#2e7d32' if nav == 'following' else '#1565c0' if nav == 'done'
                     else '#c62828' if nav.startswith('stopped') or nav == 'no_joy' else '#ef6c00')
            lines.append(f"rover: <b style='color:{color}'>{nav}</b>{prog}{xte}")
        elif n:
            lines.append("<i>rover reports no route state (rover_nav.py not running?)</i>")
        self.route_label.setText("<br>".join(lines))

    def _refresh_relay(self, snap, names) -> None:
        """Explorer list, relay targets on the map and the selected rover's relay state."""
        n = self.rover_box.currentText()
        others = [m for m in names if m != n]
        if others != [self.relay_explorer.itemText(i) for i in range(self.relay_explorer.count())]:
            cur = self.relay_explorer.currentText()
            self.relay_explorer.blockSignals(True)
            self.relay_explorer.clear()
            self.relay_explorer.addItems(others)
            if cur in others:
                self.relay_explorer.setCurrentText(cur)
            self.relay_explorer.blockSignals(False)
        cfg = snap.get('relay_cfg', {})
        status = {r['node']: r['status'] for r in snap['rovers']}
        spots = []
        for node in cfg:
            st = status.get(node, {})
            if isinstance(st.get('rtx'), float) and isinstance(st.get('rty'), float):
                spots.append({'pos': (st['rtx'], st['rty']), 'brush': pg.mkBrush(self.color(node))})
        self.relay_marks.setData(spots)
        if not n:
            self.relay_label.setText("")
            return
        st = status.get(n, {})
        lines = []
        if n in cfg:
            ex, speed, ferry = cfg[n][:3]
            lines.append(f"relay for <b>{ex}</b> at {speed:g} m/s, ferry after {ferry:g} s")
            if len(cfg[n]) >= 6:
                lines.append(f"comfort {cfg[n][3]:g} dBm, gain {cfg[n][4]:g} dB, look-ahead {cfg[n][5]:g} s")
            if len(cfg[n]) == 7 and st.get('rm') and str(st['rm']) != cfg[n][6]:
                lines.append(f"<span style='color:#c62828'>rover uses model {st['rm']}: no {cfg[n][6]} "
                             f"table on it</span>")
            elif st.get('rm'):
                lines.append(f"model {st['rm']}")
            rl = st.get('rl')
            if rl is None:
                lines.append("<i>rover hasn't reported a relay state yet</i>")
            else:
                rl = str(rl)
                color = ('#2e7d32' if rl in ('move', 'hold', 'at_target') else '#1565c0' if rl == 'ferry'
                         else '#c62828')
                lines.append(f"rover: <b style='color:{color}'>{rl}</b>"
                             + (f", explorer fix {st['rea']:.0f} s old" if isinstance(st.get('rea'), float) else ''))
                pred = [(k, lab) for k, lab in (('rsc', 'target'), ('rcu', 'here'), ('rdi', 'explorer direct'))
                        if isinstance(st.get(k), float)]
                if pred:
                    lines.append("predicted weaker link: " + ", ".join(f"{lab} {st[k]:.0f} dBm" for k, lab in pred))
                hints = {'unavailable': "radio_daemon.py needs --site (and numpy) for the relay role",
                         'no_align': "place the relay rover on the map first",
                         'no_pose': "no position from the relay rover",
                         'wait_ex': "no position from the explorer yet (place it on the map)",
                         'no_bs': "rover doesn't know the BS position yet",
                         'wrong_site': "rover was started with a different --site",
                         'no_route': "no drivable route to the chosen spot"}
                if rl in hints:
                    lines.append(f"<span style='color:#c62828'>{hints[rl]}</span>")
        elif st.get('rl') == 'unavailable':
            lines.append("<span style='color:#c62828'>relay role unavailable on this rover "
                         "(radio_daemon.py without --site or numpy)</span>")
        others_on = [f"<b>{m}</b> (for {c[0]})" for m, c in sorted(cfg.items()) if m != n]
        if others_on:
            lines.append("<span style='color:#e65100'>relay also on: " + ", ".join(others_on)
                         + " - select it to see it; Stop relays stops all</span>")
        self.relay_label.setText("<br>".join(lines))

    def on_start_relay(self) -> None:
        n, ex = self.rover_box.currentText(), self.relay_explorer.currentText()
        if not n or not ex:
            QtWidgets.QMessageBox.information(self, "Relay", "Select the relay rover above and an explorer "
                                                             "(both need a location).")
            return
        if self.core.relay_cfg.get(n, [None])[0] == ex:
            # already relaying for this explorer: just apply the settings, no confirmation
            err = self.core.set_relay(n, ex, self.relay_speed.value(), self.relay_ferry.value(),
                                      self.relay_comfort.value(), self.relay_gain.value(),
                                      self.relay_predict.value(), self.relay_model.currentText())
            if err:
                QtWidgets.QMessageBox.warning(self, "Relay", err)
            return
        ans = QtWidgets.QMessageBox.question(
            self, "Start relay",
            f"Rover {n} will keep driving to a spot between the BS and rover {ex} at "
            f"{self.relay_speed.value():.2f} m/s.\n\n" + self._drive_note(n))
        if ans != QtWidgets.QMessageBox.Yes:
            return
        err = self.core.set_relay(n, ex, self.relay_speed.value(), self.relay_ferry.value(),
                                  self.relay_comfort.value(), self.relay_gain.value(), self.relay_predict.value(),
                                  self.relay_model.currentText())
        if err:
            QtWidgets.QMessageBox.warning(self, "Relay", err)

    def _refresh_pose_mode(self, snap) -> None:
        """Checkbox = the mode the rover reports (pm), else what we last asked for."""
        n = self.rover_box.currentText()
        st = next((r['status'] for r in snap['rovers'] if r['node'] == n), {})
        mode = st.get('pm') or self.core.pose_modes.get(n)
        wanted = self.core.pose_modes.get(n)
        self.camera_check.setEnabled(bool(n))
        self.camera_check.blockSignals(True)
        self.camera_check.setChecked(mode == 'camera')
        self.camera_check.blockSignals(False)
        pending = wanted is not None and st.get('pm') is not None and st.get('pm') != wanted
        self.camera_check.setText("Camera-only pose (no wheel fusion)" + (" - switching…" if pending else ""))

    def _refresh_rb(self, snap) -> None:
        """Checkbox = what the rover reports (rb=0/1), else what we last asked for."""
        n = self.rover_box.currentText()
        st = next((r['status'] for r in snap['rovers'] if r['node'] == n), {})
        wanted_off = n in snap.get('rb_off', {})
        rep = st.get('rb')
        off = (rep == 0.0) if isinstance(rep, float) else wanted_off
        self.rb_check.setEnabled(bool(n))
        self.rb_check.blockSignals(True)
        self.rb_check.setChecked(off)
        self.rb_check.blockSignals(False)
        pending = isinstance(rep, float) and (rep == 0.0) != wanted_off
        self.rb_check.setText("Drive without RB (LB still overrides)" + (" - switching…" if pending else ""))
        self.rb_check.setStyleSheet("color: #c62828; font-weight: bold;" if off else "")

    def on_rb_toggled(self, off: bool) -> None:
        n = self.rover_box.currentText()
        if not n:
            return
        if off:
            ans = QtWidgets.QMessageBox.warning(
                self, "Drive without RB",
                f"Rover {n} will drive routes and relay moves without anyone holding RB.\n\n"
                "Keep its controller connected and within reach: holding LB takes over, Stop on the "
                "dashboard cancels the route (in radio range only), and it stops if the controller "
                "drops. Needs the Autonomy input in mux.yaml on that car.",
                QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
            if ans != QtWidgets.QMessageBox.Ok:
                self.rb_check.setChecked(False)
                return
        self.core.set_rb_required(n, not off)

    def _drive_note(self, n: str) -> str:
        if n in self.core.rb_off:
            return ("It drives WITHOUT RB held (controller must stay connected). Hold LB to take over; "
                    "Stop cancels.")
        return "It drives only while RB is held on its controller. Release RB to stop; LB overrides."

    def on_camera_toggled(self, on: bool) -> None:
        n = self.rover_box.currentText()
        if n:
            self.core.set_pose_mode(n, 'camera' if on else 'fused')

    def _on_wp_toggled(self, on: bool) -> None:
        if on:
            self.align_btn.setChecked(False)
            self.bs_btn.setChecked(False)
        self.map_vb.setCursor(QtCore.Qt.CrossCursor if on else QtCore.Qt.ArrowCursor)

    def add_waypoint(self, x, y) -> None:
        self.waypoints.append([float(x), float(y)])
        self._replan()

    def on_wp_undo(self) -> None:
        if self.waypoints:
            self.waypoints.pop()
        self._replan()

    def on_wp_clear(self) -> None:
        self.waypoints = []
        self._replan()

    def _replan(self) -> bool:
        """Preview route from the selected rover through the waypoints; False (and the reason
        in the status bar) when it can't be planned."""
        xs, ys = zip(*self.waypoints) if self.waypoints else ((), ())
        self.wp_marks.setData(list(xs), list(ys))
        for t in self.wp_labels:
            self.p_map.removeItem(t)
        self.wp_labels = []
        for i, (x, y) in enumerate(self.waypoints):
            t = pg.TextItem(str(i + 1), color='k', anchor=(-0.4, 1.2))
            t.setPos(x, y)
            self.p_map.addItem(t)
            self.wp_labels.append(t)
        self.route_plan = None
        self.route_preview.setData([], [])
        n = self.rover_box.currentText()
        if not self.waypoints or not n:
            return False
        pts, err = self.core.plan_route(n, self.waypoints)
        if pts is None:
            self.statusBar().showMessage(f"route: {err}", 6000)
            return False
        self.route_plan = pts
        self.route_preview.setData(pts[:, 0], pts[:, 1])
        return True

    def on_send_route(self) -> None:
        n = self.rover_box.currentText()
        if not n:
            QtWidgets.QMessageBox.information(self, "Route", "Select a rover with a location first.")
            return
        if not self._replan():
            QtWidgets.QMessageBox.information(
                self, "Route", "Could not plan the route (see the status bar)." if self.waypoints
                else "Click at least one waypoint first.")
            return
        length = route_plan.length(self.route_plan)
        ans = QtWidgets.QMessageBox.question(
            self, "Send route",
            f"Send a {length:.0f} m route at {self.route_speed.value():.2f} m/s to rover {n}?\n\n"
            + self._drive_note(n))
        if ans != QtWidgets.QMessageBox.Yes:
            return
        err = self.core.send_route(n, self.route_plan, self.route_speed.value())
        if err:
            QtWidgets.QMessageBox.warning(self, "Route", err)
            return
        self.wp_btn.setChecked(False)
        self.waypoints = []
        self._replan()

    def on_stop_route(self) -> None:
        n = self.rover_box.currentText()
        if n:
            self.core.stop_route(n)

    def on_stop_relays(self) -> None:
        self.core.stop_routes(list(self.core.relay_cfg) + [self.rover_box.currentText()], 'stop relays')

    def on_stop_all(self) -> None:
        nodes = set(self.core.relay_cfg) | set(self.core.nav_cmds) | set((self.last_snap or {}).get('locations', {}))
        nodes |= {r['node'] for r in (self.last_snap or {}).get('rovers', [])}
        self.core.stop_routes(nodes, 'STOP ALL')

    # map actions
    def _selected_location(self):
        n = self.rover_box.currentText()
        loc = (self.last_snap or {}).get('locations', {}).get(n)
        if not n or not loc or (loc['x'] == 0.0 and loc['y'] == 0.0)                 or not (math.isfinite(loc['x']) and math.isfinite(loc['y'])):
            return None, None
        return n, loc

    def _on_align_toggled(self, on: bool) -> None:
        if on and self._selected_location()[0] is None:
            self.align_btn.setChecked(False)
            QtWidgets.QMessageBox.information(self, "Align", "No location from any rover yet.")
            return
        if on:
            self.wp_btn.setChecked(False)
        self.map_vb.setCursor(QtCore.Qt.CrossCursor if on else QtCore.Qt.ArrowCursor)
        if not on:
            self.align_preview.setData([], [])

    def align_from_map(self, x0, y0, x1, y1, finish: bool) -> None:
        """Click (x0 == x1) places the selected rover at (x0, y0) keeping the rotation; a drag
        also sets its heading to the drag direction."""
        self.align_preview.setData([x0, x1], [y0, y1])
        if not finish:
            return
        self.align_preview.setData([], [])
        self.align_btn.setChecked(False)
        n, loc = self._selected_location()
        if n is None:
            return
        px_w, px_h = self.map_vb.viewPixelSize()
        dragged = math.hypot((x1 - x0) / px_w, (y1 - y0) / px_h) > 10      # screen pixels
        heading = math.degrees(math.atan2(y1 - y0, x1 - x0)) if dragged else None
        cur = self.core.alignments.get(n)
        al = site_map.solve_alignment(loc['x'], loc['y'], loc['yaw'], x0, y0, heading, cur['theta'])
        al['anchor'] = [x0, y0]
        note = 'placed + heading' if heading is not None and loc['yaw'] is not None else 'placed'
        self.core.set_alignment(n, al, note)
        self.track_key = None

    def on_nudge(self, dx: int, dy: int, dth: int) -> None:
        """Shift / rotate the selected rover's alignment about its placed point (or its
        current position if it was never placed)."""
        n, loc = self._selected_location()
        if n is None:
            return
        al = self.core.alignments.get(n)
        step = float(self.step_m.currentText().split()[0])
        dang = float(self.step_deg.currentText().rstrip('°')) * dth
        pivot = al.get('anchor')
        if pivot is None:
            mx, my, _ = site_map.to_map(al, loc['x'], loc['y'])
            pivot = [float(mx), float(my)]
        a = math.radians(dang)
        c, s = math.cos(a), math.sin(a)
        tx, ty = al['x'] - pivot[0], al['y'] - pivot[1]
        new = {'x': pivot[0] + c * tx - s * ty + dx * step, 'y': pivot[1] + s * tx + c * ty + dy * step,
               'theta': (al['theta'] + dang + 180.0) % 360.0 - 180.0,
               'anchor': [pivot[0] + dx * step, pivot[1] + dy * step]}
        self.core.set_alignment(n, new, '')
        self.track_key = None

    def on_reset_alignment(self) -> None:
        n, _loc = self._selected_location()
        if n is not None:
            self.core.set_alignment(n, dict(site_map.IDENTITY), 'reset')
            self.track_key = None

    def on_clear_trail(self) -> None:
        self.trail_since = time.time()
        self.track_key = None

    def _on_map_hover(self, pos) -> None:
        if not self.p_map.sceneBoundingRect().contains(pos):
            return
        p = self.map_vb.mapSceneToView(pos)
        x, y = p.x(), p.y()
        text = f"x {x:+.1f}  y {y:+.1f} m"
        site = self.site
        if site is not None:
            elev = float(site.sample(site.terrain, x, y))
            if not math.isnan(elev):
                text += f"  elev {elev:.1f} m"
                if site.drivable is not None:
                    text += "  drivable" if site.on_drivable(x, y) else "  not drivable"
                rm = self.rm_box.currentText()
                if site.radio_maps.get(rm) is not None:
                    v = float(site.sample(site.radio_maps[rm], x, y))
                    text += f"  {rm} {v:.0f} dB" if not math.isnan(v) else ""
        self.hover_label.setText(text)

    def closeEvent(self, event) -> None:
        self.timer.stop()
        self.core.shutdown()
        event.accept()


def main():
    ap = argparse.ArgumentParser(description='Live BS dashboard (PySide6 + pyqtgraph)')
    ap.add_argument('--port', default='COM5', help='BS serial port')
    ap.add_argument('--baud', type=int, default=bs_core.BAUD_RATE)
    ap.add_argument('--report-dir', default='bs_reports')
    ap.add_argument('--map-half', type=float, default=75.0, help='Map half-width in metres without --site')
    ap.add_argument('--site', default=None, help='terrain site to show on the Map tab, e.g. fountain or range')
    ap.add_argument('--terrain-dir', default=None, help='default: ./terrain, then ../terrain')
    ap.add_argument('--rm-env', default=None, help='lunar-mesh-env checkout used to generate radio maps '
                                                   'when the BS is moved (default ~/Documents/lunar-mesh-env)')
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    core = BSCore(args.port, args.baud, args.report_dir, args.site, args.terrain_dir, args.rm_env)
    core.start()
    win = Dashboard(core, args.map_half)
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
