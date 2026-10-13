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
from bs_core import BSCore, BS_NODE_ID

PALETTE = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd', '#ff7f0e', '#17becf', '#8c564b', '#e377c2']
TIMELINE_CHOICES = [(60, 1), (180, 5), (600, 10), (1800, 30), (3600, 60)]   # (window s, bin s)
EVENT_COLORS = {'data': '#2e7d32', 'ferry': '#ef6c00', 'contact': '#1565c0', 'control': '#6a1b9a',
                'marker': '#000000', 'warn': '#c62828', 'session': '#616161'}


def rssi_color(rssi: float) -> QtGui.QColor:
    """Red (-90 dBm) -> yellow (-60) -> green (-30)."""
    f = min(1.0, max(0.0, (rssi + 90) / 60.0))
    if f < 0.5:
        r, g = 220, int(60 + 340 * f)
    else:
        r, g = int(220 - 360 * (f - 0.5)), 200
    return QtGui.QColor(r, g, 60)


def fmt_age(sec) -> str:
    if sec is None:
        return '-'
    sec = int(sec)
    m, s = divmod(sec, 60)
    h, m = divmod(m, 60)
    return f"{h}h{m:02d}m" if h else (f"{m}m{s:02d}s" if m else f"{s}s")


class Dashboard(QtWidgets.QMainWindow):
    TABLE_COLS = ['Node', 'Link', 'For', 'RSSI', 'State', 'Experiment', 'Ctrl', 'Radio window',
                  'Queue own/ferr', 'Gen', 'Tx', 'Retx', 'Ack', 'Data 60s', 'Data total',
                  'Avg lat', 'Status age']

    def __init__(self, core: BSCore, map_half: float):
        super().__init__()
        self.core = core
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
        pg.setConfigOptions(antialias=True, background='w', foreground='k')

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
        }
        self.cfg_widgets['payload_bytes'] = QtWidgets.QSpinBox()
        self.cfg_widgets['payload_bytes'].setRange(0, 1024)
        self.cfg_widgets['payload_bytes'].setSpecialValueText("natural")
        self.cfg_widgets['payload_bytes'].setSuffix(" B")
        self.cfg_widgets['bw_order'] = QtWidgets.QComboBox()
        self.cfg_widgets['bw_order'].addItems(['oldest', 'newest'])
        labels = [('interval', 'Data every'), ('bw_limit', 'BW limit'), ('bw_burst', 'Burst'),
                  ('bw_order', 'Order'), ('comms_on', 'Radio on'), ('comms_off', 'Radio off'),
                  ('comms_phase', 'Phase'), ('payload_bytes', 'Payload')]
        tips = {'interval': 'seconds between generated data bundles',
                'bw_limit': 'data bundles per second over the air (beacons/acks are never limited)',
                'bw_burst': 'data bundles that may go back-to-back after idling',
                'bw_order': 'which queued bundles get the budget first',
                'comms_on': 'periodic mode: seconds the radio is on (needs Radio off > 0 too)',
                'comms_off': 'periodic mode: seconds the radio is fully off (no TX, no RX)',
                'comms_phase': "shift a rover's window schedule (fleet-wide here)",
                'payload_bytes': 'pad data bundle payloads to this size'}
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
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        tbox = QtWidgets.QGroupBox("Rovers")
        QtWidgets.QVBoxLayout(tbox).addWidget(self.table)
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

        bottom = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.p_topo = pg.PlotWidget(title="Topology (orange = ferried, red = direct delivery)")
        self.p_topo.hideAxis('left')
        self.p_topo.hideAxis('bottom')
        self.p_topo.setAspectLocked(True)
        self.p_topo.setMouseEnabled(False, False)
        bottom.addWidget(self.p_topo)

        self.p_map = pg.PlotWidget(title="Radio map: RSSI to BS (red -90 → green -30 dBm)")
        self.p_map.setAspectLocked(True)
        self.p_map.showGrid(x=True, y=True, alpha=0.25)
        self.p_map.setXRange(-self.map_half, self.map_half)
        self.p_map.setYRange(-self.map_half, self.map_half)
        self.p_map.setLabel('bottom', 'X', units='m')
        self.p_map.setLabel('left', 'Y', units='m')
        self.map_scatter = pg.ScatterPlotItem(size=9, pen=None)
        self.map_now = pg.ScatterPlotItem(size=16, pen=pg.mkPen('k', width=2), brush=pg.mkBrush('w'))
        self.p_map.addItem(self.map_scatter)
        self.p_map.addItem(self.map_now)
        self.map_labels = {}
        self.map_count = -1
        bottom.addWidget(self.p_map)
        split.addWidget(bottom)
        split.setSizes([600, 400])
        return split

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
            state = msg.get('state', '?').upper()
            exp = f" <b>'{msg['exp']}'</b>" if msg.get('exp') else ""
            starts = ""
            if msg.get('state') == 'active' and msg.get('start_at', 0) > now:
                starts = f", starts in {msg['start_at'] - now:.0f} s"
            text = (f"Fleet state v{c['version']}: <b>{state}</b>{exp}{starts} "
                    f"(set {fmt_age(now - c['sent_at'])} ago, re-sent every {bs_core.CONTROL_PERIOD_S:g} s)")
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
            self._cell(i, 7, '-' if win is None else f"{'ON' if win[0] else 'OFF'} {win[1]:.0f}s left",
                       bg=None if win is None else ('#c8e6c9' if win[0] else '#eeeeee'))
            self._cell(i, 8, f"{num('q')}/{num('qf')}")
            for col, k in ((9, 'gen'), (10, 'tx'), (11, 'retx'), (12, 'ack')):
                self._cell(i, col, num(k))
            self._cell(i, 13, r['data_60s'])
            self._cell(i, 14, r['data_total'])
            self._cell(i, 15, f"{r['avg_lat']:.0f} ms" if r['avg_lat'] is not None else '-')
            age = r['status_age']
            self._cell(i, 16, fmt_age(age), fg='#c62828' if age is not None and age > 15 else None)

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
        samples = snap['map_samples']
        count = sum(len(v) for v in samples.values())
        if count != self.map_count:          # only rebuild the (possibly large) scatter on change
            self.map_count = count
            spots = [{'pos': (x, y), 'brush': pg.mkBrush(rssi_color(r))}
                     for v in samples.values() for x, y, r in v]
            self.map_scatter.setData(spots)
            self.p_map.setTitle(f"Radio map: RSSI to BS (red -90 → green -30 dBm), {count} samples")
        locs = {n: l for n, l in snap['locations'].items() if l['x'] != 0.0 or l['y'] != 0.0}
        self.map_now.setData([l['x'] for l in locs.values()], [l['y'] for l in locs.values()],
                             brush=[pg.mkBrush(self.color(n)) for n in locs])
        for n, l in locs.items():
            if n not in self.map_labels:
                self.map_labels[n] = pg.TextItem(n, color='k', anchor=(-0.2, 1.2))
                self.p_map.addItem(self.map_labels[n])
            self.map_labels[n].setPos(l['x'], l['y'])

    def closeEvent(self, event) -> None:
        self.timer.stop()
        self.core.shutdown()
        event.accept()


def main():
    ap = argparse.ArgumentParser(description='Live BS dashboard (PySide6 + pyqtgraph)')
    ap.add_argument('--port', default='COM5', help='BS serial port')
    ap.add_argument('--baud', type=int, default=bs_core.BAUD_RATE)
    ap.add_argument('--report-dir', default='bs_reports')
    ap.add_argument('--map-half', type=float, default=75.0, help='Radio map half-width in metres')
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    core = BSCore(args.port, args.baud, args.report_dir)
    core.start()
    win = Dashboard(core, args.map_half)
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
