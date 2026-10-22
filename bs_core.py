"""
Base-station core: reads the BS ESP32 serial port, keeps all mesh / experiment state, writes
the session logs and reports, and sends fleet control to the rovers. No GUI.

bs_dashboard.py is the live display on top of this. The core can also run on its own, so
logging and reports never depend on the display:

    python bs_core.py --port COM5              # headless: log everything, Ctrl-C writes the summary
    python bs_core.py --port COM5 --site fountain   # + terrain lookups (site_map.py)

Session folder (bs_reports/session_<time>[_<tag>]/):
    serial.log              every BS line with a wall-clock timestamp (+ #CONTACT/#MARKER/#CONTROL)
    deliveries.csv          one row per delivered bundle (data vs telemetry, via, hops, latency)
    contacts.csv            one row per rover <-> BS contact (start, end, data delivered, RSSI)
    rover_status_<id>.csv   every @ROVER_STATUS report
    controls.csv            every fleet control change
    markers.csv             markers (+ every rover's map position at that moment)
    coverage.csv            every @LOCATION sample: raw odometry (lat/lon/alt_m columns = odom
                            x/y/z), yaw, RSSI, map position, and with --site the drivable flag,
                            elevation and each predicted radio map value at that point
    alignments.csv          every odometry -> map alignment change
    routes.csv              every route sent to a rover (map + odometry points) and every stop
    bs_position.json        where the BS is on the site map (with --site; see move_bs)
    dtn_summary_<time>.txt  session report
    node_<id>_boot<n>.txt   per-boot report on @NODE_RESTART
"""

import argparse
import csv
import json
import math
import os
import struct
import subprocess
import threading
import time
from collections import defaultdict, deque
from datetime import datetime
from typing import Dict, List, Optional

import serial

import rm_generate
import route_plan
import site_map

BS_NODE_ID       = 23768
BAUD_RATE        = 115200
NODE_TIMEOUT_S   = 15.0    # node disappears from the topology after this long without @NET
CONTACT_GAP_S    = 3.0     # rover link to the BS is "down" after this long without its beacon
HISTORY_S        = 3600.0  # how much delivery/contact history the display can ask for

# fleet control (laptop -> BS ESP32 -> rovers), must match radio_daemon.py
HOST_SOF          = 0xAA
HOST_CMD_WIFI_TX  = 0x30
PKT_TYPE_CONTROL  = 0x06
CONTROL_PERIOD_S  = 2.0    # desired fleet state is re-broadcast this often
START_DELAY_S     = 3.0    # experiments start this long after Start, fleet-wide

CONFIG_DEFAULTS = {        # same keys/meaning as the radio_daemon.py CLI options
    'interval': 1.0, 'bw_limit': 0.0, 'bw_burst': 1.0, 'bw_order': 'oldest',
    'comms_on': 0.0, 'comms_off': 0.0, 'comms_phase': 0.0, 'payload_bytes': 0,
    'loc_interval': 1.0,
}
# control JSON bytes per air frame. ESP-NOW v2 would carry 1470, but the firmware drops serial
# frames over HOST_MAX_PAYLOAD (1100): a rover's ESP32 hands it on as [mac 6][rssi 1][air], and the
# air frame has a 7-byte control header, so 1100 - 7 - 7.
CONTROL_MAX_JSON  = 1086
ROUTE_TOLERANCES  = (0.4, 0.6, 0.8, 1.0)   # simplification tried in turn until the route fits [m]
ROUTE_SPEED_MAX   = 1.0    # m/s; rover_nav.py caps it again
TRACK_MAX        = 20000   # location samples kept per rover for the map
ALIGN_HEADER     = ['time', 'epoch', 'node', 'x', 'y', 'theta_deg', 'note']


def _ts(t: float) -> str:
    return datetime.fromtimestamp(t).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_frame(cmd: int, payload: bytes) -> bytes:
    n = len(payload)
    body = bytes([cmd, n & 0xFF, n >> 8]) + payload
    crc = _crc16(body)
    return bytes([HOST_SOF]) + body + bytes([crc & 0xFF, crc >> 8])


def _pct(vals: List[float], p: float):
    s = sorted(vals)
    return s[min(len(s) - 1, int(round(p / 100.0 * (len(s) - 1))))] if s else None


def sanitize_name(raw: str) -> str:
    return ''.join(ch for ch in raw.strip().replace(' ', '_') if ch.isalnum() or ch in '-_')[:40]


class BSCore:
    def __init__(self, port: str, baud: int = BAUD_RATE, report_dir: str = 'bs_reports',
                 site: Optional[str] = None, terrain_dir: Optional[str] = None,
                 rm_env: Optional[str] = None):
        self.port = port
        self.baud = baud
        self.report_dir = report_dir
        self.site: Optional[site_map.Site] = None
        if site:
            tdir = site_map.find_terrain_dir(terrain_dir)
            if tdir is None:
                raise SystemExit(f"--site {site}: no terrain directory found (use --terrain-dir)")
            self.site = site_map.Site(tdir, site)
            print(f"[CORE] site {self.site.describe()}")
        self.alignments = site_map.AlignmentStore(report_dir, site or '')
        self.lock = threading.RLock()
        # BS position on the site map + radio maps generated for it (rm_generate.py)
        self.rm_env = rm_env or rm_generate.DEFAULT_ENV
        self.bs_store = os.path.join(report_dir, f"bs_position_{site}.json") if site else ''
        self.rm_version = 0                   # bumped whenever the BS or its maps change
        self.rm_job = {'status': 'idle', 'started': 0.0, 'missing': [], 'error': ''}
        self._rm_proc = None
        if self.site is not None and os.path.exists(self.bs_store):
            try:
                with open(self.bs_store) as f:
                    pos = json.load(f)
                self.site.set_bs(float(pos['x']), float(pos['y']))
            except (OSError, ValueError, KeyError) as e:
                print(f"[CORE] could not read {self.bs_store}: {e}")
        self._ser: Optional[serial.Serial] = None
        self._ser_write_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.connected = False
        self.lines_total = 0
        self.last_line_at = 0.0
        self.bs_rejects_control = False
        self.control = {'version': 0, 'msg': None, 'sent_at': 0.0, 'last_tx': 0.0}
        self.ros_cmds: Dict[str, list] = {}   # node -> ["on"|"off", cmd_id], rides on every control msg
        # node -> [cmd_id, speed, [x0, y0, ...] odom dm] (route) | [cmd_id, "stop"] | [cmd_id, "ok"]
        # (route points dropped once the rover reports it has the command, see _on_rover_status)
        self.nav_cmds: Dict[str, list] = {}
        self.routes: Dict[str, dict] = {}     # node -> {'cmd', 'speed', 'map': [[x, y], ...]} for the map
        self.pose_modes: Dict[str, str] = {}  # node -> "fused" | "camera" (rover_nav.py pose source)
        # rovers whose rover_nav.py drives without RB held (mux Autonomy input, LB overrides)
        self.rb_off: Dict[str, int] = {}      # node -> 0, carried as "rb" on every control msg
        # node -> [explorer node, speed m/s, ferry_s, comfort dBm, gain dB, look-ahead s, link model]:
        # that rover acts as a relay (relay_brain.py)
        self.relay_cfg: Dict[str, list] = {}
        # BS time each alignment was last set: rovers keep the newest copy (see _live_fields)
        self.align_ts: Dict[str, float] = {n: time.time() for n in self.alignments.data}
        self._planner = None                  # route_plan.Planner, built on first use

        self.session_dir = ''
        self._serial_log = None
        self._events_seq = 0
        self.events: deque = deque(maxlen=3000)   # (id, time, category, text)
        self._reset_state()
        self._new_session_files('')
        if self.site is not None and self.site.reload_bs_maps():
            self._start_rm_job()

    # state
    def _reset_state(self) -> None:
        self.start_time = time.time()
        self.node_last_seen: Dict[str, float] = {}
        self.edges: Dict[str, tuple] = {}                    # node -> (parent, rssi)
        self.transfers: deque = deque(maxlen=200)            # dicts: time, src, via, dst, hops, ferry
        self.delivered: Dict[tuple, dict] = {}               # (src, seq) -> {latency_ms, hops, time, kind}
        self.timeline: deque = deque(maxlen=100000)          # (time, src, kind, latency_ms, via)
        self.antipkt_fail = defaultdict(int)
        self.ferry_dup = defaultdict(int)
        self.node_rssi = defaultdict(list)
        self.seq_range: Dict[str, dict] = {}
        self.boot_count = defaultdict(int)
        self.boot_history = defaultdict(list)
        self.boot_start: Dict[str, float] = {}
        self.locations: Dict[str, dict] = {}
        self.tracks = defaultdict(lambda: deque(maxlen=TRACK_MAX))   # (t, x, y, yaw, rssi), odometry
        self.track_total = defaultdict(int)                          # samples ever added, per rover
        self.link: Dict[str, dict] = {}                      # node -> {up, since, last, data, tel, rssi[]}
        self.contacts = defaultdict(list)                    # node -> [{start, end, data, tel, avg_rssi}]
        self.rover_status: Dict[str, dict] = {}
        self.markers: List[tuple] = []
        self.data_stats = defaultdict(lambda: [0, 0])        # node -> [n data delivered, latency sum]
        self._pending_metric = None

    def _event(self, category: str, text: str) -> None:
        self._events_seq += 1
        self.events.append((self._events_seq, time.time(), category, text))

    # session files
    def _new_session_files(self, tag: str) -> None:
        name = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}" + (f"_{tag}" if tag else "")
        self.session_dir = os.path.join(self.report_dir, name)
        os.makedirs(self.session_dir, exist_ok=True)
        if self._serial_log is not None:
            try:
                self._serial_log.close()
            except OSError:
                pass
        self._serial_log = open(os.path.join(self.session_dir, 'serial.log'), 'a', buffering=1)
        self._csv('coverage.csv', self._coverage_header(), None)
        for node, al in self.alignments.data.items():
            self._csv('alignments.csv', ALIGN_HEADER,
                      [_ts(time.time()), f"{time.time():.3f}", node, f"{al['x']:.3f}", f"{al['y']:.3f}",
                       f"{al['theta']:.2f}", 'loaded'])
        self._write_bs_position(session_only=True)
        self._event('session', f"session folder {self.session_dir}")
        print(f"[CORE] session folder: {self.session_dir}")

    def _csv(self, name: str, header: list, row: Optional[list]) -> None:
        path = os.path.join(self.session_dir, name)
        try:
            new = not os.path.exists(path)
            with open(path, 'a', newline='') as f:
                w = csv.writer(f)
                if new:
                    w.writerow(header)
                if row is not None:
                    w.writerow(row)
        except OSError:
            pass

    def _log_serial(self, line: str) -> None:
        if self._serial_log is not None:
            try:
                self._serial_log.write(f"{_ts(time.time())}\t{line}\n")
            except (OSError, ValueError):
                pass

    def new_session(self, tag: str = '') -> None:
        """Save the summary of the current session and start a fresh one."""
        self.save_summary()
        with self.lock:
            self._reset_state()
            self._new_session_files(tag)

    # serial
    def start(self) -> None:
        self._thread = threading.Thread(target=self._reader, name='bs-serial', daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        self._kill_rm_proc()
        if self._thread:
            self._thread.join(timeout=2.0)
        self.save_summary()
        if self._serial_log is not None:
            self._serial_log.close()
            self._serial_log = None

    def _reader(self) -> None:
        first = True
        while not self._stop.is_set():
            try:
                ser = serial.Serial()
                ser.port, ser.baudrate, ser.timeout = self.port, self.baud, 0.1
                ser.dtr = False
                ser.rts = False
                ser.open()
                ser.dtr = False
                ser.rts = False
                self._ser = ser
                self.connected = True
                with self.lock:
                    if not first:
                        self._event('warn', f"reconnected to {self.port}")
                    self._event('session', f"connected to {self.port}")
                print(f"[CORE] connected to {self.port}")
                if not first:
                    self.new_session('reconnect')
                first = False
                self._read_loop(ser)
            except (serial.SerialException, OSError) as e:
                if self.connected:
                    with self.lock:
                        self._event('warn', f"serial lost: {e}")
                    print(f"[CORE] serial lost: {e}")
                self.connected = False
                self._ser = None
                self._stop.wait(2.0)

    def _read_loop(self, ser) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            chunk = ser.read(max(1, ser.in_waiting))
            now = time.time()
            if chunk:
                buf += chunk
                for line in self._extract_lines(buf):
                    self._handle_line(line, now)
            with self.lock:
                self._check_contacts(now)
            self._maybe_send_control(now)

    @staticmethod
    def _extract_lines(buf: bytearray):
        """Split ASCII lines out of the stream, removing binary host frames (SOF 0xAA,
        e.g. PULL_REQ from the BS host task) wherever they appear."""
        out = []
        while buf:
            aa = buf.find(HOST_SOF)
            nl = buf.find(b'\n')
            if aa >= 0 and (nl < 0 or aa < nl):
                if len(buf) < aa + 4:
                    break
                plen = buf[aa + 2] | (buf[aa + 3] << 8)
                if plen > 1200:
                    del buf[aa]                    # not a real frame header
                    continue
                if len(buf) < aa + 4 + plen + 2:
                    break                          # wait for the rest of the frame
                del buf[aa:aa + 4 + plen + 2]
                continue
            if nl < 0:
                if len(buf) > 4096:                # runaway garbage without newline
                    buf.clear()
                break
            line = bytes(buf[:nl]).decode('utf-8', errors='ignore').strip()
            del buf[:nl + 1]
            if line:
                out.append(line)
        return out

    # line parsing
    def _handle_line(self, line: str, now: float) -> None:
        self._log_serial(line)
        with self.lock:
            self.lines_total += 1
            self.last_line_at = now

        if "I am the Base Station" in line:
            with self.lock:
                self._event('warn', "BS reboot detected - new session")
            self.new_session('bs_reboot')
            return
        if "unknown cmd 0x30" in line:
            with self.lock:
                if not self.bs_rejects_control:
                    self._event('warn', "BS firmware rejected a control frame: reflash the BS")
                    print("[CORE] *** BS firmware does not forward control (unknown cmd 0x30): reflash it ***")
                self.bs_rejects_control = True
            return
        if not line.startswith('@'):
            return

        with self.lock:
            if not line.startswith('@NET:'):
                self._resolve_pending_metric(line)
            parts = line.split(':')
            tag = parts[0]
            try:
                if tag == '@NET' and len(parts) >= 4:
                    self._on_net(parts[1], parts[2], parts[3], now)
                elif tag == '@DTN_RX' and len(parts) >= 5:
                    self._on_dtn_rx(parts, now)
                elif tag == '@METRIC' and len(parts) >= 5:
                    self._on_metric(parts, now)
                elif tag == '@NODE_RESTART' and len(parts) >= 2:
                    self._on_node_restart(parts[1], now)
                elif tag == '@ANTIPKT_FAIL' and len(parts) >= 3:
                    self.antipkt_fail[parts[1]] += 1
                elif tag == '@FERRY_DUP' and len(parts) >= 3:
                    self.ferry_dup[parts[1]] += 1
                elif tag == '@LOCATION' and len(parts) >= 5:
                    self._on_location(parts, now)
                elif tag == '@ROVER_STATUS':
                    self._on_rover_status(line, now)
            except (ValueError, IndexError) as e:
                self._event('warn', f"could not parse {line!r}: {e}")

    def _on_net(self, node: str, parent: str, rssi: str, now: float) -> None:
        self.node_last_seen[node] = now
        self.boot_start.setdefault(node, now)
        if parent == '0':
            self.edges.pop(node, None)
            return
        self.node_last_seen.setdefault(parent, now)
        self.edges[node] = (parent, rssi)
        try:
            self.node_rssi[node].append(int(rssi))
        except ValueError:
            pass
        ls = self.link.get(node)
        if ls is None or not ls['up']:
            gap = (now - ls['last']) if ls else None
            self.link[node] = ls = {'up': True, 'since': now, 'last': now, 'data': 0, 'tel': 0, 'rssi': []}
            self._event('contact', f"{node} link UP" + (f" after {gap:.1f}s down" if gap else ""))
            self._log_serial(f"#CONTACT_UP:{node}")
        ls['last'] = now
        try:
            ls['rssi'].append(int(rssi))
        except ValueError:
            pass

    def _check_contacts(self, now: float) -> None:
        for node, ls in self.link.items():
            if ls['up'] and now - ls['last'] > CONTACT_GAP_S:
                ls['up'] = False
                self._close_contact(node, ls)

    def _contact_record(self, ls: dict) -> dict:
        return {'start': ls['since'], 'end': ls['last'], 'data': ls['data'], 'tel': ls['tel'],
                'avg_rssi': sum(ls['rssi']) / len(ls['rssi']) if ls['rssi'] else None}

    def _close_contact(self, node: str, ls: dict) -> None:
        rec = self._contact_record(ls)
        self.contacts[node].append(rec)
        dur = rec['end'] - rec['start']
        self._event('contact', f"{node} link DOWN after {dur:.1f}s ({rec['data']} data delivered)")
        self._log_serial(f"#CONTACT_DOWN:{node}:{dur:.1f}")
        self._csv('contacts.csv',
                  ['node', 'start', 'end', 'duration_s', 'data_delivered', 'tel_delivered',
                   'avg_rssi', 'start_epoch', 'end_epoch'],
                  [node, _ts(rec['start']), _ts(rec['end']), f"{dur:.1f}", rec['data'], rec['tel'],
                   f"{rec['avg_rssi']:.1f}" if rec['avg_rssi'] is not None else '',
                   f"{rec['start']:.3f}", f"{rec['end']:.3f}"])

    def _on_dtn_rx(self, parts: list, now: float) -> None:
        src, via, seq, dst = parts[1], parts[2], parts[3], parts[4]
        try:
            hops = int(parts[5]) if len(parts) >= 6 else 1
        except ValueError:
            hops = 1
        for n in (src, via, dst):
            self.node_last_seen[n] = max(self.node_last_seen.get(n, 0), now)
        ferry = (via != src) or hops > 1
        self.transfers.append({'time': now, 'src': src, 'via': via, 'dst': dst,
                               'hops': hops, 'ferry': ferry, 'seq': seq})

    def _on_metric(self, parts: list, now: float) -> None:
        src, seq, hops, lat = parts[1], int(parts[2]), int(parts[3]), int(parts[4])
        if not (0 <= lat < 300000):
            return
        self.delivered[(src, seq)] = {'latency_ms': lat, 'hops': hops, 'time': now, 'kind': 'tel'}
        r = self.seq_range.setdefault(src, {'min': seq, 'max': seq})
        r['min'], r['max'] = min(r['min'], seq), max(r['max'], seq)
        self._pending_metric = (src, seq, hops, lat, now)

    def _resolve_pending_metric(self, line: str) -> None:
        """The BS prints @METRIC then, for the same bundle, @DTN_RX (data bundle) or the
        telemetry payload text. Classify the pending @METRIC from the line that follows."""
        if self._pending_metric is None:
            return
        src, seq, hops, lat, t = self._pending_metric
        self._pending_metric = None
        parts = line.split(':')
        if line.startswith('@DTN_RX:') and len(parts) >= 4 and parts[1] == src and parts[3] == str(seq):
            kind, via, label = 'data', parts[2], ''
        else:
            kind, via, label = 'tel', src, parts[0]
        info = self.delivered.get((src, seq))
        if info is not None:
            info['kind'] = kind
        self.timeline.append((t, src, kind, lat, via))
        ls = self.link.get(via) or self.link.get(src)
        if ls is not None and ls['up']:
            ls[kind] += 1
        if kind == 'data':
            ds = self.data_stats[src]
            ds[0] += 1
            ds[1] += lat
            how = f"via {via}, {hops} hops" if via != src or hops > 1 else "direct"
            self._event('ferry' if via != src else 'data', f"{src} seq {seq} {how}, {lat} ms")
        self._csv('deliveries.csv',
                  ['time', 'epoch', 'src', 'seq', 'kind', 'via', 'hops', 'latency_ms', 'label'],
                  [_ts(t), f"{t:.3f}", src, seq, kind, via, hops, lat, label])

    def _on_node_restart(self, node: str, now: float) -> None:
        lats = [i['latency_ms'] for (s, _q), i in self.delivered.items() if s == node]
        rssi = self.node_rssi.get(node, [])
        rec = {
            'boot': self.boot_count[node],
            'duration_s': int(now - self.boot_start.get(node, self.start_time)),
            'seq_range': dict(self.seq_range[node]) if node in self.seq_range else None,
            'delivered': {q: i for (s, q), i in self.delivered.items() if s == node},
            'antipkt_fails': self.antipkt_fail.get(node, 0),
            'ferry_dups': self.ferry_dup.get(node, 0),
            'avg_rssi': sum(rssi) / len(rssi) if rssi else None,
            'avg_latency': sum(lats) / len(lats) if lats else None,
            'all_latencies': lats,
        }
        self.boot_history[node].append(rec)
        self.boot_count[node] += 1
        for d in (self.seq_range, self.node_rssi, self.antipkt_fail, self.ferry_dup, self.locations,
                  self.data_stats):
            d.pop(node, None)
        for k in [k for k in self.delivered if k[0] == node]:
            del self.delivered[k]
        self.boot_start[node] = now
        self._event('warn', f"node {node} restarted (boot {rec['boot']} archived)")
        self._write_boot_summary(node, rec)

    def _write_boot_summary(self, node: str, rec: dict) -> None:
        lats, d = rec['all_latencies'], rec['delivered']
        lines = [f"Node {node} boot {rec['boot']} - saved {datetime.now():%Y-%m-%d %H:%M:%S}",
                 f"Duration {rec['duration_s']} s, delivered {len(d)}, ferry dups {rec['ferry_dups']}, "
                 f"antipacket fails {rec['antipkt_fails']}"]
        if lats:
            lines.append(f"Latency avg {sum(lats)/len(lats):.0f} ms, min {min(lats)}, max {max(lats)}")
        if rec['seq_range']:
            r = rec['seq_range']
            poss = r['max'] - r['min'] + 1
            lines.append(f"Seq [{r['min']}..{r['max']}] possible {poss}, rate {100.0*len(d)/poss:.1f}%")
        try:
            with open(os.path.join(self.session_dir, f"node_{node}_boot{rec['boot']}.txt"), 'w') as f:
                f.write("\n".join(lines) + "\n")
        except OSError:
            pass

    def _coverage_header(self) -> list:
        h = ['timestamp', 'node_id', 'lat', 'lon', 'alt_m', 'rssi_to_bs_dBm', 'yaw_deg',
             'map_x', 'map_y', 'map_heading_deg']
        if self.site is not None:
            h += ['on_drivable', 'elev_m'] + [f"pred_{m}_dB" for m in self.site.radio_maps]
        return h

    def _on_location(self, parts: list, now: float) -> None:
        # @LOCATION:<node>:<x>:<y>:<z>[:<rssi>[:<yaw_deg>]]  (x/y/z = rover odometry)
        node, x, y, z = parts[1], float(parts[2]), float(parts[3]), float(parts[4])
        rssi = int(parts[5]) if len(parts) >= 6 and parts[5] else None
        yaw = float(parts[6]) if len(parts) >= 7 and parts[6] else None
        if not (math.isfinite(x) and math.isfinite(y)):
            return                               # rover pose not ready (NaN): keep the last good one
        if yaw is not None and not math.isfinite(yaw):
            yaw = None
        self.locations[node] = {'x': x, 'y': y, 'z': z, 'yaw': yaw, 'rssi': rssi, 'updated_at': now}
        if x == 0.0 and y == 0.0:
            return                                # no location source on that rover
        self.tracks[node].append((now, x, y, yaw, rssi))
        self.track_total[node] += 1
        mx, my, myaw = site_map.to_map(self.alignments.get(node), x, y, yaw)
        row = [_ts(now), node, f"{x:.6f}", f"{y:.6f}", f"{z:.1f}", rssi if rssi is not None else '',
               f"{yaw:.1f}" if yaw is not None else '', f"{float(mx):.2f}", f"{float(my):.2f}",
               f"{float(myaw):.1f}" if myaw is not None else '']
        if self.site is not None:
            row += [int(self.site.on_drivable(mx, my)),
                    f"{float(self.site.sample(self.site.terrain, mx, my)):.2f}"]
            row += [f"{float(self.site.sample(a, mx, my)):.1f}" for a in self.site.radio_maps.values()]
        self._csv('coverage.csv', self._coverage_header(), row)

    # map alignment
    def set_alignment(self, node: str, al: dict, note: str = '') -> None:
        """Store a rover's odometry -> map alignment (see site_map.py) and log the change."""
        now = time.time()
        with self.lock:
            if not site_map.finite_alignment(al):
                self._event('map', f"{node}: alignment not set (not finite: no pose/heading from the rover?)")
                return
            self.alignments.set(node, al)
            self.align_ts[node] = now
            a = self.alignments.get(node)
            self._log_serial(f"#ALIGN:{node}:{a['x']:.3f}:{a['y']:.3f}:{a['theta']:.2f}:{note}")
            self._csv('alignments.csv', ALIGN_HEADER,
                      [_ts(now), f"{now:.3f}", node, f"{a['x']:.3f}", f"{a['y']:.3f}",
                       f"{a['theta']:.2f}", note])
            if note:
                self._event('map', f"{node} aligned ({note}): offset ({a['x']:+.1f}, {a['y']:+.1f}) m, "
                                   f"rotation {a['theta']:+.1f} deg")

    # BS position on the site map
    def _bs_position(self) -> Optional[dict]:
        site = self.site
        if site is None or site.bs_rc is None:
            return None
        r, c = site.bs_rc
        return {'site': site.name, 'x': site.tx_xy[0], 'y': site.tx_xy[1], 'row': r, 'col': c,
                'moved': site.bs_moved, 'set_at': _ts(time.time())}

    def _write_bs_position(self, session_only: bool = False) -> None:
        pos = self._bs_position()
        if pos is None:
            return
        targets = [os.path.join(self.session_dir, 'bs_position.json')]
        if not session_only:
            targets.append(self.bs_store)
        for path in targets:
            try:
                with open(path, 'w') as f:
                    json.dump(pos, f, indent=1)
            except OSError as e:
                self._event('warn', f"could not write {path}: {e}")

    def move_bs(self, x: float, y: float) -> bool:
        """Put the BS at map point (x, y) (snapped to the cell centre). Radio maps for that
        cell are loaded if they were generated before, otherwise rm_generate.py is started in
        the background; until they arrive the pred_* columns are blank."""
        if self.site is None:
            return False
        with self.lock:
            r, c, missing = self.site.set_bs(x, y)
            if r < 0:
                return False
            bx, by = self.site.tx_xy
            self._write_bs_position()
            self._log_serial(f"#BS:{bx:.1f}:{by:.1f}:{c}:{r}")
            self._event('map', f"BS {'moved to' if self.site.bs_moved else 'back at its survey cell'} "
                               f"({bx:+.1f}, {by:+.1f}) m"
                               + (f"; generating radio maps ({', '.join(missing)})" if missing else ""))
            self.rm_version += 1
            self._kill_rm_proc()
            if missing:
                self._start_rm_job()
            else:
                self.rm_job = {'status': 'ready', 'started': 0.0, 'missing': [], 'error': ''}
        return True

    def reset_bs(self) -> bool:
        """BS back to the site's survey (tx) cell, with the site's own radio maps."""
        if self.site is None or self.site.base_rc is None:
            return False
        return self.move_bs(*self.site.rc_to_xy(*self.site.base_rc))

    def _kill_rm_proc(self) -> None:
        proc, self._rm_proc = self._rm_proc, None
        if proc is not None and proc.poll() is None:
            try:
                proc.kill()
            except OSError:
                pass

    def _start_rm_job(self) -> None:
        site = self.site
        py = rm_generate.env_python(self.rm_env)
        if not py:
            self.rm_job = {'status': 'error', 'started': time.time(), 'missing': site.reload_bs_maps(),
                           'error': f"no Python venv in {self.rm_env}/.venv (use --rm-env)"}
            self._event('warn', f"radio maps: {self.rm_job['error']}")
            return
        bx, by = site.tx_xy
        cmd = [py, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rm_generate.py'),
               '--site', site.name, '--terrain-dir', site.dir, '--bs', f"{bx},{by}", '--rm-env', self.rm_env]
        flags = 0x08000000 if os.name == 'nt' else 0          # CREATE_NO_WINDOW
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                    creationflags=flags)
        except OSError as e:
            self.rm_job = {'status': 'error', 'started': time.time(), 'missing': [], 'error': str(e)}
            self._event('warn', f"radio maps: could not start rm_generate.py: {e}")
            return
        self._rm_proc = proc
        self.rm_job = {'status': 'running', 'started': time.time(), 'missing': site.reload_bs_maps(), 'error': ''}
        threading.Thread(target=self._rm_watch, args=(proc, site.bs_rc), daemon=True).start()

    def _rm_watch(self, proc, rc) -> None:
        """Follow one rm_generate.py run; load each map as it lands (if the BS hasn't moved since)."""
        err = ''
        for line in proc.stdout:
            line = line.strip()
            if line.startswith('ERROR'):
                err = line[6:]
            if not line.startswith('READY'):
                continue
            with self.lock:
                if self.site.bs_rc != rc:
                    return
                self.rm_job['missing'] = self.site.reload_bs_maps()
                self.rm_version += 1
                self._event('map', f"radio map {line.split()[1]} ready for the BS at "
                                   f"({self.site.tx_xy[0]:+.1f}, {self.site.tx_xy[1]:+.1f})")
        code = proc.wait()
        with self.lock:
            if self.site.bs_rc != rc or self._rm_proc is not proc:
                return
            self._rm_proc = None
            missing = self.site.reload_bs_maps()
            if missing or code != 0:
                self.rm_job.update(status='error', missing=missing,
                                   error=err or f"rm_generate.py exited with code {code}")
                self._event('warn', f"radio maps failed: {self.rm_job['error']}")
            else:
                self.rm_job.update(status='ready', missing=[])

    def map_pose(self, node: str):
        """Latest map pose (x, y, heading or None) of a rover, or None."""
        loc = self.locations.get(node)
        if not loc or (loc['x'] == 0.0 and loc['y'] == 0.0):
            return None
        mx, my, myaw = site_map.to_map(self.alignments.get(node), loc['x'], loc['y'], loc['yaw'])
        return float(mx), float(my), (float(myaw) if myaw is not None else None)

    def get_tracks(self) -> Dict[str, list]:
        with self.lock:
            return {n: list(v) for n, v in self.tracks.items()}

    def track_stats(self, node: str, since: float = 0.0) -> Optional[dict]:
        """Length, on-drivable share and closure gaps of a rover's track in map coordinates."""
        with self.lock:
            tr = [p for p in self.tracks.get(node, []) if p[0] >= since]
            al = self.alignments.get(node)
        if len(tr) < 2:
            return None
        xs, ys = [p[1] for p in tr], [p[2] for p in tr]
        mx, my, _ = site_map.to_map(al, xs, ys)
        dx, dy = mx[1:] - mx[:-1], my[1:] - my[:-1]
        st = {'n': len(tr), 'length_m': float(sum(math.hypot(a, b) for a, b in zip(dx, dy))),
              'start_end_m': float(math.hypot(mx[-1] - mx[0], my[-1] - my[0]))}
        if self.site is not None and self.site.drivable is not None:
            st['on_drivable'] = float(self.site.on_drivable(mx, my).mean())
            st['on_drivable_1m'] = float(self.site.on_drivable(mx, my, 1.0).mean())
        anchor = al.get('anchor')
        if anchor:
            st['to_anchor_m'] = float(math.hypot(mx[-1] - anchor[0], my[-1] - anchor[1]))
        return st

    def _on_rover_status(self, line: str, now: float) -> None:
        parts = line.split(':', 2)
        if len(parts) != 3:
            return
        fields = {}
        for kv in parts[2].split(','):
            k, _, v = kv.partition('=')
            try:
                fields[k] = float(v)
            except ValueError:
                fields[k] = v
        prev = self.rover_status.get(parts[1], {})
        if prev and (prev.get('st'), prev.get('exp')) != (fields.get('st'), fields.get('exp')):
            self._event('control', f"{parts[1]} now {fields.get('st')} '{fields.get('exp')}' "
                                   f"(ctrl v{int(fields.get('cv', 0) or 0)})")
        fields['updated_at'] = now
        self.rover_status[parts[1]] = fields
        nav = self.nav_cmds.get(parts[1])
        nc = fields.get('nc')
        if nav and len(nav) == 3 and isinstance(nc, float) and nc >= nav[0]:
            # rover has it: stop re-sending the points (same version, so nobody re-acts on it)
            self.nav_cmds[parts[1]] = [nav[0], 'ok']
            sent = (self.control['msg'] or {}).get('nav') or {}
            if parts[1] in sent:
                sent[parts[1]] = [nav[0], 'ok']
        keys = sorted(k for k in fields if k != 'updated_at')
        self._csv(f"rover_status_{parts[1]}.csv", ['time', 'epoch'] + keys,
                  [_ts(now), f"{now:.3f}"] + [fields[k] for k in keys])

    # fleet control
    def _send_control_now(self) -> bool:
        msg, ser = self.control['msg'], self._ser
        if ser is None:
            return False
        # every copy carries the BS time: rovers run experiment starts and radio windows on it
        # (FleetClock in radio_daemon.py), so their own clocks don't need NTP. Before any fleet
        # state exists a time-only message goes out as version 0, which rovers never adopt.
        msg = dict(msg or {}, **self._live_fields(), t=round(time.time(), 3), hop=0)
        air = (struct.pack('<BHI', PKT_TYPE_CONTROL, BS_NODE_ID, self.control['version'] & 0xFFFFFFFF)
               + json.dumps(msg, separators=(',', ':')).encode())
        try:
            with self._ser_write_lock:
                ser.write(encode_frame(HOST_CMD_WIFI_TX, air))
            self.control['last_tx'] = time.time()
            return True
        except (serial.SerialException, OSError) as e:
            with self.lock:
                self._event('warn', f"control send failed: {e}")
            return False

    def _live_fields(self) -> dict:
        """Fleet picture added to every control copy (not versioned, never logged as a change):
            fleet  {node: [map x, map y, heading deg | null, BS time of that fix]}
            align  {node: [x, y, theta deg, BS time it was set]}   odometry -> map, site_map.py
            bs     [map x, map y] of the BS, site  the site name
        Relays carry the newest entries they have (by time), so a relay rover knows where the
        explorer is even when it can't hear it directly."""
        out = {}
        with self.lock:
            fleet = {}
            for n, loc in self.locations.items():
                pose = self.map_pose(n)
                if pose is None or not all(math.isfinite(v) for v in pose[:2]):
                    continue
                yaw = round(pose[2]) if pose[2] is not None and math.isfinite(pose[2]) else None
                fleet[n] = [round(pose[0], 1), round(pose[1], 1), yaw, round(loc['updated_at'], 1)]
            align = {}
            for n in self.alignments.data:
                a = self.alignments.get(n)
                vals = [a['x'], a['y'], a['theta']]
                if all(math.isfinite(v) for v in vals):
                    align[n] = [round(vals[0], 2), round(vals[1], 2), round(vals[2], 1),
                                int(self.align_ts.get(n, 0))]
            if fleet:
                out['fleet'] = fleet
            if align:
                out['align'] = align
            if self.site is not None and self.site.tx_xy is not None:
                out['bs'] = [round(self.site.tx_xy[0], 1), round(self.site.tx_xy[1], 1)]
                out['site'] = self.site.name
        return out

    def _maybe_send_control(self, now: float) -> None:
        if now - self.control['last_tx'] >= CONTROL_PERIOD_S:
            self._send_control_now()

    def _set_control(self, msg: dict, note: str) -> None:
        """Publish a new desired fleet state. Version = epoch seconds, so it stays monotonic
        across restarts of this program and rovers never ignore a fresh BS."""
        with self.lock:
            if self.ros_cmds:
                msg = dict(msg, ros={n: list(c) for n, c in self.ros_cmds.items()})
            if self.nav_cmds:
                msg = dict(msg, nav={n: list(c) for n, c in self.nav_cmds.items()})
            if self.pose_modes:
                msg = dict(msg, pose=dict(self.pose_modes))
            if self.relay_cfg:
                msg = dict(msg, relay={n: list(c) for n, c in self.relay_cfg.items()})
            if self.rb_off:
                msg = dict(msg, rb=dict(self.rb_off))
            self.control['version'] = max(int(time.time()), self.control['version'] + 1)
            self.control['msg'] = msg
            self.control['sent_at'] = time.time()
            v = self.control['version']
            self._event('control', f"v{v} {note}")
            self._log_serial(f"#CONTROL:v{v}:{json.dumps(msg, separators=(',', ':'))}")
            self._csv('controls.csv', ['time', 'epoch', 'version', 'state', 'exp', 'note', 'msg'],
                      [_ts(self.control['sent_at']), f"{self.control['sent_at']:.3f}", v,
                       msg.get('state', ''), msg.get('exp', ''), note, json.dumps(msg)])
        self._send_control_now()
        print(f"[CORE] control v{v} {note}: {msg}")

    def start_experiment(self, name: str, cfg: dict) -> str:
        exp = sanitize_name(name) or f"exp_{datetime.now():%H%M%S}"
        self.new_session(exp)
        start_at = time.time() + START_DELAY_S
        self._set_control({'state': 'active', 'exp': exp, 'start_at': round(start_at, 2),
                           'cfg': dict(CONFIG_DEFAULTS, **cfg)}, f"start experiment '{exp}'")
        self.add_marker(f"start:{exp}")
        return exp

    def apply_live(self, cfg: dict) -> bool:
        msg = self.control['msg']
        if not msg or msg.get('state') != 'active':
            with self.lock:
                self._event('warn', "no experiment running - use Start or Standby")
            return False
        self._set_control(dict(msg, cfg=dict(CONFIG_DEFAULTS, **cfg)), "live config change")
        self.add_marker("cfg")
        return True

    def standby(self, cfg: Optional[dict] = None) -> None:
        self.new_session('standby')
        self._set_control({'state': 'standby', 'cfg': dict(CONFIG_DEFAULTS, **(cfg or {}))}, "standby")
        self.add_marker("standby")

    def set_ros(self, nodes: List[str], on: bool) -> None:
        """Ask rovers to start/stop their ROS stack. Each press gets a new cmd_id (epoch ms) so
        the rover acts on it exactly once; re-broadcasts and later control msgs carry it along.
        Rovers need --ros-control. Without a fleet state yet, only the ROS field is sent."""
        nodes = [n for n in nodes if n and n != str(BS_NODE_ID)]
        if not nodes:
            return
        want = 'on' if on else 'off'
        with self.lock:
            cmd_id = max([int(time.time() * 1000)] + [c[1] + 1 for c in self.ros_cmds.values()])
            for n in nodes:
                self.ros_cmds[n] = [want, cmd_id]
        self._set_control(self._base_msg(), f"ROS stack {want}: {', '.join(nodes)}")

    def _base_msg(self) -> dict:
        """Current fleet state without the per-rover command fields (_set_control adds them)."""
        return {k: v for k, v in (self.control['msg'] or {}).items() if k not in ('ros', 'nav', 'pose', 'relay', 'rb')}

    def set_pose_mode(self, node: str, mode: str) -> None:
        """Pose source of rover_nav.py on a rover: 'fused' (wheel distance + T265 heading) or
        'camera' (T265 only). A setting, carried on every control message."""
        if mode not in ('fused', 'camera'):
            return
        with self.lock:
            self.pose_modes[node] = mode
        self._set_control(self._base_msg(), f"pose source {mode}: {node}")

    def set_rb_required(self, node: str, required: bool) -> None:
        """Whether rover_nav.py on a rover needs RB held to drive. Off: it drives on the mux's
        Autonomy input (above teleop) while its controller is connected; LB still overrides.
        A setting, carried on every control message."""
        with self.lock:
            if required:
                self.rb_off.pop(node, None)
            else:
                self.rb_off[node] = 0
        self._set_control(self._base_msg(), f"RB {'required' if required else 'NOT required'}: {node}")

    # routes (rover_nav.py on the rover drives them; RB on its controller must be held unless rb_off)
    def plan_route(self, node: str, waypoints: List[List[float]]):
        """A* route (map metres) from the rover's current map position through the waypoints.
        Returns (Nx2 points or None, error text)."""
        if self.site is None or self.site.drivable is None:
            return None, "route planning needs --site with a drivable map"
        pose = self.map_pose(node)
        if pose is None:
            return None, f"no location from {node} yet"
        if not waypoints:
            return None, "no waypoints"
        if self._planner is None:
            self._planner = route_plan.Planner(self.site)
        return self._planner.plan(pose[:2], waypoints)

    def _cmd_id(self) -> int:
        return max([int(time.time() * 1000)] + [c[0] + 1 for c in self.nav_cmds.values()])

    def send_route(self, node: str, pts_map, speed: float) -> str:
        """Send a planned route to a rover. Returns '' or why it was not sent."""
        import numpy as np
        pts_map = np.asarray(pts_map, float)
        speed = round(min(max(float(speed), 0.1), ROUTE_SPEED_MAX), 2)
        with self.lock:
            al = self.alignments.get(node)
            odom = route_plan.to_odom(al, pts_map)
            cmd_id = self._cmd_id()
            others = {n: c for n, c in self.nav_cmds.items() if n != node}
            for tol in ROUTE_TOLERANCES:
                keep = route_plan.simplify(odom, tol)
                cmd = [cmd_id, speed, route_plan.encode(keep)]
                trial = dict(self._base_msg(), ros={n: list(c) for n, c in self.ros_cmds.items()},
                             nav=dict(others, **{node: cmd}), pose=dict(self.pose_modes),
                             relay={n: list(c) for n, c in self.relay_cfg.items() if n != node},
                             rb=dict(self.rb_off),
                             **self._live_fields(), t=0.0, hop=0)
                if len(json.dumps(trial, separators=(',', ':'))) <= CONTROL_MAX_JSON:
                    break
            else:
                return (f"route too long for one control message ({len(keep)} points at "
                        f"{tol} m); use fewer or closer waypoints")
            self.nav_cmds[node] = cmd
            self.relay_cfg.pop(node, None)        # a hand-sent route takes over from the relay
            keep_map = route_plan.simplify(pts_map, tol)
            self.routes[node] = {'cmd': cmd_id, 'speed': speed, 'map': keep_map.tolist(), 'sent': time.time()}
            self._csv('routes.csv', ['time', 'epoch', 'node', 'cmd_id', 'action', 'speed', 'length_m',
                                     'points', 'tolerance_m', 'map_xy', 'odom_dm'],
                      [_ts(time.time()), f"{time.time():.3f}", node, cmd_id, 'route', speed,
                       f"{route_plan.length(keep_map):.1f}", len(keep), tol,
                       json.dumps(np.round(keep_map, 2).tolist()), json.dumps(cmd[2])])
        self._set_control(self._base_msg(), f"route to {node}: {len(keep)} pts, "
                                            f"{route_plan.length(keep_map):.0f} m at {speed:g} m/s")
        return ''

    def stop_route(self, node: str) -> None:
        self.stop_routes([node])

    def stop_routes(self, nodes, what: str = 'stop route') -> None:
        """Cancel the routes of these rovers and turn their relays off, in one control message."""
        nodes = sorted({str(n) for n in nodes if n})
        if not nodes:
            return
        relays_off = []
        with self.lock:
            for node in nodes:
                self._stop_locked(node)
                if self.relay_cfg.pop(node, None) is not None:
                    relays_off.append(node)
        self._set_control(self._base_msg(), f"{what}: {', '.join(nodes)}"
                          + (f" (relay off: {', '.join(relays_off)})" if relays_off else ""))

    def _stop_locked(self, node: str) -> None:
        cmd_id = self._cmd_id()
        self.nav_cmds[node] = [cmd_id, 'stop']
        self.routes.pop(node, None)
        self._csv('routes.csv', ['time', 'epoch', 'node', 'cmd_id', 'action', 'speed', 'length_m',
                                 'points', 'tolerance_m', 'map_xy', 'odom_dm'],
                  [_ts(time.time()), f"{time.time():.3f}", node, cmd_id, 'stop', '', '', '', '', '', ''])

    # relay role (relay_brain.py in the rover's radio_daemon.py, which needs --site)
    def set_relay(self, node: str, explorer: str, speed: float, ferry_s: float,
                  comfort_dbm: float = -75.0, gain_db: float = 6.0, predict_s: float = 4.0,
                  model: str = 'nn_pmnet') -> str:
        """Make a rover keep itself between the BS and the explorer (or change its settings:
        same explorer = no reset on the rover). Returns '' or why not. The tuning values are
        relay_brain.DEFAULTS comfort_dbm / gain_db / predict_s; model = link table key on the
        rover (nn_pmnet, nn) or 'distance'."""
        if not node or not explorer or node == explorer:
            return "pick a relay rover and a different explorer rover"
        speed = round(min(max(float(speed), 0.1), ROUTE_SPEED_MAX), 2)
        cfg = [str(explorer), speed, round(float(ferry_s), 1), round(float(comfort_dbm), 1),
               round(float(gain_db), 1), round(float(predict_s), 1), str(model)]
        with self.lock:
            self.relay_cfg[node] = cfg
            self.routes.pop(node, None)
            # the explorer is driven by hand: if it was itself a relay (e.g. roles swapped), stop it
            swapped = self.relay_cfg.pop(str(explorer), None) is not None
            if swapped:
                self._stop_locked(str(explorer))
        self._set_control(self._base_msg(), f"relay on: {node} for explorer {explorer} at {speed:g} m/s, "
                                            f"ferry after {cfg[2]:g} s, comfort {cfg[3]:g} dBm, "
                                            f"gain {cfg[4]:g} dB, look-ahead {cfg[5]:g} s, model {cfg[6]}"
                                            + (f" (relay on {explorer} stopped)" if swapped else ""))
        return ''

    def add_marker(self, label: str = '') -> str:
        now = time.time()
        with self.lock:
            label = sanitize_name(label.replace(':', '-')) or f"M{len(self.markers) + 1}"
            self.markers.append((now, label))
            poses = {n: self.map_pose(n) for n in self.locations}
            where = ' '.join(f"{n}@({p[0]:.1f},{p[1]:.1f})" for n, p in poses.items() if p)
            self._event('marker', label + (f"  {where}" if where else ''))
            self._log_serial(f"#MARKER:{label}")
            self._csv('markers.csv', ['time', 'epoch', 'label', 'rover_map_positions'],
                      [_ts(now), f"{now:.3f}", label, where])
        return label

    # display snapshot
    @staticmethod
    def window_estimate(st: dict, now: float):
        """Periodic rover radio state from its last @ROVER_STATUS (BS arrival time only, so
        no clock sync needed). Returns (is_on, secs_left) or None."""
        on, off, win = st.get('on', 0), st.get('off', 0), st.get('win', -1)
        if st.get('st') != 'active' or not (on and off) or not isinstance(win, float) or win < 0:
            return None
        period = on + off
        x = (now - (st['updated_at'] + win)) % period
        return (False, off - x) if x < off else (True, period - x)

    def snapshot(self, since_event: int = 0, history_s: float = HISTORY_S) -> dict:
        """Copy of everything the display needs, taken under the lock in one go."""
        now = time.time()
        t0 = now - history_s
        with self.lock:
            rovers = sorted(set(self.link) | set(self.rover_status) | set(self.contacts))
            recent = defaultdict(int)                       # data deliveries in the last 60 s
            for t, s, k, _l, _v in reversed(self.timeline):
                if now - t > 60:
                    break
                if k == 'data':
                    recent[s] += 1
            rows = []
            for nid in rovers:
                ls = self.link.get(nid)
                st = dict(self.rover_status.get(nid, {}))
                n_data, lat_sum = self.data_stats.get(nid, (0, 0))
                rows.append({
                    'node': nid,
                    'link_up': bool(ls and ls['up']),
                    'link_for': (now - ls['since']) if ls and ls['up'] else ((now - ls['last']) if ls else None),
                    'rssi': ls['rssi'][-1] if ls and ls['rssi'] else None,
                    'contacts': len(self.contacts.get(nid, [])) + (1 if ls and ls['up'] else 0),
                    'status': st,
                    'status_age': (now - st['updated_at']) if st else None,
                    'window': self.window_estimate(st, now) if st else None,
                    'data_total': n_data,
                    'data_60s': recent[nid],
                    'avg_lat': lat_sum / n_data if n_data else None,
                })
            spans = {}
            for nid in rovers:
                sp = [(c['start'], c['end']) for c in self.contacts.get(nid, []) if c['end'] >= t0]
                ls = self.link.get(nid)
                if ls and ls['up']:
                    sp.append((ls['since'], now))
                spans[nid] = sp
            snap = {
                'now': now,
                'connected': self.connected,
                'port': self.port,
                'session_dir': self.session_dir,
                'lines_total': self.lines_total,
                'last_line_age': (now - self.last_line_at) if self.last_line_at else None,
                'bs_rejects_control': self.bs_rejects_control,
                'control': dict(self.control),
                'rovers': rows,
                'nodes': {n: now - t for n, t in self.node_last_seen.items() if now - t < NODE_TIMEOUT_S},
                'edges': dict(self.edges),
                'transfers': [dict(t) for t in self.transfers if now - t['time'] < 8.0],
                'timeline': [x for x in self.timeline if x[0] >= t0],
                'spans': spans,
                'markers': [m for m in self.markers if m[0] >= t0],
                'track_total': dict(self.track_total),
                'locations': {n: dict(v) for n, v in self.locations.items()},
                'alignments': {n: self.alignments.get(n) for n in self.locations},
                'routes': {n: dict(r) for n, r in self.routes.items()},
                'relay_cfg': {n: list(c) for n, c in self.relay_cfg.items()},
                'rb_off': dict(self.rb_off),
                'bs': dict(self._bs_position() or {}, rm_version=self.rm_version, rm_job=dict(self.rm_job)),
                'events': [e for e in self.events if e[0] > since_event],
                'n_delivered': len(self.delivered),
            }
        return snap

    # summary
    def save_summary(self) -> Optional[str]:
        with self.lock:
            report = self._summary_text()
            path = os.path.join(self.session_dir, f"dtn_summary_{datetime.now():%Y%m%d_%H%M%S}.txt")
            try:
                with open(path, 'w') as f:
                    f.write(report + "\n")
                self._event('session', f"summary saved: {path}")
            except OSError as e:
                self._event('warn', f"could not save summary: {e}")
                path = None
        print("\n" + report)
        return path

    def _summary_text(self) -> str:
        now = time.time()
        dur = int(now - self.start_time)
        all_deliv = list(self.delivered.items())
        for nid, hist in self.boot_history.items():
            for rec in hist:
                all_deliv.extend(((nid, q), i) for q, i in rec['delivered'].items())
        nodes = sorted({s for (s, _q), _i in all_deliv} | set(self.link) | set(self.rover_status)
                       | set(self.contacts) | set(self.seq_range))
        W = 70
        h, rem = divmod(dur, 3600)
        m, s = divmod(rem, 60)
        L = ["=" * W, "  DTN Mesh Network  -  Session Summary Report",
             f"  Session   : {os.path.basename(self.session_dir)}",
             f"  Generated : {datetime.now():%Y-%m-%d %H:%M:%S}",
             f"  Duration  : {f'{h}h ' if h else ''}{m:02d}m {s:02d}s", "=" * W, ""]

        data_all = [i['latency_ms'] for _k, i in all_deliv if i.get('kind') == 'data']
        L += ["Totals",
              f"  Bundles delivered (all)   : {len(all_deliv)}",
              f"  Data bundles delivered    : {len(data_all)}",
              f"  Ferry duplicates          : {sum(self.ferry_dup.values())}",
              f"  Antipacket failures       : {sum(self.antipkt_fail.values())}",
              f"  Data latency avg/p50/p95  : " + (f"{sum(data_all)/len(data_all):.0f} / {_pct(data_all, 50)} / "
                                                  f"{_pct(data_all, 95)} ms" if data_all else "N/A"), ""]

        hdr = (f"  {'Node':<8} {'Data':>6} {'Tel':>6} {'AvgLat':>8} {'p50':>6} {'p95':>6}"
               f" {'RovGen':>7} {'Data/Gen':>9} {'FDup':>5} {'APFl':>5} {'RSSI':>7}")
        L += ["Per node  (data = generated sensor bundles; Data/Gen uses the rover's own counter)",
              hdr, "  " + "-" * (len(hdr) - 2)]
        for n in nodes:
            d = [i['latency_ms'] for (s, _q), i in all_deliv if s == n and i.get('kind') == 'data']
            tel = sum(1 for (s, _q), i in all_deliv if s == n and i.get('kind') != 'data')
            gen = self.rover_status.get(n, {}).get('gen')
            gen = gen if isinstance(gen, float) else None
            rssi = self.node_rssi.get(n, [])
            L.append(f"  {n:<8} {len(d):>6} {tel:>6} "
                     f"{(f'{sum(d)/len(d):.0f}' if d else '-'):>8} {(_pct(d, 50) if d else '-'):>6} "
                     f"{(_pct(d, 95) if d else '-'):>6} {(f'{gen:.0f}' if gen is not None else '-'):>7} "
                     f"{(f'{100.0*len(d)/gen:.1f}%' if gen else '-'):>9} {self.ferry_dup.get(n, 0):>5} "
                     f"{self.antipkt_fail.get(n, 0):>5} "
                     f"{(f'{sum(rssi)/len(rssi):.1f}' if rssi else '-'):>7}")
        L.append("")

        hdr = (f"  {'Node':<8} {'N':>4} {'UpTime':>8} {'Up%':>5} {'MeanLen':>8} {'DataInUp':>9} {'Data/s up':>10}")
        L += [f"Contacts with the BS  (link down after {CONTACT_GAP_S:.0f}s without a beacon)",
              hdr, "  " + "-" * (len(hdr) - 2)]
        for n in nodes:
            recs = list(self.contacts.get(n, []))
            ls = self.link.get(n)
            if ls and ls['up']:
                recs.append(self._contact_record(ls))
            up = sum(r['end'] - r['start'] for r in recs)
            data_up = sum(r['data'] for r in recs)
            L.append(f"  {n:<8} {len(recs):>4} {up:>7.0f}s {(f'{100.0*up/dur:.0f}%' if dur else '-'):>5} "
                     f"{(f'{up/len(recs):.1f}s' if recs else '-'):>8} {data_up:>9} "
                     f"{(f'{data_up/up:.2f}' if up > 0 else '-'):>10}")
        L.append("")

        if self.rover_status:
            L.append("Rovers (last @ROVER_STATUS)")
            for n, st in sorted(self.rover_status.items()):
                bw, on, off = st.get('bw', 0), st.get('on', 0), st.get('off', 0)
                L.append(f"  {n}: {st.get('st')} '{st.get('exp')}' ctrl v{int(st.get('cv', 0) or 0)}, "
                         f"{f'bw {bw:g}/s' if bw else 'bw unlimited'}, "
                         f"{f'periodic {on:g}s on/{off:g}s off' if on and off else 'always on'}, "
                         f"gen every {st.get('gi', 0):g}s | queue {st.get('q', 0):.0f} own/{st.get('qf', 0):.0f} "
                         f"ferried | gen {st.get('gen', 0):.0f} tx {st.get('tx', 0):.0f} "
                         f"retx {st.get('retx', 0):.0f} ack {st.get('ack', 0):.0f} | ros {st.get('ros', '-')}"
                         + (f", loc {st['loc']}" if st.get('loc') else ""))
            L.append("")
        if self.control['msg'] is not None:
            msg = self.control['msg']
            L += [f"Fleet control (last sent): v{self.control['version']} {msg.get('state', '(no fleet state)')}"
                  + (f" '{msg['exp']}'" if msg.get('exp') else ""),
                  f"  cfg: {json.dumps(msg.get('cfg', {}), separators=(',', ':'))}",
                  "  (full history in controls.csv)", ""]
        if self.bs_rejects_control:
            L += ["WARNING: the BS firmware rejected control frames (unknown cmd 0x30) - reflash it.", ""]
        if self.tracks:
            L.append("Map" + (f"  ({self.site.describe()})" if self.site else "  (no --site)"))
            for n in sorted(self.tracks):
                al, st = self.alignments.get(n), self.track_stats(n)
                line = (f"  {n}: alignment offset ({al['x']:+.2f}, {al['y']:+.2f}) m rotation "
                        f"{al['theta']:+.1f} deg")
                if st:
                    line += (f" | {st['n']} samples, {st['length_m']:.0f} m driven, "
                             f"start-end {st['start_end_m']:.1f} m")
                    if 'to_anchor_m' in st:
                        line += f", end-to-alignment point {st['to_anchor_m']:.1f} m"
                    if 'on_drivable' in st:
                        line += (f", on drivable {100 * st['on_drivable']:.0f}% "
                                 f"({100 * st['on_drivable_1m']:.0f}% within 1 m)")
                L.append(line)
            L += ["  (alignment history in alignments.csv)", ""]
        if self.site is not None and self.site.bs_moved:
            pos = self._bs_position()
            L += [f"BS position: moved to ({pos['x']:+.1f}, {pos['y']:+.1f}) m (cell col {pos['col']}, "
                  f"row {pos['row']}); predictions use maps generated for it (bs_position.json)", ""]
        if self.markers:
            L.append("Markers")
            L += [f"  {lab:<16} {_ts(t)}  (+{t - self.start_time:.0f}s)" for t, lab in self.markers]
            L.append("")
        restarts = {n: h for n, h in self.boot_history.items() if h}
        if restarts:
            L.append("Node restarts")
            for n, hist in sorted(restarts.items()):
                L.append(f"  {n}: {len(hist)} restart(s), now on boot {self.boot_count[n]}")
            L.append("")
        L.append("=" * W)
        return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description='BS core (headless): log the BS serial port')
    ap.add_argument('--port', default='COM5')
    ap.add_argument('--baud', type=int, default=BAUD_RATE)
    ap.add_argument('--report-dir', default='bs_reports')
    ap.add_argument('--site', default=None, help='terrain site, e.g. fountain or range')
    ap.add_argument('--terrain-dir', default=None, help='default: ./terrain, then ../terrain')
    ap.add_argument('--rm-env', default=None, help=f'lunar-mesh-env checkout for radio-map generation '
                                                   f'(default {rm_generate.DEFAULT_ENV})')
    args = ap.parse_args()
    core = BSCore(args.port, args.baud, args.report_dir, args.site, args.terrain_dir, args.rm_env)
    core.start()
    print("[CORE] running headless, Ctrl-C to stop and write the summary")
    seen = 0
    try:
        while True:
            time.sleep(1.0)
            for eid, t, cat, text in core.snapshot(since_event=seen, history_s=1)['events']:
                seen = eid
                print(f"{datetime.fromtimestamp(t):%H:%M:%S} [{cat}] {text}")
    except KeyboardInterrupt:
        pass
    finally:
        core.shutdown()


if __name__ == '__main__':
    main()
