"""
Base-station core: reads the BS ESP32 serial port, keeps all mesh / experiment state, writes
the session logs and reports, and sends fleet control to the rovers. No GUI.

bs_dashboard.py is the live display on top of this. The core can also run on its own, so
logging and reports never depend on the display:

    python bs_core.py --port COM5              # headless: log everything, Ctrl-C writes the summary

Session folder (bs_reports/session_<time>[_<tag>]/):
    serial.log              every BS line with a wall-clock timestamp (+ #CONTACT/#MARKER/#CONTROL)
    deliveries.csv          one row per delivered bundle (data vs telemetry, via, hops, latency)
    contacts.csv            one row per rover <-> BS contact (start, end, data delivered, RSSI)
    rover_status_<id>.csv   every @ROVER_STATUS report
    controls.csv            every fleet control change
    markers.csv             markers
    coverage.csv            every @LOCATION sample
    dtn_summary_<time>.txt  session report
    node_<id>_boot<n>.txt   per-boot report on @NODE_RESTART
"""

import argparse
import csv
import json
import os
import struct
import threading
import time
from collections import defaultdict, deque
from datetime import datetime
from typing import Dict, List, Optional

import serial

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
}


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
    def __init__(self, port: str, baud: int = BAUD_RATE, report_dir: str = 'bs_reports'):
        self.port = port
        self.baud = baud
        self.report_dir = report_dir
        self.lock = threading.RLock()
        self._ser: Optional[serial.Serial] = None
        self._ser_write_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.connected = False
        self.lines_total = 0
        self.last_line_at = 0.0
        self.bs_rejects_control = False
        self.control = {'version': 0, 'msg': None, 'sent_at': 0.0, 'last_tx': 0.0}

        self.session_dir = ''
        self._serial_log = None
        self._events_seq = 0
        self.events: deque = deque(maxlen=3000)   # (id, time, category, text)
        self._reset_state()
        self._new_session_files('')

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
        self.map_samples = defaultdict(lambda: deque(maxlen=5000))   # (x, y, rssi)
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
        self._csv('coverage.csv', ['timestamp', 'node_id', 'lat', 'lon', 'alt_m', 'rssi_to_bs_dBm'], None)
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

    def _on_location(self, parts: list, now: float) -> None:
        node, x, y, z = parts[1], float(parts[2]), float(parts[3]), float(parts[4])
        rssi = int(parts[5]) if len(parts) >= 6 and parts[5] else None
        self.locations[node] = {'x': x, 'y': y, 'z': z, 'rssi': rssi, 'updated_at': now}
        if rssi is not None:
            self.map_samples[node].append((x, y, rssi))
        if x != 0.0 or y != 0.0:
            self._csv('coverage.csv', ['timestamp', 'node_id', 'lat', 'lon', 'alt_m', 'rssi_to_bs_dBm'],
                      [_ts(now), node, f"{x:.6f}", f"{y:.6f}", f"{z:.1f}", rssi if rssi is not None else ''])

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
        keys = sorted(k for k in fields if k != 'updated_at')
        self._csv(f"rover_status_{parts[1]}.csv", ['time', 'epoch'] + keys,
                  [_ts(now), f"{now:.3f}"] + [fields[k] for k in keys])

    # fleet control
    def _send_control_now(self) -> bool:
        msg, ser = self.control['msg'], self._ser
        if msg is None or ser is None:
            return False
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

    def _maybe_send_control(self, now: float) -> None:
        if self.control['msg'] is not None and now - self.control['last_tx'] >= CONTROL_PERIOD_S:
            self._send_control_now()

    def _set_control(self, msg: dict, note: str) -> None:
        """Publish a new desired fleet state. Version = epoch seconds, so it stays monotonic
        across restarts of this program and rovers never ignore a fresh BS."""
        with self.lock:
            self.control['version'] = max(int(time.time()), self.control['version'] + 1)
            self.control['msg'] = msg
            self.control['sent_at'] = time.time()
            v = self.control['version']
            self._event('control', f"v{v} {note}")
            self._log_serial(f"#CONTROL:v{v}:{json.dumps(msg, separators=(',', ':'))}")
            self._csv('controls.csv', ['time', 'epoch', 'version', 'state', 'exp', 'note', 'msg'],
                      [_ts(self.control['sent_at']), f"{self.control['sent_at']:.3f}", v,
                       msg.get('state'), msg.get('exp', ''), note, json.dumps(msg)])
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

    def add_marker(self, label: str = '') -> str:
        now = time.time()
        with self.lock:
            label = sanitize_name(label.replace(':', '-')) or f"M{len(self.markers) + 1}"
            self.markers.append((now, label))
            self._event('marker', label)
            self._log_serial(f"#MARKER:{label}")
            self._csv('markers.csv', ['time', 'epoch', 'label'], [_ts(now), f"{now:.3f}", label])
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
                'map_samples': {n: list(v) for n, v in self.map_samples.items()},
                'locations': {n: dict(v) for n, v in self.locations.items()},
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
                         f"retx {st.get('retx', 0):.0f} ack {st.get('ack', 0):.0f}")
            L.append("")
        if self.control['msg'] is not None:
            msg = self.control['msg']
            L += [f"Fleet control (last sent): v{self.control['version']} {msg.get('state')}"
                  + (f" '{msg['exp']}'" if msg.get('exp') else ""),
                  f"  cfg: {json.dumps(msg.get('cfg', {}), separators=(',', ':'))}",
                  "  (full history in controls.csv)", ""]
        if self.bs_rejects_control:
            L += ["WARNING: the BS firmware rejected control frames (unknown cmd 0x30) - reflash it.", ""]
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
    args = ap.parse_args()
    core = BSCore(args.port, args.baud, args.report_dir)
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
