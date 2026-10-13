"""
Rover node daemon, runs on a rover Jetson Orin Nano.

The ESP32 attached to a rover is now a *pure broadcast relay*: it broadcasts every
over-air frame the daemon hands it (HOST_CMD_WIFI_TX) and ships every received frame to
the daemon (HOST_CMD_WIFI_RX, tagged with src MAC + RSSI). ALL DTN logic lives here:
beacon generation, peer discovery, the bundle store, epidemic forwarding, antipacket /
dedup, and ACK-map apply/propagate.

The over-air wire format is byte-identical to the legacy firmware so the unchanged Base
Station keeps interoperating (it still emits @NET/@METRIC/@DTN_RX for mesh_visualizer.py).

Usage:
    python radio_daemon.py [--serial PORT] [--baud BAUD] [--interval SECS] [--node-id ID]

    --serial   Serial port connected to ESP32 (default: /dev/ttyUSB0)
    --baud     Baud rate (default: 115200)
    --interval Seconds between generated sensor bundles (default: 1, 0 = disable)
    --node-id  Override node ID (default: auto-detected from ESP32 STATUS_RESP on startup)

    python3 radio_daemon.py --serial /dev/ttyUSB0 --ros-location-topic /car/t265/odom/sample

    python3 radio_daemon.py ... --ros-control    # let the BS dashboard start/stop the ROS stack

Comms restriction modes (combinable, see --help for all options):
    --bw-limit 1                     data bundles/s budget (beacons/ackmaps/location exempt)
    --comms-on 30 --comms-off 60     radio fully on (TX+RX) 30 s, then fully off 60 s;
                                     windows are aligned to wall-clock time so NTP-synced
                                     rovers share the same schedule

Every run writes a JSON-lines event log (rover_logs/ by default) with every TX/RX,
store change, peer change, window transition and a 1 s state snapshot, plus a
*_summary.json on shutdown.
"""

import argparse
import json
import math
import os
import platform
import signal
import struct
import subprocess
import threading
import time
import traceback
import serial
from abc import ABC, abstractmethod
from collections import defaultdict
from datetime import datetime
from typing import Callable, Dict, List, Optional, Set, Tuple

try:
    import rospy
    from nav_msgs.msg import Odometry as _Odometry
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

# host UART framing (must match mesh_main.c)
HOST_SOF              = 0xAA
HOST_MAX_PAYLOAD      = 1100
BASE_STATION_NODE_ID  = 23768

HOST_CMD_QUERY_STATUS = 0x02
HOST_CMD_STATUS_RESP  = 0x21
HOST_CMD_WIFI_TX      = 0x30   # Jetson -> ESP32: broadcast these raw over-air bytes
HOST_CMD_WIFI_RX      = 0x31   # ESP32 -> Jetson: [src_mac:6][rssi:1][air bytes]

# over-air packet types (first byte of the air payload)
PKT_TYPE_BEACON   = 0x01
PKT_TYPE_BUNDLE   = 0x02
PKT_TYPE_ACKMAP   = 0x04
PKT_TYPE_LOCATION = 0x05
PKT_TYPE_CONTROL  = 0x06   # fleet control from the BS laptop: [type][origin u16][version u32][json]

CONTROL_HDR_FMT    = '<BHI'
CONTROL_HDR_SIZE   = struct.calcsize(CONTROL_HDR_FMT)   # 7
CONTROL_REPEAT_S   = 5.0    # rovers re-broadcast their current control so out-of-range rovers get it
# config keys the BS may set at runtime (same names as the CLI options)
CONFIG_KEYS = ('interval', 'bw_limit', 'bw_burst', 'bw_order', 'bw_count_telemetry',
               'comms_on', 'comms_off', 'comms_phase', 'payload_bytes', 'retransmit_s',
               'loc_interval')

# dtn_bundle_t layout (packed, little-endian, matches ESP32 C struct)
# Fields: creation_time, sequence_number, lifetime, source_node, dest_node,
#         report_to_node, prev_node, request_delivery_report, is_telemetry,
#         hop_limit, hop_count, payload_len, payload[1024]
BUNDLE_FMT      = '<IIIHHHHBBBBI1024s'
BUNDLE_SIZE     = struct.calcsize(BUNDLE_FMT)        # 1052
BUNDLE_HDR_SIZE = BUNDLE_SIZE - 1024                 # 28 (header, no payload)
PAYLOAD_LEN_OFF = 24                                 # byte offset of payload_len field

# beacon_pkt_t: pkt_type(u8), node_id(u16), timestamp_ms(u32), boot_id(u32)
BEACON_FMT  = '<BHII'
BEACON_SIZE = struct.calcsize(BEACON_FMT)            # 11

# ackmap_pkt_t: pkt_type(u8), origin_node_id(u16), source_node(u16),
#               seq_base(u32), bitmap_len(u8), bitmap[32]
ACKMAP_BITMAP_BYTES = 32
ACKMAP_BITMAP_BITS  = ACKMAP_BITMAP_BYTES * 8        # 256
ACKMAP_FMT          = '<BHHIB32s'
ACKMAP_SIZE         = struct.calcsize(ACKMAP_FMT)    # 42
ACKMAP_HDR_SIZE     = ACKMAP_SIZE - ACKMAP_BITMAP_BYTES  # 10

# host_status_t: node_id(u16), active_peers, store_used, store_max, channel (all u8)
STATUS_FMT  = '<HBBBB'
STATUS_SIZE = struct.calcsize(STATUS_FMT)            # 6

# DTN parameters (mirror mesh_main.c)
BUNDLE_DEFAULT_LIFETIME  = 3000000  # ms (50 minutes)
BUNDLE_DEFAULT_HOP_LIMIT = 5
TELEMETRY_ACK_LIFETIME   = 10000    # ms

BEACON_INTERVAL_S          = 1.0
FORWARD_INTERVAL_S         = 1.0
PEER_TIMEOUT_S             = 10.0
BUNDLE_RETRANSMIT_S        = 5.0    # re-send a forwarded-but-unacked bundle to the BS after 5s
LOCATION_INTERVAL_S        = 5.0    # how often to broadcast own location to neighbors
LOCATION_BUNDLE_INTERVAL_S = 1.0    # default for how often own location goes to the BS (--loc-interval)
ROVER_STATUS_INTERVAL_S    = 5.0    # how often to send @ROVER_STATUS telemetry toward BS
STATE_LOG_INTERVAL_S       = 1.0    # how often to write a full state snapshot to the event log
BW_THROTTLE_LOG_S          = 1.0    # min spacing between bw_throttled log lines

# ROS stack control (--ros-control). teleop.launch only: car_startup.launch (what the
# controller's OPTIONS button runs) also includes astar_planner, which drives the goals
# mission on its own - never start that remotely without a human able to override.
ROS_STACK_CMD     = ['roslaunch', 'mushr_base', 'teleop.launch']
ROS_STOP_GRACE_S  = 20.0   # SIGTERM a stack that hasn't exited this long after SIGINT
ROS_CHECK_S       = 1.0    # how often to re-scan for running stacks

RECONNECT_DELAY_S  = 5.0
SERIAL_SETTLE_S    = 3.0
WRITE_TIMEOUT_S    = 2.0
STATUS_INTERVAL_S  = 5.0


# rover-side event log
class EventLog:
    """Append-only JSON-lines log, one object per line. Line-buffered so a power cut on
    the rover loses at most the line being written."""

    def __init__(self, path: Optional[str]):
        self.path = path
        self.exp_path: Optional[str] = None
        self.node_id: Optional[int] = None
        self._lock = threading.Lock()
        self._t0 = time.monotonic()
        self._f = None
        self._exp_f = None
        if path:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            self._f = open(path, 'a', buffering=1)

    def start_experiment(self, exp_id: str) -> Optional[str]:
        """Mirror every event into a per-experiment file as well (the daemon-lifetime log
        keeps everything). Returns the experiment log path."""
        self.end_experiment()
        if not self.path:
            return None
        d = os.path.dirname(self.path) or '.'
        path = os.path.join(d, f"exp_{exp_id}_{self.node_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl")
        with self._lock:
            self._exp_f = open(path, 'a', buffering=1)
            self.exp_path = path
        return path

    def end_experiment(self) -> None:
        with self._lock:
            if self._exp_f is not None:
                self._exp_f.close()
            self._exp_f = None
            self.exp_path = None

    def log(self, ev: str, **fields) -> None:
        if self._f is None:
            return
        rec = {'t': round(time.time(), 3),
               'up_ms': int((time.monotonic() - self._t0) * 1000),
               'node': self.node_id, 'ev': ev}
        rec.update(fields)
        line = json.dumps(rec, separators=(',', ':'), default=str)
        with self._lock:
            try:
                self._f.write(line + '\n')
                if self._exp_f is not None:
                    self._exp_f.write(line + '\n')
            except (OSError, ValueError):
                pass

    def close(self) -> None:
        self.end_experiment()
        with self._lock:
            if self._f is not None:
                self._f.close()
                self._f = None


# artificial comms restrictions
class FleetClock:
    """BS (laptop) wall-clock time as seen by this rover.

    Jetsons without internet have no NTP, so their clocks can be minutes off. The BS stamps
    every control message it sends with its time ("t") and hop count 0; rovers re-stamp
    their relays with their own estimate and hop + 1. Experiment starts and periodic radio
    windows use now() so the whole fleet runs on BS time. Before the first control message
    the local clock is used (offset 0).
    """

    JUMP_S     = 1.0     # an estimate this far from the current one replaces it outright
    HOP_HOLD_S = 30.0    # prefer estimates from fewer hops if one arrived this recently

    def __init__(self):
        self.offset = 0.0              # BS time - local time [s]
        self.synced = False
        self.hop = 0
        self._updated = 0.0            # monotonic time of the last accepted estimate

    def now(self) -> float:
        return time.time() + self.offset

    def update(self, bs_time: float, hop: int) -> Optional[float]:
        """Feed one stamped control message. Returns the step in seconds when the offset
        jumped (first sync, or the BS / local clock was changed), else None."""
        mono = time.monotonic()
        if self.synced and hop > self.hop and mono - self._updated < self.HOP_HOLD_S:
            return None
        off = bs_time - time.time()
        step = None
        if not self.synced or abs(off - self.offset) > self.JUMP_S:
            step = off - self.offset
            self.offset = off
        else:
            # smooth out UART/air delay jitter (a few tens of ms)
            self.offset += 0.2 * (off - self.offset)
        self.synced, self.hop, self._updated = True, hop, mono
        return step


FLEET_CLOCK = FleetClock()


class CommsPolicy:
    """Bandwidth budget + periodic radio windows.

    bw_limit: data bundles per second allowed over the air (token bucket, 0 = unlimited).
              Telemetry bundles (acks, location, status) are exempt unless count_telemetry.
              Beacons, ackmaps and location broadcasts are never counted.
    on_s/off_s: radio fully on for on_s, fully off (no TX, RX dropped) for off_s. Either
              0 = always on. Windows are aligned to BS time (FLEET_CLOCK, epoch + phase_s)
              so all rovers open their windows together without NTP.
    """

    def __init__(self, bw_limit: float = 0.0, bw_burst: float = 1.0,
                 count_telemetry: bool = False, on_s: float = 0.0, off_s: float = 0.0,
                 phase_s: float = 0.0):
        self.bw_limit        = max(0.0, bw_limit)
        self.bw_burst        = max(1.0, bw_burst)
        self.count_telemetry = count_telemetry
        self.on_s            = max(0.0, on_s)
        self.off_s           = max(0.0, off_s)
        self.phase_s         = phase_s
        self._tokens         = self.bw_burst
        self._t              = time.monotonic()

    @property
    def periodic(self) -> bool:
        return self.on_s > 0 and self.off_s > 0

    def window(self) -> Tuple[bool, Optional[float], int]:
        """(radio_on, seconds left in the current window or None, cycle index)."""
        if not self.periodic:
            return True, None, 0
        rel = FLEET_CLOCK.now() - self.phase_s
        period = self.on_s + self.off_s
        t = rel % period
        cycle = int(rel // period)
        if t < self.on_s:
            return True, self.on_s - t, cycle
        return False, period - t, cycle

    def counts(self, b: dict) -> bool:
        return self.bw_limit > 0 and (self.count_telemetry or not b.get('is_telemetry'))

    def _refill(self) -> None:
        now = time.monotonic()
        self._tokens = min(self.bw_burst, self._tokens + (now - self._t) * self.bw_limit)
        self._t = now

    def tokens(self) -> Optional[float]:
        if self.bw_limit <= 0:
            return None
        self._refill()
        return round(self._tokens, 3)

    def try_take(self) -> bool:
        if self.bw_limit <= 0:
            return True
        self._refill()
        if self._tokens >= 1.0:
            self._tokens -= 1.0
            return True
        return False

    def describe(self) -> dict:
        return {'bw_limit': self.bw_limit, 'bw_burst': self.bw_burst,
                'bw_count_telemetry': self.count_telemetry,
                'comms_on_s': self.on_s, 'comms_off_s': self.off_s,
                'comms_phase_s': self.phase_s, 'periodic': self.periodic}


def pack_control(version: int, msg: dict, origin: int = BASE_STATION_NODE_ID) -> bytes:
    return (struct.pack(CONTROL_HDR_FMT, PKT_TYPE_CONTROL, origin, version & 0xFFFFFFFF)
            + json.dumps(msg, separators=(',', ':')).encode())


def unpack_control(rest: bytes) -> Optional[Tuple[int, int, dict]]:
    """Returns (origin, version, msg) or None if malformed."""
    if len(rest) <= CONTROL_HDR_SIZE:
        return None
    _t, origin, version = struct.unpack_from(CONTROL_HDR_FMT, rest)
    try:
        msg = json.loads(rest[CONTROL_HDR_SIZE:].decode())
    except (ValueError, UnicodeDecodeError):
        return None
    return (origin, version, msg) if isinstance(msg, dict) else None


def _percentile(vals: List[float], p: float) -> Optional[float]:
    if not vals:
        return None
    s = sorted(vals)
    k = min(len(s) - 1, max(0, int(round(p / 100.0 * (len(s) - 1)))))
    return s[k]


# CRC-16/CCITT (poly 0x1021, init 0xFFFF)
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_frame(cmd: int, payload: bytes = b'') -> bytes:
    plen = len(payload)
    header = bytes([HOST_SOF, cmd, plen & 0xFF, (plen >> 8) & 0xFF])
    crc_data = bytes([cmd, plen & 0xFF, (plen >> 8) & 0xFF]) + payload
    crc = _crc16(crc_data)
    return header + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _read_exact(ser: serial.Serial, n: int, timeout_s: float = 1.0) -> bytes:
    deadline = time.monotonic() + timeout_s
    buf = b''
    while len(buf) < n:
        if time.monotonic() >= deadline:
            return buf
        buf += ser.read(n - len(buf))
    return buf


def recv_frame(ser: serial.Serial, timeout_s: float = 1.0, verbose: bool = False,
               on_ascii: Optional[Callable[[str], None]] = None) -> Optional[Tuple[int, bytes]]:
    """Block until a valid frame arrives or timeout. Returns (cmd, payload) or None.

    on_ascii: called with decoded text whenever a run of non-SOF bytes is flushed,
              used to scan for ESP32 boot banners without discarding other frames.
    """
    deadline = time.monotonic() + timeout_s
    skipped = bytearray()

    def _flush_skipped() -> None:
        if skipped:
            text = bytes(skipped).decode('ascii', errors='replace')
            if on_ascii:
                on_ascii(text)
            skipped.clear()

    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != HOST_SOF:
            skipped.append(b[0])
            continue
        _flush_skipped()
        header = _read_exact(ser, 3, timeout_s=0.2)
        if len(header) < 3:
            return None
        cmd = header[0]
        plen = header[1] | (header[2] << 8)
        if plen > HOST_MAX_PAYLOAD:
            print(f"[RECV] Oversized payload claim {plen} > {HOST_MAX_PAYLOAD}, discarding")
            continue
        payload = _read_exact(ser, plen, timeout_s=1.0) if plen else b''
        crc_bytes = _read_exact(ser, 2, timeout_s=0.2)
        if len(crc_bytes) < 2:
            return None
        recv_crc = crc_bytes[0] | (crc_bytes[1] << 8)
        calc_crc = _crc16(bytes([cmd, header[1], header[2]]) + payload)
        if recv_crc != calc_crc:
            print(f"[WARN] CRC mismatch cmd=0x{cmd:02x} expected=0x{calc_crc:04x} got=0x{recv_crc:04x}")
            continue
        if verbose:
            print(f"[RECV] Frame cmd=0x{cmd:02x} len={plen}")
        return cmd, payload

    _flush_skipped()
    return None


# bundle (de)serialization
def pack_bundle(b: dict) -> bytes:
    payload_bytes = b['payload']
    if isinstance(payload_bytes, str):
        payload_bytes = payload_bytes.encode()
    payload_padded = payload_bytes[:1024].ljust(1024, b'\x00')
    return struct.pack(
        BUNDLE_FMT,
        b['creation_time'],
        b['sequence_number'],
        b.get('lifetime', BUNDLE_DEFAULT_LIFETIME),
        b['source_node'],
        b.get('dest_node', BASE_STATION_NODE_ID),
        b.get('report_to_node', 0),
        b.get('prev_node', b['source_node']),
        int(b.get('request_delivery_report', False)),
        int(b.get('is_telemetry', False)),
        b.get('hop_limit', BUNDLE_DEFAULT_HOP_LIMIT),
        b.get('hop_count', 0),
        len(payload_bytes),
        payload_padded,
    )


def unpack_bundle(data: bytes) -> Optional[dict]:
    # received air bundles are trimmed to header + payload_len; pad to full size to unpack.
    if len(data) < BUNDLE_HDR_SIZE:
        return None
    if len(data) < BUNDLE_SIZE:
        data = data + b'\x00' * (BUNDLE_SIZE - len(data))
    fields = struct.unpack_from(BUNDLE_FMT, data)
    (creation_time, seq, lifetime, source_node, dest_node, report_to, prev_node,
     req_report, is_telem, hop_limit, hop_count, payload_len, payload_raw) = fields
    return {
        'creation_time':           creation_time,
        'sequence_number':         seq,
        'lifetime':                lifetime,
        'source_node':             source_node,
        'dest_node':               dest_node,
        'report_to_node':          report_to,
        'prev_node':               prev_node,
        'request_delivery_report': bool(req_report),
        'is_telemetry':            bool(is_telem),
        'hop_limit':               hop_limit,
        'hop_count':               hop_count,
        'payload':                 payload_raw[:payload_len],
    }


def encode_air_bundle(b: dict) -> bytes:
    """[PKT_TYPE_BUNDLE][28-byte header][payload_len bytes] - matches firmware bundle_tx_task."""
    full = pack_bundle(b)
    payload_len = struct.unpack_from('<I', full, PAYLOAD_LEN_OFF)[0]
    n = BUNDLE_HDR_SIZE + min(payload_len, 1024)
    return bytes([PKT_TYPE_BUNDLE]) + full[:n]


# beacon (de)serialization
def pack_beacon(node_id: int, timestamp_ms: int, boot_id: int) -> bytes:
    return struct.pack(BEACON_FMT, PKT_TYPE_BEACON, node_id,
                       timestamp_ms & 0xFFFFFFFF, boot_id & 0xFFFFFFFF)


def unpack_beacon(rest: bytes) -> Optional[Tuple[int, int, int]]:
    # accept old (7-byte, no boot_id) and new (11-byte) formats
    if len(rest) < 7:
        return None
    node_id = struct.unpack_from('<H', rest, 1)[0]
    timestamp_ms = struct.unpack_from('<I', rest, 3)[0]
    boot_id = struct.unpack_from('<I', rest, 7)[0] if len(rest) >= BEACON_SIZE else 0
    return node_id, timestamp_ms, boot_id


# ackmap (de)serialization  (origin is always the BS)
def pack_ackmap(source_node: int, seq_base: int, bitmap: bytes) -> bytes:
    return struct.pack(ACKMAP_FMT, PKT_TYPE_ACKMAP, BASE_STATION_NODE_ID, source_node,
                       seq_base & 0xFFFFFFFF, ACKMAP_BITMAP_BYTES, bytes(bitmap))


def unpack_ackmap(rest: bytes):
    if len(rest) < ACKMAP_HDR_SIZE:
        return None
    if len(rest) < ACKMAP_SIZE:
        rest = rest + b'\x00' * (ACKMAP_SIZE - len(rest))
    # (pkt_type, origin, source, seq_base, bitmap_len, bitmap)
    return struct.unpack_from(ACKMAP_FMT, rest)


# location (de)serialization
# pkt_type(u8), node_id(u16), lat(f32), lon(f32), alt(f32)- 15 bytes total
LOCATION_FMT  = '<BHfff'
LOCATION_SIZE = struct.calcsize(LOCATION_FMT)   # 15


def _yaw_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    """Heading (rotation about z) of a quaternion, degrees counter-clockwise from +x."""
    return math.degrees(math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))


def pack_location(node_id: int, lat: float, lon: float, alt: float) -> bytes:
    return struct.pack(LOCATION_FMT, PKT_TYPE_LOCATION, node_id, lat, lon, alt)


def unpack_location(data: bytes) -> Optional[Tuple[int, float, float, float]]:
    if len(data) < LOCATION_SIZE:
        return None
    _, node_id, lat, lon, alt = struct.unpack_from(LOCATION_FMT, data)
    return node_id, lat, lon, alt


# routing protocols

class RoutingProtocol(ABC):
    """Strategy interface for bundle forwarding decisions.

    Implement should_forward() at minimum. The other hooks let protocols maintain
    per-bundle state (e.g. copy counts for spray-and-wait) without touching the
    forwarding loop in RoverDaemon.
    """

    @abstractmethod
    def should_forward(self, bundle: dict, active_peer_ids: List[int],
                       node_id: int, peers: Dict[int, dict]) -> bool:
        """Return True if this bundle should be forwarded this cycle."""

    def on_bundle_added(self, bundle: dict, is_local: bool) -> None:
        """Called when a bundle enters the store. is_local=True if we originated it."""

    def on_bundle_forwarded(self, bundle: dict) -> None:
        """Called after a bundle is successfully transmitted."""

    def on_bundle_removed(self, bundle: dict) -> None:
        """Called when a bundle leaves the store (ACKed, TTL expired, hop limit)."""


class EpidemicRouting(RoutingProtocol):
    """Classic epidemic: forward every bundle to every reachable peer."""

    def should_forward(self, bundle: dict, active_peer_ids: List[int],
                       node_id: int, peers: Dict[int, dict]) -> bool:
        return True


class SprayAndWaitRouting(RoutingProtocol):
    """Binary spray-and-wait: source starts with spray_count copies; each transfer
    halves the sender's count. Nodes with 1 copy enter wait phase and only deliver
    directly to the base station.
    """

    def __init__(self, spray_count: int = 8):
        self._spray_count = spray_count
        self._copies: Dict[tuple, int] = {}   # bundle_key -> copies remaining

    def _key(self, b: dict) -> tuple:
        return (b['source_node'], b['creation_time'], b['sequence_number'])

    def on_bundle_added(self, bundle: dict, is_local: bool) -> None:
        k = self._key(bundle)
        # Source gets spray_count copies; relayed bundles arrive with 1 (wait phase)
        self._copies.setdefault(k, self._spray_count if is_local else 1)

    def should_forward(self, bundle: dict, active_peer_ids: List[int],
                       node_id: int, peers: Dict[int, dict]) -> bool:
        copies = self._copies.get(self._key(bundle), 1)
        if copies <= 1:
            return BASE_STATION_NODE_ID in active_peer_ids
        return True

    def on_bundle_forwarded(self, bundle: dict) -> None:
        k = self._key(bundle)
        c = self._copies.get(k, 1)
        if c > 1:
            self._copies[k] = c // 2   # keep half, give half away

    def on_bundle_removed(self, bundle: dict) -> None:
        self._copies.pop(self._key(bundle), None)


# ROS stack control
class RosStack:
    """Starts/stops the MuSHR ROS stack when the BS asks (control msg field
    "ros": {"<node>": ["on"|"off", cmd_id]}).

    A stack counts as running whoever started it (controller OPTIONS, a manual roslaunch, or
    us), so the radio never launches a second copy. Each cmd_id is acted on once; the last one
    is saved to disk so the BS re-broadcasting an old command after a reboot does nothing.
    """

    def __init__(self, enabled: bool, state_path: str, log_dir: str):
        self.enabled = enabled and os.path.isdir('/proc')
        if enabled and not self.enabled:
            print('[ROS] --ros-control needs Linux (/proc); ROS commands will be ignored')
        self._state_path = state_path
        self._log_dir = log_dir
        self._proc: Optional[subprocess.Popen] = None   # the stack we launched, if any
        self._proc_log = None
        self._stopping: Dict[int, float] = {}           # pid -> monotonic time of SIGINT
        self._stacks: List[int] = []
        self._last_cmd = self._load_last_cmd()

    def _load_last_cmd(self) -> int:
        try:
            with open(self._state_path) as f:
                return int(json.load(f)['cmd_id'])
        except (OSError, ValueError, KeyError, TypeError):
            return 0

    def _save_last_cmd(self, cmd_id: int) -> None:
        self._last_cmd = cmd_id
        try:
            os.makedirs(os.path.dirname(self._state_path) or '.', exist_ok=True)
            tmp = self._state_path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'cmd_id': cmd_id, 'saved': time.time()}, f)
            os.replace(tmp, self._state_path)
        except OSError as e:
            print(f"[ROS] could not save {self._state_path}: {e}")

    @staticmethod
    def find_stacks() -> List[int]:
        """PIDs of every `roslaunch mushr_base ...` process (zombies have an empty cmdline)."""
        pids = []
        for d in os.listdir('/proc'):
            if not d.isdigit():
                continue
            try:
                with open(f'/proc/{d}/cmdline', 'rb') as f:
                    argv = f.read().split(b'\0')
            except OSError:
                continue
            if any(os.path.basename(a) == b'roslaunch' and b == b'mushr_base'
                   for a, b in zip(argv, argv[1:])):
                pids.append(int(d))
        return pids

    def on_command(self, want: str, cmd_id: int) -> Optional[str]:
        """Returns a description of what was done, or None for an already-handled cmd_id."""
        if cmd_id <= self._last_cmd:
            return None
        self._save_last_cmd(cmd_id)
        if not self.enabled:
            return 'ignored (--ros-control not enabled)'
        return self._start() if want == 'on' else self._stop()

    def _start(self) -> str:
        running = self.find_stacks()
        if running:
            return f"already running (pid {running[0]})"
        path = os.path.join(self._log_dir, f"ros_stack_{datetime.now():%Y%m%d_%H%M%S}.log")
        try:
            os.makedirs(self._log_dir, exist_ok=True)
            self._proc_log = open(path, 'ab')
            # own session: Ctrl-C / restarting the daemon must not take the stack down with it
            self._proc = subprocess.Popen(ROS_STACK_CMD, stdin=subprocess.DEVNULL,
                                          stdout=self._proc_log, stderr=subprocess.STDOUT,
                                          start_new_session=True)
        except OSError as e:
            self._close_proc_log()
            self._proc = None
            return f"start failed: {e}"
        return f"started pid {self._proc.pid}, output in {path}"

    def _stop(self) -> str:
        running = self.find_stacks()
        if not running:
            return 'not running'
        for pid in running:
            self._signal(pid, signal.SIGINT)    # same as SHARE: roslaunch shuts its nodes down
            self._stopping[pid] = time.monotonic()
        return f"SIGINT sent to pid {', '.join(map(str, running))}"

    @staticmethod
    def _signal(pid: int, sig: int) -> None:
        try:
            os.kill(pid, sig)
        except OSError:
            pass

    def _close_proc_log(self) -> None:
        if self._proc_log is not None:
            self._proc_log.close()
            self._proc_log = None

    def tick(self) -> List[str]:
        """Reap our child, escalate stuck stops, stop our stack if a second one appeared."""
        if not self.enabled:
            return []
        notes = []
        if self._proc is not None and self._proc.poll() is not None:
            notes.append(f"stack pid {self._proc.pid} exited (rc {self._proc.returncode})")
            self._proc = None
            self._close_proc_log()
        self._stacks = self.find_stacks()
        now = time.monotonic()
        for pid, t in list(self._stopping.items()):
            if pid not in self._stacks:
                del self._stopping[pid]
            elif now - t > ROS_STOP_GRACE_S:
                self._signal(pid, signal.SIGTERM)
                del self._stopping[pid]
                notes.append(f"pid {pid} ignored SIGINT for {ROS_STOP_GRACE_S:.0f}s, sent SIGTERM")
        ours = self._proc.pid if self._proc is not None else None
        if len(self._stacks) > 1 and ours in self._stacks and ours not in self._stopping:
            # someone pressed OPTIONS (or ran roslaunch) while our stack was up: keep theirs
            self._signal(ours, signal.SIGINT)
            self._stopping[ours] = now
            notes.append(f"duplicate stack detected, stopping ours (pid {ours})")
        return notes

    def status(self) -> str:
        if not self.enabled:
            return 'na'
        if any(pid in self._stopping for pid in self._stacks):
            return 'stopping'
        return 'on' if self._stacks else 'off'


# daemon
class RoverDaemon:
    def __init__(self, serial_port: str, baud: int, gen_interval: float,
                 node_id_override: Optional[int], protocol: RoutingProtocol,
                 lat: float = 0.0, lon: float = 0.0, alt: float = 0.0,
                 ros_location_topic: Optional[str] = None,
                 policy: Optional[CommsPolicy] = None,
                 event_log: Optional[EventLog] = None,
                 bw_order: str = 'oldest',
                 payload_bytes: int = 0,
                 retransmit_s: float = BUNDLE_RETRANSMIT_S,
                 status_interval_s: float = ROVER_STATUS_INTERVAL_S,
                 state_interval_s: float = STATE_LOG_INTERVAL_S,
                 loc_interval_s: float = LOCATION_BUNDLE_INTERVAL_S,
                 run_config: Optional[dict] = None,
                 ros_stack: Optional[RosStack] = None):
        assert BUNDLE_SIZE == 1052, f"Bundle struct size mismatch: got {BUNDLE_SIZE}, expected 1052"
        assert ACKMAP_SIZE == 42,   f"Ackmap struct size mismatch: got {ACKMAP_SIZE}, expected 42"

        self._port             = serial_port
        self._baud             = baud
        self._interval         = gen_interval
        self._node_id_override = node_id_override
        self._node_id          = node_id_override
        self._protocol         = protocol

        self._policy        = policy or CommsPolicy()
        self._log           = event_log or EventLog(None)
        self._log.node_id   = node_id_override
        self._bw_order      = bw_order
        self._payload_bytes = max(0, min(payload_bytes, 1024))
        self._retransmit_ms = int(retransmit_s * 1000)
        self._status_interval_s = status_interval_s
        self._state_interval_s  = state_interval_s
        self._loc_interval_s    = loc_interval_s
        self._run_config    = run_config or {}
        self._ros           = ros_stack or RosStack(False, os.devnull, '.')
        # forward loop ticks fast enough to pace the bandwidth budget smoothly
        self._forward_interval_s = (min(FORWARD_INTERVAL_S, 1.0 / self._policy.bw_limit)
                                    if self._policy.bw_limit > 0 else FORWARD_INTERVAL_S)
        self._radio_on: Optional[bool] = None
        self._counters: Dict[str, int] = defaultdict(int)
        self._ack_latencies_ms: List[int] = []     # own data bundles: generation -> BS ack
        self._last_active: Set[int] = set()
        self._last_throttle_log = 0.0
        self._wall_start = time.time()

        # fleet control / experiment state
        self._mode = 'standby'                     # 'standby' | 'active'
        self._exp_id = ''
        self._exp_start = time.time()
        self._ctrl_version = 0
        self._ctrl_msg: Optional[tuple] = None     # (origin, msg) last adopted control, re-broadcast
        self._pending_start: Optional[dict] = None # {'exp', 'start_at', 'cfg'}
        self._base_cfg = {k: self._run_config.get(k) for k in CONFIG_KEYS
                          if self._run_config.get(k) is not None}

        self._store: List[dict] = []        # each: dtn fields + added_at_ms, forwarded, forwarded_ms
        self._lock = threading.Lock()
        self._seq_next = 1
        self._start_ms = int(time.monotonic() * 1000)
        self._boot_id  = int.from_bytes(os.urandom(4), 'little')

        self._delivered: Set[tuple] = set()                  # (source, creation_time, seq) acked by BS
        self._peers: Dict[int, dict] = {}                    # node_id -> {last_seen, boot_id, rssi}
        self._ack: Dict[int, dict] = {}                      # source_node -> {seq_base, bitmap, dirty}

        # lat/lon/alt carry odometry x/y/z when a ROS topic is used; yaw is degrees
        # (counter-clockwise from the odometry x axis), None without a ROS topic
        self._own_location    = {'lat': lat, 'lon': lon, 'alt': alt, 'yaw': None}
        self._ros_location_topic = ros_location_topic
        self._locations: Dict[int, dict] = {}                # node_id -> {lat, lon, alt, updated_ms}

        self._detected_node_id: Optional[int] = None
        self._restart_pending = False

        self._ser: Optional[serial.Serial] = None
        self._uart_lock = threading.Lock()

    def _next_seq(self) -> int:
        s = self._seq_next
        self._seq_next += 1
        return s

    # serial helpers
    def _send(self, cmd: int, payload: bytes = b'') -> None:
        frame = encode_frame(cmd, payload)
        with self._uart_lock:
            self._ser.write(frame)

    def _air_tx(self, payload: bytes, kind: str, **fields) -> bool:
        """Broadcast one over-air frame through the ESP32. Every air TX goes through here
        so the periodic-comms gate and TX logging cannot be bypassed."""
        if not self._radio_on:
            self._counters['tx_blocked_off_' + kind] += 1
            return False
        self._send(HOST_CMD_WIFI_TX, payload)
        self._counters['tx_' + kind] += 1
        self._counters['tx_bytes'] += len(payload)
        self._log.log('tx', kind=kind, bytes=len(payload), **fields)
        return True

    def _store_counts(self) -> dict:
        own = ferried = tel = pending = 0
        with self._lock:
            for b in self._store:
                if b.get('is_telemetry'):
                    tel += 1
                elif b['source_node'] == self._node_id:
                    own += 1
                else:
                    ferried += 1
                if not b.get('forwarded'):
                    pending += 1
            total = len(self._store)
        return {'store': total, 'q_own_data': own, 'q_ferried_data': ferried,
                'q_telemetry': tel, 'q_unforwarded': pending}

    @staticmethod
    def _bundle_fields(b: dict) -> dict:
        return {'src': b['source_node'], 'seq': b['sequence_number'],
                'prev': b.get('prev_node'), 'hops': b['hop_count'],
                'tel': bool(b.get('is_telemetry')), 'plen': len(b.get('payload', b''))}

    # bundle store
    def _add_bundle(self, b: dict, is_local: bool = False) -> bool:
        key = (b['source_node'], b['creation_time'], b['sequence_number'])
        b.setdefault('forwarded', False)
        b.setdefault('forwarded_ms', 0)
        with self._lock:
            if key in self._delivered:
                return False
            for stored in self._store:
                if (stored['source_node'] == b['source_node'] and
                        stored['creation_time'] == b['creation_time'] and
                        stored['sequence_number'] == b['sequence_number']):
                    return False
            self._store.append(b)
        self._protocol.on_bundle_added(b, is_local)
        return True

    def _expire_bundles(self) -> None:
        now = int(time.monotonic() * 1000)
        with self._lock:
            keep, drop = [], []
            for b in self._store:
                if (now - b.get('added_at_ms', now)) < b.get('lifetime', BUNDLE_DEFAULT_LIFETIME):
                    keep.append(b)
                else:
                    drop.append(b)
            self._store = keep
        for b in drop:
            self._protocol.on_bundle_removed(b)
            self._counters['expired_tel' if b.get('is_telemetry') else 'expired_data'] += 1
            self._log.log('bundle_expired', age_ms=now - b.get('added_at_ms', now),
                          tx_count=b.get('tx_count', 0), **self._bundle_fields(b))
        if drop:
            print(f"[STORE] Expired {len(drop)} bundles by TTL")

    #ACK window (sliding 256-bit bitmap per source), mirrors mark_ack_locked 
    def _mark_ack(self, source_node: int, seq: int) -> None:
        a = self._ack.get(source_node)
        if a is None:
            a = {'seq_base': seq, 'bitmap': bytearray(ACKMAP_BITMAP_BYTES), 'dirty': False}
            self._ack[source_node] = a
        if seq < a['seq_base']:
            return
        offset = seq - a['seq_base']
        if offset >= ACKMAP_BITMAP_BITS:
            advance = offset - (ACKMAP_BITMAP_BITS - 1)
            as_int = int.from_bytes(a['bitmap'], 'little') >> advance
            a['bitmap'] = bytearray(as_int.to_bytes(ACKMAP_BITMAP_BYTES, 'little'))
            a['seq_base'] += advance
            offset = seq - a['seq_base']
        a['bitmap'][offset // 8] |= (1 << (offset % 8))
        a['dirty'] = True

    def _merge_ackmap(self, source_node: int, seq_base: int, bitmap: bytes, blen: int) -> None:
        for i in range(blen * 8):
            if bitmap[i // 8] & (1 << (i % 8)):
                self._mark_ack(source_node, seq_base + i)

    def _apply_ackmap_to_store(self, source_node: int, seq_base: int, bitmap: bytes, blen: int) -> None:
        dropped = []
        with self._lock:
            keep = []
            for b in self._store:
                if b['source_node'] != source_node:
                    keep.append(b)
                    continue
                seq = b['sequence_number']
                delivered = False
                if seq < seq_base:
                    delivered = True                       # window slid past -> delivered
                else:
                    off = seq - seq_base
                    if off < blen * 8 and (bitmap[off // 8] & (1 << (off % 8))):
                        delivered = True
                if delivered:
                    self._delivered.add((source_node, b['creation_time'], seq))
                    dropped.append(b)
                else:
                    keep.append(b)
            self._store = keep
        now = int(time.monotonic() * 1000)
        for b in dropped:
            self._protocol.on_bundle_removed(b)
            own = b['source_node'] == self._node_id
            age_ms = now - b.get('added_at_ms', now)
            first_tx = b.get('first_tx_ms')
            if own and not b.get('is_telemetry'):
                self._counters['acked_own_data'] += 1
                self._ack_latencies_ms.append(age_ms)
            elif b.get('is_telemetry'):
                self._counters['acked_tel'] += 1
            else:
                self._counters['acked_ferried_data'] += 1
            self._log.log('bundle_acked', own=own, age_ms=age_ms,
                          queue_ms=(first_tx - b['added_at_ms']) if first_tx and 'added_at_ms' in b else None,
                          tx_count=b.get('tx_count', 0), **self._bundle_fields(b))
        if dropped:
            print(f"[ANTIPKT] ACKMAP freed {len(dropped)} for src:{source_node} (store now {len(self._store)})")

    # -peer table
    def _active_peer_ids(self) -> List[int]:
        now = int(time.monotonic() * 1000)
        return [pid for pid, p in self._peers.items()
                if (now - p['last_seen']) < PEER_TIMEOUT_S * 1000]

    def _on_beacon(self, node_id: int, timestamp_ms: int, boot_id: int, rssi: int) -> None:
        if node_id == self._node_id:
            return
        now = int(time.monotonic() * 1000)
        prev = self._peers.get(node_id)
        was_absent = prev is None or (now - prev['last_seen']) >= PEER_TIMEOUT_S * 1000
        restarted = (prev is not None and boot_id != 0 and
                     prev.get('boot_id', 0) != 0 and prev['boot_id'] != boot_id)
        daemon_rel_now = now - self._start_ms
        clock_offset_ms = timestamp_ms - daemon_rel_now   # peer_ts - daemon_ts; const between reboots
        self._peers[node_id] = {'last_seen': now, 'boot_id': boot_id, 'rssi': rssi,
                                'clock_offset_ms': clock_offset_ms}
        self._counters['rx_beacon'] += 1
        self._log.log('rx_beacon', peer=node_id, rssi=rssi, peer_boot_id=boot_id,
                      peer_ts_ms=timestamp_ms, clock_offset_ms=clock_offset_ms)

        if not (was_absent or restarted):
            return
        self._last_active.add(node_id)
        self._log.log('peer_up', peer=node_id, rssi=rssi,
                      reason='restarted' if restarted else ('returned' if prev else 'discovered'),
                      absent_ms=(now - prev['last_seen']) if prev else None)

        # A new/returned/restarted peer should receive everything we hold from other sources.
        bs_active = BASE_STATION_NODE_ID in self._active_peer_ids()
        restart_dropped = []
        with self._lock:
            keep = []
            for b in self._store:
                if b['source_node'] == node_id:
                    if restarted and bs_active and b.get('forwarded'):
                        # the source rebooted and the BS is in range -> its old bundles were
                        # already delivered; drop them rather than re-flooding (avoids dups).
                        self._delivered.add((b['source_node'], b['creation_time'], b['sequence_number']))
                        restart_dropped.append(b)
                        continue
                else:
                    b['forwarded'] = False
                keep.append(b)
            self._store = keep
        for b in restart_dropped:
            self._protocol.on_bundle_removed(b)
            self._log.log('bundle_drop_peer_restart', **self._bundle_fields(b))
        if restarted:
            self._ack.pop(node_id, None)   # clear stale ack window for the rebooted source
            print(f"[PEER] node {node_id} restarted (boot_id changed); reset forwarded flags"
                  + (f", dropped {len(restart_dropped)} delivered" if restart_dropped else ""))
        else:
            print(f"[PEER] node {node_id} {'returned' if prev else 'discovered'}; reset forwarded flags")

    # bundle rx
    def _on_bundle(self, rest: bytes, rssi: Optional[int] = None) -> None:
        b = unpack_bundle(rest[1:])
        if not b:
            return
        b['added_at_ms'] = int(time.monotonic() * 1000)
        new = self._add_bundle(b)
        self._counters['rx_bundle_new' if new else 'rx_bundle_dup'] += 1
        self._log.log('rx_bundle', new=new, rssi=rssi, **self._bundle_fields(b))
        if not new:
            return   # duplicate or already delivered
        print(f"[RX] bundle src:{b['source_node']} seq:{b['sequence_number']} "
              f"hops:{b['hop_count']} (store {len(self._store)})")

    def _make_telemetry_ack(self, orig: dict) -> None:
        if self._node_id is None:
            return
        now = int(time.monotonic() * 1000)
        # For ferried bundles (from another node), estimate the true latency so the BS
        # can display a meaningful value.  The BS uses src_clock_offset=0 for nodes it has
        # never heard directly, so its @METRIC line shows 0ms (clamped from negative).
        # We know the source's clock offset from its beacons, so we compute:
        #   est_latency = daemon_rel_now - (creation_time - src_clock_offset_ms)
        # which equals the wall-clock age of the bundle at the time we received it.
        latency_suffix = ''
        if orig['source_node'] != self._node_id:
            src_peer = self._peers.get(orig['source_node'])
            if src_peer and 'clock_offset_ms' in src_peer:
                daemon_rel_now = now - self._start_ms
                est_latency = daemon_rel_now - (orig['creation_time'] - src_peer['clock_offset_ms'])
                if est_latency >= 0:
                    latency_suffix = f':{est_latency}'
        text = (f"@DTN_RX:{orig['source_node']}:{orig['prev_node']}:"
                f"{orig['sequence_number']}:{self._node_id}:{orig['hop_count']}{latency_suffix}")
        ack = {
            'creation_time':   now - self._start_ms,
            'sequence_number': self._next_seq(),
            'lifetime':        TELEMETRY_ACK_LIFETIME,
            'source_node':     self._node_id,
            'dest_node':       0,
            'report_to_node':  0,
            'prev_node':       self._node_id,
            'is_telemetry':    True,
            'hop_limit':       5,
            'hop_count':       0,
            'payload':         text.encode(),
            'added_at_ms':     now,
        }
        self._add_bundle(ack, is_local=True)
        self._log.log('bundle_gen', text=text, **self._bundle_fields(ack))

    # ackmap rx
    def _on_ackmap(self, rest: bytes) -> None:
        fields = unpack_ackmap(rest)
        if fields is None:
            return
        _ptype, origin, source, seq_base, blen, bitmap = fields
        self._counters['rx_ackmap'] += 1
        self._log.log('rx_ackmap', origin=origin, ack_src=source, seq_base=seq_base, blen=blen,
                      n_acked=sum(bin(x).count('1') for x in bitmap[:blen]))
        if origin != BASE_STATION_NODE_ID:
            return
        if blen == 0 or blen > ACKMAP_BITMAP_BYTES:
            return
        # A stale pre-restart ackmap for our own node has seq_base higher than any
        # sequence we've issued this boot.  Applying it would evict new seq=1,2,...
        # bundles because seq < seq_base is treated as "window slid past → delivered".
        # Also skip merge so we don't propagate the stale window to other peers.
        if source == self._node_id and seq_base >= self._seq_next:
            return
        self._merge_ackmap(source, seq_base, bitmap, blen)        # for propagation
        self._apply_ackmap_to_store(source, seq_base, bitmap, blen)

    # forward and ack map propagation loop, runs every self._forward_interval_s seconds
    def _forward(self) -> None:
        self._expire_bundles()
        if not self._radio_on:
            return
        now = int(time.monotonic() * 1000)
        active = self._active_peer_ids()
        peers_active = len(active) > 0
        bs_active = BASE_STATION_NODE_ID in active

        with self._lock:
            snapshot = list(self._store)
        # transmission order under a bandwidth budget
        snapshot.sort(key=lambda x: x.get('added_at_ms', 0), reverse=(self._bw_order == 'newest'))

        to_remove = []
        throttled = 0
        for b in snapshot:
            if b['hop_count'] >= b['hop_limit']:
                to_remove.append(b)
                continue
            if (b.get('forwarded') and bs_active and
                    (now - b.get('forwarded_ms', 0)) >= self._retransmit_ms):
                b['forwarded'] = False   # unacked too long; allow re-send
                self._log.log('bundle_rearm', reason='unacked', **self._bundle_fields(b))
            if b.get('forwarded') or not peers_active:
                continue
            if not self._protocol.should_forward(b, active, self._node_id, self._peers):
                continue
            if self._mode != 'active' and not b.get('is_telemetry'):
                continue                          # standby: hold data, only telemetry flows
            if self._policy.counts(b) and not self._policy.try_take():
                throttled += 1
                continue
            out = dict(b)
            out['prev_node'] = self._node_id
            out['hop_count'] = b['hop_count'] + 1
            # Translate creation_time into this daemon's clock domain so the BS can
            # compute correct latency for bundles from absent sources.
            # Guards: bundle must have arrived directly from the source (prev==src, so
            # no prior relay has already done this), the source must be absent (if still
            # active the BS has its clock_offset directly), and the result must be >= 0
            # (source started earlier than us → large offset → don't wrap the uint32).
            if (bs_active
                    and b['source_node'] != self._node_id
                    and b['prev_node'] == b['source_node']
                    and b['source_node'] not in active):
                src_peer = self._peers.get(b['source_node'])
                if src_peer and 'clock_offset_ms' in src_peer:
                    adjusted = b['creation_time'] - src_peer['clock_offset_ms']
                    if adjusted >= 0:
                        out['creation_time'] = adjusted
            tx_count = b.get('tx_count', 0) + 1
            kind = 'bundle_tel' if b.get('is_telemetry') else 'bundle_data'
            if not self._air_tx(encode_air_bundle(out), kind, tx_count=tx_count,
                                age_ms=now - b.get('added_at_ms', now),
                                bs_active=bs_active, peers=active,
                                **self._bundle_fields(b)):
                continue
            b['tx_count'] = tx_count
            if tx_count == 1:
                b['first_tx_ms'] = now
            else:
                self._counters['retx_' + kind] += 1
            # Generate @DTN_RX telemetry ack for non-local ferried bundles on first
            # forward to BS.  Deferred from reception time so that clock_offset_ms for
            # the source is already populated (avoids the burst-before-first-beacon race).
            if (bs_active and not b.get('is_telemetry') and
                    b['source_node'] != self._node_id and not b.get('ack_generated')):
                b['ack_generated'] = True
                self._make_telemetry_ack(b)
            b['forwarded'] = True
            b['forwarded_ms'] = now
            self._protocol.on_bundle_forwarded(b)

        if throttled:
            self._counters['bw_denials'] += throttled
            mono = time.monotonic()
            if mono - self._last_throttle_log >= BW_THROTTLE_LOG_S:
                self._last_throttle_log = mono
                self._log.log('bw_throttled', waiting=throttled, tokens=self._policy.tokens())

        if to_remove:
            for b in to_remove:
                self._protocol.on_bundle_removed(b)
                self._log.log('bundle_drop_hoplimit', **self._bundle_fields(b))
            ids = {id(b) for b in to_remove}
            with self._lock:
                self._store = [b for b in self._store if id(b) not in ids]

        # propagate any updated ack windows so disconnected/ferried peers learn deliveries
        if peers_active:
            for source, a in self._ack.items():
                if a['dirty']:
                    if self._air_tx(pack_ackmap(source, a['seq_base'], a['bitmap']), 'ackmap',
                                    ack_src=source, seq_base=a['seq_base']):
                        a['dirty'] = False

    # node restart notification
    def _send_node_restart_notification(self) -> None:
        if self._node_id is None:
            return
        now = int(time.monotonic() * 1000)
        payload = f"@NODE_RESTART:{self._node_id}".encode()
        b = {
            'creation_time':   now - self._start_ms,
            'sequence_number': self._next_seq(),
            'lifetime':        BUNDLE_DEFAULT_LIFETIME,
            'source_node':     self._node_id,
            'dest_node':       BASE_STATION_NODE_ID,
            'prev_node':       self._node_id,
            'is_telemetry':    True,
            'hop_limit':       BUNDLE_DEFAULT_HOP_LIMIT,
            'hop_count':       0,
            'payload':         payload,
            'added_at_ms':     now,
        }
        self._add_bundle(b, is_local=True)
        self._log.log('bundle_gen', **self._bundle_fields(b))
        print(f"[RESTART] queued @NODE_RESTART:{self._node_id} notification bundle")

    # beacon
    def _send_beacon(self) -> None:
        ts = int(time.monotonic() * 1000) - self._start_ms
        self._air_tx(pack_beacon(self._node_id, ts, self._boot_id), 'beacon', ts_ms=ts)

    # location
    def _send_location(self) -> None:
        if self._node_id is None:
            return
        loc = self._own_location
        self._air_tx(pack_location(self._node_id, loc['lat'], loc['lon'], loc['alt']),
                     'location', **loc)

    def _on_location(self, data: bytes) -> None:
        result = unpack_location(data)
        if result is None:
            return
        node_id, lat, lon, alt = result
        if node_id == self._node_id:
            return
        self._locations[node_id] = {
            'lat': lat, 'lon': lon, 'alt': alt,
            'updated_ms': int(time.monotonic() * 1000),
        }
        self._log.log('rx_location', peer=node_id, lat=lat, lon=lon, alt=alt)
        print(f"[LOC] node {node_id}: lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m")

    def _queue_telemetry(self, text: str, lifetime: int = TELEMETRY_ACK_LIFETIME) -> None:
        now = int(time.monotonic() * 1000)
        b = {
            'creation_time':   now - self._start_ms,
            'sequence_number': self._next_seq(),
            'lifetime':        lifetime,
            'source_node':     self._node_id,
            'dest_node':       BASE_STATION_NODE_ID,
            'prev_node':       self._node_id,
            'is_telemetry':    True,
            'hop_limit':       BUNDLE_DEFAULT_HOP_LIMIT,
            'hop_count':       0,
            'payload':         text.encode(),
            'added_at_ms':     now,
        }
        self._add_bundle(b, is_local=True)
        self._log.log('bundle_gen', text=text, **self._bundle_fields(b))

    def _send_status_bundle(self) -> None:
        """@ROVER_STATUS telemetry so the BS/visualizer can see queue depth and comms config.
        key=value fields after the node id so the format can grow without breaking parsers."""
        if self._node_id is None or not self._radio_on:
            return
        c = self._store_counts()
        _on, left, cycle = self._window()
        fields = {
            'q':    c['q_own_data'], 'qf': c['q_ferried_data'], 'qt': c['q_telemetry'],
            'gen':  self._counters['gen_data'],
            'tx':   self._counters['tx_bundle_data'],
            'retx': self._counters['retx_bundle_data'],
            'ack':  self._counters['acked_own_data'],
            'bw':   self._policy.bw_limit, 'gi': self._interval,
            'on':   self._policy.on_s, 'off': self._policy.off_s,
            'win':  round(left, 1) if left is not None else -1, 'cyc': cycle,
            'up':   int(time.time() - self._wall_start),
            'st':   'pending' if self._pending_start else self._mode,
            'cv':   self._ctrl_version,
            'ck':   round(FLEET_CLOCK.offset, 2) if FLEET_CLOCK.synced else 'na',
            'ros':  self._ros.status(),
            'exp':  (self._pending_start or {}).get('exp') or
                    (self._exp_id if self._mode == 'active' else '') or '-',
        }
        self._queue_telemetry(f"@ROVER_STATUS:{self._node_id}:"
                              + ','.join(f"{k}={v}" for k, v in fields.items()))

    def _send_location_bundle(self) -> None:
        """Send own location as a telemetry bundle so the BS serial output captures it."""
        if self._node_id is None or not self._radio_on:
            return
        loc = self._own_location
        bs_rssi = self._peers.get(BASE_STATION_NODE_ID, {}).get('rssi')
        # @LOCATION:<node>:<x>:<y>:<z>[:<rssi>[:<yaw_deg>]]  (rssi left empty when unknown)
        tail = f":{bs_rssi if bs_rssi is not None else ''}"
        if loc.get('yaw') is not None:
            tail += f":{loc['yaw']:.1f}"
        elif bs_rssi is None:
            tail = ""
        self._queue_telemetry(f"@LOCATION:{self._node_id}:"
                              f"{loc['lat']:.6f}:{loc['lon']:.6f}:{loc['alt']:.1f}{tail}")

    # bundle generation
    def _generate_bundle(self) -> None:
        abs_ms = int(time.monotonic() * 1000)
        creation_time = abs_ms - self._start_ms
        doc = {
            'ts': int(time.time()),
            'node': self._node_id,
            'sensor': round(20.0 + (creation_time % 1000) / 100.0, 1),
        }
        payload = json.dumps(doc).encode()
        if len(payload) < self._payload_bytes:
            # pad to the requested size so bandwidth tests can use realistic payloads
            doc['pad'] = ''
            doc['pad'] = 'x' * max(0, self._payload_bytes - len(json.dumps(doc).encode()))
            payload = json.dumps(doc).encode()
        b = {
            'creation_time':   creation_time,
            'sequence_number': self._next_seq(),
            'lifetime':        BUNDLE_DEFAULT_LIFETIME,
            'source_node':     self._node_id,
            'dest_node':       BASE_STATION_NODE_ID,
            'prev_node':       self._node_id,
            'hop_limit':       BUNDLE_DEFAULT_HOP_LIMIT,
            'hop_count':       0,
            'is_telemetry':    False,
            'payload':         payload,
            'added_at_ms':     abs_ms,
        }
        if self._add_bundle(b, is_local=True):
            self._counters['gen_data'] += 1
            self._log.log('bundle_gen', radio_on=self._radio_on, **self._bundle_fields(b))
            print(f"[GEN] bundle seq:{b['sequence_number']} (store {len(self._store)})")

    # boot-banner detector
    def _handle_ascii(self, text: str) -> None:
        self._log.log('esp_ascii', text=text.strip()[:500])
        if 'I am' in text or 'rst:0x' in text:
            print(f"[RESTART] ESP32 boot banner: {text.strip()!r}")
            self._restart_pending = True

    def _dispatch(self, cmd: int, payload: bytes) -> None:
        if cmd == HOST_CMD_WIFI_RX:
            if len(payload) < 8:   # 6 mac + 1 rssi + >=1 air byte
                return
            rssi = struct.unpack_from('<b', payload, 6)[0]
            rest = payload[7:]
            ptype = rest[0]
            if not self._radio_on:
                # periodic mode: radio is "off", so nothing is heard
                self._counters['rx_dropped_off'] += 1
                self._log.log('rx_dropped_off', ptype=ptype, rssi=rssi, bytes=len(rest))
                return
            if ptype == PKT_TYPE_BEACON:
                bc = unpack_beacon(rest)
                if bc:
                    self._on_beacon(bc[0], bc[1], bc[2], rssi)
            elif ptype == PKT_TYPE_BUNDLE:
                self._on_bundle(rest, rssi)
            elif ptype == PKT_TYPE_ACKMAP:
                self._on_ackmap(rest)
            elif ptype == PKT_TYPE_LOCATION:
                self._on_location(rest)
            elif ptype == PKT_TYPE_CONTROL:
                self._on_control(rest, rssi)
            else:
                self._counters['rx_unknown'] += 1
                self._log.log('rx_unknown', ptype=ptype, rssi=rssi, bytes=len(rest))
        elif cmd == HOST_CMD_STATUS_RESP:
            if len(payload) >= STATUS_SIZE:
                self._detected_node_id = struct.unpack_from('<H', payload)[0]
        # other host commands are unused on the relay link

    # serial lifecycle
    def _open_serial(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

        ser = serial.Serial()
        ser.port     = self._port
        ser.baudrate = self._baud
        ser.timeout       = 0.05
        ser.write_timeout = WRITE_TIMEOUT_S
        ser.xonxoff       = False
        ser.dsrdtr        = False
        ser.rtscts        = False
        ser.dtr           = False
        ser.rts           = False
        ser.open()

        if platform.system() != 'Windows':
            import termios
            attrs = termios.tcgetattr(ser.fd)
            attrs[2] &= ~termios.HUPCL          # don't drop DTR/RTS on close (CH341 reset bug)
            termios.tcsetattr(ser.fd, termios.TCSANOW, attrs)
        ser.dtr = False
        ser.rts = False
        ser.reset_input_buffer()

        self._ser = ser
        time.sleep(SERIAL_SETTLE_S)

    def _detect_node_id(self) -> int:
        for attempt in range(5):
            print(f"[INIT] Attempt {attempt+1}/5: QUERY_STATUS")
            try:
                self._send(HOST_CMD_QUERY_STATUS)
            except serial.SerialException as e:
                print(f"[INIT] Write failed ({e}), settling...")
                time.sleep(2.0)
                self._ser.reset_input_buffer()
                continue
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                result = recv_frame(self._ser, timeout_s=max(0.05, remaining),
                                    on_ascii=self._handle_ascii)
                if result is None:
                    break
                cmd, payload = result
                if cmd == HOST_CMD_STATUS_RESP and len(payload) >= STATUS_SIZE:
                    node_id = struct.unpack_from('<H', payload)[0]
                    print(f"[INIT] Auto-detected node_id={node_id}")
                    return node_id
                self._dispatch(cmd, payload)
            time.sleep(1)
        raise RuntimeError("Could not auto-detect node_id from ESP32. Use --node-id to override.")

    # fleet control / experiments
    def start_on_connect(self, exp_id: str) -> None:
        """--mode active: run an experiment with the CLI settings as soon as we're connected."""
        self._pending_start = {'exp': exp_id, 'start_at': 0, 'cfg': {}}

    def _window(self) -> Tuple[bool, Optional[float], int]:
        """Comms restrictions only apply while an experiment is running; in standby the
        radio stays on so the rover can always hear control from the BS."""
        if self._mode != 'active':
            return True, None, 0
        return self._policy.window()

    def _apply_config(self, cfg: dict) -> dict:
        """Apply runtime settings (CONFIG_KEYS). Returns the values actually applied."""
        applied = {}
        for k in CONFIG_KEYS:
            if k not in cfg or cfg[k] is None:
                continue
            v = cfg[k]
            try:
                if k == 'bw_order':
                    v = 'newest' if str(v) == 'newest' else 'oldest'
                elif k == 'bw_count_telemetry':
                    v = bool(v)
                elif k == 'payload_bytes':
                    v = max(0, min(int(v), 1024))
                else:
                    v = max(0.0, float(v)) if k != 'comms_phase' else float(v)
            except (TypeError, ValueError):
                self._log.log('config_bad_value', key=k, value=cfg[k])
                continue
            applied[k] = v
        p = self._policy
        self._interval      = applied.get('interval', self._interval)
        self._bw_order      = applied.get('bw_order', self._bw_order)
        self._payload_bytes = applied.get('payload_bytes', self._payload_bytes)
        if 'retransmit_s' in applied:
            self._retransmit_ms = int(applied['retransmit_s'] * 1000)
        if 'loc_interval' in applied and applied['loc_interval'] > 0:
            self._loc_interval_s = max(0.2, applied['loc_interval'])
        self._policy = CommsPolicy(
            bw_limit=applied.get('bw_limit', p.bw_limit),
            bw_burst=applied.get('bw_burst', p.bw_burst),
            count_telemetry=applied.get('bw_count_telemetry', p.count_telemetry),
            on_s=applied.get('comms_on', p.on_s),
            off_s=applied.get('comms_off', p.off_s),
            phase_s=applied.get('comms_phase', p.phase_s))
        self._forward_interval_s = (min(FORWARD_INTERVAL_S, 1.0 / self._policy.bw_limit)
                                    if self._policy.bw_limit > 0 else FORWARD_INTERVAL_S)
        self._log.log('config_applied', applied=applied, policy=self._policy.describe(),
                      interval=self._interval, bw_order=self._bw_order,
                      payload_bytes=self._payload_bytes, retransmit_ms=self._retransmit_ms,
                      loc_interval=self._loc_interval_s)
        return applied

    def _current_config(self) -> dict:
        d = self._policy.describe()
        return {'interval': self._interval, 'bw_limit': d['bw_limit'], 'bw_burst': d['bw_burst'],
                'bw_order': self._bw_order, 'bw_count_telemetry': d['bw_count_telemetry'],
                'comms_on': d['comms_on_s'], 'comms_off': d['comms_off_s'],
                'comms_phase': d['comms_phase_s'], 'payload_bytes': self._payload_bytes,
                'retransmit_s': self._retransmit_ms / 1000.0, 'loc_interval': self._loc_interval_s}

    def _end_experiment(self, reason: str) -> None:
        if self._mode != 'active':
            return
        print(f"[EXP] experiment '{self._exp_id}' ended ({reason})")
        self._log.log('experiment_stop', exp=self._exp_id, reason=reason)
        self.write_summary()
        self._log.end_experiment()
        self._mode = 'standby'

    def _start_experiment(self, exp_id: str, cfg: dict) -> None:
        self._end_experiment('superseded')
        prev_counters = dict(self._counters)
        # fresh slate: drop queued data so the new run only measures its own bundles.
        # sequence numbers keep counting (the BS ack window per source must stay valid).
        with self._lock:
            dropped = [b for b in self._store if not b.get('is_telemetry')]
            self._store = [b for b in self._store if b.get('is_telemetry')]
        for b in dropped:
            self._protocol.on_bundle_removed(b)
        self._apply_config(dict(self._base_cfg, **cfg))
        self._counters = defaultdict(int)
        self._ack_latencies_ms = []
        self._exp_id = exp_id
        self._exp_start = time.time()
        self._mode = 'active'
        self._radio_on = None                    # re-evaluate the window from scratch
        exp_log = self._log.start_experiment(exp_id)
        self._log.log('experiment_start', exp=exp_id, config=self._current_config(),
                      dropped_data=len(dropped), prev_counters=prev_counters,
                      ctrl_version=self._ctrl_version, exp_log=exp_log)
        print(f"[EXP] experiment '{exp_id}' started: {self._current_config()}"
              + (f" (log {exp_log})" if exp_log else ""))
        self._update_comms_window()

    def _on_control(self, rest: bytes, rssi: int) -> None:
        parsed = unpack_control(rest)
        if parsed is None:
            self._log.log('rx_control_bad', bytes=len(rest))
            return
        origin, version, msg = parsed
        self._counters['rx_control'] += 1
        self._sync_clock(msg)                     # every copy, also repeats of a known version
        msg.pop('t', None)
        msg.pop('hop', None)
        if version <= self._ctrl_version:
            return                                # already have this one (or newer)
        self._log.log('rx_control', origin=origin, version=version, rssi=rssi, msg=msg)
        self._ctrl_version = version
        self._ctrl_msg = (origin, msg)
        self._relay_control()                     # relay once immediately (multi-hop)

        ros = (msg.get('ros') or {}).get(str(self._node_id))
        if isinstance(ros, list) and len(ros) == 2 and ros[0] in ('on', 'off'):
            self._on_ros_command(ros[0], int(ros[1]), version)
        if 'state' not in msg:
            return                                # ROS-only control: fleet state unchanged

        cfg = dict(msg.get('cfg') or {})
        cfg.update((msg.get('over') or {}).get(str(self._node_id), {}))   # per-rover overrides
        state = msg.get('state', 'standby')
        exp = ''.join(ch for ch in str(msg.get('exp', '')) if ch.isalnum() or ch in '-_')[:40]
        print(f"[CTRL] v{version}: state={state} exp={exp or '-'} cfg={cfg}")

        if state == 'standby':
            self._pending_start = None
            self._end_experiment('standby command')
            self._apply_config(dict(self._base_cfg, **cfg))
            self._log.log('standby', ctrl_version=version)
        elif state == 'active':
            if self._mode == 'active' and exp == self._exp_id:
                self._apply_config(cfg)           # live change within the running experiment
                self._log.log('config_live_change', exp=exp, ctrl_version=version)
            else:
                start_at = float(msg.get('start_at') or 0)
                self._pending_start = {'exp': exp or f"exp{version}", 'start_at': start_at, 'cfg': cfg}
                wait = start_at - FLEET_CLOCK.now()
                self._log.log('experiment_scheduled', exp=exp, start_at=start_at,
                              wait_s=round(wait, 2), late_join=wait < 0,
                              clock_synced=FLEET_CLOCK.synced)
                if wait > 0:
                    print(f"[EXP] '{exp}' starts in {wait:.1f}s")

    def _on_ros_command(self, want: str, cmd_id: int, version: int) -> None:
        result = self._ros.on_command(want, cmd_id)
        if result is None:
            return                                # already acted on this cmd_id
        print(f"[ROS] v{version}: stack {want} (cmd {cmd_id}): {result}")
        self._log.log('ros_command', want=want, cmd_id=cmd_id, ctrl_version=version, result=result)

    def _sync_clock(self, msg: dict) -> None:
        try:
            t, hop = float(msg['t']), int(msg.get('hop', 0))
        except (KeyError, TypeError, ValueError):
            return                                # BS software without time stamps
        step = FLEET_CLOCK.update(t, hop)
        if step is not None:
            print(f"[CLOCK] using BS time: local clock is {FLEET_CLOCK.offset:+.2f}s off "
                  f"(step {step:+.2f}s, {hop} hop(s))")
            self._log.log('clock_sync', offset_s=round(FLEET_CLOCK.offset, 3),
                          step_s=round(step, 3), hop=hop)

    def _relay_control(self) -> None:
        """Re-broadcast the adopted control message, stamped with our BS-time estimate."""
        if self._ctrl_msg is None:
            return
        origin, msg = self._ctrl_msg
        if FLEET_CLOCK.synced:
            msg = dict(msg, t=round(FLEET_CLOCK.now(), 3), hop=FLEET_CLOCK.hop + 1)
        self._air_tx(pack_control(self._ctrl_version, msg, origin), 'control',
                     version=self._ctrl_version)

    def _check_pending_start(self) -> None:
        p = self._pending_start
        if p is not None and FLEET_CLOCK.now() >= p['start_at']:
            self._pending_start = None
            self._start_experiment(p['exp'], p['cfg'])

    # comms window / state logging
    def _update_comms_window(self) -> bool:
        """Apply the periodic on/off schedule. Returns True on an off->on transition."""
        on, left, cycle = self._window()
        if on == self._radio_on:
            return False
        first = self._radio_on is None
        self._radio_on = on
        counts = self._store_counts()
        if on:
            self._counters['windows_on'] += 1
            print(f"[COMMS] radio ON  (cycle {cycle}"
                  + (f", {left:.1f}s window" if left is not None else "")
                  + f", store {counts['store']}, own data queued {counts['q_own_data']})")
            self._log.log('radio_on', cycle=cycle, window_s=left, initial=first, **counts)
        else:
            self._counters['windows_off'] += 1
            # Radio is down, so every peer is unreachable. Age them out now so the first
            # beacon of the next window registers as a returned peer and re-arms
            # forwarding, the same path as a real link outage.
            now = int(time.monotonic() * 1000)
            expired = self._active_peer_ids()
            for p in self._peers.values():
                p['last_seen'] = min(p['last_seen'], now - int(PEER_TIMEOUT_S * 1000) - 1)
            print(f"[COMMS] radio OFF (cycle {cycle}, {left:.1f}s off, store {counts['store']}, "
                  f"own data queued {counts['q_own_data']})")
            self._log.log('radio_off', cycle=cycle, off_s=left, initial=first,
                          peers_dropped=expired, **counts)
        return on and not first

    def _check_peers(self) -> None:
        active = set(self._active_peer_ids())
        for pid in self._last_active - active:
            p = self._peers.get(pid, {})
            self._log.log('peer_down', peer=pid, last_rssi=p.get('rssi'),
                          radio_on=self._radio_on)
            print(f"[PEER] node {pid} lost")
        self._last_active = active

    def _log_state(self) -> None:
        now = int(time.monotonic() * 1000)
        _on, left, cycle = self._window()
        peers = {str(pid): {'rssi': p['rssi'], 'age_ms': now - p['last_seen']}
                 for pid, p in self._peers.items()
                 if (now - p['last_seen']) < PEER_TIMEOUT_S * 1000}
        self._log.log('state', mode=self._mode, exp=self._exp_id, ctrl_version=self._ctrl_version,
                      radio_on=self._radio_on, window_left_s=left, cycle=cycle,
                      tokens=self._policy.tokens(), peers=peers,
                      bs_clock_offset=round(FLEET_CLOCK.offset, 3) if FLEET_CLOCK.synced else None,
                      loc=dict(self._own_location), counters=dict(self._counters),
                      **self._store_counts())

    def _print_status(self) -> None:
        c = self._store_counts()
        _on, left, _cycle = self._window()
        radio = 'ON ' if self._radio_on else 'OFF'
        win = f" {left:4.0f}s left" if left is not None else ""
        peers = ' '.join(f"{pid}({p['rssi']})" for pid, p in self._peers.items()
                         if pid in self._last_active) or '-'
        tok = self._policy.tokens()
        mode = (f"ACTIVE '{self._exp_id}'" if self._mode == 'active'
                else ('PENDING' if self._pending_start else 'STANDBY'))
        print(f"[STATUS] {mode} v{self._ctrl_version} | radio {radio}{win} | queue own:{c['q_own_data']} ferried:{c['q_ferried_data']} "
              f"tel:{c['q_telemetry']} | gen:{self._counters['gen_data']} "
              f"tx:{self._counters['tx_bundle_data']} retx:{self._counters['retx_bundle_data']} "
              f"acked:{self._counters['acked_own_data']}"
              + (f" | tokens:{tok:.2f}" if tok is not None else "")
              + (f" | ros:{self._ros.status()}" if self._ros.enabled else "")
              + f" | peers {peers}")

    def write_summary(self) -> dict:
        lats = self._ack_latencies_ms
        in_exp = self._mode == 'active'
        since = self._exp_start if in_exp else self._wall_start
        summary = {
            'node_id':   self._node_id,
            'experiment': self._exp_id if in_exp else None,
            'ctrl_version': self._ctrl_version,
            'started':   datetime.fromtimestamp(since).isoformat(timespec='seconds'),
            'ended':     datetime.now().isoformat(timespec='seconds'),
            'duration_s': round(time.time() - since, 1),
            'config':    self._current_config(),
            'launch_args': self._run_config,
            'counters':  dict(self._counters),
            'store_at_exit': self._store_counts(),
            'own_data_ack_latency_ms': {
                'n': len(lats),
                'mean': round(sum(lats) / len(lats)) if lats else None,
                'p50': _percentile(lats, 50), 'p95': _percentile(lats, 95),
                'max': max(lats) if lats else None,
            },
        }
        gen = self._counters['gen_data']
        summary['own_data_delivery_ratio'] = (round(self._counters['acked_own_data'] / gen, 4)
                                              if gen else None)
        self._log.log('summary', **summary)
        base = self._log.exp_path if in_exp else self._log.path
        if base:
            path = os.path.splitext(base)[0] + '_summary.json'
            try:
                with open(path, 'w') as f:
                    json.dump(summary, f, indent=2, default=str)
                print(f"[SUMMARY] written to {path}")
            except OSError as e:
                print(f"[SUMMARY] could not write: {e}")
        print(json.dumps(summary, indent=2, default=str))
        return summary

    def _main_loop(self) -> None:
        last_beacon          = 0.0
        last_forward         = 0.0
        last_gen             = time.monotonic()
        last_status          = time.monotonic()
        last_location        = 0.0
        last_location_bundle = 0.0
        last_rover_status    = 0.0
        last_state_log       = 0.0
        last_console_status  = time.monotonic()
        last_ctrl_relay      = time.monotonic()
        last_ros_check       = 0.0

        while True:
            self._check_pending_start()
            if self._update_comms_window():
                last_beacon = 0.0    # announce ourselves immediately when the window opens
                last_forward = 0.0
            if self._restart_pending:
                self._restart_pending = False
                self._boot_id = int.from_bytes(os.urandom(4), 'little')
                self._seq_next = 1
                # Clear our own ack window so the old seq_base (> 1) doesn't mark new
                # seq=1 bundles as already delivered.
                if self._node_id is not None:
                    self._ack.pop(self._node_id, None)
                # ESP32 rebooted: drop the store entirely.  Re-flooding pre-restart bundles
                # causes @ANTIPKT_FAIL at the BS because it already deduped them by
                # (source, creation_time, seq) and they arrive from the same forwarder.
                with self._lock:
                    dropped = list(self._store)
                    self._store = []
                for b in dropped:
                    self._protocol.on_bundle_removed(b)
                self._peers.clear()
                self._last_active = set()
                self._counters['esp_restarts'] += 1
                self._log.log('esp_restart', new_boot_id=self._boot_id, bundles_dropped=len(dropped),
                              own_data_dropped=sum(1 for b in dropped if not b.get('is_telemetry')
                                                   and b['source_node'] == self._node_id))
                print(f"[RESTART] new boot_id=0x{self._boot_id:08x}, seq reset; "
                      f"cleared {len(dropped)} bundles from store; cleared peer table")
                if self._node_id is None:
                    self._node_id = self._detect_node_id()
                self._send_node_restart_notification()

            result = recv_frame(self._ser, timeout_s=0.05, on_ascii=self._handle_ascii)
            if result is not None:
                self._dispatch(result[0], result[1])

            mono = time.monotonic()
            if (mono - last_beacon) >= BEACON_INTERVAL_S:
                last_beacon = mono
                self._send_beacon()
            if (mono - last_forward) >= self._forward_interval_s:
                last_forward = mono
                self._forward()
            if (self._ctrl_msg is not None and self._radio_on
                    and (mono - last_ctrl_relay) >= CONTROL_REPEAT_S):
                last_ctrl_relay = mono
                self._relay_control()
            if self._mode == 'active' and self._interval > 0 and (mono - last_gen) >= self._interval:
                last_gen = mono
                self._generate_bundle()
            if (mono - last_status) >= STATUS_INTERVAL_S:
                last_status = mono
                self._send(HOST_CMD_QUERY_STATUS)   # liveness/disconnect detection
            if (mono - last_location) >= LOCATION_INTERVAL_S:
                last_location = mono
                self._send_location()
            if (mono - last_location_bundle) >= self._loc_interval_s:
                last_location_bundle = mono
                self._send_location_bundle()
            if self._status_interval_s > 0 and (mono - last_rover_status) >= self._status_interval_s:
                last_rover_status = mono
                self._send_status_bundle()
            if (mono - last_state_log) >= self._state_interval_s:
                last_state_log = mono
                self._check_peers()
                self._log_state()
            if (mono - last_ros_check) >= ROS_CHECK_S:
                last_ros_check = mono
                for note in self._ros.tick():
                    print(f"[ROS] {note}")
                    self._log.log('ros_stack', note=note)
            if (mono - last_console_status) >= 5.0:
                last_console_status = mono
                self._print_status()

    def _close_serial(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _reset_after_disconnect(self) -> None:
        self._close_serial()
        if self._node_id_override is None:
            self._node_id = None
        # The store is the rover's source of truth, keep it across a serial reconnect but
        # re-arm forwarding and forget transient peer/ack state.
        with self._lock:
            for b in self._store:
                b['forwarded'] = False
                b['forwarded_ms'] = 0
            kept = len(self._store)
        self._peers.clear()
        self._last_active = set()
        self._ack.clear()
        self._counters['serial_reconnects'] += 1
        self._log.log('serial_reconnect', kept=kept)
        print(f"[RECONNECT] kept {kept} bundles, cleared peers/acks; retrying in {RECONNECT_DELAY_S:.0f}s")

    def _connect_and_run(self) -> None:
        print(f"[INIT] Opening {self._port} at {self._baud} baud (boot_id=0x{self._boot_id:08x})")
        self._log.log('serial_open', port=self._port, baud=self._baud, boot_id=self._boot_id)
        self._open_serial()
        self._update_comms_window()
        if self._node_id is None:
            self._node_id = self._detect_node_id()
        self._log.node_id = self._node_id
        self._log.log('node_detected', boot_id=self._boot_id)
        print(f"[INIT] Running as rover node {self._node_id}")
        self._main_loop()

    def _start_ros_location(self, topic: str) -> None:
        """Keep self._own_location current from a ROS nav_msgs/Odometry topic.

        Tries rospy first; if rospy is not importable (ROS Melodic ships Python 2
        rospy only) falls back to parsing `rostopic echo --noarr -n 0` output so
        the daemon works under plain python3 without any extra packages.
        """
        if _ROS_AVAILABLE:
            self._start_ros_location_rospy(topic)
        else:
            self._start_ros_location_subprocess(topic)

    def _start_ros_location_rospy(self, topic: str) -> None:
        def _thread() -> None:
            try:
                # disable_signals=True required when init_node is called from a
                # non-main thread (signal handlers must stay on the main thread).
                rospy.init_node('rover_daemon_loc', anonymous=True, disable_signals=True)

                def _cb(msg: _Odometry) -> None:
                    p = msg.pose.pose.position
                    q = msg.pose.pose.orientation
                    yaw = _yaw_deg(q.x, q.y, q.z, q.w)
                    with self._lock:
                        self._own_location = {'lat': p.x, 'lon': p.y, 'alt': p.z, 'yaw': yaw}

                rospy.Subscriber(topic, _Odometry, _cb)
                print(f"[LOC] Subscribed to {topic} via rospy")
                rospy.spin()
            except Exception as exc:
                print(f"[LOC] rospy location thread failed: {exc}")

        threading.Thread(target=_thread, daemon=True, name='ros-loc').start()

    def _start_ros_location_subprocess(self, topic: str) -> None:
        """Parse `rostopic echo` YAML output for pose.pose.position and pose.pose.orientation."""
        import subprocess, re
        print(f"[LOC] rospy unavailable - falling back to rostopic echo for {topic}")

        def _thread() -> None:
            key_re = re.compile(r'^(\s*)(\w+)\s*:\s*(.*)')
            while True:
                try:
                    proc = subprocess.Popen(
                        ['rostopic', 'echo', topic],
                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                        universal_newlines=True,
                    )
                    # nav_msgs/Odometry YAML: track the key path by indentation and keep
                    # pose/pose/position/{x,y,z} and pose/pose/orientation/{x,y,z,w};
                    # '---' ends each message.
                    stack: list = []          # [(indent, key)]
                    vals: dict = {}
                    for line in proc.stdout:
                        if line.rstrip() == '---':
                            pos = [vals.get(('position', k)) for k in 'xyz']
                            ori = [vals.get(('orientation', k)) for k in 'xyzw']
                            if None not in pos:
                                yaw = _yaw_deg(*ori) if None not in ori else None
                                with self._lock:
                                    self._own_location = {'lat': pos[0], 'lon': pos[1],
                                                          'alt': pos[2], 'yaw': yaw}
                            stack, vals = [], {}
                            continue
                        m = key_re.match(line)
                        if not m:
                            continue
                        indent, key, val = len(m.group(1)), m.group(2), m.group(3).strip()
                        while stack and stack[-1][0] >= indent:
                            stack.pop()
                        path = [k for _i, k in stack]
                        if not val:
                            stack.append((indent, key))
                        elif path[:2] == ['pose', 'pose'] and len(path) == 3:
                            try:
                                vals[(path[2], key)] = float(val)
                            except ValueError:
                                pass
                    proc.wait()
                    print("[LOC] rostopic echo exited; restarting in 2s")
                    time.sleep(2)
                except Exception as exc:
                    print(f"[LOC] rostopic echo failed: {exc}; retrying in 5s")
                    time.sleep(5)

        threading.Thread(target=_thread, daemon=True, name='ros-loc').start()

    def run(self) -> None:
        self._log.log('daemon_start', config=self._run_config, boot_id=self._boot_id)
        if self._ros_location_topic:
            self._start_ros_location(self._ros_location_topic)
        try:
            while True:
                try:
                    self._connect_and_run()
                except KeyboardInterrupt:
                    raise
                except Exception as e:
                    print(f"[ERROR] {type(e).__name__}: {e}")
                    traceback.print_exc()
                    self._log.log('error', error=f"{type(e).__name__}: {e}",
                                  trace=traceback.format_exc())
                    self._reset_after_disconnect()
                time.sleep(RECONNECT_DELAY_S)
        except KeyboardInterrupt:
            print('\n[INIT] Shutting down.')
        finally:
            self._close_serial()
            self._end_experiment('daemon shutdown')
            self.write_summary()
            self._log.log('daemon_stop')
            self._log.close()


def _sigterm(_signum, _frame):
    # treat `kill` / tmux kill-session like Ctrl-C so the summary still gets written
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description='Rover node daemon (ESP32 is relay)',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--serial',      default='/dev/ttyUSB0', help='Serial port to ESP32')
    parser.add_argument('--baud',        type=int, default=115200, help='Baud rate')
    parser.add_argument('--interval',    type=float, default=1.0,
                        help='Data bundle generation interval in seconds (0 = disable)')
    parser.add_argument('--payload-bytes', type=int, default=0,
                        help='Pad each data bundle payload to this many bytes (max 1024, 0 = natural ~60 B)')
    parser.add_argument('--node-id',     type=int, default=None,
                        help='Override node ID (default: auto-detect from ESP32)')
    parser.add_argument('--routing',     choices=['epidemic', 'spray-wait'], default='epidemic',
                        help='Routing protocol')
    parser.add_argument('--spray-count', type=int, default=2,
                        help='Initial copy count for spray-and-wait')
    parser.add_argument('--retransmit-s', type=float, default=BUNDLE_RETRANSMIT_S,
                        help='Re-send a forwarded bundle the BS has not acked after this many seconds')

    bw = parser.add_argument_group('bandwidth-restricted mode')
    bw.add_argument('--bw-limit', type=float, default=0.0,
                    help='Max data bundles per second over the air (0 = unlimited). '
                         'Beacons, ackmaps and location broadcasts are never limited')
    bw.add_argument('--bw-burst', type=float, default=1.0,
                    help='Token bucket size: max data bundles sent back-to-back after idling')
    bw.add_argument('--bw-order', choices=['oldest', 'newest'], default='oldest',
                    help='Which queued bundles get the budget first')
    bw.add_argument('--bw-count-telemetry', action='store_true',
                    help='Also count telemetry bundles (acks, location, status) against the budget')

    pc = parser.add_argument_group('periodic comms mode')
    pc.add_argument('--comms-on',  type=float, default=0.0,
                    help='Seconds the radio is on per cycle (0 = always on)')
    pc.add_argument('--comms-off', type=float, default=0.0,
                    help='Seconds the radio is off per cycle (no TX, RX dropped; data still generated)')
    pc.add_argument('--comms-phase', type=float, default=0.0,
                    help="Shift this rover's schedule by N seconds (windows are aligned to wall clock)")

    parser.add_argument('--mode', choices=['standby', 'active'], default='standby',
                        help='standby: wait for the BS to start an experiment (no data bundles). '
                             'active: start an experiment immediately with the CLI settings')

    lg = parser.add_argument_group('logging')
    lg.add_argument('--log-dir', default='rover_logs', help='Directory for the JSON-lines event log')
    lg.add_argument('--run-tag', default='', help='Label added to the log filename, e.g. bw1_route3')
    lg.add_argument('--state-interval', type=float, default=STATE_LOG_INTERVAL_S,
                    help='Seconds between full state snapshots in the event log')
    lg.add_argument('--status-interval', type=float, default=ROVER_STATUS_INTERVAL_S,
                    help='Seconds between @ROVER_STATUS telemetry bundles to the BS (0 = off)')
    lg.add_argument('--no-log', action='store_true', help='Disable the event log file')

    parser.add_argument('--lat',         type=float, default=0.0, help='Own latitude (static fallback)')
    parser.add_argument('--lon',         type=float, default=0.0, help='Own longitude (static fallback)')
    parser.add_argument('--alt',         type=float, default=0.0, help='Own altitude in metres (static fallback)')
    parser.add_argument('--loc-interval', type=float, default=LOCATION_BUNDLE_INTERVAL_S,
                        help='Seconds between @LOCATION telemetry bundles to the BS '
                             '(the dashboard can change it)')
    parser.add_argument('--ros-location-topic', default=None, metavar='TOPIC',
                        help='ROS 1 nav_msgs/Odometry topic for live position '
                             '(e.g. /car/t265/odom/sample); overrides --lat/--lon/--alt')
    parser.add_argument('--ros-control', action='store_true',
                        help='Let the BS start/stop the ROS stack (' + ' '.join(ROS_STACK_CMD) + '). '
                             'Linux only; needs the ROS environment sourced')
    args = parser.parse_args()

    if args.routing == 'spray-wait':
        protocol: RoutingProtocol = SprayAndWaitRouting(spray_count=args.spray_count)
        print(f'[INIT] Routing: spray-and-wait (L={args.spray_count})')
    else:
        protocol = EpidemicRouting()
        print('[INIT] Routing: epidemic')

    policy = CommsPolicy(bw_limit=args.bw_limit, bw_burst=args.bw_burst,
                         count_telemetry=args.bw_count_telemetry,
                         on_s=args.comms_on, off_s=args.comms_off, phase_s=args.comms_phase)
    if (args.comms_on > 0) != (args.comms_off > 0):
        print('[INIT] WARNING: periodic mode needs both --comms-on and --comms-off; radio stays always on')
    print(f"[INIT] Comms: bw_limit={args.bw_limit or 'unlimited'} data bundles/s "
          f"(burst {policy.bw_burst:g}, {args.bw_order} first)"
          + (f", periodic {args.comms_on:g}s on / {args.comms_off:g}s off" if policy.periodic
             else ", radio always on"))

    log_path = None
    if not args.no_log:
        tag = f"_{args.run_tag}" if args.run_tag else ''
        log_path = os.path.join(args.log_dir,
                                f"rover_{datetime.now().strftime('%Y%m%d_%H%M%S')}{tag}.jsonl")
        print(f"[INIT] Event log: {log_path}")

    signal.signal(signal.SIGTERM, _sigterm)

    daemon = RoverDaemon(
        serial_port=args.serial,
        baud=args.baud,
        gen_interval=args.interval,
        node_id_override=args.node_id,
        protocol=protocol,
        lat=args.lat,
        lon=args.lon,
        alt=args.alt,
        ros_location_topic=args.ros_location_topic,
        policy=policy,
        event_log=EventLog(log_path),
        bw_order=args.bw_order,
        payload_bytes=args.payload_bytes,
        retransmit_s=args.retransmit_s,
        status_interval_s=args.status_interval,
        state_interval_s=args.state_interval,
        loc_interval_s=max(0.2, args.loc_interval),
        run_config=dict(vars(args), **policy.describe()),
        ros_stack=RosStack(args.ros_control,
                           state_path=os.path.join(args.log_dir, 'ros_control_state.json'),
                           log_dir=args.log_dir),
    )
    if args.mode == 'active':
        daemon.start_on_connect(args.run_tag or 'cli')
    daemon.run()


if __name__ == '__main__':
    main()
