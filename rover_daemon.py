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
    python rover_daemon.py [--serial PORT] [--baud BAUD] [--interval SECS] [--node-id ID]

    --serial   Serial port connected to ESP32 (default: /dev/ttyUSB0)
    --baud     Baud rate (default: 115200)
    --interval Seconds between generated sensor bundles (default: 1, 0 = disable)
    --node-id  Override node ID (default: auto-detected from ESP32 STATUS_RESP on startup)
"""

import argparse
import itertools
import json
import os
import platform
import struct
import threading
import time
import serial
from abc import ABC, abstractmethod
from typing import Callable, Dict, List, Optional, Set, Tuple

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
PEER_TIMEOUT_S             = 15.0
BUNDLE_RETRANSMIT_S        = 5.0    # re-send a forwarded-but-unacked bundle to the BS after 5s
LOCATION_INTERVAL_S        = 5.0    # how often to broadcast own location to neighbors
LOCATION_BUNDLE_INTERVAL_S = 30.0   # how often to send location as a telemetry bundle toward BS

RECONNECT_DELAY_S  = 5.0
SERIAL_SETTLE_S    = 3.0
WRITE_TIMEOUT_S    = 2.0
STATUS_INTERVAL_S  = 5.0


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
# pkt_type(u8), node_id(u16), lat(f32), lon(f32), alt(f32)  — 15 bytes total
LOCATION_FMT  = '<BHfff'
LOCATION_SIZE = struct.calcsize(LOCATION_FMT)   # 15


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


# daemon
class RoverDaemon:
    def __init__(self, serial_port: str, baud: int, gen_interval: float,
                 node_id_override: Optional[int], protocol: RoutingProtocol,
                 lat: float = 0.0, lon: float = 0.0, alt: float = 0.0):
        assert BUNDLE_SIZE == 1052, f"Bundle struct size mismatch: got {BUNDLE_SIZE}, expected 1052"
        assert ACKMAP_SIZE == 42,   f"Ackmap struct size mismatch: got {ACKMAP_SIZE}, expected 42"

        self._port             = serial_port
        self._baud             = baud
        self._interval         = gen_interval
        self._node_id_override = node_id_override
        self._node_id          = node_id_override
        self._protocol         = protocol

        self._store: List[dict] = []        # each: dtn fields + added_at_ms, forwarded, forwarded_ms
        self._lock = threading.Lock()
        self._seq  = itertools.count(start=1)
        self._start_ms = int(time.monotonic() * 1000)
        self._boot_id  = int.from_bytes(os.urandom(4), 'little')

        self._delivered: Set[tuple] = set()                  # (source, creation_time, seq) acked by BS
        self._peers: Dict[int, dict] = {}                    # node_id -> {last_seen, boot_id, rssi}
        self._ack: Dict[int, dict] = {}                      # source_node -> {seq_base, bitmap, dirty}

        self._own_location = {'lat': lat, 'lon': lon, 'alt': alt}
        self._locations: Dict[int, dict] = {}                # node_id -> {lat, lon, alt, updated_ms}

        self._detected_node_id: Optional[int] = None
        self._restart_pending = False

        self._ser: Optional[serial.Serial] = None
        self._uart_lock = threading.Lock()

    # serial helpers
    def _send(self, cmd: int, payload: bytes = b'') -> None:
        frame = encode_frame(cmd, payload)
        with self._uart_lock:
            self._ser.write(frame)

    # ---- bundle store ----
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
        if drop:
            print(f"[STORE] Expired {len(drop)} bundles by TTL")

    # ---- ACK window (sliding 256-bit bitmap per source), mirrors mark_ack_locked ----
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
        for b in dropped:
            self._protocol.on_bundle_removed(b)
        if dropped:
            print(f"[ANTIPKT] ACKMAP freed {len(dropped)} for src:{source_node} (store now {len(self._store)})")

    # ---- peer table ----
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
        self._peers[node_id] = {'last_seen': now, 'boot_id': boot_id, 'rssi': rssi}

        if not (was_absent or restarted):
            return

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
        if restarted:
            self._ack.pop(node_id, None)   # clear stale ack window for the rebooted source
            print(f"[PEER] node {node_id} restarted (boot_id changed); reset forwarded flags"
                  + (f", dropped {len(restart_dropped)} delivered" if restart_dropped else ""))
        else:
            print(f"[PEER] node {node_id} {'returned' if prev else 'discovered'}; reset forwarded flags")

    # ---- bundle RX ----
    def _on_bundle(self, rest: bytes) -> None:
        b = unpack_bundle(rest[1:])
        if not b:
            return
        b['added_at_ms'] = int(time.monotonic() * 1000)
        if not self._add_bundle(b):
            return   # duplicate or already delivered
        print(f"[RX] bundle src:{b['source_node']} seq:{b['sequence_number']} "
              f"hops:{b['hop_count']} (store {len(self._store)})")
        # for a received DATA bundle, emit a telemetry-ACK the BS will print as @DTN_RX
        if not b['is_telemetry']:
            self._make_telemetry_ack(b)

    def _make_telemetry_ack(self, orig: dict) -> None:
        now = int(time.monotonic() * 1000)
        text = (f"@DTN_RX:{orig['source_node']}:{orig['prev_node']}:"
                f"{orig['sequence_number']}:{self._node_id}:{orig['hop_count']}")
        ack = {
            'creation_time':   now - self._start_ms,
            'sequence_number': next(self._seq),
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

    # ---- ackmap RX ----
    def _on_ackmap(self, rest: bytes) -> None:
        fields = unpack_ackmap(rest)
        if fields is None:
            return
        _ptype, origin, source, seq_base, blen, bitmap = fields
        if origin != BASE_STATION_NODE_ID:
            return
        if blen == 0 or blen > ACKMAP_BITMAP_BYTES:
            return
        self._merge_ackmap(source, seq_base, bitmap, blen)        # for propagation
        self._apply_ackmap_to_store(source, seq_base, bitmap, blen)

    # ---- forwarding + ackmap propagation ----
    def _forward(self) -> None:
        self._expire_bundles()
        now = int(time.monotonic() * 1000)
        active = self._active_peer_ids()
        peers_active = len(active) > 0
        bs_active = BASE_STATION_NODE_ID in active

        with self._lock:
            snapshot = list(self._store)

        to_remove = []
        for b in snapshot:
            if b['hop_count'] >= b['hop_limit']:
                to_remove.append(b)
                continue
            if (b.get('forwarded') and bs_active and
                    (now - b.get('forwarded_ms', 0)) >= BUNDLE_RETRANSMIT_S * 1000):
                b['forwarded'] = False   # unacked too long; allow re-send
            if b.get('forwarded') or not peers_active:
                continue
            if not self._protocol.should_forward(b, active, self._node_id, self._peers):
                continue
            out = dict(b)
            out['prev_node'] = self._node_id
            out['hop_count'] = b['hop_count'] + 1
            self._send(HOST_CMD_WIFI_TX, encode_air_bundle(out))
            b['forwarded'] = True
            b['forwarded_ms'] = now
            self._protocol.on_bundle_forwarded(b)

        if to_remove:
            for b in to_remove:
                self._protocol.on_bundle_removed(b)
            ids = {id(b) for b in to_remove}
            with self._lock:
                self._store = [b for b in self._store if id(b) not in ids]

        # propagate any updated ack windows so disconnected/ferried peers learn deliveries
        if peers_active:
            for source, a in self._ack.items():
                if a['dirty']:
                    self._send(HOST_CMD_WIFI_TX, pack_ackmap(source, a['seq_base'], a['bitmap']))
                    a['dirty'] = False

    # ---- beacon ----
    def _send_beacon(self) -> None:
        ts = int(time.monotonic() * 1000) - self._start_ms
        self._send(HOST_CMD_WIFI_TX, pack_beacon(self._node_id, ts, self._boot_id))

    # ---- location ----
    def _send_location(self) -> None:
        loc = self._own_location
        self._send(HOST_CMD_WIFI_TX,
                   pack_location(self._node_id, loc['lat'], loc['lon'], loc['alt']))

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
        print(f"[LOC] node {node_id}: lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m")

    def _send_location_bundle(self) -> None:
        """Send own location as a telemetry bundle so the BS serial output captures it."""
        loc = self._own_location
        payload = (f"@LOCATION:{self._node_id}:"
                   f"{loc['lat']:.6f}:{loc['lon']:.6f}:{loc['alt']:.1f}").encode()
        now = int(time.monotonic() * 1000)
        b = {
            'creation_time':   now - self._start_ms,
            'sequence_number': next(self._seq),
            'lifetime':        TELEMETRY_ACK_LIFETIME,
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

    # ---- bundle generator ----
    def _generate_bundle(self) -> None:
        abs_ms = int(time.monotonic() * 1000)
        creation_time = abs_ms - self._start_ms
        payload = json.dumps({
            'ts': int(time.time()),
            'node': self._node_id,
            'sensor': round(20.0 + (creation_time % 1000) / 100.0, 1),
        }).encode()
        b = {
            'creation_time':   creation_time,
            'sequence_number': next(self._seq),
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
            print(f"[GEN] bundle seq:{b['sequence_number']} (store {len(self._store)})")

    # boot-banner detector
    def _handle_ascii(self, text: str) -> None:
        if 'I am' in text or 'rst:0x' in text:
            print(f"[RESTART] ESP32 boot banner: {text.strip()!r}")
            self._restart_pending = True

    # ---- frame dispatch ----
    def _dispatch(self, cmd: int, payload: bytes) -> None:
        if cmd == HOST_CMD_WIFI_RX:
            if len(payload) < 8:   # 6 mac + 1 rssi + >=1 air byte
                return
            rssi = struct.unpack_from('<b', payload, 6)[0]
            rest = payload[7:]
            ptype = rest[0]
            if ptype == PKT_TYPE_BEACON:
                bc = unpack_beacon(rest)
                if bc:
                    self._on_beacon(bc[0], bc[1], bc[2], rssi)
            elif ptype == PKT_TYPE_BUNDLE:
                self._on_bundle(rest)
            elif ptype == PKT_TYPE_ACKMAP:
                self._on_ackmap(rest)
            elif ptype == PKT_TYPE_LOCATION:
                self._on_location(rest)
        elif cmd == HOST_CMD_STATUS_RESP:
            if len(payload) >= STATUS_SIZE:
                self._detected_node_id = struct.unpack_from('<H', payload)[0]
        # other host commands are unused on the relay link

    # ---- serial lifecycle ----
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

    # ---- main loop ----
    def _main_loop(self) -> None:
        last_beacon          = 0.0
        last_forward         = 0.0
        last_gen             = time.monotonic()
        last_status          = time.monotonic()
        last_location        = 0.0
        last_location_bundle = 0.0

        while True:
            if self._restart_pending:
                self._restart_pending = False
                # ESP32 rebooted; the daemon (and store) survived. Re-flood through the fresh
                # modem, clear peer table (will be rediscovered), keep store + delivered set.
                with self._lock:
                    for b in self._store:
                        b['forwarded'] = False
                        b['forwarded_ms'] = 0
                    count = len(self._store)
                self._peers.clear()
                print(f"[RESTART] re-armed {count} bundles for retransmit; cleared peer table")
                if self._node_id is None:
                    self._node_id = self._detect_node_id()

            result = recv_frame(self._ser, timeout_s=0.05, on_ascii=self._handle_ascii)
            if result is not None:
                self._dispatch(result[0], result[1])

            mono = time.monotonic()
            if (mono - last_beacon) >= BEACON_INTERVAL_S:
                last_beacon = mono
                self._send_beacon()
            if (mono - last_forward) >= FORWARD_INTERVAL_S:
                last_forward = mono
                self._forward()
            if self._interval > 0 and (mono - last_gen) >= self._interval:
                last_gen = mono
                self._generate_bundle()
            if (mono - last_status) >= STATUS_INTERVAL_S:
                last_status = mono
                self._send(HOST_CMD_QUERY_STATUS)   # liveness/disconnect detection
            if (mono - last_location) >= LOCATION_INTERVAL_S:
                last_location = mono
                self._send_location()
            if (mono - last_location_bundle) >= LOCATION_BUNDLE_INTERVAL_S:
                last_location_bundle = mono
                self._send_location_bundle()

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
        # The store is the rover's source of truth; keep it across a serial reconnect but
        # re-arm forwarding and forget transient peer/ack state.
        with self._lock:
            for b in self._store:
                b['forwarded'] = False
                b['forwarded_ms'] = 0
            kept = len(self._store)
        self._peers.clear()
        self._ack.clear()
        print(f"[RECONNECT] kept {kept} bundles, cleared peers/acks; retrying in {RECONNECT_DELAY_S:.0f}s")

    def _connect_and_run(self) -> None:
        print(f"[INIT] Opening {self._port} at {self._baud} baud (boot_id=0x{self._boot_id:08x})")
        self._open_serial()
        if self._node_id is None:
            self._node_id = self._detect_node_id()
        print(f"[INIT] Running as rover node {self._node_id}")
        self._main_loop()

    def run(self) -> None:
        while True:
            try:
                self._connect_and_run()
            except KeyboardInterrupt:
                print('\n[INIT] Shutting down.')
                self._close_serial()
                return
            except Exception as e:
                print(f"[ERROR] {type(e).__name__}: {e}")
                self._reset_after_disconnect()
            try:
                time.sleep(RECONNECT_DELAY_S)
            except KeyboardInterrupt:
                print('\n[INIT] Shutting down.')
                return


def main():
    parser = argparse.ArgumentParser(description='Rover node daemon (ESP32 is a dumb relay)')
    parser.add_argument('--serial',      default='/dev/ttyUSB0', help='Serial port to ESP32')
    parser.add_argument('--baud',        type=int, default=115200, help='Baud rate')
    parser.add_argument('--interval',    type=float, default=1.0,
                        help='Bundle generation interval in seconds (0 = disable)')
    parser.add_argument('--node-id',     type=int, default=None,
                        help='Override node ID (default: auto-detect from ESP32)')
    parser.add_argument('--routing',     choices=['epidemic', 'spray-wait'], default='epidemic',
                        help='Routing protocol (default: epidemic)')
    parser.add_argument('--spray-count', type=int, default=8,
                        help='Initial copy count for spray-and-wait (default: 8)')
    parser.add_argument('--lat',         type=float, default=0.0, help='Own latitude')
    parser.add_argument('--lon',         type=float, default=0.0, help='Own longitude')
    parser.add_argument('--alt',         type=float, default=0.0, help='Own altitude (metres)')
    args = parser.parse_args()

    if args.routing == 'spray-wait':
        protocol: RoutingProtocol = SprayAndWaitRouting(spray_count=args.spray_count)
        print(f'[INIT] Routing: spray-and-wait (L={args.spray_count})')
    else:
        protocol = EpidemicRouting()
        print('[INIT] Routing: epidemic')

    RoverDaemon(
        serial_port=args.serial,
        baud=args.baud,
        gen_interval=args.interval,
        node_id_override=args.node_id,
        protocol=protocol,
        lat=args.lat,
        lon=args.lon,
        alt=args.alt,
    ).run()


if __name__ == '__main__':
    main()
