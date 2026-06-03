# -*- coding: utf-8 -*-
"""
Jetson node daemon — Python 2.7 compatible port.
Runs on a rover Jetson Orin Nano.
Manages the local bundle store and bridges it to the attached ESP32 modem via UART.

Usage:
    python jetson_daemon_2.7.py [--serial PORT] [--baud BAUD] [--interval SECS] [--node-id ID]

    --serial   Serial port connected to ESP32 (default: /dev/ttyUSB0)
    --baud     Baud rate (default: 115200)
    --interval Seconds between generated sensor bundles (default: 1.0, 0 = disable)
    --node-id  Override node ID (default: auto-detected from ESP32 STATUS_RESP on startup)
"""
from __future__ import print_function

import argparse
import itertools
import json
import platform
import struct
import threading
import time
import serial

# time.monotonic() was added in Python 3.3; fall back to time.time() on Python 2.7
try:
    _monotonic = time.monotonic
except AttributeError:
    _monotonic = time.time

# protocol constants (must match mesh_main.c)
HOST_SOF              = 0xAA
HOST_MAX_PAYLOAD      = 1060
BASE_STATION_NODE_ID  = 23768

HOST_CMD_TX_BUNDLE      = 0x01
HOST_CMD_QUERY_STATUS   = 0x02
HOST_CMD_PULL_REQ       = 0x10
HOST_CMD_ACK            = 0x20
HOST_CMD_STATUS_RESP    = 0x21
HOST_CMD_BUNDLE_DATA    = 0x11
HOST_CMD_NO_BUNDLES     = 0x12
HOST_CMD_ANTIPKT_NOTIFY = 0x13
HOST_CMD_BUNDLE_PUSH    = 0x14  # ESP32 -> Jetson: store received neighbor bundle

# dtn_bundle_t layout (packed, little-endian, matches ESP32 C struct)
# Fields: creation_time, sequence_number, lifetime, source_node, dest_node,
#         report_to_node, prev_node, request_delivery_report, is_telemetry,
#         hop_limit, hop_count, payload_len, payload[1024]
BUNDLE_FMT  = '<IIIHHHHBBBBI1024s'
BUNDLE_SIZE = struct.calcsize(BUNDLE_FMT)   # must be 1052

# antipkt_id_t layout: source_node (u16), creation_time (u32), sequence_number (u32)
ANTIPKT_ID_FMT  = '<HII'
ANTIPKT_ID_SIZE = struct.calcsize(ANTIPKT_ID_FMT)  # 10 bytes

# host_status_t layout: node_id (u16), active_peers, store_used, store_max, channel (all u8)
STATUS_FMT  = '<HBBBB'
STATUS_SIZE = struct.calcsize(STATUS_FMT)  # 6 bytes

BUNDLE_DEFAULT_LIFETIME  = 3000000  # ms (50 minutes)
BUNDLE_DEFAULT_HOP_LIMIT = 5

# CRC-16/CCITT (poly 0x1021, init 0xFFFF)
def _crc16(data):
    crc = 0xFFFF
    for b in bytearray(data):  # bytearray gives ints in Python 2 and 3
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

# frame encode / decode
def encode_frame(cmd, payload=b''):
    plen     = len(payload)
    header   = bytearray([HOST_SOF, cmd, plen & 0xFF, (plen >> 8) & 0xFF])
    crc_data = bytearray([cmd, plen & 0xFF, (plen >> 8) & 0xFF]) + bytearray(payload)
    crc      = _crc16(crc_data)
    return bytes(header + bytearray(payload) + bytearray([crc & 0xFF, (crc >> 8) & 0xFF]))


def _read_exact(ser, n, timeout_s=1.0):
    deadline = _monotonic() + timeout_s
    buf = bytearray()
    while len(buf) < n:
        remaining = deadline - _monotonic()
        if remaining <= 0:
            break
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(bytearray(chunk))
    return bytes(buf)


def recv_frame(ser, timeout_s=1.0, verbose=False):
    """Block until a valid frame arrives or timeout. Returns (cmd, payload) or None."""
    deadline = _monotonic() + timeout_s
    skipped  = bytearray()
    while _monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        bval = bytearray(b)[0]
        if bval != HOST_SOF:
            skipped.append(bval)
            continue
        if skipped:
            print("[RECV] Skipped {} non-SOF bytes".format(len(skipped)))
            skipped = bytearray()
        header_raw = _read_exact(ser, 3, timeout_s=0.2)
        if len(header_raw) < 3:
            print("[RECV] Incomplete header after SOF (got {}/3 bytes)".format(len(header_raw)))
            return None
        header = bytearray(header_raw)
        cmd    = header[0]
        plen   = header[1] | (header[2] << 8)
        if plen > HOST_MAX_PAYLOAD:
            print("[RECV] Oversized payload claim {} > {}, discarding".format(plen, HOST_MAX_PAYLOAD))
            continue
        payload  = _read_exact(ser, plen, timeout_s=1.0) if plen else b''
        crc_raw  = _read_exact(ser, 2, timeout_s=0.2)
        if len(crc_raw) < 2:
            print("[RECV] Truncated CRC (got {}/2 bytes)".format(len(crc_raw)))
            return None
        crc_ba   = bytearray(crc_raw)
        recv_crc = crc_ba[0] | (crc_ba[1] << 8)
        calc_crc = _crc16(bytearray([cmd, header[1], header[2]]) + bytearray(payload))
        if recv_crc != calc_crc:
            print("[WARN] CRC mismatch cmd=0x{:02x} expected=0x{:04x} got=0x{:04x}".format(
                cmd, calc_crc, recv_crc))
            continue
        if verbose:
            print("[RECV] Frame cmd=0x{:02x} len={}".format(cmd, plen))
        return cmd, payload
    if skipped:
        print("[RECV] Timeout with {} trailing non-SOF bytes".format(len(skipped)))
    return None

# Bundle serialization
def pack_bundle(b):
    payload_bytes = b['payload']
    # In Python 2, str and bytes are the same; only encode if unicode
    try:
        if isinstance(payload_bytes, unicode):
            payload_bytes = payload_bytes.encode('utf-8')
    except NameError:
        # Python 3: unicode doesn't exist, str is already the text type
        if isinstance(payload_bytes, str):
            payload_bytes = payload_bytes.encode('utf-8')
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


def unpack_bundle(data):
    # BUNDLE_PUSH frames may be shorter than BUNDLE_SIZE (only header + actual payload).
    BUNDLE_HEADER_SIZE = BUNDLE_SIZE - 1024
    if len(data) < BUNDLE_HEADER_SIZE:
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

# daemon
class JetsonDaemon(object):
    def __init__(self, serial_port, baud, gen_interval, node_id_override):
        assert BUNDLE_SIZE == 1052, \
            "Bundle struct size mismatch: got {}, expected 1052".format(BUNDLE_SIZE)

        self._port     = serial_port
        self._baud     = baud
        self._interval = gen_interval
        self._node_id  = node_id_override  # None = auto detect on startup

        self._store     = []
        self._lock      = threading.Lock()
        self._seq       = itertools.count(1)  # count(start=1) keyword form not available in Python 2
        self._start_ms  = int(_monotonic() * 1000)  # daemon epoch

        self._delivered = set()  # set of (source_node, creation_time, seq) tuples

        self._ser       = None
        self._uart_lock = threading.Lock()  # serialises UART writes

    # serial helpers
    def _send(self, cmd, payload=b''):
        frame = encode_frame(cmd, payload)
        print("[TX] cmd=0x{:02x} len={}".format(cmd, len(payload)))
        with self._uart_lock:
            self._ser.write(frame)

    def _recv(self, timeout_s=1.0):
        with self._uart_lock:
            return recv_frame(self._ser, timeout_s, verbose=True)

    # bundle store
    def _add_bundle(self, b):
        key = (b['source_node'], b['creation_time'], b['sequence_number'])
        with self._lock:
            if key in self._delivered:
                return False
            for stored in self._store:
                if (stored['source_node'] == b['source_node'] and
                        stored['creation_time'] == b['creation_time'] and
                        stored['sequence_number'] == b['sequence_number']):
                    return False
            self._store.append(b)
        print("[STORE] Added src:{} seq:{} | store size: {}".format(
            b['source_node'], b['sequence_number'], len(self._store)))
        return True

    def _expire_bundles(self):
        now = int(_monotonic() * 1000)
        with self._lock:
            before = len(self._store)
            self._store = [
                b for b in self._store
                if (now - b.get('added_at_ms', now)) < b.get('lifetime', BUNDLE_DEFAULT_LIFETIME)
            ]
            expired = before - len(self._store)
        if expired:
            print("[STORE] Expired {} bundles by TTL".format(expired))

    def _apply_antipkt(self, source_node, creation_time, seq):
        key = (source_node, creation_time, seq)
        self._delivered.add(key)
        with self._lock:
            before = len(self._store)
            self._store = [
                b for b in self._store
                if not (b['source_node'] == source_node and
                        b['creation_time'] == creation_time and
                        b['sequence_number'] == seq)
            ]
            removed = before - len(self._store)
        if removed:
            print("[ANTIPKT] Cleared src:{} seq:{} from store".format(source_node, seq))
        else:
            print("[ANTIPKT] src:{} seq:{} (not in store, already clear)".format(source_node, seq))

    # startup: detect node ID from ESP32 STATUS_RESP
    def _detect_node_id(self):
        for attempt in range(5):
            frame = encode_frame(HOST_CMD_QUERY_STATUS)
            print("[INIT] Attempt {}/5: sending QUERY_STATUS {}".format(
                attempt + 1, frame.encode('hex')))
            try:
                with self._uart_lock:
                    self._ser.write(frame)
            except serial.SerialException as e:
                print("[INIT] Write failed ({}), waiting for device to settle...".format(e))
                time.sleep(2.0)
                self._ser.reset_input_buffer()
                continue
            # Loop within the 2s window: dispatch BUNDLE_PUSH/ANTIPKT/etc. that
            # arrive before STATUS_RESP (common now that the ESP32 pushes bundles).
            deadline = _monotonic() + 2.0
            while _monotonic() < deadline:
                remaining = deadline - _monotonic()
                result = recv_frame(self._ser, timeout_s=max(0.05, remaining), verbose=True)
                if result is None:
                    break
                cmd, payload = result
                if cmd == HOST_CMD_STATUS_RESP and len(payload) >= STATUS_SIZE:
                    node_id = struct.unpack_from('<H', payload)[0]
                    print("[INIT] Auto-detected node_id={}".format(node_id))
                    return node_id
                # dispatch other frames (bundle pushes, antipackets) so they aren't lost
                print("[INIT] Got cmd=0x{:02x} while waiting for STATUS_RESP, dispatching".format(cmd))
                self._dispatch(cmd, payload)
            print("[INIT] No STATUS_RESP in 2s window, retrying...")
            time.sleep(1)
        raise RuntimeError("Could not auto-detect node_id from ESP32. Use --node-id to override.")

    # PULL_REQ handler: drain local store to ESP32
    def _handle_pull_req(self):
        self._expire_bundles()
        with self._lock:
            pending = list(self._store)
        if not pending:
            self._send(HOST_CMD_NO_BUNDLES)
            return
        sent_bundles = []
        skipped = 0
        for b in pending:
            key = (b['source_node'], b['creation_time'], b['sequence_number'])
            if key in self._delivered:
                skipped += 1
                continue  # antipacket arrived for this during drain, don't re-push
            data = pack_bundle(b)
            self._send(HOST_CMD_BUNDLE_DATA, data)
            sent_bundles.append(b)
            # Wait for the ESP32's implicit ACK (another PULL_REQ).
            deadline = _monotonic() + 2.0
            got_pull = False
            while _monotonic() < deadline:
                remaining = deadline - _monotonic()
                nxt = recv_frame(self._ser, timeout_s=max(0.05, remaining))
                if nxt is None:
                    break  # timeout, ESP32 store full, stop draining
                if nxt[0] == HOST_CMD_PULL_REQ:
                    got_pull = True
                    break
                self._dispatch(nxt[0], nxt[1])  # antipkt etc., keep waiting
            if not got_pull:
                break
        self._send(HOST_CMD_NO_BUNDLES)
        by_src = {}
        for b in sent_bundles:
            by_src[b['source_node']] = by_src.get(b['source_node'], 0) + 1
        src_summary = ' '.join('src{}x{}'.format(n, c) for n, c in sorted(by_src.items()))
        skip_str = " skip:{}".format(skipped) if skipped else ""
        print("[PULL] Sent {}/{} bundles to ESP32{}  [{}]".format(
            len(sent_bundles), len(pending), skip_str, src_summary))

    # frame dispatcher
    def _dispatch(self, cmd, payload):
        if cmd == HOST_CMD_PULL_REQ:
            self._handle_pull_req()

        elif cmd == HOST_CMD_ANTIPKT_NOTIFY:
            if len(payload) >= ANTIPKT_ID_SIZE:
                source_node, creation_time, seq = struct.unpack_from(ANTIPKT_ID_FMT, payload)
                self._apply_antipkt(source_node, creation_time, seq)

        elif cmd == HOST_CMD_BUNDLE_PUSH:
            b = unpack_bundle(payload)
            if b:
                b['added_at_ms'] = int(_monotonic() * 1000)
                added = self._add_bundle(b)
                if added:
                    print("[PUSH] Neighbor bundle stored: src:{} seq:{}".format(
                        b['source_node'], b['sequence_number']))

        elif cmd == HOST_CMD_ACK:
            status = bytearray(payload)[0] if payload else 1
            print("[ACK] status={}".format('ok' if status == 0 else 'error/full'))

        elif cmd == HOST_CMD_STATUS_RESP:
            self._print_status(payload)

        else:
            print("[WARN] Unknown cmd 0x{:02x} len={}".format(cmd, len(payload)))

    def _print_status(self, payload):
        if len(payload) < STATUS_SIZE:
            return
        node_id, active_peers, store_used, store_max, channel = struct.unpack_from(STATUS_FMT, payload)
        jetson_store = len(self._store)
        print("[STATUS] ESP32 store={}/{}  peers={}  ch={}  jetson_store={}".format(
            store_used, store_max, active_peers, channel, jetson_store))

    def _poll_status(self):
        self._send(HOST_CMD_QUERY_STATUS)
        # response arrives asynchronously via _dispatch -> _print_status

    # bundle generator
    def _generate_bundle(self):
        abs_ms        = int(_monotonic() * 1000)
        creation_time = abs_ms - self._start_ms  # daemon-relative: same clock domain as ESP32 now_ms()
        payload = json.dumps({
            'ts':     int(time.time()),
            'node':   self._node_id,
            'sensor': round(20.0 + (creation_time % 1000) / 100.0, 1),
        }).encode('utf-8')
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
        if self._add_bundle(b):
            print("[GEN] Generated bundle seq:{}".format(b['sequence_number']))

    # main loop
    def run(self):
        print("[INIT] Opening {} at {} baud".format(self._port, self._baud))
        self._ser          = serial.Serial()
        self._ser.port     = self._port
        self._ser.baudrate = self._baud
        self._ser.timeout  = 0.1
        self._ser.xonxoff  = False
        self._ser.dsrdtr   = False
        self._ser.rtscts   = False
        self._ser.dtr      = False
        self._ser.rts      = False
        self._ser.open()

        # On Linux, disable HUPCL so the kernel never toggles DTR on open/close,
        # then force DTR low and flush any boot garbage. No-op on Windows.
        if platform.system() != 'Windows':
            import termios
            attrs = termios.tcgetattr(self._ser.fd)
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(self._ser.fd, termios.TCSANOW, attrs)
        self._ser.dtr = False
        self._ser.rts = False
        self._ser.reset_input_buffer()
        time.sleep(3.0)  # ESP32-C5 native USB JTAG re-enumerates ~1.5-2s after reset

        if self._node_id is None:
            self._node_id = self._detect_node_id()
        print("[INIT] Running as node {}".format(self._node_id))

        last_gen    = _monotonic()
        last_status = _monotonic()
        STATUS_INTERVAL_S = 30.0

        while True:
            # Receive one frame (non-blocking style via short timeout)
            result = recv_frame(self._ser, timeout_s=0.1, verbose=True)
            if result is not None:
                cmd, payload = result
                print("[RX] cmd=0x{:02x} len={}".format(cmd, len(payload)))
                self._dispatch(cmd, payload)

            # Periodic bundle generation
            if self._interval > 0 and (_monotonic() - last_gen) >= self._interval:
                last_gen = _monotonic()
                self._generate_bundle()

            # Periodic ESP32 store status poll
            if (_monotonic() - last_status) >= STATUS_INTERVAL_S:
                last_status = _monotonic()
                self._poll_status()


def main():
    parser = argparse.ArgumentParser(description='Jetson rover node daemon')
    parser.add_argument('--serial',   default='/dev/ttyUSB0', help='Serial port to ESP32')
    parser.add_argument('--baud',     type=int, default=115200, help='Baud rate')
    parser.add_argument('--interval', type=float, default=1.0,
                        help='Bundle generation interval in seconds (0 = disable)')
    parser.add_argument('--node-id',  type=int, default=None,
                        help='Override node ID (default: auto-detect from ESP32)')
    args = parser.parse_args()

    daemon = JetsonDaemon(
        serial_port=args.serial,
        baud=args.baud,
        gen_interval=args.interval,
        node_id_override=args.node_id,
    )
    try:
        daemon.run()
    except KeyboardInterrupt:
        print('\n[INIT] Shutting down.')


if __name__ == '__main__':
    main()
