"""
Jetson node daemon, runs on a rover Jetson Orin Nano.
Manages the local bundle store and bridges it to the attached ESP32 modem via UART.

Usage:
    python jetson_daemon.py [--serial PORT] [--baud BAUD] [--interval SECS] [--node-id ID]

    --serial   Serial port connected to ESP32 (default: /dev/ttyTHS1 for Jetson UART1)
    --baud     Baud rate (default: 115200)
    --interval Seconds between generated sensor bundles (default: 10, 0 = disable)
    --node-id  Override node ID (default: auto-detected from ESP32 STATUS_RESP on startup)
"""

import argparse
import itertools
import json
import platform
import struct
import threading
import time
import serial
from typing import Callable, Optional, Tuple

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

RECONNECT_DELAY_S  = 5.0   # seconds between reconnect attempts
SERIAL_SETTLE_S    = 3.0   # seconds after open before talking to ESP32
WRITE_TIMEOUT_S    = 2.0   # serial write timeout - raises SerialTimeoutException on hang
STATUS_INTERVAL_S  = 5.0   # how often to poll ESP32 status (shorter = faster disconnect detection)


# CRC-16/CCITT (poly 0x1021, init 0xFFFF)
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

# frame encode / decode
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
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return buf
        chunk = ser.read(n - len(buf))
        buf += chunk
    return buf


def recv_frame(ser: serial.Serial, timeout_s: float = 1.0, verbose: bool = False,
               on_ascii: Optional[Callable[[str], None]] = None) -> Optional[Tuple[int, bytes]]:
    """Block until a valid frame arrives or timeout. Returns (cmd, payload) or None.

    on_ascii: called with decoded text whenever a run of non-SOF bytes is flushed.
              Use this to scan for ESP32 boot banners without discarding other frames.
    """
    deadline = time.monotonic() + timeout_s
    skipped = bytearray()

    def _flush_skipped() -> None:
        if skipped:
            text = bytes(skipped).decode('ascii', errors='replace')
            print(f"[RECV] Skipped {len(skipped)} non-SOF bytes")
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
            print(f"[RECV] Incomplete header after SOF (got {len(header)}/3 bytes)")
            return None
        cmd = header[0]
        plen = header[1] | (header[2] << 8)
        if plen > HOST_MAX_PAYLOAD:
            print(f"[RECV] Oversized payload claim {plen} > {HOST_MAX_PAYLOAD}, discarding")
            continue
        payload = _read_exact(ser, plen, timeout_s=1.0) if plen else b''
        crc_bytes = _read_exact(ser, 2, timeout_s=0.2)
        if len(crc_bytes) < 2:
            print(f"[RECV] Truncated CRC (got {len(crc_bytes)}/2 bytes)")
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

# Bundle serialization
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
    # BUNDLE_PUSH frames may be shorter than BUNDLE_SIZE (only header + actual payload).
    # The minimum valid size is the header without any payload bytes.
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
class JetsonDaemon:
    def __init__(self, serial_port: str, baud: int, gen_interval: float, node_id_override: Optional[int]):
        assert BUNDLE_SIZE == 1052, f"Bundle struct size mismatch: got {BUNDLE_SIZE}, expected 1052"

        self._port             = serial_port
        self._baud             = baud
        self._interval         = gen_interval
        self._node_id_override = node_id_override        # None = auto-detect
        self._node_id          = node_id_override        # working copy, may be reset on restart

        self._store: list[dict] = []
        self._lock = threading.Lock()
        self._seq  = itertools.count(start=1)
        self._start_ms = int(time.monotonic() * 1000)

        self._delivered: set[tuple] = set()

        self._detected_node_id: Optional[int] = None
        self._restart_pending = False                    # set by _handle_ascii on boot banner

        self._ser: Optional[serial.Serial] = None
        self._uart_lock = threading.Lock()

    # serial helpers
    def _send(self, cmd: int, payload: bytes = b'') -> None:
        frame = encode_frame(cmd, payload)
        print(f"[TX] cmd=0x{cmd:02x} len={len(payload)}")
        with self._uart_lock:
            self._ser.write(frame)

    def _recv(self, timeout_s: float = 1.0):
        with self._uart_lock:
            return recv_frame(self._ser, timeout_s, verbose=True)

    # bundle store
    def _add_bundle(self, b: dict) -> bool:
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
        print(f"[STORE] Added src:{b['source_node']} seq:{b['sequence_number']} "
              f"| store size: {len(self._store)}")
        return True

    def _expire_bundles(self) -> None:
        now = int(time.monotonic() * 1000)
        with self._lock:
            before = len(self._store)
            self._store = [
                b for b in self._store
                if (now - b.get('added_at_ms', now)) < b.get('lifetime', BUNDLE_DEFAULT_LIFETIME)
            ]
            expired = before - len(self._store)
        if expired:
            print(f"[STORE] Expired {expired} bundles by TTL")

    def _apply_antipkt(self, source_node: int, creation_time: int, seq: int) -> None:
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
            print(f"[ANTIPKT] Cleared src:{source_node} seq:{seq} from store")
        else:
            print(f"[ANTIPKT] src:{source_node} seq:{seq} (not in store, already clear)")

    # boot-banner detector - called with ASCII text chunks skipped by recv_frame
    def _handle_ascii(self, text: str) -> None:
        if 'I am' in text or 'rst:0x' in text:
            print(f"[RESTART] Boot banner detected: {text.strip()!r}")
            self._restart_pending = True
            if self._node_id_override is None:
                self._node_id = None   # will be re-detected in _main_loop

    # serial lifecycle
    def _open_serial(self) -> None:
        """Open (or reopen) the serial port, closing any existing connection first."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

        ser = serial.Serial()
        ser.port     = self._port
        ser.baudrate = self._baud
        ser.timeout       = 0.1
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
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(ser.fd, termios.TCSANOW, attrs)
        ser.dtr = False
        ser.rts = False
        ser.reset_input_buffer()

        self._ser = ser
        time.sleep(SERIAL_SETTLE_S)

    # startup: detect node ID from ESP32 STATUS_RESP
    def _detect_node_id(self) -> int:
        for attempt in range(5):
            frame = encode_frame(HOST_CMD_QUERY_STATUS)
            print(f"[INIT] Attempt {attempt+1}/5: sending QUERY_STATUS {frame.hex()}")
            try:
                with self._uart_lock:
                    self._ser.write(frame)
            except serial.SerialException as e:
                print(f"[INIT] Write failed ({e}), waiting for device to settle...")
                time.sleep(2.0)
                self._ser.reset_input_buffer()
                continue
            deadline = time.monotonic() + 2.0
            found = False
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                result = recv_frame(self._ser, timeout_s=max(0.05, remaining), verbose=True,
                                    on_ascii=self._handle_ascii)
                if result is None:
                    break
                cmd, payload = result
                if cmd == HOST_CMD_STATUS_RESP and len(payload) >= STATUS_SIZE:
                    node_id = struct.unpack_from('<H', payload)[0]
                    print(f"[INIT] Auto-detected node_id={node_id}")
                    return node_id
                print(f"[INIT] Got cmd=0x{cmd:02x} while waiting for STATUS_RESP, dispatching")
                self._dispatch(cmd, payload)
                if self._detected_node_id is not None:
                    print(f"[INIT] Auto-detected node_id={self._detected_node_id} (via dispatch)")
                    return self._detected_node_id
            print(f"[INIT] No STATUS_RESP in 2s window, retrying...")
            time.sleep(1)
        raise RuntimeError("Could not auto-detect node_id from ESP32. Use --node-id to override.")

    # PULL_REQ handler: drain local store to ESP32
    def _handle_pull_req(self) -> None:
        self._expire_bundles()
        with self._lock:
            pending = list(self._store)
        if not pending:
            self._send(HOST_CMD_NO_BUNDLES)
            return
        sent_bundles: list[dict] = []
        handed_off: list[dict] = []
        skipped = 0
        for b in pending:
            key = (b['source_node'], b['creation_time'], b['sequence_number'])
            if key in self._delivered:
                skipped += 1
                continue
            data = pack_bundle(b)
            self._send(HOST_CMD_BUNDLE_DATA, data)
            sent_bundles.append(b)
            deadline = time.monotonic() + 2.0
            got_pull = False
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                nxt = recv_frame(self._ser, timeout_s=max(0.05, remaining),
                                 on_ascii=self._handle_ascii)
                if nxt is None:
                    break
                if nxt[0] == HOST_CMD_PULL_REQ:
                    got_pull = True
                    break
                self._dispatch(nxt[0], nxt[1])
            if got_pull:
                handed_off.append(b)
            else:
                break
        self._send(HOST_CMD_NO_BUNDLES)

        if handed_off:
            remove_keys = {(b['source_node'], b['creation_time'], b['sequence_number'])
                           for b in handed_off}
            with self._lock:
                before = len(self._store)
                self._store = [b for b in self._store
                               if (b['source_node'], b['creation_time'], b['sequence_number'])
                               not in remove_keys]
            print(f"[PULL] Cleared {before - len(self._store)} handed-off bundles "
                  f"(store now {len(self._store)})")

        by_src: dict[int, int] = {}
        for b in sent_bundles:
            by_src[b['source_node']] = by_src.get(b['source_node'], 0) + 1
        src_summary = ' '.join(f"src{n}×{c}" for n, c in sorted(by_src.items()))
        skip_str = f" skip:{skipped}" if skipped else ""
        print(f"[PULL] Sent {len(sent_bundles)}/{len(pending)} bundles to ESP32{skip_str}  [{src_summary}]")

    # frame dispatcher
    def _dispatch(self, cmd: int, payload: bytes) -> None:
        if cmd == HOST_CMD_PULL_REQ:
            self._handle_pull_req()

        elif cmd == HOST_CMD_ANTIPKT_NOTIFY:
            if len(payload) >= ANTIPKT_ID_SIZE:
                source_node, creation_time, seq = struct.unpack_from(ANTIPKT_ID_FMT, payload)
                self._apply_antipkt(source_node, creation_time, seq)

        elif cmd == HOST_CMD_BUNDLE_PUSH:
            b = unpack_bundle(payload)
            if b:
                b['added_at_ms'] = int(time.monotonic() * 1000)
                added = self._add_bundle(b)
                if added:
                    print(f"[PUSH] Neighbor bundle stored: src:{b['source_node']} seq:{b['sequence_number']}")

        elif cmd == HOST_CMD_ACK:
            status = payload[0] if payload else 1
            print(f"[ACK] status={'ok' if status == 0 else 'error/full'}")

        elif cmd == HOST_CMD_STATUS_RESP:
            self._print_status(payload)
            if len(payload) >= STATUS_SIZE:
                self._detected_node_id = struct.unpack_from('<H', payload)[0]

        else:
            print(f"[WARN] Unknown cmd 0x{cmd:02x} len={len(payload)}")

    def _print_status(self, payload: bytes) -> None:
        if len(payload) < STATUS_SIZE:
            return
        node_id, active_peers, store_used, store_max, channel = struct.unpack_from(STATUS_FMT, payload)
        jetson_store = len(self._store)
        print(f"[STATUS] ESP32 store={store_used}/{store_max}  "
              f"peers={active_peers}  ch={channel}  "
              f"jetson_store={jetson_store}")

    def _poll_status(self) -> None:
        self._send(HOST_CMD_QUERY_STATUS)

    # bundle generator
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
        if self._add_bundle(b):
            print(f"[GEN] Generated bundle seq:{b['sequence_number']}")

    # inner event loop - exits only by raising (SerialException, RuntimeError, etc.)
    def _main_loop(self) -> None:
        last_gen    = time.monotonic()
        last_status = time.monotonic()

        while True:
            # Re-sync state after an ESP32 restart detected via boot banner
            if self._restart_pending:
                self._restart_pending = False
                with self._lock:
                    dropped = len(self._store)
                    self._store.clear()
                self._delivered.clear()
                print(f"[RESTART] ESP32 restarted - cleared {dropped} bundles and antipacket set")
                if self._node_id is None:
                    self._node_id = self._detect_node_id()
                    print(f"[RESTART] Re-detected node_id={self._node_id}")

            result = recv_frame(self._ser, timeout_s=0.1, verbose=True,
                                on_ascii=self._handle_ascii)
            if result is not None:
                cmd, payload = result
                print(f"[RX] cmd=0x{cmd:02x} len={len(payload)}")
                self._dispatch(cmd, payload)

            if self._interval > 0 and (time.monotonic() - last_gen) >= self._interval:
                last_gen = time.monotonic()
                self._generate_bundle()

            if (time.monotonic() - last_status) >= STATUS_INTERVAL_S:
                last_status = time.monotonic()
                self._poll_status()

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
        with self._lock:
            dropped = len(self._store)
            self._store.clear()
        self._delivered.clear()
        if dropped:
            print(f"[RECONNECT] Cleared {dropped} bundles from store")
        print(f"[RECONNECT] Retrying in {RECONNECT_DELAY_S:.0f}s...")

    def _connect_and_run(self) -> None:
        """Open the port, detect node ID, then run the event loop. Raises on any failure."""
        print(f"[INIT] Opening {self._port} at {self._baud} baud")
        self._open_serial()
        if self._node_id is None:
            self._node_id = self._detect_node_id()
        print(f"[INIT] Running as node {self._node_id}")
        self._main_loop()

    # outer loop - reconnects forever; exits only on Ctrl+C
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
            # sleep separately so Ctrl+C here also exits cleanly
            try:
                time.sleep(RECONNECT_DELAY_S)
            except KeyboardInterrupt:
                print('\n[INIT] Shutting down.')
                return


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
    daemon.run()


if __name__ == '__main__':
    main()
