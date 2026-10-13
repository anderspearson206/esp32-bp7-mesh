import argparse
import csv
import json
import os
import struct
import serial
import threading
import time
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque, defaultdict
from datetime import datetime

SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200
TIMEOUT_SECONDS = 15
RADIO_MAP_HALF_M = 75.0   # half-width of the radio map in metres (map spans ±this value)
REPORT_DIR = "bs_reports"
CONTACT_GAP_S = 3.0       # no @NET from a rover for this long -> its contact with the BS has ended
TIMELINE_S = 180          # width of the live timeline panel (seconds)
TIMELINE_BIN_S = 5        # throughput bin width on the timeline

G = nx.DiGraph()
recent_transfers = []
lock = threading.Lock()
node_latencies      = defaultdict(lambda: deque(maxlen=20))  # rolling avg for live display
event_log           = deque(maxlen=20)
delivered_bundles   = {}                 # (source_node_str, seq_int) -> {latency_ms, hops, time}
antipkt_fail_counts = defaultdict(int)   # source_node_str -> count  (excluded from delivery totals)
ferry_dup_counts    = defaultdict(int)   # source_node_str -> count  (counted as deliveries)
node_rssi           = defaultdict(list)  # source_node_str -> [rssi_int, ...]  (from @NET:)
node_seq_range      = {}                 # source_node_str -> {'min': int, 'max': int}
node_boot_count     = defaultdict(int)   # node_id -> number of restarts seen this session
node_boot_history   = defaultdict(list)  # node_id -> list of archived boot records
node_boot_start_time = {}                # node_id -> time.time() of first activity in current boot
node_locations       = {}                # node_id -> {lat, lon, alt, rssi, updated_at}
radio_map_samples    = defaultdict(lambda: deque(maxlen=2000))  # node_id -> deque of (x, y, rssi)
start_time           = time.time()
_coverage_csv_path: str = ""

# experiment logging / comms tracking
delivery_timeline = deque(maxlen=20000)  # (time, src, kind 'data'|'tel', latency_ms)
link_state        = {}                   # node_id -> {'up', 'since', 'last', 'data', 'tel', 'rssi': [...]}
contact_history   = defaultdict(list)    # node_id -> [{'start', 'end', 'data', 'tel', 'avg_rssi'}]
rover_status      = {}                   # node_id -> latest @ROVER_STATUS fields + 'updated_at'
markers           = []                   # [(time, label)]
_pending_metric   = None                 # last @METRIC awaiting classification by the next line
session_dir: str  = ""
_serial_log       = None


def _ts(t: float) -> str:
    return datetime.fromtimestamp(t).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


def _csv_append(name: str, header: list, row: list) -> None:
    """Append a row to <session_dir>/<name>, writing the header when the file is new."""
    if not session_dir:
        return
    path = os.path.join(session_dir, name)
    try:
        new = not os.path.exists(path)
        with open(path, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(header)
            w.writerow(row)
    except OSError:
        pass


def _log_serial(line: str) -> None:
    if _serial_log is not None:
        try:
            _serial_log.write(f"{_ts(time.time())}\t{line}\n")
        except (OSError, ValueError):
            pass


def _init_session_files(tag: str = "") -> None:
    """Each session (visualizer start, BS reboot, reset key) gets its own folder holding
    the raw serial log, CSVs and summaries, so runs never mix."""
    global _coverage_csv_path, session_dir, _serial_log
    session_dir = os.path.join(REPORT_DIR, f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                                           + (f"_{tag}" if tag else ""))
    os.makedirs(session_dir, exist_ok=True)
    _coverage_csv_path = os.path.join(session_dir, "coverage.csv")
    with open(_coverage_csv_path, 'w', newline='') as f:
        csv.writer(f).writerow(['timestamp', 'node_id', 'lat', 'lon', 'alt_m', 'rssi_to_bs_dBm'])
    if _serial_log is not None:
        try:
            _serial_log.close()
        except OSError:
            pass
    _serial_log = open(os.path.join(session_dir, "serial.log"), 'a', buffering=1)
    print(f"[VISUALIZER] Session folder: {session_dir}")


def _append_coverage_row(node_id: str, lat: float, lon: float, alt: float, rssi) -> None:
    if not _coverage_csv_path:
        return
    try:
        with open(_coverage_csv_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                node_id, f"{lat:.6f}", f"{lon:.6f}", f"{alt:.1f}",
                rssi if rssi is not None else '',
            ])
    except OSError:
        pass


def _record_delivery(src: str, seq: int, hops: int, latency_ms: int, t: float,
                     kind: str, via: str, label: str) -> None:
    """Called once an @METRIC has been classified as data or telemetry. Caller holds lock."""
    info = delivered_bundles.get((src, seq))
    if info is not None:
        info['kind'] = kind
    delivery_timeline.append((t, src, kind, latency_ms))
    ls = link_state.get(via) or link_state.get(src)
    if ls is not None and ls['up']:
        ls[kind] += 1
    _csv_append("deliveries.csv",
                ['time', 'epoch', 'src', 'seq', 'kind', 'via', 'hops', 'latency_ms', 'label'],
                [_ts(t), f"{t:.3f}", src, seq, kind, via, hops, latency_ms, label])


def _resolve_pending_metric(line: str) -> None:
    """The BS prints @METRIC then, for the same bundle, either @DTN_RX (data bundle) or the
    telemetry payload text. Classify the pending @METRIC from the line that follows it."""
    global _pending_metric
    if _pending_metric is None:
        return
    src, seq, hops, lat, t = _pending_metric
    _pending_metric = None
    parts = line.split(':')
    if line.startswith("@DTN_RX:") and len(parts) >= 4 and parts[1] == src and parts[3] == str(seq):
        _record_delivery(src, seq, hops, lat, t, 'data', parts[2], '')
    else:
        _record_delivery(src, seq, hops, lat, t, 'tel', src, parts[0] if line.startswith('@') else '')


def _contact_update(node_id: str, rssi, now: float) -> None:
    """A rover was heard by the BS (@NET). Opens a contact if the link was down. Caller holds lock."""
    ls = link_state.get(node_id)
    if ls is None or not ls['up']:
        gap = (now - ls['last']) if ls else None
        link_state[node_id] = {'up': True, 'since': now, 'last': now,
                               'data': 0, 'tel': 0, 'rssi': []}
        ls = link_state[node_id]
        print(f"[CONTACT] {node_id} link UP" + (f" after {gap:.1f}s down" if gap else ""))
        _log_serial(f"#CONTACT_UP:{node_id}")
    ls['last'] = now
    try:
        ls['rssi'].append(int(rssi))
    except (TypeError, ValueError):
        pass


def _close_contact(node_id: str, ls: dict, ongoing: bool = False) -> dict:
    rec = {'start': ls['since'], 'end': ls['last'], 'data': ls['data'], 'tel': ls['tel'],
           'avg_rssi': sum(ls['rssi']) / len(ls['rssi']) if ls['rssi'] else None}
    if not ongoing:
        contact_history[node_id].append(rec)
        dur = rec['end'] - rec['start']
        print(f"[CONTACT] {node_id} link DOWN after {dur:.1f}s ({rec['data']} data delivered)")
        _log_serial(f"#CONTACT_DOWN:{node_id}:{dur:.1f}")
        _csv_append("contacts.csv",
                    ['node', 'start', 'end', 'duration_s', 'data_delivered', 'tel_delivered',
                     'avg_rssi', 'start_epoch', 'end_epoch'],
                    [node_id, _ts(rec['start']), _ts(rec['end']), f"{dur:.1f}", rec['data'],
                     rec['tel'], f"{rec['avg_rssi']:.1f}" if rec['avg_rssi'] is not None else '',
                     f"{rec['start']:.3f}", f"{rec['end']:.3f}"])
    return rec


def _check_contacts(now: float) -> None:
    with lock:
        for nid, ls in link_state.items():
            if ls['up'] and now - ls['last'] > CONTACT_GAP_S:
                ls['up'] = False
                _close_contact(nid, ls)


def add_marker(label: str = "") -> None:
    now = time.time()
    with lock:
        label = label or f"M{len(markers) + 1}"
        markers.append((now, label))
    print(f"[MARKER] {label} at {_ts(now)}")
    _log_serial(f"#MARKER:{label}")
    _csv_append("markers.csv", ['time', 'epoch', 'label'], [_ts(now), f"{now:.3f}", label])


def read_line_or_frame(ser):
    """Return one stripped ASCII line, skipping binary ESP32 UART frames (SOF=0xAA).
    Returns None on timeout or empty input."""
    buf = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            return None
        byte = b[0]
        if not buf and byte == 0xAA:
            # binary frame, read cmd + len (3 bytes), then skip payload + CRC
            header = ser.read(3)
            if len(header) == 3:
                plen = header[1] | (header[2] << 8)
                if plen <= 1060:
                    ser.read(plen + 2)
            continue
        buf.append(byte)
        if byte == ord('\n'):
            return buf.decode('utf-8', errors='ignore').strip() or None


def reset_session_state(tag: str = ""):
    """Wipe all per-session tracking state and restart the session clock.
    Always save the current summary before calling this."""
    global start_time
    with lock:
        G.clear()
        recent_transfers.clear()
        node_latencies.clear()
        event_log.clear()
        delivered_bundles.clear()
        antipkt_fail_counts.clear()
        ferry_dup_counts.clear()
        node_rssi.clear()
        node_seq_range.clear()
        node_boot_count.clear()
        node_boot_history.clear()
        node_boot_start_time.clear()
        node_locations.clear()
        radio_map_samples.clear()
        delivery_timeline.clear()
        link_state.clear()
        contact_history.clear()
        rover_status.clear()
        markers.clear()
        start_time = time.time()
    _init_session_files(tag)
    print("[VISUALIZER] Session state cleared - starting fresh.")


def save_boot_summary(node_id: str, boot_num: int, boot_record: dict) -> None:
    """Write a summary file for one completed boot of a single node."""
    now = datetime.now()
    duration_s = boot_record.get('duration_s', 0)
    lats = boot_record.get('all_latencies', [])
    delivered = boot_record.get('delivered', {})
    seq_range = boot_record.get('seq_range')
    ferry_dups = boot_record.get('ferry_dups', 0)
    ap_fails = boot_record.get('antipkt_fails', 0)
    avg_rssi = boot_record.get('avg_rssi')

    unique_count = len(delivered)
    h, rem = divmod(duration_s, 3600)
    m, s = divmod(rem, 60)
    dur_str = f"{h}h {m:02d}m {s:02d}s" if h else f"{m}m {s:02d}s"

    W = 58
    lines = []
    lines.append("=" * W)
    lines.append(f"  DTN Mesh Network  -  Node {node_id} Boot {boot_num} Summary")
    lines.append(f"  Saved    : {now.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"  Duration : {dur_str}")
    lines.append("=" * W)
    lines.append("")

    lines.append("Deliveries")
    lines.append(f"  Unique delivered    : {unique_count}")
    lines.append(f"  Ferry duplicates    : {ferry_dups}")
    lines.append(f"  Antipacket failures : {ap_fails}")
    lines.append("")

    lines.append("Latency")
    if lats:
        lines.append(f"  Average : {sum(lats)/len(lats):.0f} ms")
        lines.append(f"  Min     : {min(lats)} ms")
        lines.append(f"  Max     : {max(lats)} ms")
    else:
        lines.append("  No data")
    lines.append("")

    lines.append("RSSI")
    lines.append(f"  Average : {avg_rssi:.1f} dBm" if avg_rssi is not None else "  No data")
    lines.append("")

    if seq_range:
        lines.append(f"Sequence Range : [{seq_range['min']}..{seq_range['max']}]")
        possible = seq_range['max'] - seq_range['min'] + 1
        undelivered = max(0, possible - unique_count)
        pct = 100.0 * unique_count / possible if possible > 0 else 0.0
        lines.append("Delivery Estimate")
        lines.append(f"  Possible    : {possible}")
        lines.append(f"  Delivered   : {unique_count}")
        lines.append(f"  Undelivered : {undelivered}")
        lines.append(f"  Rate        : {pct:.1f}%")
    else:
        lines.append("Sequence Range : no data")
    lines.append("")
    lines.append("=" * W)

    report = "\n".join(lines)
    print(f"\n{report}")

    os.makedirs(session_dir or REPORT_DIR, exist_ok=True)
    filename = os.path.join(session_dir or REPORT_DIR, f"node_{node_id}_boot{boot_num}.txt")
    try:
        with open(filename, 'w') as f:
            f.write(report + "\n")
        print(f"[Boot summary saved to {filename}]")
    except OSError as e:
        print(f"[Could not save boot summary: {e}]")


def serial_reader_thread(port):
    """Read serial data from the BS port and update shared state."""
    global _pending_metric, _ser
    first_connect = True
    _init_session_files()
    while True:
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=1)
            ser.dtr = False
            ser.rts = False
            _ser = ser
            if not first_connect:
                # BS came back after a disconnect, save old session and reset
                print(f"[VISUALIZER] Reconnected to {port} - saving session and resetting state.")
                generate_summary()
                reset_session_state()
            first_connect = False
            print(f"Connected to {port}. Listening for DTN telemetry...")

            while True:
                line = read_line_or_frame(ser)
                _check_contacts(time.time())
                _maybe_send_control()
                if not line:
                    continue
                _log_serial(line)
                if "unknown cmd 0x30" in line and not control['bs_rejects']:
                    control['bs_rejects'] = True
                    print("[CONTROL] *** The BS firmware rejected a control frame (unknown cmd 0x30). "
                          "It is running old firmware: close the visualizer, build and flash the BS. ***")

                # Detect BS reboot via its boot-time log line (covers software resets
                # where the serial connection stays open and no SerialException fires).
                if "I am the Base Station" in line:
                    print(f"[VISUALIZER] BS reboot detected in serial stream - saving and resetting.")
                    generate_summary()
                    reset_session_state()
                    continue

                if line.startswith("@"):
                    print(f"Received: {line} : {datetime.now().strftime('%H:%M:%S')}")
                    if not line.startswith("@NET:"):
                        with lock:
                            _resolve_pending_metric(line)

                # topology heartbeat from rovers, e.g. @NET:node_id:parent_id:rssi
                if line.startswith("@NET:"):
                    parts = line.split(':')
                    if len(parts) >= 4:
                        node_id, parent, rssi = parts[1], parts[2], parts[3]
                        with lock:
                            current_time = time.time()
                            if parent != "0":
                                _contact_update(node_id, rssi, current_time)
                            edges_to_remove = [(u, v) for u, v in G.edges() if u == node_id]
                            G.remove_edges_from(edges_to_remove)
                            G.add_node(node_id, last_seen=current_time)
                            if node_id not in node_boot_start_time:
                                node_boot_start_time[node_id] = current_time
                            if parent != "0":
                                G.add_node(parent)
                                G.add_edge(node_id, parent, link_type='mesh', rssi=rssi)
                                try:
                                    node_rssi[node_id].append(int(rssi))
                                except ValueError:
                                    pass

                # packet recieved
                elif line.startswith("@DTN_RX:"):
                    # Format: @DTN_RX:<source>:<prev_node>:<seq>:<receiver>:<hop_count>
                    parts = line.split(':')
                    if len(parts) >= 6:
                        source, ferry, seq, receiver = parts[1], parts[2], parts[3], parts[4]
                        try:
                            hops = int(parts[5])
                        except ValueError:
                            hops = 1
                    elif len(parts) >= 5:
                        source, ferry, seq, receiver = parts[1], parts[2], parts[3], parts[4]
                        hops = 1
                    else:
                        continue
                    with lock:
                        current_time = time.time()
                        G.add_node(source,   last_seen=current_time)
                        G.add_node(ferry,    last_seen=current_time)
                        G.add_node(receiver, last_seen=current_time)
                        is_ferry = (ferry != source) or (hops > 1)
                        recent_transfers.append({
                            'src': source, 'ferry': ferry, 'dst': receiver,
                            'hops': hops, 'is_ferry': is_ferry, 'time': current_time,
                        })
                        event_log.append({
                            'time': current_time, 'src': source, 'ferry': ferry,
                            'seq': seq, 'hops': hops, 'is_ferry': is_ferry,
                        })

                # latency tracking
                elif line.startswith("@METRIC:"):
                    # Format: @METRIC:<source_node>:<seq_num>:<hop_count>:<latency_ms>
                    parts = line.split(':')
                    if len(parts) >= 5:
                        source_node = parts[1]
                        try:
                            seq_num    = int(parts[2])
                            hops_val   = int(parts[3])
                            latency_ms = int(parts[4])
                            if 0 <= latency_ms < 300000:
                                with lock:
                                    node_latencies[source_node].append(latency_ms)
                                    delivered_bundles[(source_node, seq_num)] = {
                                        'latency_ms': latency_ms,
                                        'hops':       hops_val,
                                        'time':       time.time(),
                                        'kind':       'tel',   # upgraded to 'data' by the next line
                                    }
                                    _pending_metric = (source_node, seq_num, hops_val,
                                                       latency_ms, time.time())
                                    if source_node not in node_seq_range:
                                        node_seq_range[source_node] = {'min': seq_num, 'max': seq_num}
                                    else:
                                        r = node_seq_range[source_node]
                                        if seq_num < r['min']: r['min'] = seq_num
                                        if seq_num > r['max']: r['max'] = seq_num
                        except ValueError:
                            pass

                # node restarted (boot_id changed): reset per-node metrics so stale sequence
                # data from before the reboot doesn't corrupt delivery rate estimates
                elif line.startswith("@NODE_RESTART:"):
                    parts = line.split(':')
                    if len(parts) >= 2:
                        node_id = parts[1]
                        with lock:
                            archived_boot = node_boot_count[node_id]
                            now_t = time.time()
                            boot_start = node_boot_start_time.get(node_id, start_time)
                            duration_s = int(now_t - boot_start)
                            lats = list(node_latencies.get(node_id, []))
                            rssi_vals = list(node_rssi.get(node_id, []))
                            boot_record = {
                                'boot':           archived_boot,
                                'duration_s':     duration_s,
                                'seq_range':      dict(node_seq_range[node_id]) if node_id in node_seq_range else None,
                                'delivered':      {seq: info for (src, seq), info in delivered_bundles.items() if src == node_id},
                                'antipkt_fails':  antipkt_fail_counts.get(node_id, 0),
                                'ferry_dups':     ferry_dup_counts.get(node_id, 0),
                                'avg_rssi':       sum(rssi_vals) / len(rssi_vals) if rssi_vals else None,
                                'avg_latency':    sum(lats) / len(lats) if lats else None,
                                'all_latencies':  lats,
                            }
                            node_boot_history[node_id].append(boot_record)
                            node_boot_count[node_id] += 1
                            new_boot = node_boot_count[node_id]
                            node_latencies.pop(node_id, None)
                            node_seq_range.pop(node_id, None)
                            node_rssi.pop(node_id, None)
                            antipkt_fail_counts.pop(node_id, None)
                            ferry_dup_counts.pop(node_id, None)
                            stale_keys = [k for k in delivered_bundles if k[0] == node_id]
                            for k in stale_keys:
                                del delivered_bundles[k]
                            node_boot_start_time[node_id] = now_t
                            node_locations.pop(node_id, None)
                            radio_map_samples.pop(node_id, None)
                        print(f"[VISUALIZER] Node {node_id} restarted - boot {archived_boot} archived, now on boot {new_boot}.")
                        save_boot_summary(node_id, archived_boot, boot_record)

                # antipacket failure, same forwarder re-sent the bundle, excluded from delivery totals
                elif line.startswith("@ANTIPKT_FAIL:"):
                    # Format: @ANTIPKT_FAIL:<src>:<seq>:<prev>
                    parts = line.split(':')
                    if len(parts) >= 3:
                        with lock:
                            antipkt_fail_counts[parts[1]] += 1

                # ferry duplicate, same bundle from a second node
                elif line.startswith("@FERRY_DUP:"):
                    # Format: @FERRY_DUP:<src>:<seq>:<first_prev>:<dup_prev>
                    parts = line.split(':')
                    if len(parts) >= 3:
                        with lock:
                            ferry_dup_counts[parts[1]] += 1

                # rover location + BS RSSI at that position
                # Format: @LOCATION:<node>:<lat>:<lon>:<alt>[:<rssi>]
                elif line.startswith("@LOCATION:"):
                    parts = line.split(':')
                    if len(parts) >= 5:
                        try:
                            node_id = parts[1]
                            lat  = float(parts[2])
                            lon  = float(parts[3])
                            alt  = float(parts[4])
                            rssi = int(parts[5]) if len(parts) >= 6 and parts[5] else None
                            with lock:
                                node_locations[node_id] = {
                                    'lat': lat, 'lon': lon, 'alt': alt,
                                    'rssi': rssi, 'updated_at': time.time(),
                                }
                                if rssi is not None:
                                    radio_map_samples[node_id].append((lat, lon, rssi))
                            if lat != 0.0 or lon != 0.0:
                                _append_coverage_row(node_id, lat, lon, alt, rssi)
                        except (ValueError, IndexError):
                            pass

                # rover daemon status: @ROVER_STATUS:<node>:k=v,k=v,...
                elif line.startswith("@ROVER_STATUS:"):
                    parts = line.split(':', 2)
                    if len(parts) == 3:
                        fields = {}
                        for kv in parts[2].split(','):
                            k, _, v = kv.partition('=')
                            try:
                                fields[k] = float(v)
                            except ValueError:
                                fields[k] = v
                        now_t = time.time()
                        with lock:
                            fields['updated_at'] = now_t
                            rover_status[parts[1]] = fields
                        keys = sorted(k for k in fields if k != 'updated_at')
                        _csv_append(f"rover_status_{parts[1]}.csv", ['time', 'epoch'] + keys,
                                    [_ts(now_t), f"{now_t:.3f}"] + [fields[k] for k in keys])

        except (serial.SerialException, serial.PortNotOpenError) as e:
            print(f"Connection lost on {port}: {e}. Retrying in 2 seconds...")
            time.sleep(2)
        except Exception as e:
            print(f"Unexpected error on {port}: {e}")
            time.sleep(2)
        finally:
            try:
                ser.close()  # type: ignore[possibly-undefined]
            except Exception:
                pass


def generate_summary():
    """Print and save a full session summary. Called on exit (window close or Ctrl-C)."""
    now = datetime.now()

    with lock:
        duration_s = int(time.time() - start_time)

        # Compute per-node metrics from delivered_bundles (current boot only)
        per_node_seqs = defaultdict(set)
        per_node_lats = defaultdict(list)
        for (src, seq), info in delivered_bundles.items():
            per_node_seqs[src].add(seq)
            per_node_lats[src].append(info['latency_ms'])

        # Aggregate archived boot history into per-node totals
        arch_unique  = defaultdict(int)
        arch_lats    = defaultdict(list)
        arch_fdups   = defaultdict(int)
        arch_apfails = defaultdict(int)
        arch_possible = defaultdict(int)
        for nid, history in node_boot_history.items():
            for rec in history:
                arch_unique[nid]  += len(rec['delivered'])
                arch_lats[nid].extend(rec.get('all_latencies', []))
                arch_fdups[nid]   += rec.get('ferry_dups', 0)
                arch_apfails[nid] += rec.get('antipkt_fails', 0)
                if rec['seq_range']:
                    r = rec['seq_range']
                    arch_possible[nid] += r['max'] - r['min'] + 1

        all_nids = (set(per_node_seqs) | set(arch_unique) | set(ferry_dup_counts)
                    | set(antipkt_fail_counts) | set(node_seq_range))
        agg_unique   = {n: len(per_node_seqs.get(n, set())) + arch_unique[n]  for n in all_nids}
        agg_lats     = {n: list(per_node_lats.get(n, [])) + arch_lats[n]      for n in all_nids}
        agg_fdups    = {n: ferry_dup_counts.get(n, 0)     + arch_fdups[n]     for n in all_nids}
        agg_apfails  = {n: antipkt_fail_counts.get(n, 0)  + arch_apfails[n]   for n in all_nids}
        agg_possible = {n: (node_seq_range[n]['max'] - node_seq_range[n]['min'] + 1
                            if n in node_seq_range else 0) + arch_possible[n]  for n in all_nids}

        total_unique     = sum(agg_unique.values())
        total_ferry_dups = sum(agg_fdups.values())
        total_delivered  = total_unique + total_ferry_dups
        total_ap_fails   = sum(agg_apfails.values())

        all_nodes = sorted(all_nids)

        all_lats_flat = [lat for lats in agg_lats.values() for lat in lats]
        all_rssi_flat = [v for vals in node_rssi.values() for v in vals]

        W = 58
        lines = []
        lines.append("=" * W)
        lines.append("  DTN Mesh Network  -  Session Summary Report")
        lines.append(f"  Generated : {now.strftime('%Y-%m-%d %H:%M:%S')}")
        h, rem = divmod(duration_s, 3600)
        m, s   = divmod(rem, 60)
        dur_str = (f"{h}h {m:02d}m {s:02d}s" if h else f"{m}m {s:02d}s")
        lines.append(f"  Duration  : {dur_str}")
        lines.append("=" * W)
        lines.append("")

        lines.append("Totals")
        lines.append(f"  Unique packets delivered         : {total_unique}")
        lines.append(f"  Total deliveries (+ ferry dups)  : {total_delivered}")
        lines.append(f"  Ferry duplicates                 : {total_ferry_dups}")
        lines.append(f"  Antipacket failures (excluded)   : {total_ap_fails}")
        if all_lats_flat:
            lines.append(f"  Overall avg latency              : {sum(all_lats_flat)/len(all_lats_flat):.0f} ms")
        else:
            lines.append(f"  Overall avg latency              : N/A")
        if all_rssi_flat:
            lines.append(f"  Overall avg RSSI                 : {sum(all_rssi_flat)/len(all_rssi_flat):.1f} dBm")
        else:
            lines.append(f"  Overall avg RSSI                 : N/A")
        lines.append("")

        # per node - all columns are aggregated across all boots
        lines.append("Per Node  (all boots)")
        hdr = (f"  {'Node':<10} {'Uniq':>6} {'Poss':>6} {'Rate':>7} {'Undlv':>6}"
               f" {'FDup':>5} {'APFl':>5} {'AvgLat':>10} {'AvgRSSI':>9}")
        lines.append(hdr)
        lines.append("  " + "-" * (len(hdr) - 2))
        for node in all_nodes:
            unique_count = agg_unique.get(node, 0)
            possible     = agg_possible.get(node, 0)
            undelivered  = max(0, possible - unique_count)
            rate_str     = f"{100.0 * unique_count / possible:.1f}%" if possible > 0 else "N/A"
            ferry_dups   = agg_fdups.get(node, 0)
            ap_fails     = agg_apfails.get(node, 0)
            lats         = agg_lats.get(node, [])
            rssi_vals    = node_rssi.get(node, [])
            lat_str      = f"{sum(lats)/len(lats):.0f} ms"  if lats     else "N/A"
            rssi_str     = f"{sum(rssi_vals)/len(rssi_vals):.1f} dBm" if rssi_vals else "N/A"
            lines.append(
                f"  {node:<10} {unique_count:>6} {possible:>6} {rate_str:>7} {undelivered:>6}"
                f" {ferry_dups:>5} {ap_fails:>5} {lat_str:>10} {rssi_str:>9}"
            )
        lines.append("")

        # per-node restart history
        nodes_with_restarts = [n for n in all_nodes if node_boot_history.get(n)]
        if nodes_with_restarts:
            lines.append("Node Restart History")
            for node in nodes_with_restarts:
                history    = node_boot_history[node]
                cur_boot   = node_boot_count[node]
                lines.append(f"  Node {node} - {len(history)} restart(s); currently boot {cur_boot}")
                for rec in history:
                    r        = rec['seq_range']
                    seq_str  = f"[{r['min']}..{r['max']}]" if r else "no data"
                    n_del    = len(rec['delivered'])
                    lat_str  = f"{rec['avg_latency']:.0f} ms"  if rec['avg_latency'] is not None else "N/A"
                    rssi_str = f"{rec['avg_rssi']:.1f} dBm"    if rec['avg_rssi']    is not None else "N/A"
                    extra    = ""
                    if rec['antipkt_fails']: extra += f"  ap_fail:{rec['antipkt_fails']}"
                    if rec['ferry_dups']:    extra += f"  dup:{rec['ferry_dups']}"
                    lines.append(
                        f"    Boot {rec['boot']}: seq {seq_str}  {n_del} pkts  "
                        f"avg {lat_str}  rssi {rssi_str}{extra}"
                    )
                # Synthesize the current (not-yet-archived) boot if it has live data
                cur_delivered = {seq: info for (src, seq), info in delivered_bundles.items() if src == node}
                cur_seq_range = node_seq_range.get(node)
                if cur_delivered or cur_seq_range:
                    cur_lats     = list(node_latencies.get(node, []))
                    cur_rssi     = list(node_rssi.get(node, []))
                    cur_lat_avg  = sum(cur_lats) / len(cur_lats) if cur_lats else None
                    cur_rssi_avg = sum(cur_rssi) / len(cur_rssi) if cur_rssi else None
                    r        = cur_seq_range
                    seq_str  = f"[{r['min']}..{r['max']}]" if r else "no data"
                    lat_str  = f"{cur_lat_avg:.0f} ms"   if cur_lat_avg  is not None else "N/A"
                    rssi_str = f"{cur_rssi_avg:.1f} dBm" if cur_rssi_avg is not None else "N/A"
                    extra    = ""
                    if antipkt_fail_counts.get(node): extra += f"  ap_fail:{antipkt_fail_counts[node]}"
                    if ferry_dup_counts.get(node):    extra += f"  dup:{ferry_dup_counts[node]}"
                    lines.append(
                        f"    Boot {cur_boot}: seq {seq_str}  {len(cur_delivered)} pkts  "
                        f"avg {lat_str}  rssi {rssi_str}{extra}  [active at exit]"
                    )
            lines.append("")

        # data vs telemetry + comms windows, per node (all boots)
        def _pct(vals, p):
            s = sorted(vals)
            return s[min(len(s) - 1, int(round(p / 100.0 * (len(s) - 1))))]

        all_delivered = [((src, seq), info) for (src, seq), info in delivered_bundles.items()]
        for nid, history in node_boot_history.items():
            for rec in history:
                all_delivered.extend(((nid, seq), info) for seq, info in rec['delivered'].items())
        comm_nodes = sorted(set(all_nodes) | set(contact_history) | set(link_state) | set(rover_status))
        if comm_nodes:
            lines.append("Data vs Telemetry  (data = generated sensor bundles only)")
            hdr = (f"  {'Node':<10} {'Data':>6} {'Tel':>6} {'DataLat':>9} {'p50':>7} {'p95':>7}"
                   f" {'RoverGen':>9} {'Data/Gen':>9}")
            lines.append(hdr)
            lines.append("  " + "-" * (len(hdr) - 2))
            for node in comm_nodes:
                d_lats = [i['latency_ms'] for (s, _q), i in all_delivered
                          if s == node and i.get('kind') == 'data']
                n_tel  = sum(1 for (s, _q), i in all_delivered if s == node and i.get('kind') != 'data')
                st     = rover_status.get(node, {})
                gen    = st.get('gen')
                ratio  = f"{100.0 * len(d_lats) / gen:.1f}%" if gen else "N/A"
                lat_s  = f"{sum(d_lats)/len(d_lats):.0f} ms" if d_lats else "N/A"
                p50    = f"{_pct(d_lats, 50)}" if d_lats else "-"
                p95    = f"{_pct(d_lats, 95)}" if d_lats else "-"
                gen_s  = f"{gen:.0f}" if gen is not None else "N/A"
                lines.append(f"  {node:<10} {len(d_lats):>6} {n_tel:>6} {lat_s:>9} {p50:>7} {p95:>7}"
                             f" {gen_s:>9} {ratio:>9}")
            lines.append("")

            lines.append(f"Contacts with BS  (link down after {CONTACT_GAP_S:.0f}s without a beacon)")
            hdr = (f"  {'Node':<10} {'N':>4} {'UpTime':>9} {'Up%':>6} {'MeanLen':>8}"
                   f" {'DataInUp':>9} {'Data/s up':>10}")
            lines.append(hdr)
            lines.append("  " + "-" * (len(hdr) - 2))
            for node in comm_nodes:
                recs = list(contact_history.get(node, []))
                ls = link_state.get(node)
                if ls and ls['up']:
                    recs.append(_close_contact(node, ls, ongoing=True))
                up = sum(r['end'] - r['start'] for r in recs)
                data_up = sum(r['data'] for r in recs)
                pct = f"{100.0 * up / duration_s:.0f}%" if duration_s else "-"
                mean_len = f"{up / len(recs):.1f}s" if recs else "-"
                rate = f"{data_up / up:.2f}" if up > 0 else "-"
                lines.append(f"  {node:<10} {len(recs):>4} {up:>8.0f}s {pct:>6} {mean_len:>8}"
                             f" {data_up:>9} {rate:>10}")
            lines.append("")

            if rover_status:
                lines.append("Rover comms config (last @ROVER_STATUS)")
                for node, st in sorted(rover_status.items()):
                    bw = st.get('bw', 0)
                    on, off = st.get('on', 0), st.get('off', 0)
                    mode = []
                    mode.append(f"bw {bw:g}/s" if bw else "bw unlimited")
                    mode.append(f"periodic {on:g}s on/{off:g}s off" if on and off else "always on")
                    lines.append(f"  {node}: {', '.join(mode)}, gen every {st.get('gi', 0):g}s,"
                                 f" queue {st.get('q', 0):.0f} own / {st.get('qf', 0):.0f} ferried,"
                                 f" gen {st.get('gen', 0):.0f} tx {st.get('tx', 0):.0f}"
                                 f" retx {st.get('retx', 0):.0f} acked {st.get('ack', 0):.0f}")
                lines.append("")

        if control['msg'] is not None:
            m = control['msg']
            lines.append(f"Fleet control (last sent): v{control['version']} {m.get('state')}"
                         + (f" '{m['exp']}'" if m.get('exp') else ""))
            lines.append(f"  cfg: {json.dumps(m.get('cfg', {}), separators=(',', ':'))}")
            lines.append("  (full history in controls.csv)")
            lines.append("")

        if markers:
            lines.append("Markers")
            for t, label in markers:
                lines.append(f"  {label:<8} {_ts(t)}  (+{t - start_time:.0f}s)")
            lines.append("")

        #  notes
        lines.append("Notes")
        lines.append("  Uniq / Poss / Rate: aggregated across all boots. Poss = sum of")
        lines.append("                      (max_seq - min_seq + 1) per boot; seq resets")
        lines.append("                      each boot so boots are counted independently.")
        lines.append("  Unique delivered  : distinct (source, boot, seq) pairs from @METRIC.")
        lines.append("  Ferry dups        : same bundle reached BS via a second forwarder;")
        lines.append("                      counted in total deliveries, not unique.")
        lines.append("  Antipacket fails  : same forwarder re-sent the bundle; excluded from")
        lines.append("                      both unique and total counts.")
        lines.append("  AvgRSSI           : current boot only (cleared on node restart).")
        lines.append("  Uniq/Poss above   : include telemetry bundles (they share the seq space).")
        lines.append("  Data/Gen          : BS-counted data deliveries / rover-reported data")
        lines.append("                      generated (rover counter spans the daemon run).")
        lines.append("  DataInUp, Data/s  : data delivered while the rover's link was up.")
        lines.append("=" * W)

        report = "\n".join(lines)

    # print and save outside the lock
    print("\n" + report)
    out_dir = session_dir or REPORT_DIR
    os.makedirs(out_dir, exist_ok=True)
    filename = os.path.join(out_dir, f"dtn_summary_{now.strftime('%Y%m%d_%H%M%S')}.txt")
    try:
        with open(filename, 'w') as f:
            f.write(report + "\n")
        print(f"\n[Summary saved to {filename}]")
    except OSError as e:
        print(f"\n[Could not save summary file: {e}]")


_PALETTE = ['dodgerblue', 'tomato', 'limegreen', 'darkorchid', 'goldenrod', 'teal']
_color_order: list = []


def _node_color(nid: str) -> str:
    if nid not in _color_order:
        _color_order.append(nid)
    return _PALETTE[_color_order.index(nid) % len(_PALETTE)]


def _fmt_age(sec: float) -> str:
    sec = int(sec)
    m, s = divmod(sec, 60)
    return f"{m}m{s:02d}s" if m else f"{s}s"


def _window_estimate(st: dict, now: float):
    """Estimate a periodic rover's radio state from its last @ROVER_STATUS.
    Uses BS arrival time only (no clock sync needed). Returns (is_on, secs_left) or None."""
    on, off, win = st.get('on', 0), st.get('off', 0), st.get('win', -1)
    if not (on and off) or win is None or win < 0:
        return None
    period = on + off
    x = (now - (st['updated_at'] + win)) % period     # time since the last expected off-start
    if x < off:
        return False, off - x
    return True, period - x


def _draw_log_panel(ax, now: float) -> None:
    """Text panel: per-rover comms status, recent deliveries, delivery metrics, positions.
    Caller holds lock."""
    ax.axis('off')
    ax.set_title("Comms Status / Bundle Events", fontsize=10, fontweight='bold')
    rows = []   # (text, color, bold)

    def row(text, color='black', bold=False):
        rows.append((text, color, bold))

    rovers = sorted(set(link_state) | set(rover_status) | set(contact_history))
    if rovers:
        row("- Rover Comms -", bold=True)
    for nid in rovers:
        ls = link_state.get(nid)
        n_contacts = len(contact_history.get(nid, [])) + (1 if ls and ls['up'] else 0)
        if ls and ls['up']:
            rssi = f", {ls['rssi'][-1]} dBm" if ls['rssi'] else ""
            row(f"Node {nid}: LINK UP {_fmt_age(now - ls['since'])} "
                f"(contact #{n_contacts}{rssi})", 'green', True)
        elif ls:
            row(f"Node {nid}: LINK DOWN {_fmt_age(now - ls['last'])} "
                f"({n_contacts} contacts)", 'firebrick', True)
        else:
            row(f"Node {nid}: no beacons heard", 'gray', True)
        st = rover_status.get(nid)
        if st:
            bw = st.get('bw', 0)
            mode = f"bw {bw:g}/s" if bw else "bw unlimited"
            est = _window_estimate(st, now)
            if est is not None:
                state = "ON" if est[0] else "OFF"
                mode += f" | {st['on']:g}s on/{st['off']:g}s off, radio ~{state} {est[1]:.0f}s left"
            row(f"   {mode}", 'dimgray')
            recent = sum(1 for t, s, k, _ in delivery_timeline
                         if s == nid and k == 'data' and now - t <= 60)
            row(f"   queue {st.get('q', 0):.0f} own/{st.get('qf', 0):.0f} ferried | "
                f"gen {st.get('gen', 0):.0f} tx {st.get('tx', 0):.0f} retx {st.get('retx', 0):.0f} "
                f"ack {st.get('ack', 0):.0f} | data 60s: {recent}  "
                f"(status {_fmt_age(now - st['updated_at'])} old)", 'dimgray')
    if rovers:
        row("")

    row("- Recent Deliveries -", bold=True)
    for entry in list(reversed(event_log))[:6]:
        age_str = _fmt_age(now - entry['time']) + " ago"
        if entry['is_ferry']:
            row(f"[FERRY]  src:{entry['src']} via:{entry['ferry']} hops:{entry['hops']} "
                f"seq:{entry['seq']} ({age_str})", 'darkorange')
        else:
            row(f"[DIRECT] src:{entry['src']} hops:{entry['hops']} seq:{entry['seq']} ({age_str})",
                'green')
    row("")

    row("- Delivery Metrics -", bold=True)
    per_source = defaultdict(lambda: {'data': [], 'tel': 0})
    for (src, _seq), info in delivered_bundles.items():
        if info.get('kind') == 'data':
            per_source[src]['data'].append(info['latency_ms'])
        else:
            per_source[src]['tel'] += 1
    n_data = sum(len(v['data']) for v in per_source.values())
    row(f"Total delivered: {len(delivered_bundles)} bundles ({n_data} data)", 'navy')
    for src in sorted(set(per_source) | set(antipkt_fail_counts) | set(ferry_dup_counts)):
        d = per_source[src]['data']
        avg = f"avg {sum(d) / len(d):.0f} ms" if d else "no data"
        extra = ""
        if node_boot_count.get(src):       extra += f"  restarts:{node_boot_count[src]}"
        if antipkt_fail_counts.get(src):   extra += f"  ap_fail:{antipkt_fail_counts[src]}"
        if ferry_dup_counts.get(src):      extra += f"  dup:{ferry_dup_counts[src]}"
        row(f"  Node {src}: {len(d)} data + {per_source[src]['tel']} tel  {avg}{extra}",
            'darkgreen')

    known_locs = {nid: loc for nid, loc in node_locations.items()
                  if loc['lat'] != 0.0 or loc['lon'] != 0.0}
    if known_locs:
        row("")
        row("- Node Positions -", bold=True)
        for nid, loc in sorted(known_locs.items()):
            rssi_str = f"  BS:{loc['rssi']} dBm" if loc['rssi'] is not None else ""
            row(f"  Node {nid}: {loc['lat']:.2f},{loc['lon']:.2f}  {loc['alt']:.0f}m{rssi_str}"
                f"  ({_fmt_age(now - loc['updated_at'])} ago)", 'darkslateblue')

    y = 0.99
    step = min(0.045, 0.93 / max(1, len(rows)))
    for text, color, bold in rows:
        ax.text(0.0, y, text, transform=ax.transAxes, fontsize=7.5, color=color,
                verticalalignment='top', fontfamily='monospace',
                fontweight='bold' if bold else 'normal')
        y -= step
    ax.text(0.0, -0.02, f"keys: m=marker  s=save summary  r=save+reset  h=help   "
                        f"[{os.path.basename(session_dir)}]",
            transform=ax.transAxes, fontsize=7, color='gray', verticalalignment='top')


def _draw_timeline(ax, now: float) -> None:
    """Bottom panel: per-rover link-up bands, data throughput, data latency, markers.
    Caller holds lock."""
    t0 = now - TIMELINE_S
    nodes = sorted(set(s for _, s, _, _ in delivery_timeline) | set(link_state)
                   | set(contact_history))
    ax.set_xlim(-TIMELINE_S, 0)
    ax.set_xlabel("seconds (0 = now)", fontsize=8)
    ax.set_ylabel("data bundles/s", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(True, alpha=0.3)
    ax.set_title(f"Comms timeline (last {TIMELINE_S}s): bands = link up to BS, "
                 f"lines = data throughput ({TIMELINE_BIN_S}s bins), dots = data latency",
                 fontsize=9, fontweight='bold')

    band_h = 0.07
    for idx, nid in enumerate(nodes):
        color = _node_color(nid)
        spans = [(c['start'], c['end']) for c in contact_history.get(nid, [])]
        ls = link_state.get(nid)
        if ls and ls['up']:
            spans.append((ls['since'], now))
        for a, b in spans:
            if b < t0:
                continue
            a = max(a, t0)
            ax.axvspan(a - now, b - now, ymin=1 - band_h * (idx + 1), ymax=1 - band_h * idx,
                       color=color, alpha=0.6, lw=0)
            ax.axvspan(a - now, b - now, color=color, alpha=0.05, lw=0)

    # throughput, bins aligned to absolute time so they don't jitter between frames
    last_edge = (int(now) // TIMELINE_BIN_S + 1) * TIMELINE_BIN_S
    n_bins = TIMELINE_S // TIMELINE_BIN_S + 1
    edges = [last_edge - TIMELINE_BIN_S * (n_bins - i) for i in range(n_bins + 1)]
    ymax = 1.0
    for nid in nodes:
        counts = [0] * n_bins
        for t, s, k, _lat in delivery_timeline:
            if s == nid and k == 'data' and t >= edges[0]:
                i = int((t - edges[0]) // TIMELINE_BIN_S)
                if 0 <= i < n_bins:
                    counts[i] += 1
        rates = [c / TIMELINE_BIN_S for c in counts]
        ymax = max(ymax, max(rates) if rates else 0)
        ax.step([e - now for e in edges], rates + rates[-1:], where='post',
                color=_node_color(nid), lw=1.8, label=f"{nid}")
    ax.set_ylim(0, ymax * 1.35)

    lat_pts = [(t - now, lat / 1000.0, _node_color(s)) for t, s, k, lat in delivery_timeline
               if k == 'data' and t >= t0]
    if lat_pts:
        ax2 = AX['tl2']
        ax2.scatter([p[0] for p in lat_pts], [p[1] for p in lat_pts],
                    c=[p[2] for p in lat_pts], s=8, alpha=0.6, marker='o', linewidths=0)
        ax2.set_ylabel("latency (s)", fontsize=8)
        ax2.tick_params(labelsize=7)
        ax2.set_ylim(bottom=0)
    ax2 = AX['tl2']
    ax2.set_xlim(-TIMELINE_S, 0)
    ax2.yaxis.set_label_position('right')
    ax2.yaxis.tick_right()

    for t, label in markers:
        if t >= t0:
            ax.axvline(t - now, color='black', ls=':', lw=1)
            ax.text(t - now, ymax * 1.3, f" {label}", fontsize=7, va='top')

    if nodes:
        ax.legend(fontsize=7, loc='upper left', title="node", title_fontsize=7)


def _on_key(event) -> None:
    if _typing():
        return                                   # keys go to the text box being edited
    if event.key == 'm':
        add_marker()
    elif event.key == 's':
        generate_summary()
    elif event.key == 'r':
        print("[VISUALIZER] Manual reset - saving session and starting fresh.")
        generate_summary()
        reset_session_state()
    elif event.key == 'h':
        print("[VISUALIZER] keys: m = drop a timestamped marker (logged + drawn on timeline), "
              "s = save summary now, r = save summary and reset session, h = this help")


# fleet control (BS laptop -> BS ESP32 -> rovers)
# The visualizer holds one desired fleet state and re-broadcasts it every CONTROL_PERIOD_S.
# Rovers adopt any version newer than theirs and report the version they run in
# @ROVER_STATUS (cv=...), so the panel shows who has applied what. Rovers in a periodic
# off-window can't hear it and pick it up when their radio comes back on.
HOST_SOF           = 0xAA
HOST_CMD_WIFI_TX   = 0x30
PKT_TYPE_CONTROL   = 0x06
BS_NODE_ID         = 23768
CONTROL_PERIOD_S   = 2.0
START_DELAY_S      = 3.0     # experiments start this long after Start is pressed, fleet-wide
CONTROL_FIELDS = [           # (config key, label, default)
    ('interval',      'gen s',     '1'),
    ('bw_limit',      'bw/s',      '0'),
    ('bw_burst',      'burst',     '1'),
    ('bw_order',      'order',     'oldest'),
    ('comms_on',      'on s',      '0'),
    ('comms_off',     'off s',     '0'),
    ('comms_phase',   'phase s',   '0'),
    ('payload_bytes', 'payload B', '0'),
]

AX: dict = {}
_widgets: list = []
_textboxes: dict = {}
_ser = None
_ser_write_lock = threading.Lock()
control = {'version': 0, 'msg': None, 'sent_at': 0.0, 'last_tx': 0.0, 'bs_rejects': False}


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def _encode_frame(cmd: int, payload: bytes) -> bytes:
    n = len(payload)
    body = bytes([cmd, n & 0xFF, n >> 8]) + payload
    crc = _crc16(body)
    return bytes([HOST_SOF]) + body + bytes([crc & 0xFF, crc >> 8])


def _send_control_now() -> bool:
    msg = control['msg']
    if msg is None or _ser is None:
        return False
    air = (struct.pack('<BHI', PKT_TYPE_CONTROL, BS_NODE_ID, control['version'] & 0xFFFFFFFF)
           + json.dumps(msg, separators=(',', ':')).encode())
    try:
        with _ser_write_lock:
            _ser.write(_encode_frame(HOST_CMD_WIFI_TX, air))
        control['last_tx'] = time.time()
        return True
    except Exception as e:   # serial hiccup: the reader thread handles reconnects
        print(f"[CONTROL] send failed: {e}")
        return False


def _maybe_send_control() -> None:
    if control['msg'] is not None and time.time() - control['last_tx'] >= CONTROL_PERIOD_S:
        _send_control_now()


def _set_control(msg: dict, note: str) -> None:
    """Publish a new desired fleet state. Version = epoch seconds (monotonic across
    visualizer restarts, so rovers never ignore a fresh visualizer)."""
    control['version'] = max(int(time.time()), control['version'] + 1)
    control['msg'] = msg
    control['sent_at'] = time.time()
    _send_control_now()
    _log_serial(f"#CONTROL:v{control['version']}:{json.dumps(msg, separators=(',', ':'))}")
    _csv_append("controls.csv", ['time', 'epoch', 'version', 'state', 'exp', 'note', 'msg'],
                [_ts(control['sent_at']), f"{control['sent_at']:.3f}", control['version'],
                 msg.get('state'), msg.get('exp', ''), note, json.dumps(msg)])
    print(f"[CONTROL] v{control['version']} {note}: {msg}")


def _read_config_fields() -> dict:
    cfg = {}
    for key, _label, default in CONTROL_FIELDS:
        raw = (_textboxes[key].text if key in _textboxes else default).strip()
        if key == 'bw_order':
            cfg[key] = 'newest' if raw.lower().startswith('n') else 'oldest'
            continue
        try:
            cfg[key] = int(raw) if key == 'payload_bytes' else float(raw)
        except ValueError:
            print(f"[CONTROL] bad value for {key!r}: {raw!r}, using {default}")
            cfg[key] = float(default) if key != 'payload_bytes' else int(default)
    return cfg


def _exp_name() -> str:
    raw = _textboxes['exp'].text if 'exp' in _textboxes else ''
    name = ''.join(ch for ch in raw.strip().replace(' ', '_') if ch.isalnum() or ch in '-_')[:40]
    return name or f"exp_{datetime.now().strftime('%H%M%S')}"


def start_experiment(_event=None) -> None:
    exp = _exp_name()
    cfg = _read_config_fields()
    start_at = time.time() + START_DELAY_S
    generate_summary()
    reset_session_state(tag=exp)
    _set_control({'state': 'active', 'exp': exp, 'start_at': round(start_at, 2), 'cfg': cfg},
                 f"start experiment '{exp}'")
    add_marker(f"start:{exp}")


def apply_live(_event=None) -> None:
    msg = control['msg']
    if not msg or msg.get('state') != 'active':
        print("[CONTROL] no experiment running - use Start (or Standby) instead")
        return
    cfg = _read_config_fields()
    _set_control(dict(msg, cfg=cfg), "live config change")
    add_marker("cfg")


def go_standby(_event=None) -> None:
    generate_summary()
    reset_session_state(tag='standby')
    _set_control({'state': 'standby', 'cfg': _read_config_fields()}, "standby")
    add_marker("standby")


def _draw_control_status(ax, now: float) -> None:
    """One line under the controls: desired fleet state and which rovers have applied it.
    Caller holds lock."""
    ax.axis('off')
    msg = control['msg']
    if control['bs_rejects']:
        ax.text(0.0, 0.5, "BS FIRMWARE DOES NOT FORWARD CONTROL (unknown cmd 0x30) - "
                          "close the visualizer, then: idf.py build && idf.py -p <BS port> flash",
                transform=ax.transAxes, fontsize=9, va='center', color='white', fontweight='bold',
                bbox=dict(facecolor='firebrick', edgecolor='none', pad=3))
        return
    if msg is None:
        head = "Fleet control: nothing sent yet (rovers keep whatever they last heard)"
    else:
        state = msg.get('state', '?').upper()
        exp = f" '{msg['exp']}'" if msg.get('exp') else ""
        starts = ""
        if msg.get('state') == 'active' and msg.get('start_at', 0) > now:
            starts = f", starts in {msg['start_at'] - now:.0f}s"
        head = (f"Fleet control v{control['version']}: {state}{exp}{starts} "
                f"(set {_fmt_age(now - control['sent_at'])} ago, re-sent every {CONTROL_PERIOD_S:g}s)")
    parts = []
    for nid, st in sorted(rover_status.items()):
        cv = int(st.get('cv', 0) or 0)
        ok = msg is not None and cv >= control['version']
        mark = "OK" if ok else ("waiting" if msg is not None else "")
        parts.append(f"{nid}: {st.get('st', '?')} '{st.get('exp', '-')}' v{cv} {mark}".rstrip())
    ax.text(0.0, 0.5, head + ("   |   " + "   ".join(parts) if parts else ""),
            transform=ax.transAxes, fontsize=8, va='center', fontfamily='monospace',
            color='navy' if msg is None or all('OK' in p for p in parts) else 'darkorange')


def build_figure():
    """Create the figure and all axes/widgets once; update_graph only redraws data axes
    (clearing the whole figure every frame would destroy the control widgets)."""
    from matplotlib.widgets import Button, TextBox
    fig = plt.figure(figsize=(21, 11))
    gs = fig.add_gridspec(3, 4, height_ratios=[3, 1.3, 0.42], width_ratios=[1, 1, 1, 0.03],
                          hspace=0.34, wspace=0.22, left=0.03, right=0.96, top=0.92, bottom=0.03)
    AX['graph'] = fig.add_subplot(gs[0, 0])
    AX['log']   = fig.add_subplot(gs[0, 1])
    AX['map']   = fig.add_subplot(gs[0, 2])
    AX['cax']   = fig.add_subplot(gs[0, 3])
    AX['tl']    = fig.add_subplot(gs[1, :3])
    AX['tl2']   = AX['tl'].twinx()

    ctrl = gs[2, :].subgridspec(2, 13, height_ratios=[1, 0.7], wspace=0.55, hspace=0.25)
    tb = TextBox(fig.add_subplot(ctrl[0, 0:2]), 'exp ', initial='')
    _textboxes['exp'] = tb
    _widgets.append(tb)
    for i, (key, label, default) in enumerate(CONTROL_FIELDS):
        t = TextBox(fig.add_subplot(ctrl[0, 2 + i]), label + ' ', initial=default)
        t.label.set_fontsize(8)
        _textboxes[key] = t
        _widgets.append(t)
    for col, (label, cb, color) in enumerate((("Start", start_experiment, 'palegreen'),
                                              ("Apply live", apply_live, 'lightyellow'),
                                              ("Standby", go_standby, 'lightcoral'))):
        b = Button(fig.add_subplot(ctrl[0, 10 + col]), label, color=color)
        b.on_clicked(cb)
        _widgets.append(b)
    AX['ctrl_status'] = fig.add_subplot(ctrl[1, :])
    fig.canvas.mpl_connect('key_press_event', _on_key)
    return fig


def _typing() -> bool:
    return any(getattr(t, 'capturekeystrokes', False) for t in _textboxes.values())


def update_graph(frame):
    for key in ('graph', 'log', 'map', 'cax', 'tl', 'tl2', 'ctrl_status'):
        AX[key].clear()
    with lock:
        current_time = time.time()

        nodes_to_remove = [
            n for n, attr in G.nodes(data=True)
            if current_time - attr.get('last_seen', 0) > TIMEOUT_SECONDS
        ]
        for n in nodes_to_remove:
            print(f"Node {n} timed out. Removing from visualizer.")
            G.remove_node(n)

        ax_graph, ax_log, ax_map, ax_tl = AX['graph'], AX['log'], AX['map'], AX['tl']
        _draw_timeline(ax_tl, current_time)
        _draw_log_panel(ax_log, current_time)
        _draw_control_status(AX['ctrl_status'], current_time)

        # radio map panel
        _NODE_COLORS = ['dodgerblue', 'tomato', 'limegreen', 'darkorchid']
        _NODE_MARKERS = ['o', 's', '^', 'D']
        ax_map.set_title("Radio Map (RSSI vs Position)", fontsize=10, fontweight='bold')
        ax_map.set_xlim(-RADIO_MAP_HALF_M, RADIO_MAP_HALF_M)
        ax_map.set_ylim(-RADIO_MAP_HALF_M, RADIO_MAP_HALF_M)
        ax_map.set_xlabel("X (m)", fontsize=8)
        ax_map.set_ylabel("Y (m)", fontsize=8)
        ax_map.set_aspect('equal')
        ax_map.grid(True, alpha=0.3)
        ax_map.axhline(0, color='gray', linewidth=0.5)
        ax_map.axvline(0, color='gray', linewidth=0.5)

        sorted_map_nodes = sorted(radio_map_samples.keys())
        _rssi_sc = None
        for idx, nid in enumerate(sorted_map_nodes):
            samples = list(radio_map_samples[nid])
            if not samples:
                continue
            xs    = [s[0] for s in samples]
            ys    = [s[1] for s in samples]
            rssis = [s[2] for s in samples]
            _rssi_sc = ax_map.scatter(
                xs, ys, c=rssis, cmap='RdYlGn',
                vmin=-90, vmax=-30, s=80, alpha=0.8,
                marker=_NODE_MARKERS[idx % len(_NODE_MARKERS)],
                linewidths=0, zorder=2,
                label=f"Node {nid}",
            )

        AX['cax'].set_visible(_rssi_sc is not None)
        if _rssi_sc is not None:
            cb = plt.gcf().colorbar(_rssi_sc, cax=AX['cax'])
            cb.set_label("RSSI (dBm)", fontsize=7)
            cb.ax.tick_params(labelsize=7)
            if len(sorted_map_nodes) > 1:
                ax_map.legend(fontsize=7, loc='upper right', markerscale=1.2)

        # current rover positions
        for idx, nid in enumerate(sorted_map_nodes):
            loc = node_locations.get(nid)
            if loc is None:
                continue
            cx, cy = loc['lat'], loc['lon']
            age = current_time - loc['updated_at']
            alpha = max(0.3, 1.0 - age / 10.0)
            ax_map.scatter([cx], [cy], s=200,
                           marker=_NODE_MARKERS[idx % len(_NODE_MARKERS)],
                           color='white', edgecolors='black',
                           linewidths=1.5, zorder=6, alpha=alpha)
            ax_map.annotate(
                nid, (cx, cy),
                textcoords='offset points', xytext=(6, 4),
                fontsize=7, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.6, lw=0),
            )

        total_samples = sum(len(v) for v in radio_map_samples.values())
        ax_map.set_title(
            f"Radio Map  ({total_samples} samples)",
            fontsize=10, fontweight='bold',
        )

        # graph panel
        if len(G.nodes) == 0:
            ax_graph.set_title("Waiting for Mesh Data...")
            ax_graph.axis('off')
            return

        active_transfers = [
            t for t in recent_transfers
            if current_time - t['time'] < (8.0 if t.get('is_ferry') else 2.0)
        ]
        recent_transfers[:] = active_transfers

        try:
            pos = nx.nx_agraph.graphviz_layout(G, prog="dot")
        except ImportError:
            pos = nx.spring_layout(G, seed=42)

        nx.draw_networkx_nodes(G, pos, node_size=700, node_color='lightblue', ax=ax_graph)
        nx.draw_networkx_labels(G, pos, font_size=12, font_weight="bold", ax=ax_graph)
        nx.draw_networkx_edges(G, pos, edge_color='gray', arrows=True, ax=ax_graph)

        edge_labels = nx.get_edge_attributes(G, 'rssi')
        formatted_labels = {k: f"{v} dBm" for k, v in edge_labels.items()}
        nx.draw_networkx_edge_labels(G, pos, edge_labels=formatted_labels, font_size=10,
                                     font_color='blue', ax=ax_graph)

        ferry_labels = {}
        for transfer in active_transfers:
            src      = transfer['src']
            ferry    = transfer['ferry']
            dst      = transfer['dst']
            hops     = transfer.get('hops', 1)
            is_ferry = (ferry != src) or (hops > 1)
            if is_ferry:
                if ferry in G and dst in G:
                    nx.draw_networkx_edges(G, pos, edgelist=[(ferry, dst)],
                                           edge_color='orange', width=3.0, style='dashed',
                                           arrows=True, ax=ax_graph)
                    ferry_labels[(ferry, dst)] = f"From {src} ({hops}h)"
            else:
                if src in G and dst in G:
                    nx.draw_networkx_edges(G, pos, edgelist=[(src, dst)],
                                           edge_color='red', width=3.0, arrows=True, ax=ax_graph)

        if ferry_labels:
            nx.draw_networkx_edge_labels(G, pos, edge_labels=ferry_labels, font_size=9,
                                         font_color='orange', ax=ax_graph)

        latency_strings = []
        for node, latencies in node_latencies.items():
            if len(latencies) > 0 and node in G.nodes:
                avg_lat = sum(latencies) / len(latencies)
                latency_strings.append(f"Rover {node}: {avg_lat:.1f} ms")

        if latency_strings:
            formatted_stats = "\n".join(
                [" | ".join(latency_strings[i:i+3]) for i in range(0, len(latency_strings), 3)]
            )
            title_str = f"Live DTN Rover Mesh\nAverage Latencies:\n{formatted_stats}"
        else:
            title_str = "Live DTN Rover Mesh\nWaiting for metric data..."

        ax_graph.set_title(title_str, fontsize=12, fontweight='bold', pad=15)
        ax_graph.axis('off')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Live DTN mesh visualizer (reads the BS serial port)')
    parser.add_argument('--port', default=SERIAL_PORT, help='BS serial port')
    parser.add_argument('--baud', type=int, default=BAUD_RATE)
    parser.add_argument('--map-half', type=float, default=RADIO_MAP_HALF_M,
                        help='Radio map half-width in metres')
    parser.add_argument('--report-dir', default=REPORT_DIR,
                        help='Parent folder for per-session report folders')
    parser.add_argument('--timeline', type=int, default=TIMELINE_S,
                        help='Seconds shown on the comms timeline')
    args = parser.parse_args()
    BAUD_RATE, RADIO_MAP_HALF_M, REPORT_DIR, TIMELINE_S = (
        args.baud, args.map_half, args.report_dir, args.timeline)

    # free up keys used by the visualizer from matplotlib's default bindings
    for km, keys in (('keymap.save', ('s',)), ('keymap.home', ('r', 'h'))):
        plt.rcParams[km] = [k for k in plt.rcParams[km] if k not in keys]

    thread = threading.Thread(target=serial_reader_thread, args=(args.port,), daemon=True)
    try:
        thread.start()
        fig = build_figure()
        print("[VISUALIZER] keys: m=marker  s=save summary  r=save+reset  h=help")
        ani = animation.FuncAnimation(fig, update_graph, interval=500, cache_frame_data=False)
        plt.show()
    except KeyboardInterrupt:
        print("\nShutting down visualizer...")
    finally:
        generate_summary()
