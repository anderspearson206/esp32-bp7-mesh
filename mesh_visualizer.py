import os
import serial
import threading
import time
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque, defaultdict
from datetime import datetime

SERIAL_PORT = os.environ.get('SERIAL_PORT', 'COM5')  # override: SERIAL_PORT=/dev/ttyUSB0 python mesh_visualizer.py
BAUD_RATE = 115200
TIMEOUT_SECONDS = 15

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
start_time          = time.time()


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


def reset_session_state():
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
        start_time = time.time()
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

    os.makedirs("bs_reports", exist_ok=True)
    filename = f"bs_reports/node_{node_id}_boot{boot_num}.txt"
    try:
        with open(filename, 'w') as f:
            f.write(report + "\n")
        print(f"[Boot summary saved to {filename}]")
    except OSError as e:
        print(f"[Could not save boot summary: {e}]")


def serial_reader_thread(port):
    """Read serial data from the BS port and update shared state."""
    first_connect = True
    while True:
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=1)
            if not first_connect:
                # BS came back after a disconnect, save old session and reset
                print(f"[VISUALIZER] Reconnected to {port} - saving session and resetting state.")
                generate_summary()
                reset_session_state()
            first_connect = False
            print(f"Connected to {port}. Listening for DTN telemetry...")

            while True:
                line = read_line_or_frame(ser)
                if not line:
                    continue

                # Detect BS reboot via its boot-time log line (covers software resets
                # where the serial connection stays open and no SerialException fires).
                if "I am the Base Station" in line:
                    print(f"[VISUALIZER] BS reboot detected in serial stream - saving and resetting.")
                    generate_summary()
                    reset_session_state()
                    continue

                if line.startswith("@"):
                    print(f"Received: {line} : {datetime.now().strftime('%H:%M:%S')}")

                # topology heartbeat from rovers, e.g. @NET:node_id:parent_id:rssi
                if line.startswith("@NET:"):
                    parts = line.split(':')
                    if len(parts) >= 4:
                        node_id, parent, rssi = parts[1], parts[2], parts[3]
                        with lock:
                            current_time = time.time()
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
                                    }
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

        # per node
        lines.append("Per Node")
        hdr = f"  {'Node':<10} {'Uniq':>6} {'FDup':>6} {'APFl':>6} {'AvgLat':>10} {'AvgRSSI':>9}  SeqRange"
        lines.append(hdr)
        lines.append("  " + "-" * (len(hdr) - 2))
        for node in all_nodes:
            unique_count = agg_unique.get(node, 0)
            ferry_dups   = agg_fdups.get(node, 0)
            ap_fails     = agg_apfails.get(node, 0)
            lats         = agg_lats.get(node, [])
            rssi_vals    = node_rssi.get(node, [])
            lat_str      = f"{sum(lats)/len(lats):.0f} ms"  if lats     else "N/A"
            rssi_str     = f"{sum(rssi_vals)/len(rssi_vals):.1f} dBm" if rssi_vals else "N/A"
            if node in node_seq_range:
                r = node_seq_range[node]
                seq_str = f"[{r['min']}..{r['max']}]"
            else:
                seq_str = "N/A"
            lines.append(
                f"  {node:<10} {unique_count:>6} {ferry_dups:>6} {ap_fails:>6} "
                f"{lat_str:>10} {rssi_str:>9}  {seq_str}"
            )
        lines.append("")

        # delivery estimate (aggregated across all boots)
        nodes_with_possible = [n for n in all_nodes if agg_possible.get(n, 0) > 0]
        if nodes_with_possible:
            lines.append("Delivery Estimate  (aggregated across all boots)")
            hdr2 = f"  {'Node':<10} {'Possible':>9} {'Delivered':>10} {'Undelivered':>12} {'Pct':>7}"
            lines.append(hdr2)
            lines.append("  " + "-" * (len(hdr2) - 2))
            for node in nodes_with_possible:
                possible     = agg_possible[node]
                unique_count = agg_unique.get(node, 0)
                undelivered  = max(0, possible - unique_count)
                pct          = 100.0 * unique_count / possible if possible > 0 else 0.0
                lines.append(
                    f"  {node:<10} {possible:>9} {unique_count:>10} {undelivered:>12} {pct:>6.1f}%"
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
            lines.append("")

        #  notes
        lines.append("Notes")
        lines.append("  Unique delivered  : distinct (source, seq) pairs from @METRIC events.")
        lines.append("  Ferry dups        : same bundle reached BS via a second forwarder;")
        lines.append("                      counted in total deliveries, not unique.")
        lines.append("  Antipacket fails  : same forwarder re-sent the bundle; excluded from")
        lines.append("                      both unique and total counts.")
        lines.append("  Undelivered est.  : sum across all boots of (last_seq - first_seq + 1)")
        lines.append("                      minus unique delivered per boot. Seq numbers reset")
        lines.append("                      each boot, so boots are summed independently.")
        lines.append("=" * W)

        report = "\n".join(lines)

    # print and save outside the lock
    print("\n" + report)
    os.makedirs("bs_reports", exist_ok=True)
    filename = f"bs_reports/dtn_summary_{now.strftime('%Y%m%d_%H%M%S')}.txt"
    try:
        with open(filename, 'w') as f:
            f.write(report + "\n")
        print(f"\n[Summary saved to {filename}]")
    except OSError as e:
        print(f"\n[Could not save summary file: {e}]")


def update_graph(frame):
    plt.clf()
    with lock:
        current_time = time.time()

        nodes_to_remove = [
            n for n, attr in G.nodes(data=True)
            if current_time - attr.get('last_seen', 0) > TIMEOUT_SECONDS
        ]
        for n in nodes_to_remove:
            print(f"Node {n} timed out. Removing from visualizer.")
            G.remove_node(n)

        ax_graph = plt.subplot(1, 2, 1)
        ax_log   = plt.subplot(1, 2, 2)

        # event log panel
        ax_log.axis('off')
        ax_log.set_title("Bundle Events / Delivery Metrics", fontsize=10, fontweight='bold')
        y = 0.97
        ax_log.text(0.02, y, "- Recent Deliveries -", transform=ax_log.transAxes,
                    fontsize=8, color='black', verticalalignment='top', fontweight='bold')
        y -= 0.05
        for entry in list(reversed(event_log))[:10]:
            age = int(current_time - entry['time'])
            mins, secs = divmod(age, 60)
            age_str = f"{mins}m{secs:02d}s ago" if mins else f"{secs}s ago"
            if entry['is_ferry']:
                ln = f"[FERRY]  src:{entry['src']} via:{entry['ferry']} hops:{entry['hops']} seq:{entry['seq']} ({age_str})"
                color = 'darkorange'
            else:
                ln = f"[DIRECT] src:{entry['src']} hops:{entry['hops']} seq:{entry['seq']} ({age_str})"
                color = 'green'
            ax_log.text(0.02, y, ln, transform=ax_log.transAxes,
                        fontsize=7.5, color=color, verticalalignment='top',
                        fontfamily='monospace')
            y -= 0.048

        # delivery metrics
        y_stats = 0.42
        ax_log.text(0.02, y_stats, "- Delivery Metrics -", transform=ax_log.transAxes,
                    fontsize=8, color='black', verticalalignment='top', fontweight='bold')
        y_stats -= 0.05

        total = len(delivered_bundles)
        ax_log.text(0.02, y_stats, f"Total delivered: {total} bundles",
                    transform=ax_log.transAxes, fontsize=8, color='navy',
                    verticalalignment='top', fontfamily='monospace')
        y_stats -= 0.045

        per_source = defaultdict(list)
        for (src, _seq), info in delivered_bundles.items():
            per_source[src].append(info['latency_ms'])
        for src, lats in sorted(per_source.items()):
            avg      = sum(lats) / len(lats)
            ap_fail  = antipkt_fail_counts.get(src, 0)
            dup      = ferry_dup_counts.get(src, 0)
            restarts = node_boot_count.get(src, 0)
            extra    = ""
            if restarts: extra += f"  restarts:{restarts}"
            if ap_fail:  extra += f"  ap_fail:{ap_fail}"
            if dup:      extra += f"  dup:{dup}"
            ax_log.text(0.02, y_stats,
                        f"  Node {src}: {len(lats)} pkts  avg {avg:.0f} ms{extra}",
                        transform=ax_log.transAxes, fontsize=8, color='darkgreen',
                        verticalalignment='top', fontfamily='monospace')
            y_stats -= 0.045

        all_dup_srcs = set(antipkt_fail_counts) | set(ferry_dup_counts)
        for src in sorted(all_dup_srcs):
            if src not in per_source:
                ap_fail = antipkt_fail_counts.get(src, 0)
                dup     = ferry_dup_counts.get(src, 0)
                extra   = ""
                if ap_fail: extra += f"  ap_fail:{ap_fail}"
                if dup:     extra += f"  dup:{dup}"
                ax_log.text(0.02, y_stats,
                            f"  Node {src}: 0 pkts{extra}",
                            transform=ax_log.transAxes, fontsize=8, color='sienna',
                            verticalalignment='top', fontfamily='monospace')
                y_stats -= 0.045

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
    thread = threading.Thread(target=serial_reader_thread, args=(SERIAL_PORT,), daemon=True)
    try:
        thread.start()
        fig = plt.figure(figsize=(14, 6))
        ani = animation.FuncAnimation(fig, update_graph, interval=500, cache_frame_data=False)
        plt.show()
    except KeyboardInterrupt:
        print("\nShutting down visualizer...")
    finally:
        generate_summary()
