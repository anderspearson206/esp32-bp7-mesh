# ESP32-C5 DTN Rover Mesh

A Delay-Tolerant Networking (DTN) mesh for a fleet of autonomous ground rovers. Each rover carries a **Jetson Orin Nano** as its main computer and an **ESP32-C5** as a wireless modem. The nodes communicate over **ESP-NOW** with epidemic-style store-and-forward routing using a BPv7-inspired bundle format.

**Deployment**: 3 rovers + 1 fixed base station (BS). Most data flows rover → BS. Rovers follow pre-planned trajectories with intermittent connectivity — bundles are carried (ferried) across disconnections by intermediary rovers.

**Architecture (rover side)**: On **rover** nodes the ESP32 is a _pure broadcast relay_ — it broadcasts every over-air frame the Jetson hands it and forwards every received frame back to the Jetson (with source MAC + RSSI). All DTN logic (beacons, bundle store, routing, antipacket/dedup, ACK-maps) runs in **`rover_daemon.py`** on the Jetson. The **Base Station** ESP32 still runs the full firmware stack unchanged, so the over-air wire format is identical and `mesh_visualizer.py` reads the BS serial port exactly as before. A single firmware binary selects its role at boot from its node ID.

## Quick Start

### 1 — Flash the ESP32-C5s

```bash
# Build firmware (ESP-IDF v6.0 required, IDF_PATH set)
idf.py build

# Flash base station
# the port will change based on device
idf.py -p COM13 flash

# Flash each rover
idf.py -p COM14 flash
```

Each node auto-detects its role at boot from its STA MAC. The base station prints `I am the Base Station`; rovers print `I am a Rover`. Optionally configure `WIFI_BAND_MODE_5G_ONLY` in firmware to restrict to 5 GHz (see Build & Flash below — `esp_wifi_set_band_mode` must be called **after** `esp_wifi_start()`).

### 2 — Start the rover daemon (one per rover)

Run on the **Jetson** (or the bench laptop) attached to each rover's ESP32 over USB/UART:

```bash
pip install pyserial
python rover_daemon.py --serial /dev/ttyUSB0   # Linux/Jetson
python rover_daemon.py --serial COM14           # Windows bench
```

The daemon owns all DTN logic for that rover: it sends beacons, generates data bundles, stores and forwards received bundles, and applies ACK maps. Nothing happens over the air until the daemon is running. If the rover tracks its own position (e.g. from a VIO or GPS source), the daemon reads that location and sends it to the base station automatically — no extra configuration needed.

### 3 — Start the visualizer on the base station laptop

```bash
pip install pyserial networkx matplotlib
python mesh_visualizer.py # you may need to adjust the serial port
```

The window shows three panels updated every 500 ms:

| Panel                       | Contents                                                                                                                                                      |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Mesh graph**              | Live topology — nodes, RSSI-labelled links, active bundle transfers (orange = ferried)                                                                        |
| **Bundle events / metrics** | Recent deliveries, per-node packet counts, avg latency, node positions                                                                                        |
| **Radio map**               | Scatter plot of every `@LOCATION:` sample received, coloured by RSSI (red = weak → green = strong); current rover position shown with a white-outlined marker |

Radio map range is set by `RADIO_MAP_HALF_M` at the top of `mesh_visualizer.py` (default `25.0` → ±25 m, i.e. a 50×50 m map).

### Running an experiment

1. Ensure `rover_daemon.py` is running on each rover.
2. Reboot all ESP32s (power-cycle or `idf.py -p <PORT> monitor` → `Ctrl-T Ctrl-R`). The visualizer detects each node's new `boot_id` and saves the previous session's metrics automatically — this gives you a clean slate for each run.
3. Start `mesh_visualizer.py` on the BS laptop.
4. Move rovers through their trajectories.
5. **End the experiment**: press `Ctrl-C` or close the visualizer window. All delivery metrics and coverage data are saved automatically to `bs_reports/`.

Per-session CSV coverage logs (`bs_reports/coverage_<timestamp>.csv`) contain every `(node, x, y, z, rssi)` sample received — useful for post-processing radio maps offline.

---

## Status

| Feature                                              | State                       |
| ---------------------------------------------------- | --------------------------- |
| ESP-NOW transport                                    | Done                        |
| Epidemic store-and-forward                           | Done                        |
| DTN ferrying (verified)                              | Done                        |
| Clock sync / latency correction                      | Done                        |
| Host UART interface (Jetson ↔ ESP32)                | Done                        |
| Antipackets (suppress redelivery)                    | Done                        |
| LCD / LED / button UI on BS                          | Done                        |
| Rover ESP32 = dumb relay, logic in `rover_daemon.py` | Done (builds + unit-tested) |
| Verify relay/daemon split on hardware                | In progress                 |
| Downlink / ACK to rovers                             | Not yet done                |
| Integration onto rover hardware                      | Not yet done                |

## Hardware

- **MCU**: ESP32-C5 (5 GHz only, no DFS channel 36)
- **Rover host**: Jetson Orin Nano, communicates over UART1 (GPIO 6/7)
- **BS peripherals**: 16×2 I2C LCD (PCF8574, addr `0x27`), pushbutton, status LED

### GPIO Pinout

| Pin     | Function                     |
| ------- | ---------------------------- |
| GPIO 9  | Pushbutton (BS only)         |
| GPIO 10 | Status LED (all nodes)       |
| GPIO 7  | LCD SDA / UART1 TX to Jetson |
| GPIO 8  | LCD SCL                      |

> When using UART1 for the Jetson interface, the LCD must be removed — they share GPIO 7/8.

### Node IDs (bench setup)

Node IDs are derived from `(mac[4] << 8) | mac[5]` of the STA MAC, printed on boot.

| Node         | COM Port | Node ID | MAC suffix |
| ------------ | -------- | ------- | ---------- |
| Base Station | COM13    | 23768   | `eb:5c:d8` |
| Rover B      | —        | 57936   | `e2:50`    |
| Rover C      | —        | 54272   | `d4:00`    |

## Build and Flash

Requires **ESP-IDF v6.0** with `IDF_PATH` set.

```bash
# Build
idf.py build

# Flash
idf.py -p COM13 flash

# Monitor serial output
idf.py -p COM13 monitor

# Build, flash, and monitor in one step
idf.py -p COM13 flash monitor
```

`menuconfig` is not needed — no router SSID, no mesh config. Channel and band are hardcoded to 5 GHz channel 36.

## Configuration

All tunable constants are at the top of `main/mesh_main.c`:

| Constant                | Default              | Notes                                                 |
| ----------------------- | -------------------- | ----------------------------------------------------- |
| `BASE_STATION_NODE_ID`  | 23768                | Match to your BS hardware                             |
| `ESPNOW_CHANNEL`        | 36                   | 5 GHz UNII-1, 5180 MHz, no DFS — all nodes must match |
| `PEER_TIMEOUT_MS`       | 15000                | ms before an unheard peer is marked inactive          |
| `HOST_UART_PORT`        | `UART_NUM_0`         | Bench: USB; swap to `UART_NUM_1` for Jetson           |
| `HOST_UART_TX_PIN`      | `UART_PIN_NO_CHANGE` | UART0 fixed by bootloader; set GPIO 6/7 for UART1     |
| `HOST_UART_RX_PIN`      | `UART_PIN_NO_CHANGE` | Same                                                  |
| `HOST_PULL_INTERVAL_MS` | 1000                 | How often ESP32 pulls bundles from Jetson (ms)        |
| `MAX_BUNDLES_IN_RAM`    | 100                  | Bundle store slots per node                           |
| `BUNDLE_PAYLOAD_SIZE`   | 1024                 | Max payload bytes per bundle                          |

## Architecture

### Node roles (one firmware, runtime branch)

`app_main` reads the node ID from the STA MAC and branches:

- **Base Station** (`node_id == BASE_STATION_NODE_ID`) — runs the full DTN stack described below (`rx_process_task`, `beacon_task`, `bundle_tx_task`, `host_uart_task`, `ui_update_task`). Unchanged; this is what `mesh_visualizer.py` consumes.
- **Rover** (anything else) — runs only the relay tasks `relay_rx_task` + `relay_host_task` (plus `ui_update_task`). The DTN logic still compiles but never runs on a rover; it lives in `rover_daemon.py`.

**Rover relay path**: `relay_rx_task` dequeues each received ESP-NOW packet and ships it to the Jetson as a `WIFI_RX` frame `[src_mac:6][rssi:1][air bytes]`. `relay_host_task` reads frames from the Jetson — `WIFI_TX` payloads are broadcast verbatim, `QUERY_STATUS` returns node ID + channel for auto-detection. No bundle store, beacons, or peer table on this path.

### Rover daemon (`rover_daemon.py`)

Reproduces the rover-side behavior in Python so the BS sees identical traffic: 1 s beacon TX, a peer table keyed by node ID (resets `forwarded` on new/returning/restarted peers), bundle RX with dedup + telemetry-ACK (`@DTN_RX:`) generation, a 1 s epidemic forward loop (rewrites `prev_node`/`hop_count` per hop, re-sends bundles unacked by the BS after 5 s), and ACK-map apply + propagate (256-bit sliding window per source). Compatible with Python 3.6+.

### Firmware (`main/mesh_main.c`) — Base Station stack

**Bundle store**: Flat array of 100 `ram_bundle_t` slots. Each holds a `dtn_bundle_t` (source, dest, prev_node, sequence number, creation_time, lifetime/TTL, hop count/limit, payload) plus `is_empty` and `forwarded` flags. A FreeRTOS mutex protects both the bundle store and peer list.

**`beacon_task`**: Broadcasts a `beacon_pkt_t` (node_id + timestamp_ms) every 1 s to the ESP-NOW broadcast MAC. Used for peer discovery and clock-offset measurement.

**`rx_process_task`**: Receives from a queue populated by the ESP-NOW recv callback. Handles:

- `PKT_TYPE_BEACON` — calls `add_or_refresh_peer`, updates clock offset at BS, emits `@NET:` line.
- `PKT_TYPE_BUNDLE` — deduplicates by `(source_node, creation_time, sequence_number)`, stores bundle, generates telemetry ACK on rovers, prints `@METRIC:` + `@DTN_RX:` at BS.
- `PKT_TYPE_ANTIPKT` — marks matching stored bundles for deletion (BS-originated suppression).

**`bundle_tx_task`**: Runs every 1 s. Expires timed-out peers, generates a data bundle every 10 s, and sends un-forwarded bundles to all active peers. Never holds the mutex during `esp_now_send`.

**`host_uart_task`**: Binary framing protocol over UART for Jetson ↔ ESP32 communication. Handles `TX_BUNDLE`, `QUERY_STATUS`, `QUERY_PEERS` from Jetson, and pushes received neighbor bundles to Jetson via `BUNDLE_PUSH`. Pulls Jetson-stored bundles every `HOST_PULL_INTERVAL_MS` when peers are active.

**Clock sync**: BS tracks per-peer `clock_offset_ms = peer_timestamp − bs_now_ms` from beacons. Latency in `@METRIC:` lines is corrected for boot-time skew.

### Host UART Protocol

Binary framing over UART. Frame format:

```
[SOF: 0xAA][CMD: 1][LEN_LO][LEN_HI][PAYLOAD: LEN bytes][CRC16_LO][CRC16_HI]
```

CRC-16/CCITT over `[CMD, LEN_LO, LEN_HI, PAYLOAD...]`.

| CMD              | Hex    | Direction      | Payload                                                           |
| ---------------- | ------ | -------------- | ----------------------------------------------------------------- |
| `TX_BUNDLE`      | `0x01` | Jetson → ESP32 | raw `dtn_bundle_t` bytes                                          |
| `QUERY_STATUS`   | `0x02` | Jetson → ESP32 | none                                                              |
| `QUERY_PEERS`    | `0x03` | Jetson → ESP32 | none                                                              |
| `PULL_REQ`       | `0x10` | ESP32 → Jetson | none                                                              |
| `BUNDLE_DATA`    | `0x11` | Jetson → ESP32 | raw `dtn_bundle_t` bytes                                          |
| `NO_BUNDLES`     | `0x12` | Jetson → ESP32 | none                                                              |
| `ANTIPKT_NOTIFY` | `0x13` | ESP32 → Jetson | `antipkt_id_t` (10 bytes)                                         |
| `BUNDLE_PUSH`    | `0x14` | ESP32 → Jetson | raw `dtn_bundle_t` bytes                                          |
| `ACK`            | `0x20` | ESP32 → Jetson | 1 byte (0=ok, 1=error)                                            |
| `STATUS_RESP`    | `0x21` | ESP32 → Jetson | `host_status_t` (6 bytes)                                         |
| `PEERS_RESP`     | `0x22` | ESP32 → Jetson | n × `host_peer_entry_t` (4 bytes each)                            |
| `WIFI_TX`        | `0x30` | Jetson → ESP32 | raw over-air bytes, broadcast verbatim (rover relay)              |
| `WIFI_RX`        | `0x31` | ESP32 → Jetson | `[src_mac:6][rssi:1][air bytes]` per received frame (rover relay) |

**Rover relay mode** (`rover_daemon.py`) uses only `WIFI_TX` / `WIFI_RX`, plus `QUERY_STATUS` / `STATUS_RESP` for node-ID detection. The bundle-centric commands (`TX_BUNDLE`, `PULL_REQ`, `BUNDLE_DATA`, `NO_BUNDLES`, `ANTIPKT_NOTIFY`, `BUNDLE_PUSH`) belong to the legacy `jetson_daemon.py` model. `HOST_MAX_PAYLOAD` is 1100 bytes (a relayed bundle is up to 6 + 1 + 1053 bytes).

### BS Serial Output (for visualizer)

| Prefix           | Format                                           | Meaning                                                                                                                            |
| ---------------- | ------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------- |
| `@NET:`          | `@NET:<node>:<parent>:<rssi>`                    | Topology heartbeat (BS emits self-heartbeat `@NET:<bs_id>:0:0` every 5 s)                                                          |
| `@DTN_RX:`       | `@DTN_RX:<src>:<prev>:<seq>:<receiver>:<hops>`   | Bundle received at BS                                                                                                              |
| `@METRIC:`       | `@METRIC:<src>:<seq>:<hops>:<latency_ms>`        | End-to-end latency (clock-corrected)                                                                                               |
| `@LOCATION:`     | `@LOCATION:<node>:<x>:<y>:<z>:<rssi>`            | Rover position in metres (x/y/z) with BS RSSI at that point; emitted by the rover daemon every 5 s when location data is available |
| `@ANTIPKT_FAIL:` | `@ANTIPKT_FAIL:<src>:<seq>:<prev>`               | Duplicate from same forwarder — antipacket didn't reach that node                                                                  |
| `@FERRY_DUP:`    | `@FERRY_DUP:<src>:<seq>:<first_prev>:<dup_prev>` | Same bundle arrived via two different forwarders (expected epidemic behaviour)                                                     |
| `@NODE_RESTART:` | `@NODE_RESTART:<node_id>`                        | BS detected a rover's `boot_id` changed; visualizer archives per-node metrics and resets counters for that node                    |

## Python Tools

Install dependencies once:

```bash
pip install pyserial networkx matplotlib
```

### Mesh Visualizer

Connects to the base station serial port and animates a three-panel live display updated every 500 ms.

```bash
SERIAL_PORT=COM13 python mesh_visualizer.py        # Windows
SERIAL_PORT=/dev/ttyUSB0 python mesh_visualizer.py # Linux/macOS
# or edit SERIAL_PORT at the top of the file
```

**Panels:**

- **Mesh graph** — live topology with RSSI-labelled edges; active bundle transfers highlighted (red = direct, orange dashed = ferried).
- **Bundle events / metrics** — scrolling delivery log, per-node packet count and average latency, current node positions with RSSI.
- **Radio map** — metre-scale scatter plot built from `@LOCATION:` messages. Each point is coloured by RSSI (`RdYlGn` colormap: red = weak, green = strong). The current rover position is shown with a white-outlined marker that fades if no update has arrived recently. Map range is controlled by `RADIO_MAP_HALF_M` at the top of the file (default `25.0` = ±25 m).

**Session management** — on serial reconnect or BS reboot detection (`"I am the Base Station"` in stream), the current session is saved to `bs_reports/` and state resets. On `@NODE_RESTART:`, only the named node's metrics reset, preserving data from other nodes. At exit (`Ctrl-C` or window close) a full summary is printed and saved.

**Output files** (all written to `bs_reports/`):

| File                          | Contents                                                 |
| ----------------------------- | -------------------------------------------------------- |
| `dtn_summary_<timestamp>.txt` | Full session delivery/latency/RSSI report                |
| `node_<id>_boot<n>.txt`       | Per-node per-boot summary saved on each `@NODE_RESTART:` |
| `coverage_<timestamp>.csv`    | Every `(timestamp, node, x, y, z, rssi)` location sample |

### BS Ferry Monitor

Watches `@DTN_RX:` lines and prints a live table showing which bundles were delivered directly vs. ferried by an intermediary rover.

```bash
python bs_ferry_monitor.py --port COM13
python bs_ferry_monitor.py --port /dev/ttyUSB0 --baud 115200
```

### Rover Daemon

Runs on the **Jetson Orin Nano** aboard each rover. Owns the bundle store and all DTN logic, driving the rover's ESP32 (which is a dumb relay) over UART. Only needs `pyserial`; compatible with Python 3.6+.

```bash
# Auto-detect node ID from ESP32 STATUS_RESP on startup
python rover_daemon.py --serial /dev/ttyUSB0

# Explicit options
python rover_daemon.py --serial /dev/ttyTHS1 --baud 115200 --interval 1 --node-id 57936
```

| Flag         | Default        | Notes                                                  |
| ------------ | -------------- | ------------------------------------------------------ |
| `--serial`   | `/dev/ttyUSB0` | Serial port connected to the ESP32                     |
| `--baud`     | 115200         | Must match `HOST_UART_BAUD` in firmware                |
| `--interval` | 1              | Seconds between generated sensor bundles (0 = disable) |
| `--node-id`  | auto           | Override node ID (normally auto-detected from ESP32)   |

> `jetson_daemon.py` is the **legacy** daemon for the old model where the ESP32 held the DTN logic (PULL/BUNDLE_PUSH). It is superseded by `rover_daemon.py` and kept only for reference.

## Jetson Deployment

1. In `main/mesh_main.c`, change:
   ```c
   #define HOST_UART_PORT    UART_NUM_1
   #define HOST_UART_TX_PIN  6   // ESP32 TX → Jetson RX
   #define HOST_UART_RX_PIN  7   // ESP32 RX ← Jetson TX
   ```
2. Rebuild and flash: `idf.py -p <PORT> flash`
3. Wire the ESP32:
   - GPIO 6 (TX) → Jetson UART RX
   - GPIO 7 (RX) ← Jetson UART TX
   - GND shared
4. On the Jetson, run:
   ```bash
   python rover_daemon.py --serial /dev/ttyTHS1
   ```
5. UART0 remains free on the ESP32 for `idf.py monitor` debugging.

> **Bench note**: When `HOST_UART_PORT = UART_NUM_0`, ASCII log lines and binary frames are interleaved on the same USB serial port. The visualizer and ferry monitor demux on the `0xAA` SOF byte automatically. Do not run `idf.py monitor` at the same time as any Python tool in this mode.

## Known Issues / Gotchas

**Band mode call order**: `esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)` must be called _after_ `esp_wifi_start()`. Calling it before returns `ESP_ERR_WIFI_NOT_STARTED`.

**Peer return not resetting `forwarded` flags**: When a timed-out peer comes back, its MAC is already in `peer_list` with `active=false`. The fix detects the inactive→active transition, resets `forwarded=false` on all stored bundles, and re-registers the peer with `esp_now_add_peer` (removed on timeout).

**UART RTS reset (Linux)**: On `/dev/ttyUSB*`, set `rts=False` immediately after `serial.Serial()` opens, or the ESP32-C5 silently resets and is unresponsive.

## Why ESP-NOW (not ESP-WIFI-MESH)

ESP-WIFI-MESH requires a router/AP to function and its root election, candidacy, and topology management all assume persistent internet uplink. For a DTN network where connectivity is **intentionally intermittent**, these assumptions are fundamentally wrong. See `TRANSITION_TO_ESP_NOW.md` for the full analysis and all bugs encountered.
