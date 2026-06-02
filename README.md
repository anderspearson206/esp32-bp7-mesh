# ESP32-C5 DTN Rover Mesh

A Delay-Tolerant Networking (DTN) mesh for a fleet of autonomous ground rovers. Each rover carries a **Jetson Orin Nano** as its main computer and an **ESP32-C5** as a wireless modem. The ESP32s communicate over **ESP-NOW** and implement epidemic-style store-and-forward routing using a BPv7-inspired bundle format.

**Deployment**: 3 rovers + 1 fixed base station (BS). Most data flows rover → BS. Rovers follow pre-planned trajectories with intermittent connectivity — bundles are carried (ferried) across disconnections by intermediary rovers.

## Status

| Feature | State |
|---|---|
| ESP-NOW transport | Done |
| Epidemic store-and-forward | Done |
| DTN ferrying (verified) | Done |
| Clock sync / latency correction | Done |
| Host UART interface (Jetson ↔ ESP32) | Done |
| Antipackets (suppress redelivery) | Done |
| LCD / LED / button UI on BS | Done |
| Downlink / ACK to rovers | Not yet done |
| Integration onto rover hardware | Not yet done |

## Hardware

- **MCU**: ESP32-C5 (5 GHz only, no DFS channel 36)
- **Rover host**: Jetson Orin Nano, communicates over UART1 (GPIO 6/7)
- **BS peripherals**: 16×2 I2C LCD (PCF8574, addr `0x27`), pushbutton, status LED

### GPIO Pinout

| Pin | Function |
|---|---|
| GPIO 9 | Pushbutton (BS only) |
| GPIO 10 | Status LED (all nodes) |
| GPIO 7 | LCD SDA / UART1 TX to Jetson |
| GPIO 8 | LCD SCL |

> When using UART1 for the Jetson interface, the LCD must be removed — they share GPIO 7/8.

### Node IDs (bench setup)

Node IDs are derived from `(mac[4] << 8) | mac[5]` of the STA MAC, printed on boot.

| Node | COM Port | Node ID | MAC suffix |
|---|---|---|---|
| Base Station | COM13 | 23768 | `eb:5c:d8` |
| Rover B | — | 57936 | `e2:50` |
| Rover C | — | 54272 | `d4:00` |

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

| Constant | Default | Notes |
|---|---|---|
| `BASE_STATION_NODE_ID` | 23768 | Match to your BS hardware |
| `ESPNOW_CHANNEL` | 36 | 5 GHz UNII-1, 5180 MHz, no DFS — all nodes must match |
| `PEER_TIMEOUT_MS` | 15000 | ms before an unheard peer is marked inactive |
| `HOST_UART_PORT` | `UART_NUM_0` | Bench: USB; swap to `UART_NUM_1` for Jetson |
| `HOST_UART_TX_PIN` | `UART_PIN_NO_CHANGE` | UART0 fixed by bootloader; set GPIO 6/7 for UART1 |
| `HOST_UART_RX_PIN` | `UART_PIN_NO_CHANGE` | Same |
| `HOST_PULL_INTERVAL_MS` | 1000 | How often ESP32 pulls bundles from Jetson (ms) |
| `MAX_BUNDLES_IN_RAM` | 100 | Bundle store slots per node |
| `BUNDLE_PAYLOAD_SIZE` | 1024 | Max payload bytes per bundle |

## Architecture

### Firmware (`main/mesh_main.c`)

**Bundle store**: Flat array of 100 `ram_bundle_t` slots. Each holds a `dtn_bundle_t` (source, dest, prev\_node, sequence number, creation\_time, lifetime/TTL, hop count/limit, payload) plus `is_empty` and `forwarded` flags. A FreeRTOS mutex protects both the bundle store and peer list.

**`beacon_task`**: Broadcasts a `beacon_pkt_t` (node\_id + timestamp\_ms) every 1 s to the ESP-NOW broadcast MAC. Used for peer discovery and clock-offset measurement.

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

| CMD | Hex | Direction | Payload |
|---|---|---|---|
| `TX_BUNDLE` | `0x01` | Jetson → ESP32 | raw `dtn_bundle_t` bytes |
| `QUERY_STATUS` | `0x02` | Jetson → ESP32 | none |
| `QUERY_PEERS` | `0x03` | Jetson → ESP32 | none |
| `PULL_REQ` | `0x10` | ESP32 → Jetson | none |
| `BUNDLE_DATA` | `0x11` | Jetson → ESP32 | raw `dtn_bundle_t` bytes |
| `NO_BUNDLES` | `0x12` | Jetson → ESP32 | none |
| `ANTIPKT_NOTIFY` | `0x13` | ESP32 → Jetson | `antipkt_id_t` (10 bytes) |
| `BUNDLE_PUSH` | `0x14` | ESP32 → Jetson | raw `dtn_bundle_t` bytes |
| `ACK` | `0x20` | ESP32 → Jetson | 1 byte (0=ok, 1=error) |
| `STATUS_RESP` | `0x21` | ESP32 → Jetson | `host_status_t` (6 bytes) |
| `PEERS_RESP` | `0x22` | ESP32 → Jetson | n × `host_peer_entry_t` (4 bytes each) |

### BS Serial Output (for visualizer)

| Prefix | Format | Meaning |
|---|---|---|
| `@NET:` | `@NET:<node>:<parent>:<rssi>` | Topology heartbeat |
| `@DTN_RX:` | `@DTN_RX:<src>:<prev>:<seq>:<receiver>:<hops>` | Bundle received at BS |
| `@METRIC:` | `@METRIC:<src>:<seq>:<hops>:<latency_ms>` | End-to-end latency (clock-corrected) |
| `@ANTIPKT_FAIL:` | `@ANTIPKT_FAIL:<src>:<seq>:<prev>` | Duplicate from same forwarder |
| `@FERRY_DUP:` | `@FERRY_DUP:<src>:<seq>:<first_prev>:<dup_prev>` | Bundle arrived via two forwarders |

## Python Tools

Install dependencies once:
```bash
pip install pyserial networkx matplotlib
```

### Mesh Visualizer

Connects to the base station serial port, parses `@NET:` / `@DTN_RX:` / `@METRIC:` lines, and animates the live network graph.

```bash
# Set port via environment variable, then run
SERIAL_PORT=COM13 python mesh_visualizer.py

# Or edit SERIAL_PORT at the top of the file
python mesh_visualizer.py
```

On Linux/macOS substitute `SERIAL_PORT=/dev/ttyUSB0`.

### BS Ferry Monitor

Watches `@DTN_RX:` lines and prints a live table showing which bundles were delivered directly vs. ferried by an intermediary rover.

```bash
python bs_ferry_monitor.py --port COM13
python bs_ferry_monitor.py --port /dev/ttyUSB0 --baud 115200
```

### Jetson Daemon

Runs on the **Jetson Orin Nano** aboard each rover. Manages the local bundle store and bridges it to the attached ESP32 modem over UART.

```bash
# Auto-detect node ID from ESP32 STATUS_RESP on startup
python jetson_daemon.py

# Explicit options
python jetson_daemon.py --serial /dev/ttyTHS1 --baud 115200 --interval 10 --node-id 57936
```

| Flag | Default | Notes |
|---|---|---|
| `--serial` | `/dev/ttyTHS1` | Jetson UART1 connected to ESP32 |
| `--baud` | 115200 | Must match `HOST_UART_BAUD` in firmware |
| `--interval` | 10 | Seconds between generated sensor bundles (0 = disable) |
| `--node-id` | auto | Override node ID (normally auto-detected from ESP32) |

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
   python jetson_daemon.py --serial /dev/ttyTHS1
   ```
5. UART0 remains free on the ESP32 for `idf.py monitor` debugging.

> **Bench note**: When `HOST_UART_PORT = UART_NUM_0`, ASCII log lines and binary frames are interleaved on the same USB serial port. The visualizer and ferry monitor demux on the `0xAA` SOF byte automatically. Do not run `idf.py monitor` at the same time as any Python tool in this mode.

## Known Issues / Gotchas

**Band mode call order**: `esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)` must be called *after* `esp_wifi_start()`. Calling it before returns `ESP_ERR_WIFI_NOT_STARTED`.

**Peer return not resetting `forwarded` flags**: When a timed-out peer comes back, its MAC is already in `peer_list` with `active=false`. The fix detects the inactive→active transition, resets `forwarded=false` on all stored bundles, and re-registers the peer with `esp_now_add_peer` (removed on timeout).

**UART RTS reset (Linux)**: On `/dev/ttyUSB*`, set `rts=False` immediately after `serial.Serial()` opens, or the ESP32-C5 silently resets and is unresponsive.

## Why ESP-NOW (not ESP-WIFI-MESH)

ESP-WIFI-MESH requires a router/AP to function and its root election, candidacy, and topology management all assume persistent internet uplink. For a DTN network where connectivity is **intentionally intermittent**, these assumptions are fundamentally wrong. See `TRANSITION_TO_ESP_NOW.md` for the full analysis and all bugs encountered.
