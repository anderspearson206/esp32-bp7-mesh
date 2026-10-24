# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This project builds a DTN (Delay-Tolerant Networking) mesh architecture for a fleet of autonomous ground rovers. Each rover carries a Jetson Orin Nano as its main computer and an ESP32-C5 as its wireless "modem." The ESP32s communicate over ESP-NOW and implement epidemic-style store-and-forward routing using a custom bundle struct inspired by BPv7.

**Deployment target**: 3 rovers + 1 fixed base station (BS), designed to scale to N rovers. Most data flows rover → BS. Rovers follow pre-planned trajectories that affect connectivity windows — the network must tolerate intermittent links and carry bundles across disconnections.

**ESP32 role**: On **rover** nodes the ESP32 is now a *pure broadcast relay* — it broadcasts every over-air frame the Jetson hands it and ships every received frame back to the Jetson (with src MAC + RSSI). All DTN logic (beacons, bundle store, epidemic routing, antipacket/dedup, ACK-maps) lives in `rover_daemon.py` on the Jetson. The **Base Station** ESP32 still runs the full firmware stack unchanged (see below).

### Current Status (as of 2026-06-16)

- **Architecture split: rover ESP32 = dumb relay** — all routing/antipacket/storage logic moved off the rover ESP32 onto `rover_daemon.py`. The rover firmware path is just `relay_rx_task` (air → serial) + `relay_host_task` (serial → broadcast). The **over-air wire format is unchanged** so the BS interoperates with no firmware change.
- **Base Station: unchanged** — the BS keeps the full firmware stack and still emits `@NET/@METRIC/@DTN_RX/...` on its serial port, read directly by `mesh_visualizer.py`. A single firmware binary branches on `node_id` at boot (BS = full stack, rover = relay).
- **Transport layer: ESP-NOW** — complete and verified. See `TRANSITION_TO_ESP_NOW.md` for history of why ESP-WIFI-MESH was abandoned.
- **DTN ferrying: verified** — rover B successfully carried rover C's bundles to the BS across a disconnection. `forwarded` flags reset correctly when a timed-out peer returns (logic now ported to `rover_daemon.py`).
- **Clock sync: implemented** — BS tracks per-peer clock offsets from beacon timestamps; latency in `@METRIC:` lines is corrected for boot-time skew.
- **Rover hardware integration started (2026-10-13)**: rover B's ESP32 is mounted on the MuSHR car `goose` (see "Rover Hardware" below). `radio_daemon.py` runs there as a systemd service, sends T265 visual-odometry location to the BS (wheel odometry until 2026-10-14), and the BS dashboard can start/stop the car's ROS stack over the mesh.
- **Pose fusion + route driving (2026-10-15)**: on `goose` the pose comes from `rover_nav.py`: distance from the wheels, heading from the T265 (the T265 alone under-read distance ~6x outdoors). The dashboard can plan an A* route through clicked waypoints and send it; the car follows it while RB is held. Indoor test worked; not yet tested outdoors at the fountain. See "Route driving" below.
- **Second rover (2026-10-20)**: rover C's ESP32 is on the second MuSHR car (`192.168.2.22`), set up with the same files and services as rover 1.
- **Odometry calibration (2026-10-20)**: wheel distance gain set on both cars, and both cars' steering centres (see "Calibration" under Rover Hardware). Rover 1's steering centre was redone on 2026-10-21 after a loose front-wheel part was fixed.
- **Not yet done**: downlink/ACK, two-rover test on hardware, re-verify ferrying with the relay/daemon split on hardware.
- **Relay role (2026-10-22)**: rover-side relay brain + dashboard "Relay" box + "Drive without RB" built and tested offline (see "Relay role" and "Route driving"). On both cars: new files deployed (previous versions as `*.bak_20261022` in the `src/` folder), terrain in `~/terrain`, numpy installed, mux `Autonomy` input added. `radio_daemon.service` has `--site fountain --terrain-dir /home/robot/terrain` on rover 1; rover 2's still needs it. Link table being generated on Colab.
- **Next steps (2026-10-20)**:
  1. ~~Redo rover 1's steering centre~~: done 2026-10-21 (0.445).
  2. Start pads + drive-out heading fix on the dashboard (no rover change). A pad is a start point + direction on the site map, one per rover near the BS. To align, put the car on the pad's start mark, press Start, drive ~5 m along a marked line, press Done; the heading comes from the direction actually driven (not where the car points), the position from the start mark. Plus a quick "Place at pad" (heading by eye) and an "At pad" loop-closure check logged per session. Still to decide: pads tied to a straight edge visible on the satellite map (preferred; a phone compass is only ±5°) or relative to the BS, and whether 5 m fits.
  3. Two-rover outdoor test, logging loop-closure / end-of-route error per rover.
  4. Optional: rover 2's low-speed sputter (see "Low-speed limit" under Rover Hardware; options there), servo-saver play on both cars, a persistent journal on the Jetsons (`sudo mkdir -p /var/log/journal`) to diagnose power cuts, pin rover 1's AP channel.

### Testing Constraints

All 3 nodes connect to the same Windows machine for bench testing:
- **BS** = COM13, node ID 23768, MAC `d0:cf:13:eb:5c:d8`
- **Rover B** = node 57936, MAC ending `e2:50`
- **Rover C** = node 54272, MAC ending `d4:00`

Node IDs are derived from `(mac[4] << 8) | mac[5]` using the STA MAC. Printed on boot as "I am the Base Station" / "I am a Rover".

Only one additional computer (physically movable) and one node on a power bank are available outside the bench setup.

### Rover Hardware: MuSHR cars (rover 1 = `.21`, rover 2 = `.22`)

- **Names** (2026-10-22): the user uses **rover 1 = rover A = `.21`** (ESP32 node 57936) and **rover 2 = rover B = `.22`** (node 54272) interchangeably. The older "rover B"/"rover C" labels in this file are the ESP32 nodes from bench testing (B = 57936 on `.21`, C = 54272 on `.22`), so "rover B" from the user means `.22`. Either car can be explorer or relay; both have the same files and setup.
- **Two identical MuSHR cars**, both with hostname `goose`. Refer to them by Ethernet address, not hostname: **rover 1** = `robot@192.168.2.21`, carries **rover B (node 57936)**; **rover 2** = `robot@192.168.2.22` (added 2026-10-20), carries **rover C (node 54272)**. Same build (racecar-uw-nano, same VESC and wheelbase, T265 on both). Everything below was set up on rover 1 and applies to both unless noted.
- **Login**: `robot@192.168.2.21` / `robot@192.168.2.22` (Jetson, Ubuntu 18.04, ROS melodic, MuSHR catkin workspace in `~/catkin_ws/src/mushr`). The ESP32 is `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` on both (`/dev/ttyUSB0`).
- **Rover 2 differences**: AP profile and SSID are `Robot AP` (not `Robot AP 36`), pinned to 2.4 GHz ch 6 with power save off (2026-10-20); rover 1's AP is still on auto (ch 1 when last checked). Its OPTIONS mission differs (`astar_planner.launch` drive_speed 0.4, `goals.yaml` has an extra return to (0, 0)); left as is. Its `sensors.launch` disables the D435 (and the lidar/push-button includes) in one comment block, and there is no `sensors.launch.bak`. Its original `teleop.launch` (T265 inside `/car` as `t265`) is in `teleop.launch.bak`; the live one is rover 1's copy. The June `radio_daemon.py` was kept as `radio_daemon_jun.py`.
- **Calibration** (2026-10-20, `mushr_base/vesc/vesc_main/config/racecar-uw-nano/vesc.yaml`, originals in `vesc.yaml.orig`; values are read only at stack start, so Stop/Start ROS after editing):

  | | `speed_to_erpm_gain` | `steering_angle_to_servo_offset` |
  |---|---|---|
  | rover 1 (.21) | -3745 (stock -3500; check runs 100.2%, 99.5% and 99.3% on 2026-10-21) | 0.445 (stock 0.545; 2026-10-21, after the front-wheel fix: left/right releases -8.3° / +3.3° over 6.6 m) |
  | rover 2 (.22) | -3690 (stock -3500; check runs 100.6%) | 0.485 (stock 0.45) |

  Measured with `calib_distance.py` (repo, copied next to `rover_nav.py` on both cars): drive a taped 6.6 m line; it compares `/car/vesc/odom` path length with the true distance, logs speed per run, and estimates the steering centre from the T265 heading change on runs where the right stick wasn't touched (it watches `/car/teleop/joy` axis 2). With SSH over Ethernet, run it as `nohup python -u calib_distance.py --true 6.6 --joy > ~/calib.log 2>&1 &`, unplug, and use the controller: X start/stop a run, square cancel/drop, triangle finish (`/car/teleop/joy` indices on these controllers: 0 square, 1 cross, 2 circle, 3 triangle; no rumble). It suggests values but writes nothing. Findings: the stock gain made the wheel odometry read ~5–7% long; steered runs still count for distance (path length). **Steering has a lot of spring play (servo saver)**: after a release the wheels settle a few degrees either side of centre depending on the last direction, so a single drift run is meaningless. Centre the offset with two drift runs, one after releasing the stick from full left and one from full right, and aim for their mean heading change to be ~0 (rover 2 at 0.485: -8.0° / +7.9° over 6.6 m). Don't straighten the wheels by hand before a run (that's what made rover 1's first offset runs misleading). Higher offset = steers left, also on rover 2 (which has `invert_servo: true`). The script's offset suggestion uses `steering_angle_to_servo_gain` (0.95), but on rover 1 a change moves the drift about twice as much as it predicts: after the front-wheel fix, 0.01 moved it ~24° over 6.6 m, so step by ~0.001 per 2.5° there. The fix shifted rover 1's centre a lot (pre-fix values ~0.55 no longer apply) and left it with much less play (left/right releases within ~3°). Rover 2's VESC overshoots at low speed: even a light throttle spikes to 2–2.5 m/s briefly (above the ~0.93 m/s cap from `speed_max: 3250` erpm), so expect a lurch when a route starts; see "Low-speed limit" below. The T265's own distance read ~5–15% long indoors (vs ~6x short outdoors), so it stays heading-only in the fused pose.
- **Low-speed limit (2026-10-21, not fixed)**: both VESCs run firmware 2.18 with **byte-identical motor configs** (sensorless BLDC, read over `/dev/vesc` with the stack stopped), so the cars differ only in hardware. Below **900 ERPM (~0.24 m/s)** the VESC speed controller doesn't drive the motor at all, on both cars, so any speed command between 0 and ~0.25 m/s does nothing (this includes the dashboard's 0.2 m/s route minimum). Wheels-off-ground step test: rover 2 draws 15–20% more current at every speed (slightly more drag or a weaker motor; the drag feels the same by hand, and the motor/phase wires are hard to reach) and is rough right at 900 ERPM (519–1361 ERPM, current spikes to 2.45 A); above 1200 ERPM both are steady. On the ground the sensorless start from standstill near the cutoff gives the sputter/lurch, worse on rover 2. Options: (1) raise the dashboard route minimum to 0.3 m/s and/or have `rover_nav.py` round non-zero commands below 0.3 up to 0.3; (2) repeat the step test on the ground (900–1500 ERPM, ~3 m per car) to measure the loaded difference; (3) switch the VESCs to FOC (needs motor detection with a BLDC Tool matching fw 2.18, or a firmware upgrade; risky). The read-only query and step-test scripts (`vesc_query.py`, `vesc_ramp.py`; COMM_GET_MCCONF / COMM_SET_RPM over the VESC serial protocol) were only copied to `/tmp` on the cars, so they're gone after a reboot.
- **Power**: two batteries. The motor pack plugs into the VESC (drive motor + steering servo); the Jetson and sensors run off a separate pack. The Jetson can be fully up while the car can't move, so check both.
- **Wired**: `eth0`, `192.168.2.21` / `192.168.1.9`.
- **Wireless**: the Jetson hosts its own AP on `wlan0`. NM profile and SSID are both `Robot AP 36`, and the rover is at **`10.42.0.1`** (clients get `10.42.0.x`). Connect with `ssh robot@10.42.0.1`. The PSK is empty in the NM profile. The AP has no internet, so Windows may drop it for a known network; turn off auto-connect on other networks while driving. If the AP itself drops, disable Wi-Fi power save (`sudo iw dev wlan0 set power_save off`).
- **AP channel is auto**: band is unset and channel is `0`, so the "36" in the name is *not* the channel. The AP can land on 5 GHz ch 36, which is `ESPNOW_CHANNEL`, and then contend with the mesh. Check the live channel with `iw dev wlan0 info`. To pin it elsewhere (do this while on Ethernet, because it drops Wi-Fi): `sudo nmcli connection modify "Robot AP 36" 802-11-wireless.band a 802-11-wireless.channel 149 && sudo nmcli connection up "Robot AP 36"`.
- **Driving**: `mushr_launcher.service` (systemd, runs `mushr_base/scripts/controller_launcher.sh`) waits at boot for the controller on `/dev/input/js0`. It does **not** start ROS by itself: press **OPTIONS (button 10)** to start the stack (`roslaunch mushr_base car_startup.launch`, run as `robot` in the background; 2 beeps). **`car_startup.launch` = `teleop.launch` + `astar_planner`, so OPTIONS starts the autonomous goals mission (`mushr_base/config/goals.yaml`, drive_speed 1.0) and the car drives off by itself.** The launcher waits for `/dev/input/js0` before it listens for OPTIONS, so with no controller connected OPTIONS does nothing, and nothing can override a running planner except stopping the stack. Press **SHARE (button 9)** to stop it (SIGINT to that roslaunch; 1 long beep). The script lives at `mushr_base/scripts/controller_launcher.sh`. **A controller disconnect must not count as OPTIONS** (2026-10-23): the original `wait_button.py` crashed when `/dev/input/js0` vanished and the script launched `car_startup.launch` anyway, so a car driving out of Bluetooth range started the goals mission with nothing blocking it (rover 1, twice on 2026-10-22). Both cars now run the fixed pair (copies in the repo: `controller_launcher.sh`, `wait_button.py`; rover 1's originals in `*.bak_20261023`): `wait_button.py` exits 2 on a lost controller, the launcher starts only on exit 0, and a disconnect while an OPTIONS stack runs stops it. It is a strict OPTIONS→SHARE→OPTIONS loop: while it waits for one button, it ignores the other. Check it with `systemctl status mushr_launcher.service` / `journalctl -u mushr_launcher.service -f`. Don't also run a `roslaunch mushr_base ...` by hand: two stacks fight ("new node registered with same name"), and a manual launch survives an SSH drop as an orphan (find it with `pgrep -af roslaunch`). Controls: hold LB as the deadman, left stick for throttle, right stick for steering (mapping in `mushr_base/config/joy_teleop.yaml`). The motor battery must be plugged into the VESC. Without it, `/dev/vesc` doesn't exist and `vesc_driver` restart-loops.
- **Radio daemon on the car**: lives at `~/catkin_ws/src/mushr/mushr_base/mushr_base/mushr_base/src/radio_daemon.py` and runs at boot as `radio_daemon.service` (`/etc/systemd/system/radio_daemon.service`, `Restart=always`). The unit sources the ROS melodic + catkin setup and runs `python3 -u radio_daemon.py --serial /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --interval 1 --ros-location-topic /car/fused_odom --ros-control`, with `KillMode=process` so restarting the daemon doesn't kill a ROS stack it started. **Location comes from `rover_nav.py`** (`/car/fused_odom`, see "Route driving"): wheel distance + T265 heading by default, or the T265 alone if "Camera-only pose" is ticked on the dashboard. History: wheel odometry until 2026-10-14 (heading drifted with steering), then the T265 alone (`/camera/odom/sample`) for one fountain run, where it under-read distance about 6x (lost visual tracking on sunny pavement; its heading stayed good). The pose starts at (0, 0) whenever the stack (and `rover_nav.py`) starts, so place the rover on the map after Start ROS. Fused mode needs the motor battery (no `/car/vesc/odom` without the VESC). The x/y metres go into the `lat`/`lon` fields as-is. The daemon still has its own fusion (`--ros-wheel-topic /car/vesc/odom` with `--ros-location-topic /camera/odom/sample`) for use without `rover_nav.py`; don't combine it with `/car/fused_odom`. Files on the car: `radio_daemon.py`, `odom_fusion.py` and `rover_nav.py`, all in that `src/` folder (copy all three when updating). Under python3 the location reader runs `rostopic echo --noarr` (the arrays are the unused covariances). It restarts `rostopic echo` when it exits ("rostopic echo exited; restarting in 2s") and also when no message has arrived for `ROS_LOC_STALE_S` (10 s, "no ... messages for 10s"). The second case matters because a Stop/Start ROS restarts `roscore`, and `rostopic echo` would otherwise stay attached to the dead master forever without exiting. While the stack is off, expect that line every ~10 s. Logs: `journalctl -u radio_daemon -f`. To change flags, edit `ExecStart` and run `sudo systemctl daemon-reload && sudo systemctl restart radio_daemon`. Don't also run it by hand, because two copies will fight over the serial port.
- **T265 camera**: Intel RealSense T265 (serial 8122110458), on a USB 3 port; it needs a data-capable USB 3 Micro-B cable. **Everything is built and tested against librealsense 2.48.0** (`librealsense2*` 2.48.0-5ubuntu5~1.gbp156c1b, `ros-melodic-librealsense2` 2.48.0-1bionic.20210701.140003), installed on both rovers. The packages are **not** `apt-mark hold`-ed, so avoid `apt upgrade`: versions 2.54 and later dropped the T265. `rs-enumerate-devices` should list it with Product Id `0B37` (`03e7:2150` in `lsusb` means it's still in bootloader mode before librealsense has loaded the firmware). The driver is started by `teleop.launch use_t265:=true`, which `car_startup.launch` (OPTIONS) sets and which the radio start now passes explicitly (`teleop.launch` defaults it to `false`). It runs outside the `/car` namespace as `/camera/realsense2_camera` and publishes `/camera/odom/sample`, **not** `/car/t265/odom/sample`. With `use_t265` on, `racecar_state` subscribes to `/car/t265/odom/sample`, which nothing publishes, so the car's own pose/TF from `racecar_state` is likely not updating (not yet fixed; the radio daemon doesn't use it). **D435 driver disabled**: `mushr_hardware/launch/racecar-uw-nano/sensors.launch` used to start `rs_camera.launch` (a D435 driver with no device filter) 10 s after launch. It grabbed the T265 and fought the T265 driver (`failed to claim usb interface ... RS2_USB_STATUS_BUSY` every 6 s in the stack log). Its `<node>` is now commented out, and the original file is in `sensors.launch.bak`. If a D435 is ever fitted, re-enable it with `device_type:=d435` in its args. The T265 can get stuck after an interrupted open; unplug and re-plug it, then restart the stack. **A low Jetson battery stops the T265 from enumerating**: the driver node runs but the stack log repeats `No RealSense devices were found!` and `/camera/odom/sample` never appears (only `/camera/realsense2_camera_manager/bond`). Swap the Jetson battery, then restart the stack.
- **Remote ROS start/stop** (`--ros-control`): the BS dashboard's Start ROS / Stop ROS buttons put `"ros": {"<node>": ["on"|"off", cmd_id]}` into the control message. `bs_core.set_ros` keeps that field on every later control message. `RosStack` in `radio_daemon.py` acts on each `cmd_id` once and saves the last one to `rover_logs/ros_control_state.json`, so a reboot never replays an old command. A radio start runs **`teleop.launch use_t265:=true` only, never the planner**: an earlier version ran `car_startup.launch`, and the car drove off on its mission with no controller connected. After a radio start the car only moves while LB is held on a connected controller. A lit controller isn't necessarily delivering input: check that `/dev/input/js0` exists and that `rostopic echo /car/teleop/joy` changes when you press buttons. It treats any `roslaunch mushr_base ...` process (found via `/proc`) as "running", whoever started it, and never launches a second one. If a duplicate appears (OPTIONS pressed while a radio-started stack is up), it stops its own copy. Stack output goes to `rover_logs/ros_stack_<time>.log`. The status line reports `ros=on|off|stopping|na`. **Frozen controller = runaway** (2026-10-21, rover 1): the Bluetooth link can stall without the kernel logging a disconnect. `/dev/input/js0` stays, `joy_node` keeps republishing the last state (LB + throttle), and the car keeps driving after release; nothing in `joy_teleop` or `rover_nav` detects it. Check: `rostopic echo /car/teleop/joy` doesn't change when you touch the controller. Stop with the dashboard's Stop ROS or by unplugging the motor battery, then `bluetoothctl disconnect 28:C6:3F:66:69:5C`, power-cycle and charge the controller. A hidraw-silence watchdog was proposed but not built. Controller gotchas: a radio-started stack can't be stopped with SHARE (the launcher is waiting for OPTIONS), and after a radio stop of an OPTIONS-started stack, press SHARE once before OPTIONS works again.

### Route driving (`route_plan.py`, `rover_nav.py`)

- **Who drives the car (ackermann mux priorities)**: safety 10 > teleop 5 > navigation 1 > default 0 (`ackermann_cmd_mux/param/mux.yaml`). While a controller is connected and idle, `joy_teleop` sends a zero command on the teleop input, which blocks navigation; holding **RB** (button 5) stops that and lets navigation through; **LB** (button 4) is manual drive and always wins. With **no controller connected nothing blocks navigation**, which is why the OPTIONS planner (`car_startup.launch`) drove off by itself. The stock `astar_planner.py` isn't used for routes: it plans once from `goals.yaml` over the `map_server` map and steers on `/car/car_pose`, which isn't updated with the T265 on.
- **`rover_nav.py`** (ROS node, Python 2, in `teleop.launch` as `<node pkg="mushr_base" type="rover_nav.py" name="rover_nav" output="screen" />`, original in `teleop.launch.bak`; needs `odom_fusion.py` next to it and `chmod +x`). It publishes the pose on `/car/fused_odom` (50 Hz) and follows routes with pure pursuit (wheelbase 0.225 m, from `racecar-uw-nano/vesc.yaml`) on `/car/mux/ackermann_cmd_mux/input/navigation`. **It drives only while RB is held** and `/car/teleop/joy` is fresh, the heading source is live (`loc` fused/camera) and the wheel odometry is fresh. Otherwise it sends zero speed. RB is required by default. **"Drive without RB"** (dashboard checkbox in "Drive route", per rover, 2026-10-22): the BS sends `"rb": {"<node>": 0}`, the daemon writes `/tmp/rover_nav/rb.json` (a reboot requires RB again), and `rover_nav.py` publishes on an extra mux input **`Autonomy` (`input/autonomy`, priority 6, above teleop)** instead of navigation, because joy_teleop's idle zero on teleop would block navigation; `joy_teleop.yaml` is unchanged, so the OPTIONS planner still needs RB. It still needs fresh `/car/teleop/joy` (controller connected, else `nav=no_joy` and zero), and goes silent while LB is held so manual teleop takes over within the 0.2 s mux timeout (`nav=manual`). Status `rb=1|0`. The `Autonomy` block is in `ackermann_cmd_mux/param/mux.yaml` on both cars (checked 2026-10-23; rover 1's original in `mux.yaml.orig`). **Radio deadman** (2026-10-23, dashboard "No controller: radio deadman" next to the checkbox, default 8 s, 0 = off; `"dm": {"<node>": s}`): with RB off, a rover whose controller dropped (out of Bluetooth range) keeps driving while it heard the BS within that many seconds, directly or through a relay, then stops (`nav=no_radio`). A connected controller works as before (LB takes over). The daemon tracks the BS time the BS was last reachable: a copy straight from the BS (hop 0) gives its `t`, relayed copies carry `bsh` (a relayed copy's `t` is only the relay's clock estimate); rovers re-broadcast control every `CONTROL_REPEAT_S` = 2 s (was 5). It writes `/tmp/rover_nav/radio.json` `{t, bs_age}` every second; `rover_nav.py` treats a file older than 3 s as no BS. Status: `dm`, `go=joy|radio` (what lets it drive), `bsa` (s since the BS was reachable). A BS beacon heard directly also counts as BS contact (2026-10-23): on the double-ferry runs that day the BS's control copies were lost for minutes while its beacons got through, `bsa` climbed to 300-600 s and the deadman stopped cars that were in range. **Pose reset warning** (2026-10-23): `rover_nav.py` reports its start time (`ns` in the status); when that changes, or the stack is seen stopping/gone, for a rover with an alignment, the BS flags it (red banner on the Map tab, red "POSE RESET" next to the rover, Relay box line, `#POSE_RESET` in serial.log) until it is placed again. The frozen-controller runaway is not helped (a stuck LB just yields to the stale teleop). It stops for good at the end of the route, more than 2.5 m off the route, or on Stop from the BS. A route already in `route.json` when the node starts is never driven.
- **Pose source** (dashboard "Camera-only pose" checkbox in "Align rover to map"): `fused` (default; `odom_fusion.py`) or `camera` (the T265's own x/y/yaw). The BS sends `"pose": {"<node>": "fused"|"camera"}` on every control message; the daemon writes `/tmp/rover_nav/pose.json`; the node re-anchors on a switch so the pose doesn't jump (alignment and active route stay valid). Status reports `pm=` (mode in use) and `loc=` (`fused|wheel|wait` or `camera|no_t265`).
- **BS side**: Map tab "Drive route" box. Select the rover, "Add waypoints…", click points in driving order; `route_plan.Planner` runs A* over the drivable mask (steps within 3 m of the edge cost more, so routes keep to the middle of paths), smooths it, and the dashed orange preview starts at the rover's current map pose. "Send route" replans from the current pose, converts to the rover's odom frame with its alignment, simplifies (Douglas-Peucker, 0.4 m, raised until the control JSON fits in `CONTROL_MAX_JSON`, 700 bytes since 2026-10-23 (hard limit 1086, see "Control message loss" under Relay role); the firmware drops serial frames over `HOST_MAX_PAYLOAD` 1100, and a rover's ESP32 adds 7 bytes when it hands a received frame to the Jetson; a ring lap is ~29 points) and sends `"nav": {"<node>": [cmd_id, speed, [x0, y0, ...] odom decimetres]}` (or `[cmd_id, "stop"]`). Speed 0.2–1.0 m/s, default 0.5. Once the rover reports `nc>=cmd_id` the BS replaces the points with `[cmd_id, "ok"]`. Every route/stop goes to `routes.csv` in the session folder. `python route_plan.py --site fountain x,y x,y ...` plans and plots offline.
- **Rover side**: `radio_daemon.py`'s `NavBridge` acts on each nav `cmd_id` once (last saved in `rover_logs/nav_state.json`) and writes `/tmp/rover_nav/route.json`; the node writes `/tmp/rover_nav/status.json` (2 Hz), which the daemon puts into `@ROVER_STATUS` as `nav=` (`idle|hold_rb|no_t265|no_wheels|following|done|stopped_off_route|stopped_bs|no_node`), `ni=`/`np=` (metres along / route length), `xte=` (m off route). The rover's `[STATUS]` line shows the same.
- **Don't press OPTIONS while a route is active**: the stock planner publishes on the same navigation input. Place the rover on the map (with heading) after Start ROS, then send routes. First test with the wheels off the ground: they should turn only while RB is held, and stop on RB release, Stop and controller off.

### Relay role (`relay_brain.py`, 2026-10-22, not yet tested on hardware)

- **Idea**: an explorer rover is driven by hand; a relay rover keeps choosing a spot on the drivable area that maximises `min(BS<->relay, relay<->explorer)` predicted RSS and drives there with `rover_nav.py` (RB still required). Set from the dashboard Map tab "Relay" box (its own Relay and Explorer dropdowns, independent of the rover selected at the top, plus Swap; starting a relay re-selects the active pair; then speed, "ferry after", comfort dBm, gain dB, look-ahead s; `"relay": {"<node>": [explorer, speed, ferry_s, comfort, gain, predict_s]}`). Pressing Start relay again with the same explorer applies changed settings without resetting the relay. Stop (either box) turns the relay off and stops its route; a hand-sent route also turns it off.
- **Fleet picture**: the BS sends unversioned `fleet` (each rover's map pose + BS time of the fix), `align` (each rover's odom->map alignment + time set), `bs` and `site` (`bs_core._live_fields`). Since 2026-10-23 they go in a second control copy with **version 0** next to each versioned copy (BS every 2 s, rovers likewise when they relay), so the versioned copy stays small; rovers read `t`, `hop`, `bsh` and the fleet fields from every copy before the version check and never adopt version 0. Rovers keep the newest entry per node and add what they hold to their relays, so the relay knows the explorer's position even out of direct range. It also uses the explorer's own location broadcast (`LOCATION_INTERVAL_S`, now 2 s) when that's newer. Non-finite alignments are never sent (rover C's saved alignment in `bs_reports/alignment_fountain.json` was all NaN on 2026-10-22).
- **Link model** (dashboard "Model" per relay; 7th relay cfg field): `nn_pmnet` (default), `nn` or `distance`. The rover loads every `<site>_linktab_*.npz` in its terrain folder at startup (both cars and the laptop keep them in the terrain folder, `../terrain` on the laptop, `~/terrain` on the cars; 84 MB each, 0.3 s to load on a Jetson; git-ignored). Tables come from `rm_linktab_colab.ipynb` (Colab GPU; inputs packed by `rm_colab_bundle.py`, upload with lunar-mesh-env's `pretrained_models_network.zip` to `MyDrive/rm_linktab`). `rss[i, j]` = int8 dBm at drivable cell `rx_rc[j]` with the transmitter at `tx_rc[i]`; the brain averages both directions. Without a table: log-distance fallback (best spot ~ the midpoint). PMNet on Colab matched the laptop's map to 0.00 dB. **Two-way consistency** (`linktab_reciprocity.py`, report in `bs_reports/reciprocity/`, 2026-10-22): |a->b - b->a| mean 7.5 dB for both tables; PMNet median 4 / p90 20 dB, diffusion median 1 / p90 36 dB. Big gaps are one-direction "no link" failures, mostly in -90..-70 dBm links and at transmitters within 5 m of the map edge. Against measured RSSI (7 outdoor runs) PMNet's directions are equally good, and for `nn` the max of both directions beats the mean in 6/7 runs (not implemented; the brain averages). One direction alone moves the chosen relay spot by a median 17-21 m. Measured ESP32 RSSI is about 18 dB below the model's numbers (~25 dB in the 2026-10-22 relay runs).
- **Measured RSSI** (2026-10-23, dashboard "Measured RSSI" checkbox, on by default; 8th relay cfg field 1|0): the relay takes the median beacon RSSI it heard from the BS and from the explorer over the last 8 s (`RoverDaemon._measured_rssi`; nothing for 5 s = -95 dBm) and shifts each link's prediction by measured − predicted at the current positions (clamped to -50..+20 dB). The weaker link "here" is then the measured one and comfort/gain are real ESP32 dBm; the model only ranks the other spots. Off = model only (comfort in model units). Status `rmb`/`rme` = measured from BS / explorer; `rdi` stays model-only. `relay_replay.py` has no measurements, so it stays model-only. Run 2 on 2026-10-22 held because the model said -51 dBm while the real link was about -78.
- **Behaviour** (`relay_brain.DEFAULTS`): explorer predicted 4 s ahead; stays put while its weaker link is >= -75 dBm and no spot is 6 dB better; retargets on +6 dB, or +2 dB once below -75; retargets >= 3 m and >= 4 s apart. Explorer silent 10 s -> hold; 10 s + ferry_s (default 15) -> drive to its last known position until heard again.
- **Rover side**: `radio_daemon.py --site fountain --terrain-dir <dir> [--linktab <npz>]` (terrain dir needs `<site>_meta.json`, `_terrain.npy`, `_drivable.npy`). Every 2 s `_relay_tick` decides and hands routes to `NavBridge.local_route` (cmd_id = BS time ms, so later BS routes/stops still win). `rwhy` = reason for the last decision (e.g. `here -60 dBm; best -58 dBm`), shown as "why:" in the Relay box. Status in `@ROVER_STATUS`: `rl=` (`off|hold|move|at_target|lost_hold|ferry|wait_ex|no_pose|no_align|no_bs|no_route|wrong_site|unavailable|error`), `rtx/rty` target, `rsc/rcu/rdi` predicted weaker link at target / here / explorer direct, `rea` explorer fix age. The dashboard draws targets as diamonds. Needs numpy for `python3` on the Jetson: `python3-numpy` 1.13.3 installed on rover 1 via apt (2026-10-22); rover 2 has no DNS/internet, so its `.deb` was fetched on rover 1 with `apt-get download` and installed with `dpkg -i`. route_plan/relay_brain are kept compatible with numpy 1.13 (`np.pad` needs an explicit `mode`; avoid `nan_to_num(nan=)`). Without `--site` or numpy the daemon runs as before and reports `rl=unavailable` if asked.
- **Offline**: `python relay_replay.py <session> --explorer <node> --site fountain [--linktab ...] [--dropout A:B]` feeds a recorded track through the same `RelayDriver`, simulates the relay driving, and plots both plus predicted direct vs relayed link. `python relay_brain.py --site fountain --bs=x,y --explorer=x,y` gives one decision.

- **Control message loss** (2026-10-23, double_ferry1/3, analysis in `data_analysis/double_ferry1|3/relay.md`): once a BS control copy grew past ~640 bytes on air (experiment start + relay + rb + dm + live fleet fields), neither rover received another BS copy for minutes, even at -40 to -50 dBm, while BS beacons, bundles and the rovers' own ~656-byte copies to each other got through. So in double_ferry1 the relay setting never arrived (the relay never moved), and in double_ferry3 the experiment start never arrived (rovers stayed in standby, no data bundles). Cause not found: on the bench (rover 1, BS a few metres away) BS frames of 300-700 bytes arrived 60-95% of the time with the ROS stacks idle or running, 800-1000 bytes about half as often. Changes: fleet fields in a separate version-0 copy (the ferry-3 start copy went from ~640 to 428 bytes), `CONTROL_MAX_JSON` 700, and the dashboard warns when a rover hasn't taken a new control version within `CONTROL_LAG_S` (10 s): red "CONTROL NOT RECEIVED" line in the Map tab banner, an event, and `#CONTROL_LAG:<node>:<has>:<sent>` in serial.log (`bs_core._check_control_lag`). The rover daemon now counts serial frames it drops (CRC mismatch, oversized, cut off) in its counters (`frame_err_*`) and reports `fe=` in `@ROVER_STATUS` when non-zero; `rx_control` log events carry `bytes`. Rover 2's clock restarts at the same Oct 20 time on every boot, so its `rover_logs/` file names collide across boots and its timestamps jump; prefer rover 1's logs for timing. `rover_status_<node>.csv` in a session now has every key the rover reported (the file is rewritten under a wider header when a new key appears); in older sessions later rows are shifted against the header, so read `@ROVER_STATUS` from serial.log there. **Likely cause found afterwards**: the BS firmware sent a `PULL_REQ` to the laptop every second (`HOST_PULL_INTERVAL_MS` was 1000, not the 10000 this file used to say) and waited up to 500 ms for an answer the laptop never sends, discarding any `WIFI_TX` (control) frame that arrived meanwhile. The laptop resent every 2.0 s, nearly phase-locked with the 1 s pull, so when its resends fell in the wait window every copy was lost until the timing drifted or a button press sent one at a new moment. Fixes (BS firmware, needs a reflash; the rovers' relay firmware is unaffected): pulls off (`HOST_PULL_ENABLED` 0; a control frame arriving during a pull is now sent, not dropped); control frames unicast to each active peer (`host_wifi_tx`; ESP-NOW unicast is acknowledged and retried, broadcast is not; broadcast if no peer is active); send statistics as `@TXSTAT`. BS software: versioned copy every 2 s +-15% jitter, every 0.5 s for 2 s after a new version and while a rover the BS hears still reports an older `cv`; the fleet copy every ~2 s, at least 0.3 s from a versioned copy (`bs_core._maybe_send_control`). Rovers: re-broadcast every 2 s +-15%, fleet copy half a period after the control copy (`CONTROL_JITTER`). `cfg` is still sent in full: a rover keeps its current value for a missing key, so leaving defaults out would need a shared defaults table.

### Key Bugs Fixed (ESP-NOW era)

**Band mode call order**: `esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)` must be called *after* `esp_wifi_start()`, not before. Returns `ESP_ERR_WIFI_NOT_STARTED` otherwise.

**Peer return not resetting `forwarded` flags**: When a timed-out peer (e.g. the BS after power-off) comes back, its MAC is already in `peer_list` with `active=false`. The fix detects the inactive→active transition and resets `forwarded=false` on all stored bundles so the returning peer receives everything accumulated during its absence. Also re-registers the peer with `esp_now_add_peer` since it was removed on timeout.

### Open Design Decisions

- **Payload size**: Flexible — the project goal is demonstrating trajectory-driven network performance. Real sensor data from the Jetson is desirable to validate end-to-end.
- **Fragmentation**: Large payloads should be chunked before being handed to the ESP32. Fragmentation logic stays on the Jetson side.
- **Downlink**: Rovers need to receive network info and delivery acknowledgements. Full command downlink is a stretch goal.

### Roadmap

1. ~~**Port transport layer to ESP-NOW**~~ — done
2. ~~**Verify ferrying**~~ — done (2026-05-17)
3. ~~**Jetson ↔ ESP32 host interface**~~ — implemented (UART binary protocol)
4. **Downlink / ACK** — deliver network state and bundle acknowledgements back to rovers.
5. **Integration** — attach ESP32 modems to actual Jetson Orin Nano rover hardware. In progress: rover B on `goose` (2026-10-13); fused pose + dashboard route driving (2026-10-15, indoor test OK, fountain lap next).

## Build & Flash Commands

Requires ESP-IDF v6.0 installed and `IDF_PATH` set.

```bash
# Build
idf.py build

# Flash to device
idf.py -p COM13 flash

# Monitor serial output
idf.py -p COM13 monitor

# Build + flash + monitor in one step
idf.py -p COM13 flash monitor
```

`menuconfig` is not needed for normal operation — no router SSID, no mesh config. Channel and band are hardcoded.

## Running the system

The BS laptop's Python environment (PySide6, pyqtgraph, pyserial, numpy) is the venv one level up, `..\.venv` (`C:\Users\Anders\Documents\esp32mesh\.venv`); the system Python lacks PySide6/pyserial. Dashboard: `python bs_dashboard.py --port COM5 --site fountain`. Site maps live in `..\terrain` (outside the repo). The Map tab's Layers → Background "Satellite" option uses `<site>_sat.png` there, made with `python fetch_sat.py fountain range` (USGS NAIP aerial photo, ~0.6 m/px, fetched for the site's UTM square; lines up with the drivable mask to ~1 m; `--force` to refetch).

```bash
# Base Station: visualizer reads its serial port directly (BS firmware emits @-lines)
pip install pyserial networkx matplotlib
python mesh_visualizer.py            # edit SERIAL_PORT at top if needed

# Each rover: run the daemon against that rover's ESP32 serial port
pip install pyserial
python rover_daemon.py --serial COM14 --interval 1   # node_id auto-detected
```

Legacy `jetson_daemon.py` (ESP32-does-the-logic model) is superseded by `rover_daemon.py`
but kept for reference. `bs_ferry_monitor.py` / `test_serial_open.py` are unchanged.

## Key Configuration

All configuration constants are at the top of `main/mesh_main.c`:

| Constant | Value | Notes |
|---|---|---|
| `BASE_STATION_NODE_ID` | 23768 | Update if using different hardware |
| `ESPNOW_CHANNEL` | 36 | 5GHz UNII-1, 5180 MHz, no DFS — all nodes must match |
| `PEER_TIMEOUT_MS` | 15000 | ms before an unheard peer is marked inactive |
| `HOST_UART_PORT` | `UART_NUM_0` | Bench: reuse USB; swap to `UART_NUM_1` for Jetson |
| `HOST_UART_TX_PIN` | `UART_PIN_NO_CHANGE` | UART0 pins fixed by bootloader; set GPIO6/7 for UART1 |
| `HOST_UART_RX_PIN` | `UART_PIN_NO_CHANGE` | Same |
| `HOST_PULL_INTERVAL_MS` | 1000 | How often ESP32 pulls bundles from Jetson (legacy `jetson_daemon.py` only) |
| `HOST_PULL_ENABLED` | 0 | Pulls off since 2026-10-23: the BS laptop never answers them, and the 500 ms wait threw away control frames |

## Architecture

### Node roles (single firmware, runtime branch)

`app_main` reads the node's `node_id` (from its STA MAC) and branches:
- **Base Station** (`node_id == BASE_STATION_NODE_ID`): starts the full DTN stack —
  `rx_process_task`, `beacon_task`, `bundle_tx_task`, `host_uart_task`, `ui_update_task`.
  This path is **unchanged** and is what `mesh_visualizer.py` consumes.
- **Rover** (everything else): starts only the relay tasks `relay_rx_task` + `relay_host_task`
  (plus `ui_update_task`). The heavy logic below still compiles but is never run on a rover —
  it lives in `rover_daemon.py` instead.

**Rover relay path**: `relay_rx_task` dequeues ESP-NOW packets (from the shared `rx_queue`
fed by `espnow_recv_cb`) and sends each as a `WIFI_RX` frame `[src_mac:6][rssi:1][air bytes]`.
`relay_host_task` reads frames from the Jetson: `WIFI_TX` payloads are broadcast verbatim via
`esp_now_send(BROADCAST_MAC, …)`; `QUERY_STATUS` returns node_id + channel so the daemon can
auto-detect which node it's attached to. No bundle store, beacons, or peer table on this path.

### Rover daemon (`rover_daemon.py`)

Reproduces the rover-side firmware behavior in Python so the BS sees identical traffic:
beacon TX (1 s), peer table keyed by node_id (resets `forwarded` on new/returning/restarted
peers), bundle RX with dedup + telemetry-ACK generation (`@DTN_RX:` payload), epidemic forward
loop (1 s, `prev_node`/`hop_count` rewritten per hop, 5 s retransmit of unacked bundles to the
BS), and ACK-map apply + propagate (256-bit sliding window per source, ports `mark_ack_locked`
/`merge_ackmap_locked`). Air (de)serializers: `pack_beacon`/`unpack_beacon` (`<BHII`),
`encode_air_bundle` (`[0x02]` + trimmed `dtn_bundle_t`), `pack_ackmap`/`unpack_ackmap`
(`<BHHIB32s`).

### Firmware (`main/mesh_main.c`) — Base Station stack

**Bundle store**: Flat array of 100 `ram_bundle_t` slots. Each slot holds a `dtn_bundle_t` (BPv7-inspired struct: source, dest, prev_node, sequence number, creation_time, lifetime/TTL, hop count/limit, payload) plus `is_empty` and `forwarded` flags. `data_mutex` protects both `bundle_store[]` and `peer_list[]`.

**`beacon_task`**: Broadcasts a `beacon_pkt_t` (node_id + timestamp_ms) every 1s to the ESP-NOW broadcast MAC. Used for peer discovery and clock-offset measurement.

**`rx_process_task`**: Receives from a FreeRTOS queue populated by the ESP-NOW recv callback (callbacks must not call `esp_now_send` directly). Handles two packet types:
- `PKT_TYPE_BEACON`: calls `add_or_refresh_peer`, updates per-peer clock offset at the BS, emits `@NET:` line if BS.
- `PKT_TYPE_BUNDLE`: deduplicates by `(source_node, creation_time, sequence_number)`, stores, generates telemetry ACK on rovers, prints `@METRIC:` + `@DTN_RX:` at BS. BS frees the slot after printing (it doesn't accumulate bundles).

**`bundle_tx_task`**: Runs every 1s. Expires timed-out peers (calls `esp_now_del_peer`). Generates a data bundle every 10s. For each un-forwarded bundle, snapshots active peer MACs (brief mutex hold), releases mutex, then sends — never holds mutex during `esp_now_send`. Marks `forwarded=true` after a successful send.

**`host_uart_task`**: Binary framing protocol for Jetson ↔ ESP32 communication. Handles incoming `TX_BUNDLE`, `QUERY_STATUS`, `QUERY_PEERS` commands. `WIFI_TX` control frames from the BS laptop go unicast to each active peer (`host_wifi_tx`). With `HOST_PULL_ENABLED` (off since 2026-10-23; only the legacy `jetson_daemon.py` answers pulls), every `HOST_PULL_INTERVAL_MS` when peers are active, drains the Jetson bundle store via `PULL_REQ` / `BUNDLE_DATA` / `NO_BUNDLES` loop. See Host Interface section below.

**`add_or_refresh_peer`**: On new peer or returning-after-timeout peer, resets `forwarded=false` on all stored bundles so the new/returning peer receives everything. Returning peers are detected by the `active=false` → `active=true` transition; `esp_now_add_peer` is re-called since it was removed on timeout.

**Clock sync**: `peer_entry_t` stores `clock_offset_ms = peer_timestamp - bs_now_ms` updated on every beacon receipt at the BS. Latency = `(now_ms() - bundle.creation_time) + src_clock_offset`, clamped to 0. Only meaningful for nodes whose beacons the BS has received recently; ferried bundles from absent nodes use a stale but approximately correct offset.

### Host Interface Protocol (UART)

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
| `ACK` | `0x20` | ESP32 → Jetson | 1 byte (0=ok, 1=error) |
| `STATUS_RESP` | `0x21` | ESP32 → Jetson | `host_status_t` (6 bytes) |
| `PEERS_RESP` | `0x22` | ESP32 → Jetson | n × `host_peer_entry_t` (4 bytes each) |
| `WIFI_TX` | `0x30` | Jetson → ESP32 | raw over-air bytes to broadcast verbatim (rover relay) |
| `WIFI_RX` | `0x31` | ESP32 → Jetson | `[src_mac:6][rssi:1][air bytes]` for each received frame (rover relay) |

**Rover relay mode** uses only `WIFI_TX`/`WIFI_RX` (+ `QUERY_STATUS`/`STATUS_RESP` for node-id
detection). The bundle-centric commands (`TX_BUNDLE`/`PULL_REQ`/`BUNDLE_DATA`/`NO_BUNDLES`/
`ANTIPKT_NOTIFY`/`BUNDLE_PUSH`) belong to the legacy `jetson_daemon.py` model and are not used
by `rover_daemon.py`. `HOST_MAX_PAYLOAD` is 1100 (a relayed bundle is up to 6+1+1053 bytes).

**Bench testing**: `HOST_UART_PORT = UART_NUM_0` reuses the existing USB/flash connection. Log lines (ASCII, starting with `I (`, `W (` etc.) and binary frames (starting with `0xAA`) are interleaved on the same port — the host-side script must demux on the SOF byte. Cannot run `idf.py monitor` simultaneously.

**Jetson deployment**: Change to `HOST_UART_PORT = UART_NUM_1`, `HOST_UART_TX_PIN = 6`, `HOST_UART_RX_PIN = 7`. Wire GPIO6 → Jetson UART RX, GPIO7 ← Jetson UART TX, GND shared. UART0 remains free for debugging.

### Serial Protocol (BS output, for visualizer)

| Prefix | Format | Meaning |
|---|---|---|
| `@NET:` | `@NET:<node>:<parent>:<rssi>` | Topology heartbeat; BS emits for each rover beacon received, plus `@NET:<bs_id>:0:0` self-heartbeat every 5s |
| `@DTN_RX:` | `@DTN_RX:<src>:<prev>:<seq>:<receiver>:<hops>` | Bundle received at BS (from telemetry ACK payload) |
| `@METRIC:` | `@METRIC:<src>:<seq>:<hops>:<latency_ms>` | End-to-end latency, clock-skew corrected |
| `@ANTIPKT_FAIL:` | `@ANTIPKT_FAIL:<src>:<seq>:<prev>` | Duplicate from same forwarder `<prev>` — that node's antipacket didn't arrive |
| `@FERRY_DUP:` | `@FERRY_DUP:<src>:<seq>:<first_prev>:<dup_prev>` | Same bundle arrived via two different forwarders — expected epidemic behavior |
| `@TXSTAT:` | `@TXSTAT:<ctrl_in>:<ctrl_sent>:<ctrl_err>:<uni_ok>:<uni_fail>` | Every 10 s, cumulative since BS boot: control frames from the laptop, unicast copies queued, `esp_now_send` errors on them, unicast frames acknowledged / not acknowledged after the radio's retries. `bs_core` sums them per session (summary, snapshot `txstat`) and warns on queue errors |
| `@NODE_RESTART:` | `@NODE_RESTART:<node_id>` | BS detected a rover's boot_id changed; visualizer resets per-node metrics. boot_id is now generated by `rover_daemon.py` at startup, so this fires on daemon restart (an ESP32-only reboot keeps the same boot_id since the store survives on the Jetson) |

### Visualizer (`mesh_visualizer.py`)

Parses the serial message types above using a background thread, maintains a `networkx.DiGraph`, and animates it with matplotlib every 500ms. Nodes time out after 15s without a `@NET:` heartbeat. DTN transfers are overlaid as colored edges. Per-node average latency shown in title.

Restart resilience: on serial reconnect or on detecting `"I am the Base Station"` in the stream, saves the session summary and resets all state. On `@NODE_RESTART:<id>`, resets per-node metrics (latency, seq range, delivered bundles, RSSI, dup/fail counts) for just that node so the delivery rate estimate stays accurate across node reboots.
