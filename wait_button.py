#!/usr/bin/env python3
"""
Blocks until a specific joystick button is pressed.
Usage: wait_button.py <joystick_device> <button_index>

Exit codes (the launcher must only act on 0):
  0  the button was pressed
  1  usage / permission error
  2  joystick missing or lost (controller off, out of range, battery dead)
"""
import struct
import sys

if len(sys.argv) != 3:
    print(f"Usage: {sys.argv[0]} <joystick_device> <button_index>")
    sys.exit(1)

joystick = sys.argv[1]
target = int(sys.argv[2])

# /dev/input/js0 event format: uint32 time, int16 value, uint8 type, uint8 number
EVENT_SIZE = struct.calcsize('IhBB')
JS_EVENT_BUTTON = 0x01

print(f"[wait_button] Listening on {joystick} for button {target}...")

try:
    with open(joystick, 'rb') as js:
        while True:
            event = js.read(EVENT_SIZE)
            if not event:
                print(f"[wait_button] {joystick} closed (controller lost).")
                sys.exit(2)
            if len(event) < EVENT_SIZE:
                continue
            _, value, typ, number = struct.unpack('IhBB', event)
            # Ignore init events (type has 0x80 bit set)
            if typ & 0x80:
                continue
            if typ == JS_EVENT_BUTTON and number == target and value == 1:
                print(f"[wait_button] Button {target} pressed.")
                sys.exit(0)
except PermissionError:
    print(f"[wait_button] ERROR: cannot read {joystick} — add user to 'input' group:")
    print(f"  sudo usermod -a -G input $USER && reboot")
    sys.exit(1)
except FileNotFoundError:
    print(f"[wait_button] {joystick} not found (controller off).")
    sys.exit(2)
except OSError as e:
    # ENODEV when the controller disconnects mid-read
    print(f"[wait_button] {joystick} lost: {e}")
    sys.exit(2)
