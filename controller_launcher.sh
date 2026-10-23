#!/bin/bash
# Waits for the PlayStation controller to connect, then:
#   Press OPTIONS (button 10) -> launch the full MuSHR stack
#   Press SHARE  (button 9)  -> kill the stack
# Loops back to waiting after a stop so you can restart without rebooting.
#
# Only a real button press (wait_button.py exit 0) starts the stack. If the controller
# drops while waiting for OPTIONS, we just wait for it to come back. If it drops while the
# stack is running, the stack is stopped (the planner in car_startup.launch would otherwise
# drive with nothing blocking navigation).
#
# Confirmed Linux PS4/PS5 button map on this rover:
#   0=Cross  1=Circle  2=Triangle  3=Square
#   4=L1     5=R1      6=L2        7=R2
#   9=Share  10=Options

START_BUTTON=${START_BUTTON:-10}  # Options
STOP_BUTTON=${STOP_BUTTON:-9}     # Share
JOYSTICK=${JOYSTICK:-/dev/input/js0}
ROS_DISTRO=${ROS_DISTRO:-noetic}
WORKSPACE=${WORKSPACE:-/home/nvidia/Documents/mushr_base}
LAUNCH_ARGS=${LAUNCH_ARGS:-""}

wait_for_joystick() {
    if [ ! -e "${JOYSTICK}" ]; then
        echo "[controller_launcher] Waiting for joystick at ${JOYSTICK}..."
        while [ ! -e "${JOYSTICK}" ]; do sleep 1; done
        echo "[controller_launcher] Controller connected."
    fi
}

wait_for_joystick

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WORKSPACE}/devel/setup.bash

while true; do
    wait_for_joystick
    echo "[controller_launcher] Controller ready."
    echo "[controller_launcher]   OPTIONS (button ${START_BUTTON}) -> start stack"
    echo "[controller_launcher]   SHARE   (button ${STOP_BUTTON})  -> stop stack"

    # Wait for OPTIONS button press
    python3 $(dirname "$0")/wait_button.py ${JOYSTICK} ${START_BUTTON}
    rc=$?
    if [ ${rc} -ne 0 ]; then
        echo "[controller_launcher] No OPTIONS press (wait_button rc ${rc}); not starting."
        sleep 1
        continue
    fi

    # Launch stack in background, capture PID
    roslaunch mushr_base car_startup.launch ${LAUNCH_ARGS} &
    LAUNCH_PID=$!
    echo "[controller_launcher] Stack running (PID ${LAUNCH_PID})."
    python3 $(dirname "$0")/beep.py 2        # 2 pulses = started

    # Wait for SHARE button press
    python3 $(dirname "$0")/wait_button.py ${JOYSTICK} ${STOP_BUTTON}
    rc=$?
    if [ ${rc} -ne 0 ]; then
        echo "[controller_launcher] Controller lost while stack running (rc ${rc}); stopping it."
    fi

    # Send SIGINT to roslaunch so it cleanly shuts down all child nodes
    echo "[controller_launcher] Shutting down..."
    kill -SIGINT ${LAUNCH_PID} 2>/dev/null
    wait ${LAUNCH_PID} 2>/dev/null
    sleep 2
    python3 $(dirname "$0")/beep.py 1 500    # 1 long pulse = stopped
    echo "[controller_launcher] Stopped. Press OPTIONS to restart."
done
