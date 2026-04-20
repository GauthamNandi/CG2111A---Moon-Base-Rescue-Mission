#!/usr/bin/env python3
"""
Studio 13: Sensor Mini-Project
Raspberry Pi Sensor Interface — pi_sensor.py

Extends the communication framework from Studio 12 with:
  - Magic number + checksum framing for reliable packet delivery
  - Hold-to-move: hold w/a/s/d to drive; releasing the key stops the robot
    automatically after a 150 ms timeout (exploits OS key-repeat).
  - Commands requiring a numeric argument (arm joints, arm speed) temporarily
    restore normal line-mode so the user can type a number and press Enter,
    then return to cbreak mode automatically.

Packet framing format (19 bytes total):
  MAGIC (2 B) | TPacket (16 B) | CHECKSUM (1 B)

Usage:
  source env/bin/activate
  python3 pi_sensor.py
"""
from second_terminal import relay

import struct
import serial
import time
import sys
import select
import tty
import termios
import os

# Activity 3: camera library (alex_camera.py is in the same folder)
import alex_camera

# Activity 4: LIDAR libraries
# alex_lidar.py lives in lidar/; lidar_example_cli_plot.py is in the same folder.
from lidar.alex_lidar import (
    lidarConnect, lidarDisconnect, lidarStatus, performSingleScan
)
from lidar_example_cli_plot import convert_to_cartesian, points_to_grid, render_to_cli

# ----------------------------------------------------------------
# SERIAL PORT SETUP
# ----------------------------------------------------------------

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600

_ser = None


def openSerial():
    """Open the serial port and wait for the Arduino to boot."""
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    time.sleep(2)
    print("Ready.\n")


def closeSerial():
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()


# ----------------------------------------------------------------
# TPACKET CONSTANTS
# (must match sensor_miniproject_v8.ino / packets.h)
# ----------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP        = 0
COMMAND_COLOR        = 1   # Activity 2: request R/G/B frequencies from TCS3200
COMMAND_FORWARD      = 2   # Studio 15: drive forward
COMMAND_BACKWARD     = 3   # Studio 15: drive backward
COMMAND_LEFT         = 4   # Studio 15: turn CCW
COMMAND_RIGHT        = 5   # Studio 15: turn CW
COMMAND_STOP_MOVE    = 6   # Studio 15: stop motors (not E-Stop)
COMMAND_SET_SPEED    = 7   # Studio 15: update speed in-place without changing direction
COMMAND_ARM_BASE     = 8   # Arm: move base servo;     params[0] = angle (0-180)
COMMAND_ARM_SHOULDER = 9   # Arm: move shoulder servo; params[0] = angle (0-100)
COMMAND_ARM_ELBOW    = 10  # Arm: move elbow servo;    params[0] = angle (60-160)
COMMAND_ARM_GRIPPER  = 11  # Arm: move gripper servo;  params[0] = angle (90-105)
COMMAND_ARM_HOME     = 12  # Arm: return all servos to home position
COMMAND_ARM_SPEED    = 13  # Arm: set movement speed;  params[0] = ms per degree

RESP_OK     = 0
RESP_STATUS = 1
RESP_COLOR  = 2      # Activity 2: response carrying R/G/B frequencies in Hz

STATE_RUNNING = 0
STATE_STOPPED = 1

PARAMS_COUNT = 3
TPACKET_SIZE = 1 + 1 + 2 + (PARAMS_COUNT * 4)  # = 16
TPACKET_FMT  = f'<BB2x{PARAMS_COUNT}I'

# ----------------------------------------------------------------
# RELIABLE FRAMING: magic number + XOR checksum
# ----------------------------------------------------------------

MAGIC      = b'\xDE\xAD'                        # 2-byte magic number (0xDEAD)
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1      # 2 + 16 + 1 = 19


def computeChecksum(data: bytes) -> int:
    """Return the XOR of all bytes in data."""
    result = 0
    for b in data:
        result ^= b
    return result


def packFrame(packetType, command, params=None):
    """
    Build a framed packet: MAGIC | TPacket bytes | checksum.

    Args:
        packetType: integer packet type constant
        command:    integer command / response constant
        params:     list of PARAMS_COUNT uint32 values (default: all zeros)

    Returns:
        bytes of length FRAME_SIZE (19)
    """
    if params is None:
        params = [0] * PARAMS_COUNT
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def unpackTPacket(raw):
    """Deserialise a 16-byte TPacket into a dict."""
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'params':     list(fields[2:]),
    }


def receiveFrame():
    """
    Read bytes from the serial port until a valid framed packet is found.

    Synchronises to the magic number, reads the TPacket body, then
    validates the checksum.  Discards corrupt or out-of-sync bytes.

    Returns:
        A packet dict (see unpackTPacket), or None on timeout / error.
    """
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]

    while True:
        # Read and discard bytes until we see the first magic byte.
        b = _ser.read(1)
        if not b:
            return None          # timeout
        if b[0] != MAGIC_HI:
            continue

        # Read the second magic byte.
        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_LO:
            continue

        # Magic matched; now read the TPacket body.
        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        # Read and verify the checksum.
        cs_byte = _ser.read(1)
        if not cs_byte:
            return None
        expected = computeChecksum(raw)
        if cs_byte[0] != expected:
            # Checksum mismatch: corrupted packet, try to resync.
            continue

        return unpackTPacket(raw)


def sendCommand(commandType, params=None):
    """Send a framed COMMAND packet to the Arduino."""
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, params=params)
    _ser.write(frame)


# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING


def isEstopActive():
    """Return True if the E-Stop is currently active (system stopped)."""
    return _estop_state == STATE_STOPPED


# ----------------------------------------------------------------
# MOVEMENT (Studio 15)
# ----------------------------------------------------------------

_move_speed        = 200   # Current motor speed (0-255). Adjusted with + / -.
_is_moving         = False  # True while a movement key is held.
_last_move_time    = 0.0    # Timestamp of the last movement keypress.
_current_direction = None   # Tracks direction to suppress repeated prints.
_MOVE_TIMEOUT      = 0.15   # Seconds: send STOP if no move key within this window.

_MOVE_COMMANDS = {
    'forward':  COMMAND_FORWARD,
    'backward': COMMAND_BACKWARD,
    'left':     COMMAND_LEFT,
    'right':    COMMAND_RIGHT,
}


def handleMoveCommand(direction):
    """
    Send a movement command to Alex.

    direction must be one of 'forward', 'backward', 'left', 'right'.
    Refuses with a clear message when the E-Stop is active.
    The current _move_speed is passed in params[0].
    Called continuously while the key is held (via OS key-repeat); only
    prints a message on the first press or when direction changes.
    """
    global _is_moving, _last_move_time, _current_direction
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    params = [_move_speed] + [0] * (PARAMS_COUNT - 1)
    sendCommand(_MOVE_COMMANDS[direction], params=params)
    if not _is_moving or _current_direction != direction:
        print(f"Moving {direction} at speed {_move_speed}")
        _current_direction = direction
    _is_moving = True
    _last_move_time = time.time()


def handleStopMoveCommand():
    """Stop the motors without triggering the E-Stop."""
    global _current_direction
    sendCommand(COMMAND_STOP_MOVE)
    _current_direction = None
    print("Motors stopped")


def adjustSpeed(delta):
    """
    Increase or decrease _move_speed by delta (positive or negative).
    Clamps to [0, 255], then immediately sends COMMAND_SET_SPEED so the
    Arduino updates all four motors in-place without changing direction.
    """
    global _move_speed
    _move_speed = max(0, min(255, _move_speed + delta))
    params = [_move_speed] + [0] * (PARAMS_COUNT - 1)
    sendCommand(COMMAND_SET_SPEED, params=params)
    print(f"Speed set to {_move_speed}")


# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def printPacket(pkt):
    """Print a received TPacket in human-readable form."""
    global _estop_state
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")

        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_state = state
            if state == STATE_RUNNING:
                print("Status: RUNNING")
            else:
                print("Status: STOPPED")

        elif cmd == RESP_COLOR:
            # Activity 2: display the three channel frequencies.
            r = pkt['params'][0]
            g = pkt['params'][1]
            b = pkt['params'][2]
            print(f"Color: R={r} Hz, G={g} Hz, B={b} Hz")

        else:
            print(f"Response: unknown command {cmd}")

    elif ptype == PACKET_TYPE_MESSAGE:
        print(f"Packet: type={ptype}, cmd={cmd}")
    else:
        print(f"Packet: type={ptype}, cmd={cmd}")


# ----------------------------------------------------------------
# ACTIVITY 2: COLOR SENSOR
# ----------------------------------------------------------------

def handleColorCommand():
    """
    Request a color reading from the Arduino and display it.

    Checks the E-Stop state first; if stopped, refuses with a clear message.
    Otherwise sends COMMAND_COLOR; the response is printed by printPacket().
    """
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    print("Sending color command...")
    sendCommand(COMMAND_COLOR)


# ----------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ----------------------------------------------------------------

_camera = None           # Opened in main; closed in finally.
_frames_remaining = 15    # Maximum captures allowed (simulates limited uplink).


def handleCameraCommand():
    """
    Capture and display a greyscale frame from the Raspberry Pi Camera.

    Checks E-Stop state first.  Refuses once the five-frame budget is
    exhausted.  Uses alex_camera without any modifications.
    """
    global _frames_remaining, _camera

    # Gate 1: E-Stop
    if isEstopActive():
        print("Refused: E-Stop is active")
        return

    # Gate 2: frame budget
    if _frames_remaining <= 0:
        print("Refused: No frames remaining (0 / 15 left)")
        return

    print(f"Capturing frame ({_frames_remaining} remaining)...")
    frame = alex_camera.captureGreyscaleFrame(_camera)
    alex_camera.renderGreyscaleFrame(frame)
    _frames_remaining -= 1
    print(f"Frames remaining: {_frames_remaining}")


# ----------------------------------------------------------------
# ACTIVITY 4: LIDAR
# ----------------------------------------------------------------

_lidar = None   # Connected on first use; disconnected in finally.


def handleLidarCommand():
    """
    Perform a single LIDAR scan and render it in the terminal.

    Gates on E-Stop state.  Connects to the LIDAR on the first call
    (lazy connect avoids a 2-second startup delay if the user never
    types 'l').
    """
    global _lidar

    if isEstopActive():
        print("Refused: E-Stop is active")
        return

    try:
        if _lidar is None:
            print("Connecting to LIDAR (takes ~2 s)...")
            _lidar = lidarConnect()

        print("Performing LIDAR scan...")
        status    = lidarStatus(_lidar, verbose=False)
        scan_data = performSingleScan(_lidar, status['typical_scan_mode'])

        # scan_data = (angles_tuple, distances_tuple, quality_tuple)
        xs, ys = convert_to_cartesian(scan_data[0], scan_data[1])
        grid   = points_to_grid(xs, ys)
        print(render_to_cli(grid))

    except Exception as exc:
        print(f"LIDAR error: {exc}")
        _lidar = None   # Force reconnect on next attempt.


# ----------------------------------------------------------------
# ROBOTIC ARM
# ----------------------------------------------------------------

# Angle limits per joint (matches Arduino-side constrain() calls)
_ARM_LIMITS = {
    'base':     (0,   180),
    'shoulder': (0,   100),
    'elbow':    (60,  180),
    'gripper':  (90,  105),
}

_ARM_COMMANDS = {
    'base':     COMMAND_ARM_BASE,
    'shoulder': COMMAND_ARM_SHOULDER,
    'elbow':    COMMAND_ARM_ELBOW,
    'gripper':  COMMAND_ARM_GRIPPER,
}


def handleArmCommand(joint_or_action, angle=None):
    """
    Send a robotic arm command to the Arduino.

    Call with:
      handleArmCommand('home')              — return all servos to home
      handleArmCommand('base',     angle)   — move base     0-180 °
      handleArmCommand('shoulder', angle)   — move shoulder 0-100 °
      handleArmCommand('elbow',    angle)   — move elbow    60-160 °
      handleArmCommand('gripper',  angle)   — move gripper  90-105 °

    Refuses with a clear message when the E-Stop is active.
    """
    if isEstopActive():
        print("Refused: E-Stop is active")
        return

    if joint_or_action == 'home':
        sendCommand(COMMAND_ARM_HOME)
        print("Arm: homing all servos")
        return

    if joint_or_action not in _ARM_COMMANDS:
        print(f"Unknown arm joint: '{joint_or_action}'")
        return

    if angle is None:
        print(f"Arm: angle required for '{joint_or_action}'")
        return

    lo, hi = _ARM_LIMITS[joint_or_action]
    angle = max(lo, min(hi, int(angle)))
    params = [angle] + [0] * (PARAMS_COUNT - 1)
    sendCommand(_ARM_COMMANDS[joint_or_action], params=params)
    print(f"Arm: moving {joint_or_action} to {angle}°")


def handleArmSpeedCommand(ms_per_deg):
    """Set arm servo movement speed (ms per degree, 1-999)."""
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    ms_per_deg = max(1, min(999, int(ms_per_deg)))
    params = [ms_per_deg] + [0] * (PARAMS_COUNT - 1)
    sendCommand(COMMAND_ARM_SPEED, params=params)
    print(f"Arm speed set to {ms_per_deg} ms/degree")


# ----------------------------------------------------------------
# RAW TERMINAL MODE HELPERS
# ----------------------------------------------------------------

def _enter_cbreak_mode(fd):
    """
    Switch stdin to cbreak mode: keypresses are delivered immediately
    without needing Enter, and echo is disabled.  Output processing
    (e.g. newline translation) is left intact so print() still works.

    Returns the old terminal settings so they can be restored later.
    """
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old


def _restore_terminal(fd, old):
    """Restore terminal to its original settings."""
    termios.tcsetattr(fd, termios.TCSADRAIN, old)


def promptAngle(fd, old_settings, key):
    """
    Temporarily restore cooked/line mode to prompt the user for an angle
    or speed value, then switch back to cbreak mode.

    Args:
        fd:           file descriptor of stdin
        old_settings: saved terminal settings from _enter_cbreak_mode()
        key:          the key that triggered this prompt ('b','o','k','g','v')
    """
    _restore_terminal(fd, old_settings)
    try:
        if key == 'v':
            val = input("Arm speed (ms/degree, 1-999): ")
            handleArmSpeedCommand(int(val))
        else:
            joint = {'b': 'base', 'o': 'shoulder', 'k': 'elbow', 'g': 'gripper'}[key]
            lo, hi = _ARM_LIMITS[joint]
            val = input(f"Arm {joint} angle ({lo}-{hi}): ")
            handleArmCommand(joint, int(val))
    except (ValueError, EOFError):
        print("Invalid input — command ignored.")
    finally:
        tty.setcbreak(fd)


# ----------------------------------------------------------------
# KEY DISPATCH (replaces the old line-mode handleUserInput)
# ----------------------------------------------------------------

def handleKeyPress(ch, fd, old_settings):
    """
    Dispatch a single character received in cbreak mode.

      e   →  toggle E-Stop
      c   →  request colour reading
      p   →  capture camera frame
      l   →  perform LIDAR scan
      w   →  drive forward  (hold to keep moving; release to stop)
      s   →  drive backward (hold to keep moving; release to stop)
      a   →  turn left CCW  (hold to keep moving; release to stop)
      d   →  turn right CW  (hold to keep moving; release to stop)
      x   →  stop motors immediately
      +   →  increase speed by 25
      -   →  decrease speed by 25
      h   →  arm: home all servos
      b   →  arm: prompt for base angle    (0-180)
      o   →  arm: prompt for shoulder angle (0-100)
      k   →  arm: prompt for elbow angle   (60-160)
      g   →  arm: prompt for gripper angle (90-105)
      v   →  arm: prompt for speed (ms/degree)
    """
    global _is_moving

    if ch == 'e':
        print("Sending E-Stop command...")
        sendCommand(COMMAND_ESTOP)

    elif ch == 'c':
        handleColorCommand()

    elif ch == 'p':
        handleCameraCommand()

    elif ch == 'l':
        handleLidarCommand()

    elif ch == 'w':
        handleMoveCommand('forward')

    elif ch == 's':
        handleMoveCommand('backward')

    elif ch == 'a':
        handleMoveCommand('left')

    elif ch == 'd':
        handleMoveCommand('right')

    elif ch == 'x':
        handleStopMoveCommand()
        _is_moving = False

    elif ch == '+':
        adjustSpeed(+25)

    elif ch == '-':
        adjustSpeed(-25)

    elif ch == 'h':
        handleArmCommand('home')

    elif ch in ('b', 'o', 'k', 'g', 'v'):
        promptAngle(fd, old_settings, ch)

    elif ch == '\x03':   # Ctrl+C delivered in cbreak mode
        raise KeyboardInterrupt

    # All other characters are silently ignored.


def runCommandInterface():
    """
    Main command loop.

    Puts the terminal into cbreak mode so movement keys (w/a/s/d) are
    received character-by-character without pressing Enter.  Holding a
    movement key keeps the robot moving via OS key-repeat (~30 Hz);
    releasing causes the auto-stop timeout (150 ms) to fire and send
    COMMAND_STOP_MOVE.

    Commands that need a numeric argument (arm joints, arm speed) prompt
    the user by temporarily restoring normal line mode, then returning
    to cbreak mode automatically.
    """
    global _is_moving, _last_move_time

    print("Sensor interface ready. Hold movement keys to drive; release to stop.")
    print("  Movement : w=forward  s=backward  a=left  d=right  x=stop  +/-=speed")
    print("  Arm      : h=home  b=base  o=shoulder  k=elbow  g=gripper  v=speed")
    print("  Other    : e=estop  c=colour  p=camera  l=lidar")
    print("Press Ctrl+C to exit.\n")

    fd = sys.stdin.fileno()
    old_settings = _enter_cbreak_mode(fd)

    try:
        while True:
            # 1. Forward any waiting Arduino packets.
            if _ser.in_waiting >= FRAME_SIZE:
                pkt = receiveFrame()
                if pkt:
                    printPacket(pkt)
                    relay.onPacketReceived(packFrame(pkt['packetType'],
                                                     pkt['command'],
                                                     pkt['params']))

            # 2. Read a single keypress (non-blocking).
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                ch = os.read(fd, 1).decode('ascii', errors='ignore').lower()
                if ch:
                    handleKeyPress(ch, fd, old_settings)

            # 3. Auto-stop: if a movement key hasn't been repeated within
            #    the timeout window, the key has been released — stop motors.
            if _is_moving and (time.time() - _last_move_time > _MOVE_TIMEOUT):
                handleStopMoveCommand()
                _is_moving = False

            # 4. Relay commands from the second terminal to the Arduino.
            relay.checkSecondTerminal(_ser)

            time.sleep(0.02)

    finally:
        _restore_terminal(fd, old_settings)


# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    openSerial()
    relay.start()

    # Open the camera once at startup so the first 'p' command
    # responds immediately (camera initialisation takes ~0.5 s).
    _camera = alex_camera.cameraOpen()

    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        # Clean up camera and LIDAR before closing serial.
        if _camera is not None:
            alex_camera.cameraClose(_camera)
        if _lidar is not None:
            lidarDisconnect(_lidar)
        relay.shutdown()
        closeSerial()
