#!/usr/bin/env python3
"""
Studio 16: Robot Integration
second_terminal.py  -  Second operator terminal.

This terminal connects to pi_sensor.py over TCP.  It:
  - Displays every TPacket forwarded from the robot (via pi_sensor.py).
  - Sends a software E-Stop command when you type 'e'.
  - Controls the robotic arm via the arm commands below.

Architecture
------------
   [Arduino] <--USB serial--> [pi_sensor.py] <--TCP--> [second_terminal.py]
                                (TCP server,               (TCP client,
                                 port 65432)                localhost:65432)

Run pi_sensor.py FIRST (it starts the TCP server), then run this script.
Both scripts run on the same Raspberry Pi.

Commands
--------
  e         Send a software E-Stop to the robot.
  h         Home all arm servos.
  b <n>     Move base servo to n degrees      (0-180).
  o <n>     Move shoulder servo to n degrees  (0-100).
  k <n>     Move elbow servo to n degrees     (60-160).
  g <n>     Move gripper servo to n degrees   (90-105).
  v <n>     Set arm movement speed to n ms/degree.
  q         Quit.

Usage
-----
    source env/bin/activate
    python3 second_terminal/second_terminal.py

Press Ctrl+C to exit.
"""

import select
import struct
import sys
import time

from net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame


# ---------------------------------------------------------------------------
# Connection settings
# ---------------------------------------------------------------------------
PI_HOST = 'localhost'
PI_PORT = 65432


# ---------------------------------------------------------------------------
# TPacket constants  (keep in sync with pi_sensor.py and packets.h)
# ---------------------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP        = 0
COMMAND_ARM_BASE     = 8   # params[0] = angle (0-180)
COMMAND_ARM_SHOULDER = 9   # params[0] = angle (0-100)
COMMAND_ARM_ELBOW    = 10  # params[0] = angle (60-160)
COMMAND_ARM_GRIPPER  = 11  # params[0] = angle (90-105)
COMMAND_ARM_HOME     = 12  # no params
COMMAND_ARM_SPEED    = 13  # params[0] = ms per degree

RESP_OK     = 0
RESP_STATUS = 1

STATE_RUNNING = 0
STATE_STOPPED = 1

PARAMS_COUNT = 3
TPACKET_SIZE = 1 + 1 + 2 + (PARAMS_COUNT * 4)   # = 16
TPACKET_FMT  = f'<BB2x{PARAMS_COUNT}I'

MAGIC      = b'\xDE\xAD'
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # = 19


# ---------------------------------------------------------------------------
# TPacket helpers
# ---------------------------------------------------------------------------

def _computeChecksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result


def _packFrame(packetType, command, params=None):
    """Pack a TPacket into a 19-byte framed byte string."""
    if params is None:
        params = [0] * PARAMS_COUNT
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command, *params)
    return MAGIC + packet_bytes + bytes([_computeChecksum(packet_bytes)])


def _unpackFrame(frame: bytes):
    """Validate checksum and unpack a 19-byte frame.  Returns None if corrupt."""
    if len(frame) != FRAME_SIZE or frame[:2] != MAGIC:
        return None
    raw = frame[2:2 + TPACKET_SIZE]
    if frame[-1] != _computeChecksum(raw):
        return None
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'params':     list(fields[2:]),
    }


# ---------------------------------------------------------------------------
# Packet display
# ---------------------------------------------------------------------------

_estop_active = False


def _printPacket(pkt):
    """Pretty-print a TPacket forwarded from the robot."""
    global _estop_active

    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("[robot] OK")
        elif cmd == RESP_STATUS:
            state         = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            print(f"[robot] Status: {'STOPPED' if _estop_active else 'RUNNING'}")
        else:
            print(f"[robot] Response: unknown command {cmd}")

    elif ptype == PACKET_TYPE_MESSAGE:
        print(f"[robot] Packet: type={ptype}, cmd={cmd}")

    else:
        print(f"[robot] Packet: type={ptype}, cmd={cmd}")


# ---------------------------------------------------------------------------
# Arm helpers
# ---------------------------------------------------------------------------

_ARM_LIMITS = {
    'base':     (0,   180),
    'shoulder': (0,   100),
    'elbow':    (60,  160),
    'gripper':  (90,  105),
}

_ARM_COMMANDS = {
    'base':     COMMAND_ARM_BASE,
    'shoulder': COMMAND_ARM_SHOULDER,
    'elbow':    COMMAND_ARM_ELBOW,
    'gripper':  COMMAND_ARM_GRIPPER,
}


def _sendArmCommand(client, joint_or_action, angle=None):
    """Pack and forward an arm command through the TCP link to pi_sensor.py."""
    if _estop_active:
        print("[second_terminal] Refused: E-Stop is active")
        return

    if joint_or_action == 'home':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_HOME)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: ARM HOME")
        return

    if joint_or_action == 'speed':
        if angle is None:
            print("[second_terminal] Usage: v <ms_per_degree>  (1-999)")
            return
        ms = max(1, min(999, int(angle)))
        params = [ms] + [0] * (PARAMS_COUNT - 1)
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_SPEED, params=params)
        sendTPacketFrame(client.sock, frame)
        print(f"[second_terminal] Sent: ARM SPEED {ms} ms/deg")
        return

    if joint_or_action not in _ARM_COMMANDS:
        print(f"[second_terminal] Unknown arm joint: '{joint_or_action}'")
        return

    if angle is None:
        lo, hi = _ARM_LIMITS[joint_or_action]
        print(f"[second_terminal] Angle required.  Range: {lo}-{hi}")
        return

    lo, hi = _ARM_LIMITS[joint_or_action]
    angle = max(lo, min(hi, int(angle)))
    params = [angle] + [0] * (PARAMS_COUNT - 1)
    frame = _packFrame(PACKET_TYPE_COMMAND, _ARM_COMMANDS[joint_or_action], params=params)
    sendTPacketFrame(client.sock, frame)
    print(f"[second_terminal] Sent: ARM {joint_or_action.upper()} → {angle}°")


# ---------------------------------------------------------------------------
# Input handling
# ---------------------------------------------------------------------------

def _handleInput(line: str, client):
    """
    Handle one line of keyboard input.

      e         E-Stop
      h         arm: home all servos
      b <n>     arm: base to n°      (0-180)
      o <n>     arm: shoulder to n°  (0-100)
      k <n>     arm: elbow to n°     (60-160)
      g <n>     arm: gripper to n°   (90-105)
      v <n>     arm: speed in ms/degree
      q         quit
    """
    parts = line.strip().lower().split()
    if not parts:
        return
    key = parts[0]

    if key == 'e':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: E-STOP")

    elif key == 'q':
        print("[second_terminal] Quitting.")
        raise KeyboardInterrupt

    elif key == 'h':
        _sendArmCommand(client, 'home')

    elif key in ('b', 'o', 'k', 'g'):
        joint = {'b': 'base', 'o': 'shoulder', 'k': 'elbow', 'g': 'gripper'}[key]
        if len(parts) < 2:
            lo, hi = _ARM_LIMITS[joint]
            print(f"[second_terminal] Usage: {key} <angle>  ({lo}-{hi})")
        else:
            try:
                _sendArmCommand(client, joint, int(parts[1]))
            except ValueError:
                print(f"[second_terminal] Angle must be an integer, got: '{parts[1]}'")

    elif key == 'v':
        if len(parts) < 2:
            print("[second_terminal] Usage: v <ms_per_degree>  (1-999)")
        else:
            try:
                _sendArmCommand(client, 'speed', int(parts[1]))
            except ValueError:
                print(f"[second_terminal] Speed must be an integer, got: '{parts[1]}'")

    else:
        print(f"[second_terminal] Unknown: '{key}'.  Valid: e h b o k g v q")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run():
    client = TCPClient(host=PI_HOST, port=PI_PORT)
    print(f"[second_terminal] Connecting to pi_sensor.py at {PI_HOST}:{PI_PORT}...")

    if not client.connect(timeout=10.0):
        print("[second_terminal] Could not connect.")
        print("  Make sure pi_sensor.py is running and waiting for a"
              " second terminal connection.")
        sys.exit(1)

    print("[second_terminal] Connected!")
    print("[second_terminal] Commands:")
    print("  e = E-Stop   q = quit")
    print("  h = arm home   b <n> = base   o <n> = shoulder   k <n> = elbow   g <n> = gripper   v <n> = speed")
    print("[second_terminal] Incoming robot packets will be printed below.\n")

    try:
        while True:
            # Check for forwarded TPackets from pi_sensor.py (non-blocking).
            if client.hasData():
                frame = recvTPacketFrame(client.sock)
                if frame is None:
                    print("[second_terminal] Connection to pi_sensor.py closed.")
                    break
                pkt = _unpackFrame(frame)
                if pkt:
                    _printPacket(pkt)

            # Check for keyboard input (non-blocking via select).
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                line = sys.stdin.readline()
                _handleInput(line, client)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[second_terminal] Exiting.")
    finally:
        client.close()


if __name__ == '__main__':
    run()
