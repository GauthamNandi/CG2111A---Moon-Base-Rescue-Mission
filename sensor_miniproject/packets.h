/*
 * packets.h
 * Studio 13: Sensor Mini-Project
 *
 * TPacket protocol: enums, struct, and framing constants.
 * This file must be kept in sync with the constants in pi_sensor.py.
 */

#pragma once

#include <stdint.h>

// =============================================================
// TPacket protocol
// =============================================================

typedef enum {
    PACKET_TYPE_COMMAND  = 0,
    PACKET_TYPE_RESPONSE = 1,
    PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

typedef enum {
    COMMAND_ESTOP     = 0,
    COMMAND_COLOR     = 1,   // Activity 2: request R/G/B channel frequencies from TCS3200
    COMMAND_FORWARD   = 2,   // Studio 15: drive forward;  params[0] = speed (0-255)
    COMMAND_BACKWARD  = 3,   // Studio 15: drive backward; params[0] = speed (0-255)
    COMMAND_LEFT      = 4,   // Studio 15: turn CCW;       params[0] = speed (0-255)
    COMMAND_RIGHT     = 5,   // Studio 15: turn CW;        params[0] = speed (0-255)
    COMMAND_STOP_MOVE = 6,   // Studio 15: stop motors (not an E-Stop)
    COMMAND_SET_SPEED    = 7,   // Studio 15: update speed on all motors without changing direction
    COMMAND_ARM_BASE     = 8,   // Arm: move base servo;     params[0] = angle (0-180)
    COMMAND_ARM_SHOULDER = 9,   // Arm: move shoulder servo; params[0] = angle (0-100)
    COMMAND_ARM_ELBOW    = 10,  // Arm: move elbow servo;    params[0] = angle (60-160)
    COMMAND_ARM_GRIPPER  = 11,  // Arm: move gripper servo;  params[0] = angle (90-105)
    COMMAND_ARM_HOME     = 12,  // Arm: return all servos to home position
    COMMAND_ARM_SPEED    = 13,  // Arm: set movement speed;  params[0] = ms per degree
} TCommandType;

typedef enum {
    RESP_OK     = 0,
    RESP_STATUS = 1,
    RESP_COLOR  = 2,   // Activity 2: response carrying R/G/B frequencies in Hz
} TResponseType;

typedef enum {
    STATE_RUNNING = 0,
    STATE_STOPPED = 1,
} TState;

typedef struct {
    uint8_t  packetType;
    uint8_t  command;
    uint8_t  dummy[2];
    uint32_t params[3];
} TPacket;

// =============================================================
// Framing constants
// =============================================================

#define MAGIC_HI        0xDE
#define MAGIC_LO        0xAD
#define TPACKET_SIZE    ((uint8_t)sizeof(TPacket))   // 16 bytes
#define FRAME_SIZE      (2 + TPACKET_SIZE + 1)       // 19 bytes
