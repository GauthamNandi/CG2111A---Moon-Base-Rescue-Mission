/*
 * sensor_miniproject_with_arm.ino
 * Studio 13 Sensor Mini-Project  +  Robotic Arm control
 *
 * Files in this folder:
 *   packets.h        - TPacket protocol (must stay in sync with pi_sensor.py)
 *   serial_driver.h  - Transport layer (sendFrame / receiveFrame)
 *   sensor_miniproject_with_arm.ino  (this file)
 *
 * =============================================================
 * PIN ASSIGNMENTS
 * =============================================================
 *
 *  E-Stop button   : Arduino Mega pin 3  (PD1 / INT1)
 *
 *  TCS3200 S0      : Arduino Mega pin 26 (PA4)
 *  TCS3200 S1      : Arduino Mega pin 27 (PA5)
 *  TCS3200 S2      : Arduino Mega pin 28 (PA6)
 *  TCS3200 S3      : Arduino Mega pin 29 (PA7)
 *  TCS3200 OUT     : Arduino Mega pin 30 (PC7)
 *
 *  Arm servos (Timer 5 / Port K):
 *    BASE     : PK0  (Arduino Mega pin A8)
 *    SHOULDER : PK1  (Arduino Mega pin A9)
 *    ELBOW    : PK2  (Arduino Mega pin A10)
 *    GRIPPER  : PK3  (Arduino Mega pin A11)
 *
 * Arm commands arrive as TPacket COMMAND packets from the RPi:
 *   COMMAND_ARM_BASE     params[0] = angle  0-180
 *   COMMAND_ARM_SHOULDER params[0] = angle  0-100
 *   COMMAND_ARM_ELBOW    params[0] = angle  60-160
 *   COMMAND_ARM_GRIPPER  params[0] = angle  90-105
 *   COMMAND_ARM_HOME     (no params)
 *   COMMAND_ARM_SPEED    params[0] = ms per degree
 *
 * Add the six entries above to packets.h TCommandType enum and keep
 * pi_sensor.py in sync.
 */

#include <AFMotor.h>
#include "packets.h"
#include "serial_driver.h"

// =============================================================
// Robotic Arm — servo definitions
// =============================================================

#define ARM_BASE     0   // PK0
#define ARM_SHOULDER 1   // PK1
#define ARM_ELBOW    2   // PK2
#define ARM_GRIPPER  3   // PK3

#define DEGREE_90 3000

// Arm state (volatile where shared with ISRs)
volatile int armPIN   = (1 << ARM_BASE);
volatile int armIndex = 0;

int armTarget[4] = {2733, 1000, 2555, DEGREE_90};
int armPulse[4]  = {2733, 1000, 2555, DEGREE_90};
int armHome[4]   = {2733, 1000, 2555, 3333};
int armStep[4]   = {1, 1, 1, 1};
int armMsPerDeg  = 1;
//int armSpeed = 2;

/*
 * calculateArmTarget() — compute step direction for one servo.
 * angle is in degrees (0-180 range before caller constrains it).
 */
static void calculateArmTarget(int angleDeg, int servo) {
    armTarget[servo] = 1000 + ((long)angleDeg * 4000L) / 180L;
    if (armPulse[servo] == armTarget[servo]) return;
    armStep[servo] = (armTarget[servo] > armPulse[servo]) ? 1 : -1;
}

/*
 * homeArm() — send all servos back to their home positions.
 */
static void homeArm() {
    for (int i = 0; i < 4; i++) {
        armTarget[i] = armHome[i];
        armStep[i] = (armTarget[i] > armPulse[i]) ? 1 : -1;
    }
}

// =============================================================
// Timer 5 ISRs — servo PWM generation (20 ms period, 4 servos)
// =============================================================

/*
 * COMPA fires at the start of every 20 ms frame.
 * Reset to servo 0 and raise its pin.
 */
ISR(TIMER5_COMPA_vect) {
    armIndex = 0;
    armPIN   = (1 << armIndex);
    OCR5B    = armPulse[armIndex];
    PORTK   |= armPIN;
}

/*
 * COMPB fires after each pulse-width interval.
 * Lower the current pin, then advance to the next servo.
 */
ISR(TIMER5_COMPB_vect) {
    PORTK &= ~armPIN;
    if (armIndex < 3) {
        armIndex++;
        armPIN  = (1 << armIndex);
        OCR5B  += armPulse[armIndex];
        PORTK  |= armPIN;
    }
}

// =============================================================
// Packet helpers
// =============================================================

static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

ISR(INT2_vect) {
    static uint32_t lastDebounceTime = 0;
    uint32_t now = millis();
    if (now - lastDebounceTime < 100) return;
    lastDebounceTime = now;

    bool pressed = (PIND & (1 << PD2)) != 0;

    if (pressed && buttonState == STATE_RUNNING) {
        buttonState  = STATE_STOPPED;
        stateChanged = true;
    } else if (!pressed && buttonState == STATE_STOPPED) {
        buttonState  = STATE_RUNNING;
        stateChanged = true;
    }
}

// =============================================================
// TCS3200 Color Sensor
// =============================================================

static uint32_t measureChannel(uint32_t window_ms) {
    uint32_t settle = millis();
    while (millis() - settle < 2) { /* wait */ }

    uint32_t count = 0;
    bool prev = (PINC & (1 << 7)) != 0;

    uint32_t start = millis();
    while (millis() - start < window_ms) {
        bool curr = (PINC & (1 << 7)) != 0;
        if (curr && !prev) count++;
        prev = curr;
    }
    return count;
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    PORTA &= ~((1 << PA6) | (1 << PA7));   // Red:   S2=L, S3=L
    *r = measureChannel(100) * 10;

    PORTA |=  (1 << PA6) | (1 << PA7);     // Green: S2=H, S3=H
    *g = measureChannel(100) * 10;

    PORTA &= ~(1 << PA6);                  // Blue:  S2=L, S3=H
    PORTA |=  (1 << PA7);
    *b = measureChannel(100) * 10;

    PORTA &= ~((1 << PA6) | (1 << PA7));  // Leave at Red
    
}

// =============================================================
// Command handler
// =============================================================

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {

        // --- E-Stop ---
        case COMMAND_ESTOP:
            cli();
            buttonState  = (buttonState == STATE_RUNNING) ? STATE_STOPPED : STATE_RUNNING;
            stateChanged = false;
            sei();
            if (buttonState == STATE_STOPPED) stop();
            sendResponse(RESP_OK, 0);
            sendStatus(buttonState);
            break;

        // --- Movement commands ---
        case COMMAND_FORWARD:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            forward((int)cmd->params[0]);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_BACKWARD:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            backward((int)cmd->params[0]);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_LEFT:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            ccw((int)cmd->params[0]);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_RIGHT:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            cw((int)cmd->params[0]);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_STOP_MOVE:
            stop();
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_SET_SPEED:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            setMotorSpeed((int)cmd->params[0]);
            sendResponse(RESP_OK, 0);
            break;

        // --- Color sensor ---
        case COMMAND_COLOR: {
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            uint32_t r, g, b;
            readColorChannels(&r, &g, &b);
            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command    = RESP_COLOR;
            pkt.params[0]  = r;
            pkt.params[1]  = g;
            pkt.params[2]  = b;
            sendFrame(&pkt);
            break;
        }

        // -------------------------------------------------------
        // --- Robotic Arm commands (new) ---
        // -------------------------------------------------------

        case COMMAND_ARM_BASE:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            calculateArmTarget(constrain((int)cmd->params[0], 0, 180), ARM_BASE);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_ARM_SHOULDER:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            calculateArmTarget(constrain((int)cmd->params[0], 0, 100), ARM_SHOULDER);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_ARM_ELBOW:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            calculateArmTarget(constrain((int)cmd->params[0], 60, 180), ARM_ELBOW);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_ARM_GRIPPER:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            calculateArmTarget(constrain((int)cmd->params[0], 90, 105), ARM_GRIPPER);
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_ARM_HOME:
            if (buttonState == STATE_STOPPED) { sendStatus(STATE_STOPPED); break; }
            homeArm();
            sendResponse(RESP_OK, 0);
            break;

        case COMMAND_ARM_SPEED:
            armMsPerDeg = constrain((int)cmd->params[0], 1, 10);
           // for(int i = 0; i < 4; i++){
           //     armStep[i] = (armStep[i] > 0) ? armSpeed : -1*armSpeed;
           // }
            sendResponse(RESP_OK, 0);
            break;
    }
}

// =============================================================
// setup()
// =============================================================

void setup() {
    // --- Serial link to RPi (baud must match pi_sensor.py) ---
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif

    // --- E-Stop button: PD1 (pin 3 / INT1), external 10 kΩ pull-down ---
    DDRD  &= ~(1 << PD2);
    PORTD &= ~(1 << PD2);
    EICRA = (EICRA & ~((1 << ISC21) | (1 << ISC20))) | (1 << ISC20);
    EIMSK |= (1 << INT2);

    // --- TCS3200 colour sensor pins ---
    DDRA |= (1 << PA4) | (1 << PA5) | (1 << PA6) | (1 << PA7);
    DDRC  &= ~(1 << 7);
    PORTC &= ~(1 << 7);
    PORTA |= (1 << PA4);                       // S0 = HIGH (20 % scaling)
    PORTA &= ~(1 << PA5);                       // S1 = LOW
    PORTA &= ~((1 << PA6) | (1 << PA7));        // Default channel: Red

    // --- Arm servos: PK0-PK3 as outputs ---
    DDRK |= (1 << ARM_BASE) | (1 << ARM_SHOULDER) | (1 << ARM_ELBOW) | (1 << ARM_GRIPPER);

    // --- Timer 5: CTC mode, 20 ms period at prescaler /8 (16 MHz) ---
    // Period   = 40000 ticks × (8 / 16 000 000) = 20 ms  ✓
    // Pulse OCR5B is set dynamically in the ISR.
    TCCR5A  = 0b00000000;          // Normal port operation
    TIMSK5 |= 0b00000110;          // Enable OCIE5A and OCIE5B
    OCR5A   = 40000 - 1;           // 20 ms frame
    OCR5B   = DEGREE_90;           // Initial compare value
    TCNT5   = 0;
    TCCR5B  = 0b00001010;          // CTC (WGM52), prescaler /8

    // --- Enable global interrupts ---
    sei();
}

// =============================================================
// loop()
// =============================================================
unsigned long prev_millis = 0;
void loop() {
    // --- 1. Report E-Stop state changes to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();

        if (state == STATE_STOPPED) stop();
        sendStatus(state);
    }

    // --- 2. Process incoming packets from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }

    // --- 3. Step arm servos toward their targets ---
    // Each call advances every servo by one tick, then waits armMsPerDeg ms.
    // This runs every loop iteration so it interleaves smoothly with
    // packet reception; the delay is short (default 10 ms) and does not
    // block the E-Stop ISR.
    unsigned long current_millis = millis();
    if(current_millis - prev_millis >= armMsPerDeg) {
        prev_millis = current_millis;
        
        for (int i = 1; i < 4; i++) {
            if (armPulse[i] != armTarget[i]) {
                armPulse[i] += armStep[i];
            }
        }

        for (int i = 0; i < 5; i++) {
            if (armPulse[0] != armTarget[0]) {
                armPulse[0] += armStep[0];
            }
        }
    }
}
