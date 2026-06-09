/**
 * @file    serial_protocol.c
 * @brief   Byte-level state-machine parser for the 12-byte ROS2 serial bridge protocol.
 *
 * Parsing flow:
 *   SYNC  — wait for 0xFC
 *   FUNC  — read function code
 *   DATA  — accumulate 8 payload bytes
 *   CS    — read XOR checksum
 *   FOOTER— verify 0xDF, validate checksum, dispatch by function
 *
 * Only func=0x06 (Motor) is dispatched in the motor command struct.
 * Other functions are recognised but ignored for now (Phase-2 telemetry).
 *
 * Designed to be called from interrupt context (e.g. CDC_Receive_HS).
 * All state is file-static — no heap allocations, no blocking calls.
 */

#include "serial_protocol.h"
#include <string.h>   /* memset */

/* ========================================================================
 * File-static state
 * ======================================================================== */

static serial_cmd_t       s_cmd;                  /* parsed motor command */
static serial_parse_state_t s_state = SERIAL_PARSE_SYNC;
static uint8_t            s_frame_buf[SERIAL_FRAME_SIZE];
static uint8_t            s_data_idx;
static uint8_t            s_checksum;             /* running XOR of bytes 0..9 */

/* ========================================================================
 * Helpers
 * ======================================================================== */

/**
 * @brief  XOR checksum of the first 10 bytes (matching protocol.hpp::checksum_10).
 */
static uint8_t compute_checksum(const uint8_t *bytes_0_to_9)
{
    uint8_t cs = 0;
    for (uint8_t i = 0; i < 10U; i++) {
        cs ^= bytes_0_to_9[i];
    }
    return cs;
}

/**
 * @brief  Decode a big-endian int16 from two consecutive bytes.
 */
static int16_t i16_from_be(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

/**
 * @brief  Dispatch a validated frame by function code.
 */
static void dispatch_frame(uint8_t func, const uint8_t *data)
{
    switch (func) {
    case SERIAL_FUNC_MOTOR:
        /* Only first 2 motors for tracked chassis (2 motors).
         * ROS2 sends 4 values; bytes 6-9 (motor 3+4) are ignored. */
        s_cmd.left_rpm  = i16_from_be(data[0], data[1]);
        s_cmd.right_rpm = i16_from_be(data[2], data[3]);
        s_cmd.fresh       = 1;
        s_cmd.last_frame_ms = HAL_GetTick();
        s_cmd.valid_frames++;
        break;
    case SERIAL_FUNC_PID:
        /* PID parameters received — store for future PID controller */
        break;
    case SERIAL_FUNC_SERVO:
        /* Servo command received — no servos on this chassis, ignore */
        break;
    default:
        /* Unknown or PC->MCU function — ignore */
        break;
    }
}

/* ========================================================================
 * Public API
 * ======================================================================== */

/**
 * @brief  Initialise parser state.
 */
void serial_protocol_init(void)
{
    memset(&s_cmd, 0, sizeof(s_cmd));
    s_state    = SERIAL_PARSE_SYNC;
    s_data_idx = 0;
    s_checksum = 0;
}

/**
 * @brief  Return pointer to the parsed motor command struct.
 */
const serial_cmd_t *serial_get_cmd(void)
{
    return &s_cmd;
}

/**
 * @brief  Return 1 if valid motor frames were received within the timeout window.
 */
uint8_t serial_is_active(void)
{
    if (s_cmd.valid_frames == 0) {
        return 0;
    }
    return (HAL_GetTick() - s_cmd.last_frame_ms) < SERIAL_TIMEOUT_MS;
}

/**
 * @brief  Feed one byte into the parser (safe to call from ISR).
 *
 * This implements a classic 5-state frame parser:
 *   SYNC → FUNC → DATA(8) → CS → FOOTER → (back to SYNC)
 *
 * On a valid frame the payload is dispatched via dispatch_frame().
 * On any error (bad header, bad footer, checksum mismatch) the parser
 * resets to SYNC immediately — no buffering of partial garbage.
 */
void serial_parse_byte(uint8_t byte)
{
    switch (s_state) {

    /* ---- waiting for 0xFC sync byte ---- */
    case SERIAL_PARSE_SYNC:
        if (byte == SERIAL_HEADER) {
            s_frame_buf[0] = byte;
            s_checksum      = byte;           /* start XOR with header */
            s_state         = SERIAL_PARSE_FUNC;
        }
        /* else: stay in SYNC — skip garbage bytes */
        break;

    /* ---- read function code ---- */
    case SERIAL_PARSE_FUNC:
        s_frame_buf[1] = byte;
        s_checksum    ^= byte;
        s_data_idx     = 0;
        s_state        = SERIAL_PARSE_DATA;
        break;

    /* ---- accumulate 8 data bytes ---- */
    case SERIAL_PARSE_DATA:
        s_frame_buf[2 + s_data_idx] = byte;
        s_checksum                 ^= byte;
        s_data_idx++;
        if (s_data_idx >= SERIAL_DATA_SIZE) {
            s_state = SERIAL_PARSE_CS;
        }
        break;

    /* ---- read XOR checksum ---- */
    case SERIAL_PARSE_CS:
        s_frame_buf[10] = byte;               /* checksum byte at position 10 */
        s_state         = SERIAL_PARSE_FOOTER;
        break;

    /* ---- verify 0xDF footer + checksum ---- */
    case SERIAL_PARSE_FOOTER:
        if (byte == SERIAL_FOOTER) {
            s_frame_buf[11] = byte;
            if (s_checksum == s_frame_buf[10]) {
                /* Valid frame — dispatch */
                dispatch_frame(s_frame_buf[1], &s_frame_buf[2]);
            } else {
                /* Checksum mismatch */
                s_cmd.error_count++;
            }
        } else {
            /* Bad footer */
            s_cmd.error_count++;
        }
        /* Always go back to hunting for the next header */
        s_state = SERIAL_PARSE_SYNC;
        break;

    default:
        s_state = SERIAL_PARSE_SYNC;
        break;
    }
}
