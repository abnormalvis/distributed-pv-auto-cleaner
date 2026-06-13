/**
 * @file    serial_protocol.h
 * @brief   Serial protocol parser for ROS2 control bridge
 *
 * Implements the 12-byte binary frame protocol defined in
 * car_ws/src/serial_boost_bridge/include/serial_boost_bridge/protocol.hpp
 *
 * Frame format:
 *   Byte  0:     Header (0xFC)
 *   Byte  1:     Function code
 *   Bytes 2-9:   8-byte data payload (big-endian integers)
 *   Byte 10:     XOR checksum of bytes 0-9
 *   Byte 11:     Footer (0xDF)
 *
 * PC-to-MCU functions:
 *   0x06 MOTOR  �? 4x int16 RPM (we use only first 2 for tracked chassis)
 *   0x07 PID    �? 3x uint16 Kp, Ki, Kd
 *   0x08 SERVO  �? 2x uint8 angle
 */

#ifndef __SERIAL_PROTOCOL_H__
#define __SERIAL_PROTOCOL_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* === Frame constants (match protocol.hpp) === */
#define SERIAL_FRAME_SIZE         12U
#define SERIAL_DATA_SIZE           8U
#define SERIAL_HEADER            0xFC
#define SERIAL_FOOTER            0xDF

/* === Function codes (PC -> MCU) === */
#define SERIAL_FUNC_BATTERY      0x01U   /* MCU->PC: battery mV (uint16)   */
#define SERIAL_FUNC_ENCODERS     0x02U   /* MCU->PC: 4x encoder (uint16)   */
#define SERIAL_FUNC_GYRO         0x03U   /* MCU->PC: 3x gyro (int16)       */
#define SERIAL_FUNC_ACCEL        0x04U   /* MCU->PC: 3x accel (int16)      */
#define SERIAL_FUNC_EULER        0x05U   /* MCU->PC: 3x euler (int16)      */
#define SERIAL_FUNC_MOTOR        0x06U   /* PC->MCU: 4x RPM (int16)        */
#define SERIAL_FUNC_PID          0x07U   /* PC->MCU: 3x PID (uint16)       */
#define SERIAL_FUNC_SERVO        0x08U   /* PC->MCU: 2x servo (uint8)      */
#define SERIAL_FUNC_CHANNELS_1   0x09U   /* PC->MCU: channels 0-2 (3x i16) */
#define SERIAL_FUNC_CHANNELS_2   0x0AU   /* PC->MCU: channels 3-5 (3x i16) */

/* === Timing === */
#define SERIAL_TIMEOUT_MS         300U   /* 3x the 100 Hz ROS2 cycle */

/* === Parser states === */
typedef enum {
    SERIAL_PARSE_SYNC,     /* waiting for 0xFC header                     */
    SERIAL_PARSE_FUNC,     /* reading function byte                       */
    SERIAL_PARSE_DATA,     /* accumulating 8 data bytes                   */
    SERIAL_PARSE_CS,       /* reading XOR checksum                        */
    SERIAL_PARSE_FOOTER,   /* verifying 0xDF footer                       */
} serial_parse_state_t;

/* === Parsed motor command === */
typedef struct {
    int16_t  left_rpm;         /* left  track target speed (signed 16-bit) */
    int16_t  right_rpm;        /* right track target speed (signed 16-bit) */
    uint8_t  fresh;            /* set to 1 when a new valid frame arrives  */
    uint32_t last_frame_ms;    /* systick timestamp of last valid motor frame */
    uint32_t valid_frames;     /* total successfully-parsed motor frames   */
    uint32_t error_count;      /* CRC/footer mismatch count                */
} serial_cmd_t;

/* === 6-channel command (func 0x09 + 0x0A) === */
typedef struct {
    int16_t  channels[6];       /* ch[0..5], PWM 1000-2000, center=1500     */
    uint8_t  fresh;             /* set to 1 when a complete pair arrives     */
    uint32_t last_frame_ms;     /* systick of last valid CHANNELS_2 frame    */
    uint32_t valid_frames;      /* total successfully-parsed channel pairs   */
} serial_channels_t;

/* === API === */
void              serial_protocol_init(void);
const serial_cmd_t       *serial_get_cmd(void);
const serial_channels_t  *serial_get_channels(void);
uint8_t           serial_is_active(void);
void              serial_parse_byte(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_PROTOCOL_H__ */
