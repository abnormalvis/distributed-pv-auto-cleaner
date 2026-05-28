#include "crsf.h"
#include "usart.h"

/* CRC8 DVB-S2 lookup table (polynomial 0xD5)
 * Same as Betaflight/INAV: src/main/rx/crsf.c
 */
static const uint8_t crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
};

crsf_data_t crsf_data;

typedef enum {
    CRSF_STATE_SYNC,
    CRSF_STATE_LEN,
    CRSF_STATE_TYPE,
    CRSF_STATE_PAYLOAD,
    CRSF_STATE_CRC,
} crsf_state_t;

static crsf_state_t parser_state = CRSF_STATE_SYNC;
static uint8_t  frame_buf[CRSF_MAX_FRAME_SIZE];
static uint8_t  frame_len;
static uint8_t  frame_type;
static uint8_t  payload_idx;
static uint8_t  payload_expect;
static uint8_t  crc_accum;

static void crsf_unpack_channels(const uint8_t *payload)
{
    const uint8_t *p = payload;

    /* 22 bytes → 16 channels × 11 bits, LSB-first packed
     * ExpressLRS / EdgeTX compatible format.
     * Each channel: lower byte | (upper bits << 8), masked to 11 bits.
     */
    crsf_data.channels[0]  = ((uint16_t)p[0]  | (uint16_t)p[1]<<8)  & 0x07FF;
    crsf_data.channels[1]  = ((uint16_t)p[1]>>3 | (uint16_t)p[2]<<5)  & 0x07FF;
    crsf_data.channels[2]  = ((uint16_t)p[2]>>6 | (uint16_t)p[3]<<2 | (uint16_t)p[4]<<10) & 0x07FF;
    crsf_data.channels[3]  = ((uint16_t)p[4]>>1 | (uint16_t)p[5]<<7)  & 0x07FF;
    crsf_data.channels[4]  = ((uint16_t)p[5]>>4 | (uint16_t)p[6]<<4)  & 0x07FF;
    crsf_data.channels[5]  = ((uint16_t)p[6]>>7 | (uint16_t)p[7]<<1 | (uint16_t)p[8]<<9)  & 0x07FF;
    crsf_data.channels[6]  = ((uint16_t)p[8]>>2 | (uint16_t)p[9]<<6)  & 0x07FF;
    crsf_data.channels[7]  = ((uint16_t)p[9]>>5 | (uint16_t)p[10]<<3) & 0x07FF;
    crsf_data.channels[8]  = ((uint16_t)p[11] | (uint16_t)p[12]<<8) & 0x07FF;
    crsf_data.channels[9]  = ((uint16_t)p[12]>>3 | (uint16_t)p[13]<<5) & 0x07FF;
    crsf_data.channels[10] = ((uint16_t)p[13]>>6 | (uint16_t)p[14]<<2 | (uint16_t)p[15]<<10) & 0x07FF;
    crsf_data.channels[11] = ((uint16_t)p[15]>>1 | (uint16_t)p[16]<<7) & 0x07FF;
    crsf_data.channels[12] = ((uint16_t)p[16]>>4 | (uint16_t)p[17]<<4) & 0x07FF;
    crsf_data.channels[13] = ((uint16_t)p[17]>>7 | (uint16_t)p[18]<<1 | (uint16_t)p[19]<<9) & 0x07FF;
    crsf_data.channels[14] = ((uint16_t)p[19]>>2 | (uint16_t)p[20]<<6) & 0x07FF;
    crsf_data.channels[15] = ((uint16_t)p[20]>>5 | (uint16_t)p[21]<<3) & 0x07FF;
}

void crsf_init(void)
{
    uint8_t i;
    for (i = 0; i < CRSF_MAX_CHANNELS; i++) {
        crsf_data.channels[i] = CRSF_CHANNEL_MID;
    }
    crsf_data.last_frame_ms = 0;
    crsf_data.connected = 0;
    crsf_data.valid_frames = 0;
    crsf_data.callback_count = 0;
    crsf_data.rx_bytes_total = 0;
    crsf_data.error_count = 0;
    crsf_data.sync_found = 0;
    crsf_data.invalid_len = 0;
    crsf_data.unknown_type = 0;
    crsf_data.link_stats_frames = 0;
    crsf_data.last_frame_type = 0;
    crsf_data.last_raw_byte = 0;
    crsf_data.raw_idx = 0;
    parser_state = CRSF_STATE_SYNC;

    __HAL_UART_ENABLE_IT(&huart10, UART_IT_RXNE);
}

void crsf_parse_byte(uint8_t byte)
{
    crsf_data.callback_count++;
    crsf_data.rx_bytes_total++;
    crsf_data.last_raw_byte = byte;
    crsf_data.raw_buf[crsf_data.raw_idx & 0x1F] = byte;
    crsf_data.raw_idx++;

    switch (parser_state) {
    case CRSF_STATE_SYNC:
        /* Match Betaflight: CRSF_ADDRESS_FLIGHT_CONTROLLER (0xC8)
         * Also accept CRSF_ADDRESS_CRSF_TRANSMITTER (0xEE) for ELRS */
        if (byte == CRSF_ADDR_RC_CHANNELS || byte == CRSF_ADDR_ELRS) {
            crsf_data.sync_found++;
            frame_buf[0] = byte;
            parser_state = CRSF_STATE_LEN;
        }
        break;

    case CRSF_STATE_LEN:
        /* CRSF frame length = type + payload bytes (1..30)
         * Valid RC channels: 1(type) + 22(payload) = 23
         * CRC is NOT included in the length field */
        if (byte < 2 || byte > CRSF_MAX_FRAME_SIZE - 3) {
            crsf_data.invalid_len++;
            parser_state = CRSF_STATE_SYNC;
            break;
        }
        frame_len = byte;
        frame_buf[1] = byte;
        parser_state = CRSF_STATE_TYPE;
        break;

    case CRSF_STATE_TYPE:
        frame_type = byte;
        crsf_data.last_frame_type = byte;
        frame_buf[2] = byte;
        crc_accum = crc8_table[byte];
        payload_idx = 0;
        if (frame_type == CRSF_TYPE_RC_CHANNELS) {
            /* ELRS uses len=24 (CRC included), TBS uses len=23 (CRC excluded) */
            if (frame_len == CRSF_RC_PAYLOAD_SIZE + 1) {
                payload_expect = CRSF_RC_PAYLOAD_SIZE;     /* 22, CRC after payload */
                parser_state = CRSF_STATE_PAYLOAD;
            } else if (frame_len == CRSF_RC_PAYLOAD_SIZE + 2) {
                payload_expect = CRSF_RC_PAYLOAD_SIZE;     /* 22, CRC counted in len */
                parser_state = CRSF_STATE_PAYLOAD;
            } else {
                crsf_data.unknown_type++;
                parser_state = CRSF_STATE_SYNC;
            }
        } else {
            crsf_data.unknown_type++;
            if (frame_type == 0x14) {
                crsf_data.link_stats_frames++;
            }
            parser_state = CRSF_STATE_SYNC;
        }
        break;

    case CRSF_STATE_PAYLOAD:
        frame_buf[3 + payload_idx] = byte;
        crc_accum = crc8_table[crc_accum ^ byte];
        payload_idx++;
        if (payload_idx >= payload_expect) {
            parser_state = CRSF_STATE_CRC;
        }
        break;

    case CRSF_STATE_CRC:
        if (byte == crc_accum) {
            if (frame_type == CRSF_TYPE_RC_CHANNELS) {
                /* Save raw payload for diagnostics */
                for (uint8_t i = 0; i < CRSF_RC_PAYLOAD_SIZE; i++) {
                    crsf_data.last_payload[i] = frame_buf[3 + i];
                }
                crsf_unpack_channels(&frame_buf[3]);
                crsf_data.last_frame_ms = HAL_GetTick();
                crsf_data.connected = 1;
                crsf_data.valid_frames++;
            }
        } else {
            crsf_data.error_count++;
        }
        parser_state = CRSF_STATE_SYNC;
        break;
    }
}

void crsf_process_data(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        crsf_parse_byte(data[i]);
    }
}

const crsf_data_t *crsf_get_data(void)
{
    return &crsf_data;
}

uint8_t crsf_is_connected(void)
{
    uint32_t now = HAL_GetTick();
    if (now - crsf_data.last_frame_ms > 500) {
        crsf_data.connected = 0;
    }
    return crsf_data.connected;
}
