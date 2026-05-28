#ifndef __CRSF_H__
#define __CRSF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define CRSF_RX_BUF_SIZE        256
#define CRSF_MAX_FRAME_SIZE     32
#define CRSF_MAX_CHANNELS       16

/* CRSF sync bytes */
#define CRSF_ADDR_RC_CHANNELS   0xC8
#define CRSF_ADDR_ELRS          0xEE

/* CRSF frame types */
#define CRSF_TYPE_RC_CHANNELS   0x16

/* RC channels packed payload size: 16 channels × 11 bits = 176 bits = 22 bytes */
#define CRSF_RC_PAYLOAD_SIZE    22

/* Channel value range (ELRS extended) */
#define CRSF_CHANNEL_MIN        172
#define CRSF_CHANNEL_MID        992
#define CRSF_CHANNEL_MAX        1811

typedef struct {
    uint16_t channels[CRSF_MAX_CHANNELS];
    uint32_t last_frame_ms;       /* systick timestamp of last valid frame */
    uint16_t valid_frames;        /* count of successfully parsed frames */
    uint16_t callback_count;      /* total bytes fed to parser */
    uint16_t rx_bytes_total;      /* total bytes received */
    uint16_t error_count;         /* CRC failures */
    uint16_t sync_found;          /* times sync byte (0xC8/0xEE) was seen */
    uint16_t invalid_len;         /* times frame length was out of range */
    uint16_t unknown_type;        /* times frame type wasn't 0x16 */
    uint16_t link_stats_frames;   /* type 0x14 (Link Statistics) frames seen */
    uint8_t  last_frame_type;     /* most recent CRSF frame type byte */
    uint8_t  last_raw_byte;       /* most recent byte received */
    uint8_t  raw_buf[32];         /* ring buffer of last 32 raw bytes */
    uint8_t  raw_idx;             /* ring buffer write index */
    uint8_t  connected;           /* 1 if receiving valid frames */
    uint8_t  last_payload[22];    /* raw 22-byte payload of most recent valid frame */
} crsf_data_t;

void crsf_init(void);
const crsf_data_t *crsf_get_data(void);
uint8_t crsf_is_connected(void);

/* Called from UART RXNE interrupt — feeds one byte to the parser state machine */
void crsf_parse_byte(uint8_t byte);

/* Bulk parse from buffer (used with DMA+IDLE mode) */
void crsf_process_data(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CRSF_H__ */
