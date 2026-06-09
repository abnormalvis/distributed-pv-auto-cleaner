#include "vofa.h"
#include "usart.h"
#include <stdio.h>

#define MAX_FRAME_SIZE 256

static uint8_t send_buf[MAX_FRAME_SIZE];

/**
 * @brief  Send CRSF channels as FireWater ASCII frame over USART1.
 * @note   FireWater ASCII format: ch0,ch1,...,chN\n
 *         VOFA+ auto-detects channel count from the first frame.
 */
void vofa_send_channels(const uint16_t *channels, uint8_t num_channels)
{
    int len = 0;

    for (uint8_t i = 0; i < num_channels; i++) {
        if (i > 0) {
            send_buf[len++] = ',';
        }
        /* Simple itoa-style conversion: channel value is 172..1811, fits in 4 chars */
        len += sprintf((char *)(send_buf + len), "%u", channels[i]);
    }
    send_buf[len++] = '\n';

    usart1_debug_transmit(send_buf, (uint16_t)len);
}
