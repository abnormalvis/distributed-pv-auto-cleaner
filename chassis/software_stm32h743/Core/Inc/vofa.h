#ifndef __VOFA_H__
#define __VOFA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define VOFA_MAX_CHANNELS  16

/* Send CRSF channels as FireWater ASCII format (comma-separated + \n) */
void vofa_send_channels(const uint16_t *channels, uint8_t num_channels);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */
