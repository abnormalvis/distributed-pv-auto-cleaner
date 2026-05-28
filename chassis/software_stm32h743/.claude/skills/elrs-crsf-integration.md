# ELRS CRSF Receiver Integration — STM32H743

## Overview

ExpressLRS receiver connected to USART10 (PE2 RX / PE3 TX), 420000 bps 8N1.
CRSF frames received via **UART RXNE interrupt** (byte-by-byte), parsed into 16 RC channels.

## Hardware Connection

| Signal | Pin  | AF  |
|--------|------|-----|
| USART10_RX | PE2 | AF4  |
| USART10_TX | PE3 | AF11 |

## Architecture

```
ELRS RX ──UART──> USART10 RXNE interrupt
                         │
                  USART10_IRQHandler:
                    read RDR → crsf_parse_byte()
                         │
                  State Machine Parser
                    (byte by byte)
                         │
                  crsf_unpack_channels()
                         │
                  crsf_data.channels[0..15]
                         │
                  main() loop reads via crsf_get_data()
```

## File Map

| File | Role |
|------|------|
| `Core/Inc/crsf.h` | CRSF protocol constants, `crsf_data_t` struct, public API |
| `Core/Src/crsf.c` | CRC8 table, state machine parser, channel unpacking |
| `Core/Src/usart.c` | USART10 420kbps init (CubeMX) + NVIC enable (manual) |
| `Core/Src/stm32h7xx_it.c` | `USART10_IRQHandler` reads RDR → feeds `crsf_parse_byte()` |
| `Core/Src/main.c` | `crsf_init()` after peripheral init, channel read in while(1) |
| `cmake/stm32cubemx/CMakeLists.txt` | Must list `crsf.c` in `MX_Application_Src` |

## USER CODE Modifications (CubeMX-Safe)

All manual additions live inside `USER CODE` sections. CubeMX preserves these on regeneration.

### 1. Core/Src/usart.c

**USER CODE BEGIN USART10_MspInit 1** — NVIC for USART10:
```c
/* USER CODE BEGIN USART10_MspInit 1 */
HAL_NVIC_SetPriority(USART10_IRQn, 1, 0);
HAL_NVIC_EnableIRQ(USART10_IRQn);
/* USER CODE END USART10_MspInit 1 */
```

**USER CODE BEGIN USART10_MspDeInit 1** — cleanup:
```c
/* USER CODE BEGIN USART10_MspDeInit 1 */
HAL_NVIC_DisableIRQ(USART10_IRQn);
/* USER CODE END USART10_MspDeInit 1 */
```

### 2. Core/Src/stm32h7xx_it.c

**USER CODE BEGIN Includes**:
```c
/* USER CODE BEGIN Includes */
#include "crsf.h"
/* USER CODE END Includes */
```

**USER CODE BEGIN EV** — extern declaration:
```c
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart10;
/* USER CODE END EV */
```

**USER CODE BEGIN 1** — RXNE-based ISR:
```c
/* USER CODE BEGIN 1 */

void USART10_IRQHandler(void)
{
  if (__HAL_UART_GET_FLAG(&huart10, UART_FLAG_RXNE)) {
    uint8_t byte = (uint8_t)(huart10.Instance->RDR & 0xFF);
    crsf_parse_byte(byte);
  }
  HAL_UART_IRQHandler(&huart10);
}

/* USER CODE END 1 */
```

### 3. Core/Inc/stm32h7xx_it.h

**USER CODE BEGIN EFP**:
```c
/* USER CODE BEGIN EFP */
void USART10_IRQHandler(void);
/* USER CODE END EFP */
```

### 4. Core/Src/main.c

**USER CODE BEGIN Includes**:
```c
/* USER CODE BEGIN Includes */
#include "crsf.h"
/* USER CODE END Includes */
```

**USER CODE BEGIN 2** — initialize after peripheral init:
```c
/* USER CODE BEGIN 2 */
crsf_init();
/* USER CODE END 2 */
```

**USER CODE BEGIN 3** — read channels in main loop:
```c
/* USER CODE BEGIN 3 */
if (crsf_is_connected()) {
    const crsf_data_t *rc = crsf_get_data();
    /* rc->channels[0..15] available for chassis motor control */
    /* Channel values range 172-1811, center ~992 */
    (void)rc;
}
/* USER CODE END 3 */
```

### 5. cmake/stm32cubemx/CMakeLists.txt

Add `crsf.c` to `MX_Application_Src`:
```cmake
set(MX_Application_Src
    ...
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/crsf.c          # <-- ADD THIS LINE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32h7xx_it.c
    ...
)
```

## CRSF Protocol Reference

### Frame Format (RC Channels, Type 0x16)

```
Byte 0:    Sync/Address (0xC8 = RC channels, 0xEE = ELRS)
Byte 1:    Frame Length (payload bytes + 1 for type = 23 for RC)
Byte 2:    Type (0x16 = RC Channels Packed)
Bytes 3-24: Payload (22 bytes: 16 channels × 11 bits)
Byte 25:   CRC8 DVB-S2 (polynomial 0xD5, covers byte 2 through 24)
```

### Channel Unpacking (11-bit Big-Endian)

```
ch[0]  = (p[0]  << 3) | (p[1]  >> 5)
ch[1]  = (p[1]  << 6) | (p[2]  >> 2)   // masked & 0x1F
ch[2]  = (p[2]  << 9) | (p[3]  << 1) | (p[4] >> 7)  // masked & 0x03
ch[3]  = (p[4]  << 4) | (p[5]  >> 4)   // masked & 0x7F
...
ch[15] = (p[20] << 8) | p[21]           // masked & 0x07
```

### Channel Value Range

| Label | Value |
|-------|-------|
| Min | 172 |
| Center | 992 |
| Max | 1811 |

Typical mapping: CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw, CH5=Arm.

## API

```c
void     crsf_init(void);                    // Init parser, enable RXNE interrupt
uint8_t  crsf_is_connected(void);            // Returns 1 if valid frames received within 500ms
const crsf_data_t *crsf_get_data(void);      // Returns pointer to channel data
void     crsf_parse_byte(uint8_t byte);      // Feed one byte from USART10 RXNE ISR

// crsf_data_t fields:
//   channels[0..15] — 11-bit RC values (172–1811)
//   valid_frames    — count of successfully parsed frames
//   callback_count  — total bytes received (= rx_bytes_total, same value in RXNE mode)
//   rx_bytes_total  — total bytes received
//   error_count     — CRC failures / invalid frames
//   connected       — 1 if receiving valid frames
//   last_frame_ms   — HAL_GetTick() timestamp of last valid frame
```

## Debug Watch Expressions (VSCode / GDB)

```
crsf_data.channels[0]@16    → all 16 channels
crsf_data.callback_count     → total bytes received
crsf_data.valid_frames       → successfully decoded frames
crsf_data.error_count        → CRC failures
crsf_data.connected          → 1 = receiving valid frames
```

## Debugging Flow

If channels stay at 992 and don't react to transmitter sticks:

| Symptom | Likely Cause | Check |
|---------|-------------|-------|
| `callback_count == 0` | No UART data arriving | Logic analyzer on PE2 (420kbps, 26B frames) |
| `callback_count > 0` but `valid_frames == 0` | Wrong baud or protocol | Verify ELRS output is CRSF at 420k |
| `error_count` keeps rising | CRC mismatch, bad sync | Check if sync byte is 0xC8 or 0xEE |
| `valid_frames > 0` but channels don't change | TX not bound / mixer not configured | Check ELRS TX module settings |

## Interrupt Priority

| IRQ | Prio | Purpose |
|-----|------|---------|
| DMA1_Stream0 | 0,0 | USART10 TX done (CubeMX default) |
| DMA1_Stream1 | 0,0 | USART1 TX done (CubeMX default) |
| USART10 | 1,0 | RXNE byte reception + error handling |

## CubeMX Regeneration Checklist

After regenerating from `.ioc`:

1. `cmake/stm32cubemx/CMakeLists.txt` — add `crsf.c` line back
2. `Core/Src/usart.c` — add NVIC for USART10 in MSP Init + DeInit USER CODE
3. `Core/Src/stm32h7xx_it.c` — add `#include "crsf.h"` + `extern huart10` + `USART10_IRQHandler` (RXNE mode)
4. `Core/Inc/stm32h7xx_it.h` — add `void USART10_IRQHandler(void);`
5. Verify `Core/Src/main.c` has `#include "crsf.h"`, `crsf_init()`, channel read loop
6. Verify USART10 baud rate is 420000 (CubeMX may reset to default 115200 or 921600)

## Design Note: Why RXNE Instead of DMA+IDLE

The original implementation used `HAL_UARTEx_ReceiveToIdle_DMA` with DMA2_Stream0 for reception. This approach required:
- DMA stream configuration + DMAMUX routing
- IDLE interrupt detection in the HAL callback chain
- DMA interrupt handler for transfer completion
- Restarting DMA after each IDLE event

In practice, the HAL's DMA+IDLE callback chain proved fragile — data was visible on the wire but `HAL_UARTEx_RxEventCallback` was never invoked. Switching to direct RXNE interrupt byte reading eliminates all DMA complexity. At 420kbps (~42 KB/s, one byte per ~24µs), the STM32H7 at 480MHz handles each byte with ~11,500 CPU cycles available — far more than needed.
