#ifndef __CAN_IO_H__
#define __CAN_IO_H__

#include "main.h"

#define CANIO_ADDRESS           0x01
#define CANIO_FUNC_WRITE        0x01
#define CANIO_FUNC_READ_RELAY   0x02
#define CANIO_FUNC_READ_INPUT   0x03

/* Standard frame ID = (function_code << 8) | address */
#define CANIO_ID_WRITE          ((CANIO_FUNC_WRITE << 8)      | CANIO_ADDRESS)
#define CANIO_ID_READ_RELAY     ((CANIO_FUNC_READ_RELAY << 8) | CANIO_ADDRESS)
#define CANIO_ID_READ_INPUT     ((CANIO_FUNC_READ_INPUT << 8) | CANIO_ADDRESS)

void can_io_init(void);
HAL_StatusTypeDef can_io_write_relay(uint8_t relay_byte);
HAL_StatusTypeDef can_io_read_relay_status(void);
HAL_StatusTypeDef can_io_read_input_status(void);

extern volatile uint8_t can_io_relay_status;
extern volatile uint8_t can_io_input_status;

#endif
