#include "can_io.h"
#include "fdcan.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan3;

volatile uint8_t can_io_relay_status = 0;
volatile uint8_t can_io_input_status = 0;

static FDCAN_TxHeaderTypeDef   TxHeader;
static FDCAN_RxHeaderTypeDef   RxHeader;
static uint8_t                 TxData[8];
static uint8_t                 RxData[8];

void can_io_init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CANIO_ADDRESS;
    sFilterConfig.FilterID2 = 0x0FF;

    HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

static HAL_StatusTypeDef send_can_message(uint32_t id)
{
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    memset(TxData, 0, 8);
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);
}

HAL_StatusTypeDef can_io_write_relay(uint8_t relay_byte)
{
    memset(TxData, 0, 8);
    TxData[0] = relay_byte;

    TxHeader.Identifier = CANIO_ID_WRITE;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);
}

HAL_StatusTypeDef can_io_read_relay_status(void)
{
    return send_can_message(CANIO_ID_READ_RELAY);
}

HAL_StatusTypeDef can_io_read_input_status(void)
{
    return send_can_message(CANIO_ID_READ_INPUT);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == FDCAN3) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            uint8_t func = (RxHeader.Identifier >> 8) & 0x07;

            if (func == CANIO_FUNC_WRITE || func == CANIO_FUNC_READ_RELAY) {
                can_io_relay_status = RxData[0];
            } else if (func == CANIO_FUNC_READ_INPUT) {
                can_io_input_status = RxData[0];
            }
        }
    }
}
