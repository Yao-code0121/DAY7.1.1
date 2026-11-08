#include "bsp_can.h"

/* ============================================================================
 * BSP = Board Support Package（板级支持包）
 *
 * 这一层只负责一件事：把 8 个字节丢到 CAN 总线上，或者从总线上收下来。
 * 它完全不知道"这 8 个字节是电机的转速指令还是别的东西"。
 *
 * 协议怎么解释、填第几个字节，全部放到 dji_motor.c。
 * 好处：以后换电机型号（比如换成达妙、宇树），只改 dji_motor.c，
 *       这一层一个字都不用动。
 * ==========================================================================*/

void BSP_CAN_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_Start(hcan) != HAL_OK) {
        Error_Handler();
    }

    HAL_CAN_ActivateNotification(hcan,
                                 CAN_IT_RX_FIFO0_MSG_PENDING |
                                 CAN_IT_RX_FIFO0_OVERRUN     |
                                 CAN_IT_BUSOFF);
}

uint8_t BSP_CAN_Send(CAN_HandleTypeDef *hcan, uint32_t std_id,
                     const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) {
        return 0U;
    }

    tx.StdId              = std_id;
    tx.IDE                = CAN_ID_STD;
    tx.RTR                = CAN_RTR_DATA;
    tx.DLC                = len;
    tx.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(hcan, &tx, (uint8_t *)data, &mailbox) == HAL_OK)
           ? 1U : 0U;
}