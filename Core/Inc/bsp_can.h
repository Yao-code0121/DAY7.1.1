#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"

void BSP_CAN_Init(CAN_HandleTypeDef *hcan);

uint8_t BSP_CAN_Send(CAN_HandleTypeDef *hcan, uint32_t std_id,
                     const uint8_t *data, uint8_t len);

#endif /* BSP_CAN_H */