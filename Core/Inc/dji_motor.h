#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "main.h"

/* ---------------- GM6020 协议常量 ---------------- */

#define GM6020_FB_ID_BASE    0x204U

#define GM6020_CTRL_ID_LOW   0x1FFU
#define GM6020_CTRL_ID_HIGH  0x2FFU

#define GM6020_CTRL_MAX      25000
#define GM6020_ECD_MAX       8192

typedef struct {
    uint8_t  id;
    uint16_t ecd;
    int16_t  speed_rpm;
    int16_t  current;
    uint8_t  temperature;
    uint32_t last_tick;
    uint8_t  online;
} GM6020_t;

void  GM6020_Init(GM6020_t *motor, uint8_t id);
void  GM6020_Decode(GM6020_t *motor, const uint8_t data[8]);
void  GM6020_SetCurrent(uint8_t id, int16_t cur);
void  GM6020_SendCommands(CAN_HandleTypeDef *hcan);
void  GM6020_RxDispatcher(uint32_t std_id, const uint8_t data[8]);
void  GM6020_UpdateOnline(GM6020_t *motor, uint32_t timeout_ms);
float GM6020_GetAngleDeg(const GM6020_t *motor);

#endif /* DJI_MOTOR_H */