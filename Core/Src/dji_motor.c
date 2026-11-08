#include "dji_motor.h"
#include "bsp_can.h"
#include <string.h>

static GM6020_t *motor_map[8];
static int16_t   cur_cmd[8];

void GM6020_Init(GM6020_t *motor, uint8_t id)
{
    memset(motor, 0, sizeof(GM6020_t));
    motor->id        = id;
    motor->last_tick = HAL_GetTick() - 1000U;
    motor->online    = 0U;

    if (id >= 1U && id <= 7U) {
        motor_map[id] = motor;
    }
}

void GM6020_Decode(GM6020_t *motor, const uint8_t data[8])
{
    motor->ecd         = (uint16_t)((data[0] << 8) | data[1]);
    motor->speed_rpm   = (int16_t)((data[2] << 8) | data[3]);
    motor->current     = (int16_t)((data[4] << 8) | data[5]);
    motor->temperature = data[6];

    motor->last_tick = HAL_GetTick();
    motor->online    = 1U;
}

void GM6020_SetCurrent(uint8_t id, int16_t cur)
{
    if (id < 1U || id > 7U) {
        return;
    }
    if (cur >  GM6020_CTRL_MAX) { cur =  GM6020_CTRL_MAX; }
    if (cur < -GM6020_CTRL_MAX) { cur = -GM6020_CTRL_MAX; }

    cur_cmd[id] = cur;
}

static uint8_t GM6020_GroupUsed(uint8_t first_id, uint8_t last_id)
{
    for (uint8_t id = first_id; id <= last_id; id++) {
        if (motor_map[id] != NULL) {
            return 1U;
        }
    }
    return 0U;
}

void GM6020_SendCommands(CAN_HandleTypeDef *hcan)
{
    uint8_t data[8] = {0};

    if (GM6020_GroupUsed(1U, 4U)) {
        for (uint8_t i = 0; i < 4U; i++) {
            data[2U * i]      = (uint8_t)(cur_cmd[i + 1U] >> 8);
            data[2U * i + 1U] = (uint8_t)(cur_cmd[i + 1U] & 0xFF);
        }
        (void)BSP_CAN_Send(hcan, GM6020_CTRL_ID_LOW, data, 8U);
    }

    if (GM6020_GroupUsed(5U, 7U)) {
        for (uint8_t i = 0; i < 3U; i++) {
            data[2U * i]      = (uint8_t)(cur_cmd[i + 5U] >> 8);
            data[2U * i + 1U] = (uint8_t)(cur_cmd[i + 5U] & 0xFF);
        }
        data[6] = 0U;
        data[7] = 0U;
        (void)BSP_CAN_Send(hcan, GM6020_CTRL_ID_HIGH, data, 8U);
    }
}

void GM6020_RxDispatcher(uint32_t std_id, const uint8_t data[8])
{
    if (std_id < GM6020_FB_ID_BASE || std_id > (GM6020_FB_ID_BASE + 7U)) {
        return;
    }

    GM6020_t *motor = motor_map[std_id - GM6020_FB_ID_BASE];
    if (motor != NULL) {
        GM6020_Decode(motor, data);
    }
}

void GM6020_UpdateOnline(GM6020_t *motor, uint32_t timeout_ms)
{
    uint32_t now = HAL_GetTick();
    motor->online = ((now - motor->last_tick) <= timeout_ms) ? 1U : 0U;
}

float GM6020_GetAngleDeg(const GM6020_t *motor)
{
    return (float)motor->ecd * 360.0f / (float)GM6020_ECD_MAX;
}