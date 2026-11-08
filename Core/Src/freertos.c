/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "tim.h"          /* htim1 / htim2 / htim5 */
#include "usart.h"        /* huart6 */
#include "can.h"          /* hcan1 */
#include "pid.h"          /* PID_t 与 PID_* 函数 */
#include "dji_motor.h"    /* GM6020_t 与 GM6020_* 函数 */
#include "bsp_can.h"      /* BSP_CAN_Init */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ================= 电机 ================= */
#define MOTOR_ID             1U
#define MOTOR_OFFLINE_MS     100U     /* 超过 100ms 没收到反馈 -> 判离线 */

/* ================= PID ================= */
#define PID_DT               0.001f   /* 必须等于 TIM2 周期：1ms */
#define PID_OUT_MAX          15000.0f /* 电压单位，满量程 25000 */
#define PID_KP_DEFAULT       75.0f
#define PID_KI_DEFAULT       100.0f
#define PID_KD_DEFAULT       0.02f
#define TARGET_SPEED_DEFAULT 0.0f

/* ================= 串口 ================= */
#define UART_RX_BUF_SIZE     64U      /* 一行指令最长 64 字节 */

/* ================= 舵机 ================= */
#define SERVO_MIN_US         500U     /* 0.5ms 一端极限 */
#define SERVO_MAX_US         2500U    /* 2.5ms 另一端极限 */
#define SERVO_STEP_US        223U     /* 每按一次 223us，约 20 度 */
#define Lastingtime          50U      /* 两次按键最小间隔，防抖 */

/* ========= 电机状态：队列里传的就是这三个数 ========= */
#define MOTOR_ST_OFFLINE     0U       /* 离线 */
#define MOTOR_ST_SPINUP      1U       /* 正在追目标速度 */
#define MOTOR_ST_READY       2U       /* 已经稳定在目标附近 */

/* ================= LED ================= */
/* C 板三颗灯：PH10 蓝 / PH11 绿 / PH12 红。
   红灯改成了 TIM5_CH3 的硬件 PWM，亮度由 CCR3 决定，不再写 GPIO；
   绿灯保持原样，还是普通推挽输出，写 1 亮、写 0 灭。 */
#define LED_PORT             GPIOH
#define LED_GREEN_PIN        GPIO_PIN_11   /* 绿灯：GPIO 常亮 */

#define LED_CH_RED           TIM_CHANNEL_3 /* 红灯：PH12 = TIM5_CH3 */
#define LED_CCR_MAX          1000U         /* PSC=83, ARR=999 -> 1000Hz，CCR 0~1000 = 0~100% */
#define LED_STEP_CCR         10U           /* 每级 1%，共 100 级 */
#define LED_STEP_MS          10U           /* 每 10ms 变一级：100 级 x 10ms = 1 秒走完全程 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* ---- 从 main.c 借来的信号量：这里只是声明，定义在 main.c ---- */
extern osSemaphoreId_t ctrl_sem;

/* ---- 电机与 PID 对象 ---- */
GM6020_t motor1;
PID_t    speed_pid;

/* ---- 观测变量：CubeMonitor 看这几个 ---- */
volatile uint32_t g_rx_count     = 0;    /* 收到多少帧 */
volatile uint32_t g_rx_id        = 0;    /* 最近一帧的 ID，应为 0x205 */
volatile float    g_actual_speed = 0.0f; /* 实测转速 rpm */
volatile float    g_pid_out      = 0.0f; /* PID 输出（电压指令） */
volatile float    g_target_speed = TARGET_SPEED_DEFAULT;

/* ---- 串口 DMA 接收缓冲 ---- */
uint8_t          g_uart_rx_buf[UART_RX_BUF_SIZE];
volatile uint8_t g_uart_rx_len  = 0;
volatile uint8_t g_uart_rx_flag = 0;

/* ---- 舵机当前脉宽 ---- */
uint16_t servo_ccr     = SERVO_MIN_US;   /* 当前脉宽，从 500 开始 */
int8_t   dir           = 1;              /* +1 往右走，-1 往左走 */
uint32_t key_last_time = 0;              /* 上次按键时刻，必须是 uint32_t */


/* USER CODE END Variables */
/* Definitions for LEDTASK */
osThreadId_t LEDTASKHandle;
const osThreadAttr_t LEDTASK_attributes = {
  .name = "LEDTASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usartTASK */
osThreadId_t usartTASKHandle;
const osThreadAttr_t usartTASK_attributes = {
  .name = "usartTASK",
  .stack_size = 258 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for dianji6020TASK */
osThreadId_t dianji6020TASKHandle;
const osThreadAttr_t dianji6020TASK_attributes = {
  .name = "dianji6020TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for duojiTASK */
osThreadId_t duojiTASKHandle;
const osThreadAttr_t duojiTASK_attributes = {
  .name = "duojiTASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myQueue02 */
osMessageQueueId_t myQueue02Handle;
const osMessageQueueAttr_t myQueue02_attributes = {
  .name = "myQueue02"
};
/* Definitions for myQueue03 */
osMessageQueueId_t myQueue03Handle;
const osMessageQueueAttr_t myQueue03_attributes = {
  .name = "myQueue03"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static uint8_t App_ParseMotorCmd(const char *s, uint8_t len, float *out);
static void    App_UartSend(const char *s, uint16_t n);
/* USER CODE END FunctionPrototypes */

void StartLEDTASK(void *argument);
void StartusartTASK(void *argument);
void Startdianji6020TASK(void *argument);
void StartduojiTASK(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* ctrl_sem 定义在 main.c，这里创建它（真正分配内存） */
  ctrl_sem = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

  /* creation of myQueue02 */
  myQueue02Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue02_attributes);

  /* creation of myQueue03 */
  myQueue03Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue03_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LEDTASK */
  LEDTASKHandle = osThreadNew(StartLEDTASK, NULL, &LEDTASK_attributes);

  /* creation of usartTASK */
  usartTASKHandle = osThreadNew(StartusartTASK, NULL, &usartTASK_attributes);

  /* creation of dianji6020TASK */
  dianji6020TASKHandle = osThreadNew(Startdianji6020TASK, NULL, &dianji6020TASK_attributes);

  /* creation of duojiTASK */
  duojiTASKHandle = osThreadNew(StartduojiTASK, NULL, &duojiTASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLEDTASK */
/**
  * @brief  Function implementing the LEDTASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLEDTASK */
void StartLEDTASK(void *argument)
{
  /* USER CODE BEGIN StartLEDTASK */
  uint16_t state  = MOTOR_ST_OFFLINE;
  uint16_t got;
  int16_t  duty   = 0;      /* 0~1000。必须是 int16_t，int8_t 装不下 1000 */
  int8_t   sgn    = 1;      /* +1 变亮，-1 变暗 */
  uint32_t t_lvl  = 0U;

  /* 绿灯仍是普通 GPIO，先灭掉 */
  HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);

  /* 红灯改成 TIM5_CH3 硬件 PWM：启动通道 + 先把占空比清 0 */
  (void)HAL_TIM_PWM_Start(&htim5, LED_CH_RED);
  __HAL_TIM_SET_COMPARE(&htim5, LED_CH_RED, 0U);

  t_lvl = HAL_GetTick();

  for(;;)
  {
    if (osMessageQueueGet(myQueue01Handle, &got, NULL, 0) == osOK)
    {
      state = got;
    }

    if (state == MOTOR_ST_OFFLINE)
    {
      HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);

      __HAL_TIM_SET_COMPARE(&htim5, LED_CH_RED, (uint32_t)duty);

      if ((HAL_GetTick() - t_lvl) >= LED_STEP_MS)
      {
        t_lvl += LED_STEP_MS;
        duty = (int16_t)(duty + sgn * (int16_t)LED_STEP_CCR);
        if (duty >= (int16_t)LED_CCR_MAX) { duty = (int16_t)LED_CCR_MAX; sgn = -1; }
        if (duty <= 0)                    { duty = 0;                    sgn =  1; }
      }
    }
    else
    {
      __HAL_TIM_SET_COMPARE(&htim5, LED_CH_RED, 0U);
      HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
      duty  = 0;
      sgn   = 1;
      t_lvl = HAL_GetTick();
    }

    osDelay(1);
  }
  /* USER CODE END StartLEDTASK */
}

/* USER CODE BEGIN Header_StartusartTASK */
/**
* @brief Function implementing the usartTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartusartTASK */
void StartusartTASK(void *argument)
{
  /* USER CODE BEGIN StartusartTASK */
  static char tx[96];
  uint8_t     local[UART_RX_BUF_SIZE + 1U];  /* +1 是留给结尾 '\0'，否则 n=64 时 local[64] 会越界 */
  uint8_t     n;
  uint8_t     i;
  float       v;
  int         len;
  uint32_t    last_print = 0;

  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, g_uart_rx_buf, sizeof(g_uart_rx_buf));

  for(;;)
  {
    if (g_uart_rx_flag == 1U)
    {
      n = g_uart_rx_len;
      g_uart_rx_flag = 0U;

      if (n > UART_RX_BUF_SIZE) { n = UART_RX_BUF_SIZE; }
      for (i = 0U; i < n; i++) { local[i] = g_uart_rx_buf[i]; }
      local[n] = 0U;

      if (App_ParseMotorCmd((const char *)local, n, &v) == 1U)
      {
        g_target_speed = v;
        PID_SetTarget(&speed_pid, v);
        len = snprintf(tx, sizeof(tx), "target=%d\r\n", (int)v);
        if (len > 0) { App_UartSend(tx, (uint16_t)len); }
      }
      else
      {
        len = snprintf(tx, sizeof(tx), "bad cmd\r\n");
        if (len > 0) { App_UartSend(tx, (uint16_t)len); }
      }
    }

    if ((HAL_GetTick() - last_print) >= 2000U)
    {
      last_print = HAL_GetTick();
      len = snprintf(tx, sizeof(tx), "tgt=%d act=%d out=%d\r\n",
                     (int)g_target_speed, (int)g_actual_speed, (int)g_pid_out);
      if (len > 0) { App_UartSend(tx, (uint16_t)len); }
    }

    osDelay(10);
  }
  /* USER CODE END StartusartTASK */
}

/* USER CODE BEGIN Header_Startdianji6020TASK */
/**
* @brief Function implementing the dianji6020TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startdianji6020TASK */
void Startdianji6020TASK(void *argument)
{
  /* USER CODE BEGIN Startdianji6020TASK */
  uint16_t last_state = 0xFFFFU;
  uint16_t state;
  float    err;
  BSP_CAN_Init(&hcan1);
  GM6020_Init(&motor1, MOTOR_ID);
  PID_Init(&speed_pid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
           PID_DT, PID_OUT_MAX);
  PID_SetTarget(&speed_pid, TARGET_SPEED_DEFAULT);
  HAL_TIM_Base_Start_IT(&htim2);
  /* Infinite loop */
  for(;;)
  {
    if (osSemaphoreAcquire(ctrl_sem, osWaitForever) != osOK)
    {
      continue;
    }

    GM6020_UpdateOnline(&motor1, MOTOR_OFFLINE_MS);
    if (motor1.online == 0U)
    {
      PID_Reset(&speed_pid);
      GM6020_SetCurrent(MOTOR_ID, 0);
      g_actual_speed = 0.0f;
      g_pid_out      = 0.0f;
    }
    else
    {
      speed_pid.measure = (float)motor1.speed_rpm;
      (void)PID_Calc(&speed_pid);
      GM6020_SetCurrent(MOTOR_ID, (int16_t)speed_pid.out);
      g_actual_speed = speed_pid.measure;
      g_pid_out      = speed_pid.out;
    }
    g_target_speed = speed_pid.target;

    /* 每 1ms 必须发一帧，不发大疆电机会在 100ms 后自己停 */
    GM6020_SendCommands(&hcan1);

    /* ---- 判状态：只在变化时才丢进队列 ---- */
    if (motor1.online == 0U)
    {
      state = MOTOR_ST_OFFLINE;
    }
    else
    {
      err = g_actual_speed - speed_pid.target;
      if (err < 0.0f) { err = -err; }
      state = (err < 10.0f) ? MOTOR_ST_READY : MOTOR_ST_SPINUP;
    }

    if (state != last_state)
    {
      (void)osMessageQueuePut(myQueue01Handle, &state, 0, 0);
      last_state = state;
    }
  }
  /* USER CODE END Startdianji6020TASK */
}

/* USER CODE BEGIN Header_StartduojiTASK */
/**
* @brief Function implementing the duojiTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartduojiTASK */
void StartduojiTASK(void *argument)
{
  /* USER CODE BEGIN StartduojiTASK */
  /* Infinite loop */
  uint8_t key_now  = 1U;   /* 本次读到的电平：1=松开，0=按下 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, servo_ccr);

  for(;;)
  {
    key_now = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    if ((HAL_GetTick() - key_last_time) >= Lastingtime && (key_now == 0U))
    {

        key_last_time = HAL_GetTick();

        if (dir == 1)
        {
          servo_ccr = (uint16_t)(servo_ccr + SERVO_STEP_US);
          if (servo_ccr >= SERVO_MAX_US)
          {
            servo_ccr = SERVO_MAX_US;
            dir = -1;
          }
        }
        else
        {
          servo_ccr = (uint16_t)(servo_ccr - SERVO_STEP_US);
          if (servo_ccr <= SERVO_MIN_US)
          {
            servo_ccr = SERVO_MIN_US;
            dir = 1;
          }
        }

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, servo_ccr);
      }
    }

    key_now=1;
    osDelay(10);
  }
  /* USER CODE END StartduojiTASK */


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static uint8_t App_ParseMotorCmd(const char *s, uint8_t len, float *out)
{
  int32_t v = 0;
  int8_t  sign = 1;
  uint8_t i = 5U;

  if (len < 7U) { return 0U; }
  if (strncmp(s, "motor", 5U) != 0) { return 0U; }

  if (s[i] == '+')      { sign =  1; i++; }
  else if (s[i] == '-') { sign = -1; i++; }
  else                  { return 0U; }

  if (i >= len) { return 0U; }

  while ((i < len) && (s[i] >= '0') && (s[i] <= '9'))
  {
    v = v * 10 + (int32_t)(s[i] - '0');
    i++;
  }

  if ((i < len) && (s[i] != '\r')) { return 0U; }

  v *= sign;
  if (v >  300) { v =  300; }
  if (v < -300) { v = -300; }

  *out = (float)v;
  return 1U;
}

static void App_UartSend(const char *s, uint16_t n)
{
  (void)HAL_UART_Transmit(&huart6, (const uint8_t *)s, n, 100U);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (hcan->Instance != CAN1) { return; }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      break;
    }

    g_rx_count++;
    g_rx_id = rx_header.StdId;

    GM6020_RxDispatcher(rx_header.StdId, rx_data);
  }
}

void HAL_CAN_RxFifo0OverrunCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (hcan->Instance != CAN1) { return; }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      break;
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART6) { return; }

  if (Size > UART_RX_BUF_SIZE) { Size = UART_RX_BUF_SIZE; }

  g_uart_rx_len  = (uint8_t)Size;
  g_uart_rx_flag = 1U;

  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, g_uart_rx_buf, sizeof(g_uart_rx_buf));
}

/* USER CODE END Application */

