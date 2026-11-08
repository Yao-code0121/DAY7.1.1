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
#define MOTOR_OFFLINE_MS     100U

/* ================= PID ================= */
#define PID_DT               0.001f   /* 必须等于 TIM2 周期：1ms */
#define PID_OUT_MAX          15000.0f
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
#define KEY_DEBOUNCE_MS      50U      /* 两次按键最小间隔，防抖 */

/* ========= 电机状态：队列里传的就是这三个数 ========= */
typedef enum
{
  MOTOR_ST_OFFLINE = 0,   /* 离线 */
  MOTOR_ST_SPINUP  = 1,   /* 正在追目标速度 */
  MOTOR_ST_READY   = 2,   /* 已经稳定在目标附近 */
  MOTOR_ST_NONE    = 3    /* 还没收到过状态，只拿来当初始值 */
} MotorState_t;

/* ========= 呼吸灯的一组状态 ========= */
typedef struct
{
  int16_t  duty;      /* 当前亮度 0~1000 */
  int8_t   sgn;       /* +1 变亮，-1 变暗 */
  uint32_t last_ms;   /* 上一次变亮度的时刻 */
} LedBreath_t;
/* ================= LED ================= */
#define LED_PORT             GPIOH
#define LED_GREEN_PIN        GPIO_PIN_11   /* 绿灯：GPIO 常亮 */

#define LED_CH_RED           TIM_CHANNEL_3 /* 红灯：PH12 = TIM5_CH3 */
#define LED_CCR_MAX          1000U         /* PSC=83, ARR=999 -> 1000Hz，CCR 0~1000 = 0~100% */
#define LED_STEP_CCR         10U           /* 每级 1%，共 100 级 */
#define LED_STEP_MS          10U           /* 每 10ms 变一级：100 级 x 10ms = 1 秒走完全程 */

/* ========= 队列 / 串口 / 判定阈值 ========= */
#define QUEUE_LEN            16U      /* 队列里最多排 16 条消息 */
#define UART_TX_BUF_SIZE     96U      /* 串口发送缓冲长度 */
#define UART_PRINT_MS        2000U    /* 每隔 2 秒打印一次状态 */
#define UART_TX_TIMEOUT_MS   100U     /* 串口发送超时 */
#define READY_ERR_RPM        10.0f    /* 误差小于 10rpm 就算稳住了 */
#define SPEED_LIMIT_RPM      300.0f   /* 目标转速上限 */

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
uint16_t g_servo_ccr_us     = SERVO_MIN_US;   /* 当前脉宽，从 500 开始 */
int8_t   g_servo_dir           = 1;              /* +1 往右走，-1 往左走 */
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
  myQueue01Handle = osMessageQueueNew (QUEUE_LEN, sizeof(MotorState_t), &myQueue01_attributes);

  /* creation of myQueue02 */
  myQueue02Handle = osMessageQueueNew (QUEUE_LEN, sizeof(MotorState_t), &myQueue02_attributes);

  /* creation of myQueue03 */
  myQueue03Handle = osMessageQueueNew (QUEUE_LEN, sizeof(MotorState_t), &myQueue03_attributes);

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

/**
 * @brief  呼吸灯走一步：更新亮度并写进 CCR
 * @param  led 呼吸灯的状态（亮度、方向、上次时刻）
 * @retval None
 */
static void Led_Breath_Step(LedBreath_t *led)
{
  /* 把当前亮度写进比较寄存器，占空比立刻改变 */
  __HAL_TIM_SET_COMPARE(&htim5, LED_CH_RED, (uint32_t)led->duty);

  /* 每 10ms 才动一格 */
  if ((HAL_GetTick() - led->last_ms) >= LED_STEP_MS)
  {
    led->last_ms += LED_STEP_MS;
    led->duty = (int16_t)(led->duty + led->sgn * (int16_t)LED_STEP_CCR);

    if (led->duty >= (int16_t)LED_CCR_MAX) { led->duty = (int16_t)LED_CCR_MAX; led->sgn = -1; }
    if (led->duty <= 0)                    { led->duty = 0;                    led->sgn =  1; }
  }
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
  MotorState_t state = MOTOR_ST_OFFLINE;   /* 当前电机状态 */
  MotorState_t got;                        /* 从队列收到的状态 */
  LedBreath_t  led   = {0, 1, 0U};         /* 呼吸灯：亮度0，方向+1，时刻0 */

  /* 绿灯仍是普通 GPIO，先灭掉 */
  HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);

  /* 红灯 = TIM5_CH3 硬件 PWM：启动通道，占空比先清 0 */
  (void)HAL_TIM_PWM_Start(&htim5, LED_CH_RED);
  __HAL_TIM_SET_COMPARE(&htim5, LED_CH_RED, 0U);

  led.last_ms = HAL_GetTick();

  for(;;)
  {
    /* 1. 有状态就先收下 */
    if (osMessageQueueGet(myQueue01Handle, &got, NULL, 0) == osOK)
    {
      state = got;
    }

    /* 2. 按状态分头处理 */
    switch (state)
    {
      case MOTOR_ST_OFFLINE:            /* 离线：红灯呼吸，绿灯灭 */
        HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
        Led_Breath_Step(&led);
        break;

      case MOTOR_ST_SPINUP:             /* 正在追速度 */
      case MOTOR_ST_READY:              /* 已经稳住 */
      default:                          /* 红灯灭，绿灯常亮 */
        __HAL_TIM_SET_COMPARE(&htim5, LED_CH_RED, 0U);
        HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
        led.duty    = 0;
        led.sgn     = 1;
        led.last_ms = HAL_GetTick();
        break;
    }

    /* 3. 让出 CPU */
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
  static char tx[UART_TX_BUF_SIZE];
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

    if ((HAL_GetTick() - last_print) >= UART_PRINT_MS)
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
  MotorState_t last_state = MOTOR_ST_NONE;
  MotorState_t state;
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
      state = (err < READY_ERR_RPM) ? MOTOR_ST_READY : MOTOR_ST_SPINUP;
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
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, g_servo_ccr_us);

  for(;;)
  {
    key_now = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    if (((HAL_GetTick() - key_last_time) >= KEY_DEBOUNCE_MS) && (key_now == 0U))
    {
      key_last_time = HAL_GetTick();

      if (g_servo_dir == 1)
      {
        g_servo_ccr_us = (uint16_t)(g_servo_ccr_us + SERVO_STEP_US);
        if (g_servo_ccr_us >= SERVO_MAX_US)
        {
          g_servo_ccr_us = SERVO_MAX_US;
          g_servo_dir = -1;
        }
      }
      else
      {
        g_servo_ccr_us = (uint16_t)(g_servo_ccr_us - SERVO_STEP_US);
        if (g_servo_ccr_us <= SERVO_MIN_US)
        {
          g_servo_ccr_us = SERVO_MIN_US;
          g_servo_dir = 1;
        }
      }

      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, g_servo_ccr_us);
    }

    osDelay(10);
  }
  /* USER CODE END StartduojiTASK */
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief  解析串口收到的电机指令，格式 "motor+100\r\n"
 * @param  s    收到的字符串，必须以 '\0' 结尾
 * @param  len  字符串长度（不含 '\0'）
 * @param  out  解析出的目标转速写到这里，单位 rpm
 * @return 1 表示解析成功，0 表示格式不对
 * @note   会修改 *out，其他全局变量都不动；只在串口任务里调用，不涉及并发
 */
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
  if (v >  (int32_t)SPEED_LIMIT_RPM) { v =  (int32_t)SPEED_LIMIT_RPM; }
  if (v < -(int32_t)SPEED_LIMIT_RPM) { v = -(int32_t)SPEED_LIMIT_RPM; }

  *out = (float)v;
  return 1U;
}

/**
 * @brief  阻塞发送一段字符串到 USART6
 * @param  s  要发的数据
 * @param  n  字节数
 * @retval None
 * @note   阻塞发送，96 字节 @115200 约 8ms；只在串口任务里调用，不要在中断里调
 */
static void App_UartSend(const char *s, uint16_t n)
{
  (void)HAL_UART_Transmit(&huart6, (const uint8_t *)s, n, UART_TX_TIMEOUT_MS);
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

