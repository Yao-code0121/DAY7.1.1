# 任务七 代码规范重构说明 //一级
## 改动概览
本次对代码做规范重构，使用枚举、结构体整理状态，消除魔法数字，抽取函数、统一命名并补充注释。
## 1. 枚举替换状态宏
- 改动位置：头文件
- 改动前：使用宏定义表示电机状态
- 改动后：typedef enum MotorState_t
## 2. 队列元素类型跟着同步
- 改动位置：MX_FREERTOS_Init 里三处 osMessageQueueNew
- 改动前：sizeof(uint16_t)
- 改动后：sizeof(MotorState_t)
## 3. 状态机替换 if/else，并抽出 Led_Breath_Step
- 改动LED任务
- 改动前：if else
- 改动后：单独封装 Led_Breath_Step 函数
- 原因：逻辑拆分，主循环更清爽，函数可以复用
## 4. 宏定义消除数字
- 改动LED相关参数
- 改动前：直接写死 1000、10 这类数字
- 改动后：用 LED_CCR_MAX、LED_STEP_CCR 等宏
## 5 命名统一
改动位置：全局变量、函数、类型名
## 6. Doxygen注释
