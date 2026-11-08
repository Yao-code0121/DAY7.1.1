#ifndef __PID_H__
#define __PID_H__

typedef struct {
    /* ---- 参数：调这三个 ---- */
    float kp;              /* 比例系数 */
    float ki;              /* 积分系数 */
    float kd;              /* 微分系数 */

    /* ---- 输入：每拍更新 ---- */
    float target;          /* 目标值（rpm） */
    float measure;         /* 实测值（rpm） */
    float dt;              /* 采样周期（秒） */

    /* ---- 内部状态：不要手动改 ---- */
    float err;             /* 本次误差 */
    float last_err;        /* 上次误差（算微分用） */
    float integral;        /* 积分累计 */
    float p_out;           /* P 项输出 */
    float i_out;           /* I 项输出 */
    float d_out;           /* D 项输出 */

    /* ---- 输出 ---- */
    float out;             /* 最终输出（已限幅） */
    float out_max;         /* 输出上限 */
    float out_min;         /* 输出下限 */
    float integral_max;    /* 积分限幅（抗饱和） */
} PID_t;

/* ---- 6 个函数声明：告诉别的文件"这些函数存在" ---- */
void  PID_Init(PID_t *pid, float kp, float ki, float kd,
               float dt, float out_max);
float PID_Calc(PID_t *pid);
void  PID_Reset(PID_t *pid);
void  PID_SetTarget(PID_t *pid, float target);
void  PID_SetTunings(PID_t *pid, float kp, float ki, float kd);

#endif /* __PID_H__ */