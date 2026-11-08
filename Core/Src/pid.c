#include "pid.h"

static float PID_Clamp(float x, float lo, float hi)
{
    if (x > hi) {
        return hi;
    }
    if (x < lo) {
        return lo;
    }
    return x;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float dt, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->dt      = (dt > 0.0f) ? dt : 0.001f;
    pid->out_max = out_max;
    pid->out_min = -out_max;

    pid->integral_max = out_max * 0.6f;

    pid->target  = 0.0f;
    pid->measure = 0.0f;

    pid->err      = 0.0f;
    pid->last_err = 0.0f;
    pid->integral = 0.0f;

    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->out   = 0.0f;
}

float PID_Calc(PID_t *pid)
{
    pid->err = pid->target - pid->measure;

    pid->p_out = pid->kp * pid->err;

    pid->d_out = pid->kd * (pid->err - pid->last_err) / pid->dt;

    float integral_new = pid->integral + pid->ki * pid->err * pid->dt;

    integral_new = PID_Clamp(integral_new, -pid->integral_max, pid->integral_max);

    float out_preview = pid->p_out + integral_new + pid->d_out;

    if ((out_preview >= pid->out_max && pid->err > 0.0f) ||
        (out_preview <= pid->out_min && pid->err < 0.0f)) {
        integral_new = pid->integral;
        }

    pid->integral = integral_new;
    pid->i_out    = pid->integral;

    pid->out = PID_Clamp(pid->p_out + pid->i_out + pid->d_out,
                         pid->out_min, pid->out_max);

    pid->last_err = pid->err;

    return pid->out;
}

void PID_Reset(PID_t *pid)
{
    pid->err      = 0.0f;
    pid->last_err = 0.0f;
    pid->integral = 0.0f;
    pid->p_out    = 0.0f;
    pid->i_out    = 0.0f;
    pid->d_out    = 0.0f;
    pid->out      = 0.0f;
}

void PID_SetTarget(PID_t *pid, float target)
{
    pid->target = target;
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}