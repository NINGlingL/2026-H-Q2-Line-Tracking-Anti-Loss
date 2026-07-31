#include "pid.h"
#include <math.h>

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit,
              float separation_error)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->separation_error = separation_error;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

float PID_Update(PID_Controller *pid, float error, float dt_seconds)
{
    float derivative;
    float output;

    if (!isfinite(error) || !isfinite(dt_seconds) ||
        dt_seconds <= 0.0f) {
        PID_Reset(pid);
        return 0.0f;
    }

    if (fabsf(error) <= pid->separation_error) {
        pid->integral += error * dt_seconds;
        if (pid->integral > pid->integral_limit) {
            pid->integral = pid->integral_limit;
        }
        if (pid->integral < -pid->integral_limit) {
            pid->integral = -pid->integral_limit;
        }
    }

    derivative = (error - pid->previous_error) / dt_seconds;
    pid->previous_error = error;
    output = pid->kp * error + pid->ki * pid->integral +
             pid->kd * derivative;

    if (!isfinite(output)) {
        PID_Reset(pid);
        return 0.0f;
    }
    if (output > pid->output_limit) {
        output = pid->output_limit;
    }
    if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    return output;
}
