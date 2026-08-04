#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float integral_limit;
    float output_limit;
    float separation_error;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit,
              float separation_error);
void PID_Reset(PID_Controller *pid);
float PID_Update(PID_Controller *pid, float error, float dt_seconds);

#endif
