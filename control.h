#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

typedef enum {
    CONTROL_SAFE = 0,
    CONTROL_DIAGNOSTIC,
    CONTROL_AUTO_TRACK,
    CONTROL_FAULT
} Control_Mode;

typedef struct {
    Control_Mode mode;
    int16_t left_command;
    int16_t right_command;
    int16_t speed_target_mm_s;
    uint8_t lap_count;
    uint8_t hardware_locked;
    uint32_t mode_enter_ms;
} Control_State;

typedef struct {
    int16_t pwm_limit_permille;
    int16_t start_pwm_permille;
    uint32_t start_ramp_ms;
    int16_t curve_pwm_permille;
    int16_t straight_pwm_permille;
    int16_t diagnostic_pwm_permille;
    int16_t speed_target_mm_s;
    float line_kp;
    float line_ki;
    float line_kd;
    float line_filter_alpha;
    float line_center_deadband;
    float yaw_kp;
    float yaw_ki;
    float yaw_rate_per_position;
    float straight_yaw_rate_dps;
    uint8_t marker_active_count;
    uint32_t marker_clear_ms;
    uint32_t marker_min_lap_ms;
    uint32_t line_lost_stop_ms;
    uint32_t auto_timeout_ms;
} Control_Tuning;

extern const Control_Tuning g_control_tuning;

void Control_Init(uint32_t now_ms);
void Control_Service(uint32_t now_ms);
Control_State Control_GetState(void);
void Control_RequestSafe(void);

#endif
