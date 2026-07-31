#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * Hardware safety gate.
 *
 * Keep this at 0 until the WHEELTEC D153C ADC output is confirmed connected
 * directly to PA27 and the displayed voltage is checked against a meter.
 *
 * PB14 may connect directly to a genuine TB6612FNG STBY input because the
 * device datasheet specifies an internal 200 kOhm pull-down on STBY.
 *
 * The firmware still builds and all sensors/diagnostics run while locked,
 * but Moto_SetLR() cannot raise STBY.
 */
#define APP_MOTOR_HW_READY             (0U)

/*
 * Screen-only line-sensor monitor. Keep the IMU out of this test so a second
 * I2C device cannot interfere with OLED diagnosis.
 */
#define APP_ENABLE_OLED                (1U)
#define APP_ENABLE_IMU                 (0U)

/* User-confirmed timing and actuator limits. */
#define APP_CONTROL_PERIOD_MS          (10U)
#define APP_I2C_TIMEOUT_MS             (10U)
#define APP_PWM_MAX_PERMILLE           (800)

/*
 * WHEELTEC D153C onboard battery divider:
 *   V1.0: 10k / 1k, V1.1: 100k / 10k; both produce VIN / 11.
 * Connect the module ADC pin directly to PA27. Do not add another divider.
 * The low threshold remains zero until the pack series count is confirmed.
 */
#define APP_BATTERY_DIV_TOP_OHM        (100000UL)
#define APP_BATTERY_DIV_BOTTOM_OHM     (10000UL)
/*
 * Per-vehicle calibration from the connected D153C:
 * firmware 12.277 V versus meter 11.440 V on 2026-07-31.
 */
#define APP_BATTERY_CALIBRATION_PPM    (931824UL)
#define APP_BATTERY_LOW_MV             (9900UL)
#define APP_BATTERY_NOMINAL_MV         (11800UL)
#define APP_BATTERY_VALID_MIN_MV       (7000UL)
#define APP_BATTERY_VALID_MAX_MV       (13500UL)

/* Eight-channel infrared module requires a warm-up after every power-up. */
#define APP_IR_WARMUP_MS               (20000UL)
#define APP_IR_FRAME_TIMEOUT_MS        (150UL)

/* Conservative first bench-test values, tunable through Bluetooth. */
#define APP_DIAG_PWM_PERMILLE          (250)
#define APP_AUTO_BASE_PWM_PERMILLE     (300)

/*
 * User-confirmed MG513XP28_12V with Hall encoder and 65 mm wheel:
 *   13 PPR * 28:1 gearbox * 4x quadrature = 1456 counts/wheel revolution.
 * The first closed-loop target is deliberately limited to 0.25 m/s.
 */
#define APP_MOTOR_GEAR_RATIO           (28UL)
#define APP_ENCODER_PPR                (13UL)
#define APP_ENCODER_QUADRATURE         (4UL)
#define APP_ENCODER_COUNTS_PER_REV     \
    (APP_MOTOR_GEAR_RATIO * APP_ENCODER_PPR * APP_ENCODER_QUADRATURE)
#define APP_WHEEL_DIAMETER_MM          (65UL)
#define APP_WHEEL_CIRCUMFERENCE_UM     (204204UL)
#define APP_MOTOR_MAX_VALID_RPM        (500UL)
#define APP_AUTO_TARGET_SPEED_MM_S     (250)

/* x1-left is provisional; Bluetooth command "IRREV 1" reverses it. */
#define APP_IR_REVERSED_DEFAULT        (0U)

#endif
