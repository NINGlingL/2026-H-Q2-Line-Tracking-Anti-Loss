#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * Hardware safety gate.
 *
 * Keep this at 0 until BOTH conditions are physically verified:
 *   1. PB14/STBY has an external 10 kOhm pull-down.
 *   2. PA17 is connected to the motor supply through a validated divider.
 *
 * The firmware still builds and all sensors/diagnostics run while locked,
 * but Moto_SetLR() cannot raise STBY.
 */
#define APP_MOTOR_HW_READY             (0U)

/* User-confirmed timing and actuator limits. */
#define APP_CONTROL_PERIOD_MS          (10U)
#define APP_I2C_TIMEOUT_MS             (10U)
#define APP_PWM_MAX_PERMILLE           (800)

/*
 * Battery divider proposal: supply -- 39k -- PA17 -- 10k -- GND.
 * The low threshold remains zero until the battery chemistry is confirmed.
 */
#define APP_BATTERY_DIV_TOP_OHM        (39000UL)
#define APP_BATTERY_DIV_BOTTOM_OHM     (10000UL)
#define APP_BATTERY_LOW_MV             (0UL)
#define APP_BATTERY_NOMINAL_MV         (12000UL)

/* Eight-channel infrared module requires a warm-up after every power-up. */
#define APP_IR_WARMUP_MS               (20000UL)
#define APP_IR_FRAME_TIMEOUT_MS        (150UL)

/* Conservative first bench-test values, tunable through Bluetooth. */
#define APP_DIAG_PWM_PERMILLE          (250)
#define APP_AUTO_BASE_PWM_PERMILLE     (300)
#define APP_ENCODER_TARGET_PER_TICK    (0)

/* x1-left is provisional; Bluetooth command "IRREV 1" reverses it. */
#define APP_IR_REVERSED_DEFAULT        (0U)

#endif
