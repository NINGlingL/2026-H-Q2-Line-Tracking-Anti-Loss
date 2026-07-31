#include "icm20948.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"
#include <math.h>
#include <string.h>

#define ICM20948_ID             (0xEAU)
#define REG_WHO_AM_I            (0x00U)
#define REG_PWR_MGMT_1          (0x06U)
#define REG_PWR_MGMT_2          (0x07U)
#define REG_ACCEL_XOUT_H        (0x2DU)
#define REG_BANK_SEL            (0x7FU)
#define REG_GYRO_SMPLRT_DIV_B2  (0x00U)
#define REG_GYRO_CONFIG_1_B2    (0x01U)
#define REG_ODR_ALIGN_EN_B2     (0x09U)
#define REG_ACCEL_SMPLRT_DIV_1_B2 (0x10U)
#define REG_ACCEL_SMPLRT_DIV_2_B2 (0x11U)
#define REG_ACCEL_CONFIG_B2     (0x14U)
#define BANK_0                  (0x00U)
#define BANK_2                  (0x20U)
#define GYRO_RANGE_DPS          (250.0f)
#define GYRO_CAL_SAMPLES        (100U)
#define GYRO_CAL_MAX_DPS        (10.0f)
#define ACCEL_CAL_MIN_MS2       (7.0f)
#define ACCEL_CAL_MAX_MS2       (12.5f)
#define I2C_TIMEOUT_LOOPS       \
    ((CPUCLK_FREQ / 1000U) * APP_I2C_TIMEOUT_MS)

static IMU_Data g_last;
static uint8_t g_address = 0x68U;
static float g_cal_sum_x;
static float g_cal_sum_y;
static float g_cal_sum_z;

static uint8_t wait_idle(void)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;
    while ((DL_I2C_getControllerStatus(IMU20948_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (timeout == 0U) {
            return 0U;
        }
        timeout--;
    }
    return 1U;
}

static void recover_bus(void)
{
    uint8_t pulse;

    g_last.stale = 1U;
    DL_I2C_resetControllerTransfer(IMU20948_INST);
    DL_I2C_flushControllerTXFIFO(IMU20948_INST);
    DL_I2C_flushControllerRXFIFO(IMU20948_INST);
    DL_I2C_disableController(IMU20948_INST);
    DL_I2C_disablePower(IMU20948_INST);
    delay_cycles((CPUCLK_FREQ / 1000U) * APP_I2C_TIMEOUT_MS);
    DL_I2C_enablePower(IMU20948_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    SYSCFG_DL_IMU20948_init();

    DL_GPIO_initDigitalOutput(GPIO_IMU20948_IOMUX_SCL);
    DL_GPIO_enableOutput(
        GPIO_IMU20948_SCL_PORT, GPIO_IMU20948_SCL_PIN);
    DL_GPIO_setPins(GPIO_IMU20948_SCL_PORT, GPIO_IMU20948_SCL_PIN);
    for (pulse = 0U; pulse < 9U; pulse++) {
        DL_GPIO_clearPins(
            GPIO_IMU20948_SCL_PORT, GPIO_IMU20948_SCL_PIN);
        delay_cycles(CPUCLK_FREQ / 200000U);
        DL_GPIO_setPins(
            GPIO_IMU20948_SCL_PORT, GPIO_IMU20948_SCL_PIN);
        delay_cycles(CPUCLK_FREQ / 200000U);
    }

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_IMU20948_IOMUX_SDA,
        GPIO_IMU20948_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_IMU20948_IOMUX_SCL,
        GPIO_IMU20948_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_IMU20948_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_IMU20948_IOMUX_SCL);
    SYSCFG_DL_IMU20948_init();
}

static uint8_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = {reg, value};

    if (wait_idle() == 0U) {
        recover_bus();
        return 0U;
    }
    DL_I2C_fillControllerTXFIFO(IMU20948_INST, bytes, 2U);
    DL_I2C_startControllerTransfer(IMU20948_INST, g_address,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    if (wait_idle() == 0U) {
        recover_bus();
        return 0U;
    }
    DL_I2C_flushControllerTXFIFO(IMU20948_INST);
    return 1U;
}

static uint8_t read_registers(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t i;
    uint32_t timeout;

    if (wait_idle() == 0U) {
        recover_bus();
        return 0U;
    }

    DL_I2C_fillControllerTXFIFO(IMU20948_INST, &reg, 1U);
    DL_I2C_startControllerTransfer(IMU20948_INST, g_address,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    if (wait_idle() == 0U) {
        recover_bus();
        return 0U;
    }
    DL_I2C_flushControllerTXFIFO(IMU20948_INST);

    DL_I2C_startControllerTransfer(IMU20948_INST, g_address,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);
    for (i = 0U; i < length; i++) {
        timeout = I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(IMU20948_INST)) {
            if (timeout == 0U) {
                recover_bus();
                return 0U;
            }
            timeout--;
        }
        data[i] = DL_I2C_receiveControllerData(IMU20948_INST);
    }
    if (wait_idle() == 0U) {
        recover_bus();
        return 0U;
    }
    DL_I2C_flushControllerRXFIFO(IMU20948_INST);
    return 1U;
}

static uint8_t select_bank(uint8_t bank)
{
    return write_register(REG_BANK_SEL, bank);
}

void ICM20948_StartCalibration(void)
{
    g_cal_sum_x = 0.0f;
    g_cal_sum_y = 0.0f;
    g_cal_sum_z = 0.0f;
    g_last.gyro_bias_x = 0.0f;
    g_last.gyro_bias_y = 0.0f;
    g_last.gyro_bias_z = 0.0f;
    g_last.calibration_samples = 0U;
    g_last.calibrated = 0U;
}

uint8_t ICM20948_Init(void)
{
    static const uint8_t addresses[2] = {0x68U, 0x69U};
    uint8_t address_index;
    uint8_t attempt;
    uint8_t who = 0U;

    memset(&g_last, 0, sizeof(g_last));

    for (address_index = 0U; address_index < 2U; address_index++) {
        g_address = addresses[address_index];
        for (attempt = 0U; attempt < 3U; attempt++) {
            if (select_bank(BANK_0) != 0U &&
                read_registers(REG_WHO_AM_I, &who, 1U) != 0U &&
                who == ICM20948_ID) {
                break;
            }
            recover_bus();
        }
        if (who == ICM20948_ID) {
            break;
        }
    }

    g_last.who_am_i = who;
    g_last.i2c_address = g_address;
    if (who != ICM20948_ID) {
        g_last.valid = 0U;
        g_last.stale = 1U;
        return 0U;
    }

    if (write_register(REG_PWR_MGMT_1, 0x80U) == 0U) {
        return 0U;
    }
    delay_cycles(CPUCLK_FREQ / 10U);
    if (select_bank(BANK_0) == 0U ||
        read_registers(REG_WHO_AM_I, &who, 1U) == 0U ||
        who != ICM20948_ID ||
        write_register(REG_PWR_MGMT_1, 0x01U) == 0U ||
        write_register(REG_PWR_MGMT_2, 0x00U) == 0U) {
        g_last.valid = 0U;
        g_last.stale = 1U;
        return 0U;
    }
    delay_cycles(CPUCLK_FREQ / 100U);

    if (select_bank(BANK_2) == 0U ||
        write_register(REG_GYRO_SMPLRT_DIV_B2, 10U) == 0U ||
        write_register(REG_GYRO_CONFIG_1_B2, 0x19U) == 0U ||
        write_register(REG_ODR_ALIGN_EN_B2, 0x01U) == 0U ||
        write_register(REG_ACCEL_SMPLRT_DIV_1_B2, 0x00U) == 0U ||
        write_register(REG_ACCEL_SMPLRT_DIV_2_B2, 10U) == 0U ||
        write_register(REG_ACCEL_CONFIG_B2, 0x1BU) == 0U ||
        select_bank(BANK_0) == 0U) {
        g_last.valid = 0U;
        g_last.stale = 1U;
        return 0U;
    }

    ICM20948_StartCalibration();
    g_last.valid = 1U;
    g_last.stale = 0U;
    return 1U;
}

uint8_t ICM20948_Read(IMU_Data *data, uint32_t now_ms)
{
    uint8_t bytes[14];
    int16_t raw[7];
    IMU_Data next = g_last;
    float raw_gx;
    float raw_gy;
    float raw_gz;
    float accel_magnitude_sq;
    uint8_t i;

    if (data == NULL) {
        return 0U;
    }
    if (read_registers(REG_ACCEL_XOUT_H, bytes, sizeof(bytes)) == 0U) {
        g_last.failures++;
        g_last.stale = 1U;
        *data = g_last;
        return 0U;
    }

    for (i = 0U; i < 7U; i++) {
        raw[i] = (int16_t) (((uint16_t) bytes[i * 2U] << 8) |
                            bytes[i * 2U + 1U]);
    }

    next.ax = (float) raw[0] * 4.0f * 9.80665f / 32768.0f;
    next.ay = (float) raw[1] * 4.0f * 9.80665f / 32768.0f;
    next.az = (float) raw[2] * 4.0f * 9.80665f / 32768.0f;
    raw_gx = (float) raw[3] * GYRO_RANGE_DPS / 32768.0f;
    raw_gy = (float) raw[4] * GYRO_RANGE_DPS / 32768.0f;
    raw_gz = (float) raw[5] * GYRO_RANGE_DPS / 32768.0f;
    next.gx = raw_gx - next.gyro_bias_x;
    next.gy = raw_gy - next.gyro_bias_y;
    next.gz = raw_gz - next.gyro_bias_z;
    next.temperature_c = ((float) raw[6] / 333.87f) + 21.0f;

    if (!isfinite(next.ax) || !isfinite(next.ay) || !isfinite(next.az) ||
        !isfinite(next.gx) || !isfinite(next.gy) || !isfinite(next.gz) ||
        fabsf(next.ax) > 45.0f || fabsf(next.ay) > 45.0f ||
        fabsf(next.az) > 45.0f || fabsf(raw_gx) > 275.0f ||
        fabsf(raw_gy) > 275.0f || fabsf(raw_gz) > 275.0f) {
        g_last.failures++;
        g_last.stale = 1U;
        *data = g_last;
        return 0U;
    }

    if (g_last.valid != 0U && g_last.stale == 0U &&
        g_last.calibrated != 0U &&
        (fabsf(next.gz - g_last.gz) > 150.0f)) {
        g_last.failures++;
        g_last.stale = 1U;
        *data = g_last;
        return 0U;
    }

    if (next.calibrated == 0U) {
        accel_magnitude_sq =
            next.ax * next.ax + next.ay * next.ay + next.az * next.az;
        if (fabsf(raw_gx) <= GYRO_CAL_MAX_DPS &&
            fabsf(raw_gy) <= GYRO_CAL_MAX_DPS &&
            fabsf(raw_gz) <= GYRO_CAL_MAX_DPS &&
            accel_magnitude_sq >=
                ACCEL_CAL_MIN_MS2 * ACCEL_CAL_MIN_MS2 &&
            accel_magnitude_sq <=
                ACCEL_CAL_MAX_MS2 * ACCEL_CAL_MAX_MS2) {
            g_cal_sum_x += raw_gx;
            g_cal_sum_y += raw_gy;
            g_cal_sum_z += raw_gz;
            next.calibration_samples++;
            if (next.calibration_samples >= GYRO_CAL_SAMPLES) {
                next.gyro_bias_x =
                    g_cal_sum_x / (float) GYRO_CAL_SAMPLES;
                next.gyro_bias_y =
                    g_cal_sum_y / (float) GYRO_CAL_SAMPLES;
                next.gyro_bias_z =
                    g_cal_sum_z / (float) GYRO_CAL_SAMPLES;
                next.gx = raw_gx - next.gyro_bias_x;
                next.gy = raw_gy - next.gyro_bias_y;
                next.gz = raw_gz - next.gyro_bias_z;
                next.calibrated = 1U;
            }
        } else {
            g_cal_sum_x = 0.0f;
            g_cal_sum_y = 0.0f;
            g_cal_sum_z = 0.0f;
            next.calibration_samples = 0U;
        }
    }

    next.valid = 1U;
    next.stale = 0U;
    next.timestamp_ms = now_ms;
    g_last = next;
    *data = g_last;
    return 1U;
}

IMU_Data ICM20948_GetLast(void)
{
    return g_last;
}
