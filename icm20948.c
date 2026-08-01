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
#define GYRO_CAL_SAMPLES        ICM20948_CALIBRATION_SAMPLES
#define CAL_STILL_GYRO_DPS      (12.0f)
#define CAL_STILL_ACCEL_ERR     (1.2f)
#define YAW_LPF_TAU_S           (0.030f)
#define YAW_ZERO_RATE_DPS       (0.35f)
#define STATIONARY_GYRO_DPS     (1.5f)
#define STATIONARY_ACCEL_ERR    (0.8f)
#define STATIONARY_SAMPLES      (10U)
#define BIAS_TRACK_RATE         (0.002f)
#define IMU_NOMINAL_PERIOD_MS   (20U)
#define I2C_TIMEOUT_LOOPS       \
    ((CPUCLK_FREQ / 1000U) * APP_I2C_TIMEOUT_MS)
#define I2C_HALF_PERIOD         (CPUCLK_FREQ / 200000U)

static IMU_Data g_last;
static uint8_t g_address = 0x68U;
static float g_cal_sum_x;
static float g_cal_sum_y;
static float g_cal_sum_z;
static float g_yaw_rate_filtered;
static float g_yaw_rate_previous;
static uint8_t g_stationary_samples;

static void i2c_delay(void)
{
    delay_cycles(I2C_HALF_PERIOD);
}

static void sda_low(void)
{
    DL_GPIO_clearPins(GPIO_IMU20948_SDA_PORT, GPIO_IMU20948_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_IMU20948_SDA_PORT, GPIO_IMU20948_SDA_PIN);
}

static void sda_release(void)
{
    DL_GPIO_disableOutput(GPIO_IMU20948_SDA_PORT, GPIO_IMU20948_SDA_PIN);
}

static void scl_low(void)
{
    DL_GPIO_clearPins(GPIO_IMU20948_SCL_PORT, GPIO_IMU20948_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_IMU20948_SCL_PORT, GPIO_IMU20948_SCL_PIN);
}

static void scl_release(void)
{
    DL_GPIO_disableOutput(GPIO_IMU20948_SCL_PORT, GPIO_IMU20948_SCL_PIN);
}

static void configure_gpio_i2c(void)
{
    DL_I2C_disableController(IMU20948_INST);
    DL_GPIO_initDigitalInputFeatures(GPIO_IMU20948_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_IMU20948_IOMUX_SCL,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(GPIO_IMU20948_SDA_PORT, GPIO_IMU20948_SDA_PIN);
    DL_GPIO_clearPins(GPIO_IMU20948_SCL_PORT, GPIO_IMU20948_SCL_PIN);
    sda_release();
    scl_release();
}

static uint8_t wait_scl_high(void)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;

    scl_release();
    while (DL_GPIO_readPins(
            GPIO_IMU20948_SCL_PORT, GPIO_IMU20948_SCL_PIN) == 0U) {
        if (timeout == 0U) {
            return 0U;
        }
        timeout--;
    }
    return 1U;
}

static uint8_t i2c_start(void)
{
    sda_release();
    if (wait_scl_high() == 0U) {
        return 0U;
    }
    i2c_delay();
    if (DL_GPIO_readPins(
            GPIO_IMU20948_SDA_PORT, GPIO_IMU20948_SDA_PIN) == 0U) {
        return 0U;
    }
    sda_low();
    i2c_delay();
    scl_low();
    return 1U;
}

static void i2c_stop(void)
{
    sda_low();
    i2c_delay();
    (void) wait_scl_high();
    i2c_delay();
    sda_release();
    i2c_delay();
}

static uint8_t i2c_write_byte(uint8_t value)
{
    uint8_t bit;
    uint8_t ack;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        i2c_delay();
        if (wait_scl_high() == 0U) {
            scl_low();
            return 0U;
        }
        i2c_delay();
        scl_low();
        value <<= 1U;
    }
    sda_release();
    i2c_delay();
    if (wait_scl_high() == 0U) {
        scl_low();
        return 0U;
    }
    ack = (DL_GPIO_readPins(
        GPIO_IMU20948_SDA_PORT, GPIO_IMU20948_SDA_PIN) == 0U) ? 1U : 0U;
    i2c_delay();
    scl_low();
    return ack;
}

static uint8_t i2c_read_byte(uint8_t *value, uint8_t ack)
{
    uint8_t bit;
    uint8_t next = 0U;

    sda_release();
    for (bit = 0U; bit < 8U; bit++) {
        next <<= 1U;
        i2c_delay();
        if (wait_scl_high() == 0U) {
            scl_low();
            return 0U;
        }
        if (DL_GPIO_readPins(
                GPIO_IMU20948_SDA_PORT, GPIO_IMU20948_SDA_PIN) != 0U) {
            next |= 1U;
        }
        i2c_delay();
        scl_low();
    }
    if (ack != 0U) {
        sda_low();
    } else {
        sda_release();
    }
    i2c_delay();
    if (wait_scl_high() == 0U) {
        scl_low();
        sda_release();
        return 0U;
    }
    i2c_delay();
    scl_low();
    sda_release();
    *value = next;
    return 1U;
}

static void recover_bus(void)
{
    uint8_t pulse;

    g_last.stale = 1U;
    configure_gpio_i2c();
    sda_release();
    for (pulse = 0U; pulse < 9U; pulse++) {
        scl_low();
        i2c_delay();
        (void) wait_scl_high();
        i2c_delay();
    }
    i2c_stop();
}

static uint8_t write_register(uint8_t reg, uint8_t value)
{
    if (i2c_start() == 0U ||
        i2c_write_byte((uint8_t) (g_address << 1U)) == 0U ||
        i2c_write_byte(reg) == 0U ||
        i2c_write_byte(value) == 0U) {
        i2c_stop();
        recover_bus();
        return 0U;
    }
    i2c_stop();
    return 1U;
}

static uint8_t read_registers(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t i;

    if (length == 0U || i2c_start() == 0U ||
        i2c_write_byte((uint8_t) (g_address << 1U)) == 0U ||
        i2c_write_byte(reg) == 0U) {
        i2c_stop();
        recover_bus();
        return 0U;
    }
    i2c_stop();
    if (i2c_start() == 0U ||
        i2c_write_byte((uint8_t) ((g_address << 1U) | 1U)) == 0U) {
        i2c_stop();
        recover_bus();
        return 0U;
    }
    for (i = 0U; i < length; i++) {
        if (i2c_read_byte(
                &data[i], (i + 1U < length) ? 1U : 0U) == 0U) {
            i2c_stop();
            recover_bus();
            return 0U;
        }
    }
    i2c_stop();
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
    g_last.stationary = 0U;
    g_last.yaw_deg = 0.0f;
    g_last.yaw_rate_dps = 0.0f;
    g_yaw_rate_filtered = 0.0f;
    g_yaw_rate_previous = 0.0f;
    g_stationary_samples = 0U;
}

void ICM20948_ZeroYaw(void)
{
    if (g_last.calibrated == 0U) {
        return;
    }
    g_last.yaw_deg = 0.0f;
    g_last.yaw_rate_dps = 0.0f;
    g_yaw_rate_filtered = 0.0f;
    g_yaw_rate_previous = 0.0f;
}

uint8_t ICM20948_Init(void)
{
    static const uint8_t addresses[2] = {0x68U, 0x69U};
    uint8_t address_index;
    uint8_t attempt;
    uint8_t who = 0U;

    memset(&g_last, 0, sizeof(g_last));
    configure_gpio_i2c();

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
    float accel_magnitude;
    float dt_s;
    float alpha;
    uint8_t stationary_candidate;
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

    accel_magnitude =
        sqrtf(next.ax * next.ax + next.ay * next.ay + next.az * next.az);

    if (next.calibrated == 0U) {
        /*
         * Calibrate only from a continuous stationary window.  The generous
         * raw-rate limit accepts normal factory bias but rejects a vehicle
         * being carried or turned while the zero offset is being measured.
         */
        if (fabsf(raw_gx) <= CAL_STILL_GYRO_DPS &&
            fabsf(raw_gy) <= CAL_STILL_GYRO_DPS &&
            fabsf(raw_gz) <= CAL_STILL_GYRO_DPS &&
            fabsf(accel_magnitude - 9.80665f) <= CAL_STILL_ACCEL_ERR) {
            g_cal_sum_x += raw_gx;
            g_cal_sum_y += raw_gy;
            g_cal_sum_z += raw_gz;
            next.calibration_samples++;
        } else {
            g_cal_sum_x = 0.0f;
            g_cal_sum_y = 0.0f;
            g_cal_sum_z = 0.0f;
            next.calibration_samples = 0U;
        }
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
            next.yaw_deg = 0.0f;
            next.yaw_rate_dps = 0.0f;
            g_yaw_rate_filtered = 0.0f;
            g_yaw_rate_previous = 0.0f;
            g_stationary_samples = 0U;
        }
    }

    if (next.calibrated != 0U) {
        stationary_candidate =
            (fabsf(next.gx) <= STATIONARY_GYRO_DPS &&
             fabsf(next.gy) <= STATIONARY_GYRO_DPS &&
             fabsf(next.gz) <= STATIONARY_GYRO_DPS &&
             fabsf(accel_magnitude - 9.80665f) <=
                STATIONARY_ACCEL_ERR) ? 1U : 0U;

        if (stationary_candidate != 0U) {
            if (g_stationary_samples < STATIONARY_SAMPLES) {
                g_stationary_samples++;
            }
        } else {
            g_stationary_samples = 0U;
        }
        next.stationary =
            (g_stationary_samples >= STATIONARY_SAMPLES) ? 1U : 0U;

        /*
         * Track slow thermal bias only after sustained stillness. This keeps
         * real turns intact while suppressing long-term yaw drift at rest.
         */
        if (next.stationary != 0U) {
            next.gyro_bias_x +=
                (raw_gx - next.gyro_bias_x) * BIAS_TRACK_RATE;
            next.gyro_bias_y +=
                (raw_gy - next.gyro_bias_y) * BIAS_TRACK_RATE;
            next.gyro_bias_z +=
                (raw_gz - next.gyro_bias_z) * BIAS_TRACK_RATE;
            next.gx = raw_gx - next.gyro_bias_x;
            next.gy = raw_gy - next.gyro_bias_y;
            next.gz = raw_gz - next.gyro_bias_z;
        }

        if (g_last.timestamp_ms == 0U || now_ms <= g_last.timestamp_ms) {
            dt_s = (float) IMU_NOMINAL_PERIOD_MS / 1000.0f;
        } else {
            dt_s = (float) (now_ms - g_last.timestamp_ms) / 1000.0f;
        }
        if (dt_s < 0.005f) {
            dt_s = 0.005f;
        }
        if (dt_s > 0.100f) {
            dt_s = 0.100f;
        }

        alpha = dt_s / (YAW_LPF_TAU_S + dt_s);
        g_yaw_rate_filtered +=
            alpha * (next.gz - g_yaw_rate_filtered);
        if (next.stationary != 0U &&
            fabsf(g_yaw_rate_filtered) < YAW_ZERO_RATE_DPS) {
            g_yaw_rate_filtered = 0.0f;
        }

        if (g_last.calibrated != 0U) {
            next.yaw_deg +=
                0.5f * (g_yaw_rate_previous + g_yaw_rate_filtered) * dt_s;
            while (next.yaw_deg > 180.0f) {
                next.yaw_deg -= 360.0f;
            }
            while (next.yaw_deg < -180.0f) {
                next.yaw_deg += 360.0f;
            }
        }
        g_yaw_rate_previous = g_yaw_rate_filtered;
        next.yaw_rate_dps = g_yaw_rate_filtered;
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
