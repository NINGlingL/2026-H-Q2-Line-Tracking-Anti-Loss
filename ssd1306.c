/**
 * SSD1306 128x64 OLED 驱动实现 (I2C)
 * Uses the SysConfig-generated OLED_INST.
 */
#include "ssd1306.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"
#include <string.h>

#define I2C_TO            ((CPUCLK_FREQ / 1000UL) * APP_I2C_TIMEOUT_MS)
#define I2C_HALF_PERIOD   (CPUCLK_FREQ / 200000UL)
#define OLED_LIVE_PAGES   (7U)
#define OLED_PAGES_PER_UPDATE (3U)
static uint8_t s_buf[SSD1306_PAGES][SSD1306_WIDTH];
static uint8_t s_online;
static uint8_t s_address = SSD1306_I2C_ADDR;
static uint8_t s_next_page;
static volatile uint32_t s_error_count;

static void i2c_delay(void)
{
    delay_cycles(I2C_HALF_PERIOD);
}

static void sda_low(void)
{
    DL_GPIO_clearPins(GPIO_OLED_SDA_PORT, GPIO_OLED_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_OLED_SDA_PORT, GPIO_OLED_SDA_PIN);
}

static void sda_release(void)
{
    DL_GPIO_disableOutput(GPIO_OLED_SDA_PORT, GPIO_OLED_SDA_PIN);
}

static void scl_low(void)
{
    DL_GPIO_clearPins(GPIO_OLED_SCL_PORT, GPIO_OLED_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_OLED_SCL_PORT, GPIO_OLED_SCL_PIN);
}

static void scl_release(void)
{
    DL_GPIO_disableOutput(GPIO_OLED_SCL_PORT, GPIO_OLED_SCL_PIN);
}

static void configure_gpio_i2c(void)
{
    DL_I2C_disableController(OLED_INST);
    DL_GPIO_initDigitalInputFeatures(GPIO_OLED_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_OLED_IOMUX_SCL,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(GPIO_OLED_SDA_PORT, GPIO_OLED_SDA_PIN);
    DL_GPIO_clearPins(GPIO_OLED_SCL_PORT, GPIO_OLED_SCL_PIN);
    sda_release();
    scl_release();
}

static uint8_t wait_scl_high(void)
{
    volatile uint32_t to = I2C_TO;

    scl_release();
    while (DL_GPIO_readPins(GPIO_OLED_SCL_PORT, GPIO_OLED_SCL_PIN) == 0U) {
        if (to == 0U) {
            return 0U;
        }
        to--;
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
    if (DL_GPIO_readPins(GPIO_OLED_SDA_PORT, GPIO_OLED_SDA_PIN) == 0U) {
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
    ack = (DL_GPIO_readPins(GPIO_OLED_SDA_PORT, GPIO_OLED_SDA_PIN) == 0U) ?
        1U : 0U;
    i2c_delay();
    scl_low();
    return ack;
}

static uint8_t write_transfer(const uint8_t *data, uint8_t length)
{
    uint8_t index;

    if (length == 0U || i2c_start() == 0U ||
        i2c_write_byte((uint8_t) (s_address << 1U)) == 0U) {
        i2c_stop();
        s_error_count++;
        return 0U;
    }
    for (index = 0U; index < length; index++) {
        if (i2c_write_byte(data[index]) == 0U) {
            i2c_stop();
            s_error_count++;
            return 0U;
        }
    }
    i2c_stop();
    return 1U;
}

/* ── I2C 底层 ── */
static void _cmd(uint8_t cmd)
{
    uint8_t tx[2] = {0x00U, cmd};

    if (s_online != 0U && write_transfer(tx, sizeof(tx)) == 0U) {
        s_online = 0U;
    }
}

static uint8_t write_data(const uint8_t *data, uint8_t length)
{
    uint8_t index;

    if (i2c_start() == 0U ||
        i2c_write_byte((uint8_t) (s_address << 1U)) == 0U ||
        i2c_write_byte(0x40U) == 0U) {
        i2c_stop();
        s_error_count++;
        return 0U;
    }
    for (index = 0U; index < length; index++) {
        if (i2c_write_byte(data[index]) == 0U) {
            i2c_stop();
            s_error_count++;
            return 0U;
        }
    }
    i2c_stop();
    return 1U;
}

static uint8_t write_page(uint8_t page)
{
    _cmd((uint8_t) (0xB0U + page));
    _cmd(0x00U);
    _cmd(0x10U);
    if (s_online == 0U) {
        return 0U;
    }
    if (write_data(s_buf[page], SSD1306_WIDTH) == 0U) {
        s_online = 0U;
        return 0U;
    }
    return 1U;
}

/* ── 写入多字节 (寄存器地址 + 数据) ── */
void SSD1306_WriteBytes(uint8_t *data, uint8_t len)
{
    if (s_online != 0U && write_transfer(data, len) == 0U) {
        s_online = 0U;
    }
}

/* ── 初始化 ── */
uint8_t SSD1306_Init(void)
{
    static const uint8_t cmds[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
        0x8DU, 0x14U, 0x20U, 0x02U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
        0x81U, 0x7FU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U,
        0x2EU, 0xAFU
    };
    static const uint8_t addresses[] = {SSD1306_I2C_ADDR, 0x3DU};
    uint8_t address_index;
    uint8_t command_index;

    configure_gpio_i2c();
    s_error_count = 0U;
    s_next_page = 0U;
    delay_cycles(CPUCLK_FREQ / 10U);
    for (address_index = 0U;
         address_index < (uint8_t) sizeof(addresses);
         address_index++) {
        s_address = addresses[address_index];
        s_online = 1U;
        for (command_index = 0U;
             command_index < (uint8_t) sizeof(cmds);
             command_index++) {
            _cmd(cmds[command_index]);
            if (s_online == 0U) {
                break;
            }
        }
        if (s_online != 0U) {
            SSD1306_Clear();
            for (command_index = 0U;
                 command_index < SSD1306_PAGES;
                 command_index++) {
                if (write_page(command_index) == 0U) {
                    break;
                }
            }
            return s_online;
        }
    }
    s_online = 0U;
    return 0U;
}

void SSD1306_Clear(void) { memset(s_buf, 0, sizeof(s_buf)); }

uint8_t SSD1306_Update(void)
{
    uint8_t count;

    if (s_online == 0U) return 0U;
    for (count = 0U; count < OLED_PAGES_PER_UPDATE; count++) {
        if (write_page(s_next_page) == 0U) {
            break;
        }
        s_next_page++;
        if (s_next_page >= OLED_LIVE_PAGES) {
            s_next_page = 0U;
        }
    }
    return s_online;
}

uint8_t SSD1306_IsOnline(void)
{
    return s_online;
}

uint8_t SSD1306_TryRecover(void)
{
    uint8_t pulse;

    if (s_online != 0U) return 1U;
    s_online = 0U;
    configure_gpio_i2c();
    sda_release();
    for (pulse = 0U; pulse < 9U; pulse++) {
        scl_low();
        i2c_delay();
        (void) wait_scl_high();
        i2c_delay();
    }
    i2c_stop();
    return SSD1306_Init();
}

/* ── 5x7 字体 ── */
static void _char(uint8_t pg, uint8_t col, char ch)
{
    if (pg >= SSD1306_PAGES || ch < 0x20 || ch > 0x7E) return;
    static const uint8_t f[][5] = {
        {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},
        {0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
        {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
        {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
        {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},
        {0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
        {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},
        {0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
        {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
        {0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
        {0x00,0x08,0x14,0x22,0x41},{0x14,0x14,0x14,0x14,0x14},
        {0x41,0x22,0x14,0x08,0x00},{0x02,0x01,0x51,0x09,0x06},
        {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},
        {0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
        {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},
        {0x7F,0x09,0x09,0x01,0x01},{0x3E,0x41,0x41,0x51,0x32},
        {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
        {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
        {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x04,0x02,0x7F},
        {0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
        {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},
        {0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
        {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
        {0x1F,0x20,0x40,0x20,0x1F},{0x7F,0x20,0x18,0x20,0x7F},
        {0x63,0x14,0x08,0x14,0x63},{0x03,0x04,0x78,0x04,0x03},
        {0x61,0x51,0x49,0x45,0x43},{0x00,0x00,0x7F,0x41,0x41},
        {0x02,0x04,0x08,0x10,0x20},{0x41,0x41,0x7F,0x00,0x00},
        {0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
        {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
        {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
        {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},
        {0x08,0x7E,0x09,0x01,0x02},{0x08,0x14,0x54,0x54,0x3C},
        {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},
        {0x20,0x40,0x44,0x3D,0x00},{0x00,0x7F,0x10,0x28,0x44},
        {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
        {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
        {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},
        {0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
        {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},
        {0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
        {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
        {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},
        {0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},
        {0x08,0x04,0x08,0x10,0x08},
    };
    uint8_t idx = ch - 0x20;
    if (col + 5 < SSD1306_WIDTH)
        for (uint8_t i = 0; i < 5; i++) s_buf[pg][col + i] = f[idx][i];
}

void SSD1306_ShowString(uint8_t pg, uint8_t col, const char *s)
{
    while (*s && col + 6 <= SSD1306_WIDTH) { _char(pg, col, *s); col += 6; s++; }
}

void SSD1306_WriteFloat(uint8_t pg, uint8_t col, float v, uint8_t dec)
{
    char b[16]; int i = 0, neg = 0;
    if (v < 0) { neg = 1; v = -v; }
    int ip = (int)v; float fp = v - ip;
    if (ip == 0) b[i++] = '0';
    else { char t[8]; int ti = 0; while (ip > 0) { t[ti++] = '0'+(ip%10); ip/=10; } while (ti) b[i++] = t[--ti]; }
    if (neg) { for (int j = i; j > 0; j--) b[j] = b[j-1]; b[0] = '-'; i++; }
    if (dec) { b[i++] = '.'; for (uint8_t d = 0; d < dec; d++) fp *= 10.0f;
        int frac = (int)(fp + 0.5f); char t[8]; int ti = 0;
        for (uint8_t d = 0; d < dec; d++) { t[ti++] = '0'+(frac%10); frac/=10; }
        while (ti) b[i++] = t[--ti]; }
    b[i] = '\0';
    SSD1306_ShowString(pg, col, b);
}
