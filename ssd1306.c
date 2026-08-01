/**
 * SSD1306 驱动实现 (I2C)
 * MSPM0G3507 + DriverLib
 */

#include "ssd1306.h"

/* ========== 显示缓冲区 (128x64 = 1024 字节) ========== */
static uint8_t OLED_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static bool s_oled_healthy = true;
static uint8_t s_oled_address = SSD1306_I2C_ADDR;
static uint8_t s_next_page = 0U;
static uint8_t s_recovery_wait = 0U;

#define OLED_I2C_TIMEOUT_LOOPS   ((CPUCLK_FREQ / 1000UL) * 10UL)
#define OLED_I2C_HALF_PERIOD     (CPUCLK_FREQ / 200000UL)
#define OLED_PAGES_PER_UPDATE    1U
#define OLED_RECOVERY_BACKOFF_UPDATES 10U

/* ========== 6x8 ASCII 字库 (字符 0x20 ~ 0x7E) ========== */
static const uint8_t Font6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* 空格 */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* $ */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /* % */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /* & */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /* ) */
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, /* * */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /* + */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /* - */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /* ; */
    {0x00,0x08,0x14,0x22,0x41,0x00}, /* < */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /* = */
    {0x41,0x22,0x14,0x08,0x00,0x00}, /* > */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /* A */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /* B */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /* D */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /* E */
    {0x7F,0x09,0x09,0x01,0x01,0x00}, /* F */
    {0x3E,0x41,0x41,0x51,0x32,0x00}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /* H */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /* J */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /* K */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /* L */
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /* O */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /* R */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /* S */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /* V */
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, /* W */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /* X */
    {0x03,0x04,0x78,0x04,0x03,0x00}, /* Y */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /* Z */
    {0x00,0x00,0x7F,0x41,0x41,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /* 反斜杠 */
    {0x41,0x41,0x7F,0x00,0x00,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /* ^ */
    {0x40,0x40,0x40,0x40,0x40,0x00}, /* _ */
    {0x00,0x01,0x02,0x04,0x00,0x00}, /* ` */
    {0x20,0x54,0x54,0x54,0x78,0x00}, /* a */
    {0x7F,0x48,0x44,0x44,0x38,0x00}, /* b */
    {0x38,0x44,0x44,0x44,0x20,0x00}, /* c */
    {0x38,0x44,0x44,0x48,0x7F,0x00}, /* d */
    {0x38,0x54,0x54,0x54,0x18,0x00}, /* e */
    {0x08,0x7E,0x09,0x01,0x02,0x00}, /* f */
    {0x08,0x14,0x54,0x54,0x3C,0x00}, /* g */
    {0x7F,0x08,0x04,0x04,0x78,0x00}, /* h */
    {0x00,0x44,0x7D,0x40,0x00,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00,0x00}, /* j */
    {0x00,0x7F,0x10,0x28,0x44,0x00}, /* k */
    {0x00,0x41,0x7F,0x40,0x00,0x00}, /* l */
    {0x7C,0x04,0x18,0x04,0x78,0x00}, /* m */
    {0x7C,0x08,0x04,0x04,0x78,0x00}, /* n */
    {0x38,0x44,0x44,0x44,0x38,0x00}, /* o */
    {0x7C,0x14,0x14,0x14,0x08,0x00}, /* p */
    {0x08,0x14,0x14,0x18,0x7C,0x00}, /* q */
    {0x7C,0x08,0x04,0x04,0x08,0x00}, /* r */
    {0x48,0x54,0x54,0x54,0x20,0x00}, /* s */
    {0x04,0x3F,0x44,0x40,0x20,0x00}, /* t */
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, /* v */
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, /* w */
    {0x44,0x28,0x10,0x28,0x44,0x00}, /* x */
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, /* y */
    {0x44,0x64,0x54,0x4C,0x44,0x00}, /* z */
    {0x00,0x08,0x36,0x41,0x00,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00,0x00}, /* | */
    {0x00,0x41,0x36,0x08,0x00,0x00}, /* } */
    {0x08,0x04,0x08,0x10,0x08,0x00}, /* ~ */
};

/* ========== 稳定的软件 I2C（参考“电赛备用2”） ========== */

static void i2c_delay(void)
{
    delay_cycles(OLED_I2C_HALF_PERIOD);
}

static void sda_low(void)
{
    DL_GPIO_clearPins(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN);
}

static void sda_release(void)
{
    DL_GPIO_disableOutput(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN);
}

static void scl_low(void)
{
    DL_GPIO_clearPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
}

static void scl_release(void)
{
    DL_GPIO_disableOutput(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
}

static void configure_gpio_i2c(void)
{
    DL_I2C_disableController(I2C_0_INST);
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_0_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_0_IOMUX_SCL,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN);
    DL_GPIO_clearPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
    sda_release();
    scl_release();
}

static bool wait_scl_high(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT_LOOPS;

    scl_release();
    while (DL_GPIO_readPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN) == 0U) {
        if (timeout-- == 0U) {
            return false;
        }
    }
    return true;
}

static bool i2c_start(void)
{
    sda_release();
    if (!wait_scl_high()) {
        return false;
    }
    i2c_delay();
    if (DL_GPIO_readPins(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN) == 0U) {
        return false;
    }
    sda_low();
    i2c_delay();
    scl_low();
    return true;
}

static void i2c_stop(void)
{
    sda_low();
    i2c_delay();
    (void)wait_scl_high();
    i2c_delay();
    sda_release();
    i2c_delay();
}

static bool i2c_write_byte(uint8_t value)
{
    uint8_t bit;
    bool acknowledged;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        i2c_delay();
        if (!wait_scl_high()) {
            scl_low();
            return false;
        }
        i2c_delay();
        scl_low();
        value <<= 1U;
    }

    sda_release();
    i2c_delay();
    if (!wait_scl_high()) {
        scl_low();
        return false;
    }
    acknowledged =
        (DL_GPIO_readPins(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN) == 0U);
    i2c_delay();
    scl_low();
    return acknowledged;
}

static bool write_command(uint8_t command)
{
    bool ok = i2c_start() &&
              i2c_write_byte((uint8_t)(s_oled_address << 1U)) &&
              i2c_write_byte(SSD1306_CMD_SINGLE) &&
              i2c_write_byte(command);

    i2c_stop();
    if (!ok) {
        s_oled_healthy = false;
    }
    return ok;
}

static bool write_page(uint8_t page)
{
    uint8_t column;
    const uint8_t *data = &OLED_Buffer[(uint16_t)page * SSD1306_WIDTH];

    if (!write_command((uint8_t)(0xB0U + page)) ||
        !write_command(0x00U) || !write_command(0x10U)) {
        return false;
    }
    if (!i2c_start() ||
        !i2c_write_byte((uint8_t)(s_oled_address << 1U)) ||
        !i2c_write_byte(SSD1306_DATA_STREAM)) {
        i2c_stop();
        s_oled_healthy = false;
        return false;
    }
    for (column = 0U; column < SSD1306_WIDTH; column++) {
        if (!i2c_write_byte(data[column])) {
            i2c_stop();
            s_oled_healthy = false;
            return false;
        }
    }
    i2c_stop();
    return true;
}

static bool recover_i2c_bus(void)
{
    uint8_t pulse;

    sda_release();
    for (pulse = 0U; pulse < 9U; pulse++) {
        scl_low();
        i2c_delay();
        if (!wait_scl_high()) {
            /* SCL 被硬件持续拉低时立即退出，不能累计等待 9 个超时。 */
            scl_low();
            i2c_stop();
            return false;
        }
        i2c_delay();
    }
    i2c_stop();
    return true;
}

/* ========== 基本操作 ========== */

void SSD1306_Init(void)
{
    static const uint8_t init_sequence[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
        0x8DU, 0x14U, 0x20U, 0x02U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
        0x81U, 0x7FU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U,
        0x2EU, 0xAFU
    };
    static const uint8_t addresses[] = {SSD1306_I2C_ADDR, 0x3DU};
    uint8_t address_index;
    uint8_t command_index;
    uint8_t page;

    configure_gpio_i2c();
    (void)recover_i2c_bus();
    s_next_page = 0U;
    s_recovery_wait = 0U;

    /* 与“电赛备用2”实机驱动一致：100 kHz 软件 I2C + page addressing。 */
    delay_cycles(CPUCLK_FREQ / 10U);
    for (address_index = 0U; address_index < sizeof(addresses); address_index++) {
        s_oled_address = addresses[address_index];
        s_oled_healthy = true;
        for (command_index = 0U; command_index < sizeof(init_sequence);
             command_index++) {
            if (!write_command(init_sequence[command_index])) {
                break;
            }
        }
        if (s_oled_healthy) {
            SSD1306_Clear();
            for (page = 0U; page < SSD1306_PAGES; page++) {
                if (!write_page(page)) {
                    break;
                }
            }
            if (s_oled_healthy) {
                return;
            }
        }
    }
    s_oled_healthy = false;
}

bool SSD1306_IsHealthy(void)
{
    return s_oled_healthy;
}

void SSD1306_Clear(void)
{
    memset(OLED_Buffer, 0x00, sizeof(OLED_Buffer));
}

void SSD1306_Fill(uint8_t data)
{
    memset(OLED_Buffer, data, sizeof(OLED_Buffer));
}

void SSD1306_Update(void)
{
    uint8_t count;

    /*
     * 一次只发送一页，避免 OLED 刷新长时间占住 20 Hz PID 主循环。
     * 瞬态 NACK/拉低总线后不永久冻结显示：约每 1 s 做一次有界总线恢复。
     */
    if (!s_oled_healthy) {
        s_recovery_wait++;
        if (s_recovery_wait < OLED_RECOVERY_BACKOFF_UPDATES) {
            return;
        }
        s_recovery_wait = 0U;
        configure_gpio_i2c();
        if (!recover_i2c_bus()) {
            return;
        }
        s_oled_healthy = true;
    }

    for (count = 0U; count < OLED_PAGES_PER_UPDATE; count++) {
        if (!write_page(s_next_page)) {
            return;
        }
        s_next_page++;
        if (s_next_page >= SSD1306_PAGES) {
            s_next_page = 0U;
        }
    }
}

void SSD1306_SetCursor(uint8_t page, uint8_t col)
{
    /* 仅用于 ShowChar 定位，不在底层发 I2C */
    (void)page;
    (void)col;
}

/* ========== 显示功能 ========== */

void SSD1306_ShowChar(uint8_t page, uint8_t col, char ch)
{
    if (ch < 0x20 || ch > 0x7E) ch = ' ';  /* 非法字符显示空格 */
    if (page >= SSD1306_PAGES) return;
    if (col + 6 > SSD1306_WIDTH) return;

    uint8_t idx = ch - 0x20;
    for (uint8_t i = 0; i < 6; i++) {
        OLED_Buffer[page * SSD1306_WIDTH + col + i] = Font6x8[idx][i];
    }
}

void SSD1306_ShowString(uint8_t page, uint8_t col, const char *str)
{
    while (*str) {
        if (col + 6 > SSD1306_WIDTH) {
            col = 0;
            page++;
            if (page >= SSD1306_PAGES) return;
        }
        SSD1306_ShowChar(page, col, *str);
        col += 6;
        str++;
    }
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    uint8_t page = y / 8;
    uint8_t bit  = y % 8;
    uint16_t idx = page * SSD1306_WIDTH + x;

    if (color)
        OLED_Buffer[idx] |= (1 << bit);
    else
        OLED_Buffer[idx] &= ~(1 << bit);
}

void SSD1306_DrawHLine(uint8_t x, uint8_t y, uint8_t len, uint8_t color)
{
    for (uint8_t i = 0; i < len; i++)
        SSD1306_DrawPixel(x + i, y, color);
}

void SSD1306_ShowNum(uint8_t page, uint8_t col, int32_t num, uint8_t len)
{
    char buf[12];
    if (num < 0) {
        /* 简单处理负数 */
        SSD1306_ShowChar(page, col, '-');
        num = -num;
        col += 6;
        len--;
    }
    /* 转换为字符串 */
    uint8_t i = 0;
    char tmp[12];
    do {
        tmp[i++] = '0' + (num % 10);
        num /= 10;
    } while (num > 0 && i < 11);

    /* 补齐前导零 */
    while (i < len && i < 11) tmp[i++] = '0';

    /* 反转输出 */
    while (i > 0) {
        SSD1306_ShowChar(page, col, tmp[--i]);
        col += 6;
    }
}
