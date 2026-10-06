/*
 * i2c_diag.c - see i2c_diag.h.
 *
 * The board has one I2C bus shared by the BMI160 (0x68) and the magnetometer.
 * Because the BMI160 answers reliably at 100 Hz, the MCU's I2C peripheral, the
 * GPIO configuration, the code path and the bus levels are all already known
 * good. That means an absent magnetometer can only be one of:
 *
 *   - the module gets no power (VCC/GND broken, cold solder joint)
 *   - its SDA/SCL are not connected, or are swapped
 *   - the module is dead, or is not the chip it was sold as
 *   - the module answers at a different address (clones: HMC5883L at 0x1E,
 *     QMC5883P at 0x2C, ...)
 *
 * Note that "SDA or SCL shorted to GND" is already excluded by the BMI160
 * working: a short would take down the whole bus, not one branch of it. This
 * report is written to make the remaining possibilities separable instead of
 * leaving them as one vague "sensor missing".
 */
#include "i2c_diag.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "board_pins.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2cdev.h"

static const char *TAG = "i2cdiag";

#define SCAN_FIRST_ADDR  0x08
#define SCAN_LAST_ADDR   0x77
#define REG_DUMP_BYTES   16
#define IDLE_SAMPLES     20
#define REPORT_LINE_MAX  192

/* Addresses sold as / commonly found on boards sold as "QMC5883L" modules. */
static const uint8_t MAG_CANDIDATES[] = { 0x0D, 0x1E, 0x2C, 0x0E };

/* ------------------------------------------------------------------ output */

static void append(char *out, size_t out_len, size_t *off, const char *s)
{
    const size_t n = strlen(s);
    if (*off + n >= out_len) {
        return;                 /* keep what fits, stay NUL-terminated */
    }
    memcpy(out + *off, s, n);
    *off += n;
    out[*off] = '\0';
}

static void appendf(char *out, size_t out_len, size_t *off, const char *fmt, ...)
{
    char line[REPORT_LINE_MAX];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    append(out, out_len, off, line);
}

/* ------------------------------------------------------- pre-init line check */

esp_err_t i2c_diag_lines_pre_init(void)
{
    const gpio_config_t as_input = {
        .pin_bit_mask = (1ULL << PIN_I2C_SDA) | (1ULL << PIN_I2C_SCL),
        .mode         = GPIO_MODE_INPUT,
        /* Enable the internal pull-ups. If no module carries its own, these are
         * the only thing holding the lines high, and without them this test
         * would report a perfectly good bus as stuck. */
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&as_input);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot put SDA/SCL into input mode for the idle check: %s",
                 esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(5));

    const int sda = gpio_get_level(PIN_I2C_SDA);
    const int scl = gpio_get_level(PIN_I2C_SCL);

    if (sda && scl) {
        ESP_LOGI(TAG, "bus idle check (pre-init): SDA=1 SCL=1, both released");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "bus idle check (pre-init): SDA=%d SCL=%d, a line is pulled low",
             sda, scl);

    if (!scl) {
        ESP_LOGE(TAG, "SCL reads low while nothing is driving it. Either it is "
                 "shorted to GND or a device is holding it, and no scan can work "
                 "until that is fixed. Measure SCL against GND with a meter.");
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * SCL is fine but SDA is held. The usual cause is a slave that was
     * interrupted part-way through a byte and is waiting for the clocks that
     * finish the transfer. Releasing SDA and pulsing SCL walks it to a stop
     * condition, which is what a bus recovery is.
     */
    ESP_LOGW(TAG, "SDA held low, sending 9 recovery clocks on SCL");

    err = gpio_set_direction(PIN_I2C_SCL, GPIO_MODE_OUTPUT_OD);
    if (err == ESP_OK) {
        for (int i = 0; i < 9; i++) {
            gpio_set_level(PIN_I2C_SCL, 0);
            esp_rom_delay_us(5);
            gpio_set_level(PIN_I2C_SCL, 1);
            esp_rom_delay_us(5);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    if (gpio_get_level(PIN_I2C_SDA)) {
        ESP_LOGI(TAG, "SDA released by the recovery clocks: a slave was stuck, "
                 "not the wiring");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "SDA is still low after 9 recovery clocks, so it is shorted to "
             "GND or something holds it permanently. Measure SDA against GND, and "
             "try again with the magnetometer module unplugged.");
    return ESP_ERR_INVALID_STATE;
}

/* ------------------------------------------------------------- bus scanning */

static bool read_registers(i2c_master_bus_handle_t bus, uint8_t addr,
                           uint8_t *regs, size_t n)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = APP_I2C_FREQ_HZ,
    };

    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK || !dev) {
        return false;
    }

    const uint8_t start_reg = 0x00;
    const esp_err_t err = i2c_master_transmit_receive(dev, &start_reg, 1, regs, n,
                                                      APP_I2C_SCAN_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev);
    return err == ESP_OK;
}

/*
 * Best-effort identification from the register dump.
 *
 * Getting this wrong is harmless - the raw bytes are printed next to it - and
 * it exists so that a clone announces itself instead of looking like a broken
 * QMC5883L, which is exactly the confusion this whole file is meant to end.
 * Register offsets differ per family: the QMC parts put their chip id in 0x00,
 * the HMC58xx in 0x0A..0x0C.
 */
static const char *identify(uint8_t addr, const uint8_t *r)
{
    if ((addr == 0x68 || addr == 0x69) && r[0] == 0xD1) {
        return "BMI160";
    }
    if (addr == 0x0D && r[0] == 0xFF) {
        return "QMC5883L / VCM5883L";
    }
    if (addr == 0x2C && r[0] == 0x80) {
        return "QMC5883P";
    }
    if (addr == 0x1E && r[10] == 'H' && r[11] == '4' && r[12] == '3') {
        return "HMC5883L";
    }
    if (addr == 0x0E) {
        return "IST8310?";
    }
    if (addr == 0x1C) {
        return "LIS3MDL?";
    }
    return "unknown";
}

static void hex_dump(const uint8_t *regs, size_t n, char *out, size_t out_len)
{
    size_t off = 0;
    for (size_t i = 0; i < n && off + 4 < out_len; i++) {
        off += (size_t)snprintf(out + off, out_len - off, "%02X ", regs[i]);
    }
}

unsigned i2c_diag_report(char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return 0;
    }
    out[0] = '\0';

    size_t off = 0;

    appendf(out, out_len, &off, "bus: SDA=GPIO%d SCL=GPIO%d @%d Hz\n",
            PIN_I2C_SDA, PIN_I2C_SCL, APP_I2C_FREQ_HZ);

    /*
     * Sample the lines several times. A single read can land inside a
     * transaction and look like a stuck line; "some samples low" is just
     * traffic, "every sample low" is a real hold.
     */
    unsigned low_sda = 0, low_scl = 0;
    for (int i = 0; i < IDLE_SAMPLES; i++) {
        low_sda += (gpio_get_level(PIN_I2C_SDA) == 0);
        low_scl += (gpio_get_level(PIN_I2C_SCL) == 0);
        esp_rom_delay_us(100);
    }
    appendf(out, out_len, &off, "idle: SDA low %u/%u, SCL low %u/%u%s\n",
            low_sda, (unsigned)IDLE_SAMPLES, low_scl, (unsigned)IDLE_SAMPLES,
            (low_sda == IDLE_SAMPLES || low_scl == IDLE_SAMPLES)
                ? "   <== a line is HELD low" : "");

    void *bus_v = NULL;
    if (i2cdev_get_shared_handle((i2c_port_t)APP_I2C_PORT_NUM, &bus_v) != ESP_OK ||
        !bus_v) {
        append(out, out_len, &off, "note: the I2C bus does not exist yet\n");
        return 0;
    }
    i2c_master_bus_handle_t bus = (i2c_master_bus_handle_t)bus_v;

    appendf(out, out_len, &off, "scan 0x%02X..0x%02X:\n",
            SCAN_FIRST_ADDR, SCAN_LAST_ADDR);

    unsigned count = 0;
    for (uint8_t addr = SCAN_FIRST_ADDR; addr <= SCAN_LAST_ADDR; addr++) {
        if (i2c_master_probe(bus, addr, APP_I2C_SCAN_TIMEOUT_MS) != ESP_OK) {
            continue;
        }
        count++;

        uint8_t regs[REG_DUMP_BYTES];
        if (!read_registers(bus, addr, regs, sizeof(regs))) {
            appendf(out, out_len, &off,
                    "  0x%02X ACK, but the register read failed\n", addr);
            continue;
        }

        char hex[REG_DUMP_BYTES * 3 + 1];
        hex_dump(regs, sizeof(regs), hex, sizeof(hex));
        appendf(out, out_len, &off, "  0x%02X  regs[00..0F] = %s -> %s\n",
                addr, hex, identify(addr, regs));
    }

    if (count == 0) {
        append(out, out_len, &off, "  nothing answered anywhere on the bus\n");
    } else {
        appendf(out, out_len, &off, "%u device(s) answered\n", count);
    }

    /* The specific question this report usually gets asked. */
    append(out, out_len, &off, "magnetometer candidates:");
    for (size_t i = 0; i < sizeof(MAG_CANDIDATES); i++) {
        const bool ack = (i2c_master_probe(bus, MAG_CANDIDATES[i],
                                           APP_I2C_SCAN_TIMEOUT_MS) == ESP_OK);
        appendf(out, out_len, &off, " 0x%02X:%s", MAG_CANDIDATES[i],
                ack ? "ACK" : "no");
    }
    append(out, out_len, &off,
           "\n(a wrong address is ruled out by a scan that answers nowhere; "
           "a swapped SDA/SCL produces exactly this result)\n");

    return count;
}
