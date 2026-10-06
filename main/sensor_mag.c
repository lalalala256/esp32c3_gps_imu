/*
 * sensor_mag.c - QMC5883L acquisition.
 *
 * Unlike the BMI160 the QMC5883L has no wired data-ready line on this board, so
 * the device's DRDY status bit is polled at the configured rate. Values are
 * reported in milligauss. A hard/soft-iron calibration hook is provided in the
 * form of two constants which are applied to every sample; they stay at their
 * neutral values until a calibration run produces real numbers.
 */
#include "sensor_mag.h"

#include <string.h>

#include "app_config.h"
#include "board_pins.h"
#include "data_types.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "qmc5883l.h"

static const char *TAG = "mag";

/* ------------------------------------------- magnetometer calibration hook */

/* Hard-iron offset per axis, in milligauss. */
static const float MAG_HARD_IRON[3] = { 0.0f, 0.0f, 0.0f };

/* Soft-iron correction matrix, row-major. Identity until calibrated. */
static const float MAG_SOFT_IRON[3][3] = {
    { 1.0f, 0.0f, 0.0f },
    { 0.0f, 1.0f, 0.0f },
    { 0.0f, 0.0f, 1.0f },
};

static void mag_apply_calibration(const qmc5883l_data_t *in, float out[3])
{
    const float v[3] = {
        in->x - MAG_HARD_IRON[0],
        in->y - MAG_HARD_IRON[1],
        in->z - MAG_HARD_IRON[2],
    };

    for (int r = 0; r < 3; r++) {
        out[r] = MAG_SOFT_IRON[r][0] * v[0] +
                 MAG_SOFT_IRON[r][1] * v[1] +
                 MAG_SOFT_IRON[r][2] * v[2];
    }
}

/* ------------------------------------------------- Kconfig -> driver enums */

#if defined(CONFIG_APP_QMC5883L_ODR_10)
#define MAG_ODR   QMC5883L_DR_10
#elif defined(CONFIG_APP_QMC5883L_ODR_50)
#define MAG_ODR   QMC5883L_DR_50
#elif defined(CONFIG_APP_QMC5883L_ODR_200)
#define MAG_ODR   QMC5883L_DR_200
#else
#define MAG_ODR   QMC5883L_DR_100
#endif

#if defined(CONFIG_APP_QMC5883L_OSR_64)
#define MAG_OSR   QMC5883L_OSR_64
#elif defined(CONFIG_APP_QMC5883L_OSR_128)
#define MAG_OSR   QMC5883L_OSR_128
#elif defined(CONFIG_APP_QMC5883L_OSR_512)
#define MAG_OSR   QMC5883L_OSR_512
#else
#define MAG_OSR   QMC5883L_OSR_256
#endif

#if defined(CONFIG_APP_QMC5883L_RANGE_8G)
#define MAG_RNG   QMC5883L_RNG_8
#else
#define MAG_RNG   QMC5883L_RNG_2
#endif

/* ------------------------------------------------------------------ state */

static qmc5883l_t s_dev;
static volatile bool s_ready;

/* -------------------------------------------------------------------- task */

static void mag_task(void *arg)
{
    (void)arg;

    TickType_t period = pdMS_TO_TICKS(1000 / APP_MAG_RATE_HZ);
    if (period == 0) {
        period = 1;
    }
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        bool ready = false;

        if (qmc5883l_data_ready(&s_dev, &ready) == ESP_OK && ready) {
            qmc5883l_data_t d;
            mag_sample_t s = {
                .t_us  = esp_timer_get_time(),
                .valid = false,
            };

            if (qmc5883l_get_data(&s_dev, &d) == ESP_OK) {
                float c[3];
                mag_apply_calibration(&d, c);
                s.mx = c[0];
                s.my = c[1];
                s.mz = c[2];
                s.valid = true;
            } else {
                g_stats.mag_errors++;
            }

            if (xQueueSend(q_mag, &s, 0) == pdPASS) {
                g_stats.mag_samples++;
            } else {
                g_stats.mag_drops++;
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/* -------------------------------------------------------------------- init */

esp_err_t sensor_mag_init(void)
{
    esp_err_t err;

    err = qmc5883l_init_desc(&s_dev, APP_QMC5883L_I2C_ADDR,
                             (i2c_port_t)APP_I2C_PORT_NUM,
                             PIN_I2C_SDA, PIN_I2C_SCL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "qmc5883l_init_desc failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Keep the shared bus at the same speed as the BMI160. */
    s_dev.i2c_dev.cfg.master.clk_speed = APP_I2C_FREQ_HZ;

    /* Probe a few times: a module that has just been powered up can need a
     * moment, and one NACK should not cost us the whole magnetometer stream. */
    uint8_t id = 0;
    for (int attempt = 1; attempt <= 3; attempt++) {
        id = 0;
        err = qmc5883l_get_chip_id(&s_dev, &id);
        if (err == ESP_OK && id == 0xFF) {
            break;
        }
        ESP_LOGW(TAG, "QMC5883L probe %d/3 failed (id 0x%02X, %s)",
                 attempt, id, esp_err_to_name(err));
        if (attempt < 3) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    if (err != ESP_OK || id != 0xFF) {
        ESP_LOGE(TAG, "QMC5883L not found at 0x%02X (chip id 0x%02X, %s)",
                 APP_QMC5883L_I2C_ADDR, id, esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "QMC5883L detected (chip id 0x%02X)", id);

    err = qmc5883l_reset(&s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "qmc5883l_reset failed: %s", esp_err_to_name(err));
        return err;
    }

    err = qmc5883l_set_config(&s_dev, MAG_ODR, MAG_OSR, MAG_RNG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "qmc5883l_set_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = qmc5883l_set_mode(&s_dev, QMC5883L_MODE_CONTINUOUS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "qmc5883l_set_mode failed: %s", esp_err_to_name(err));
        return err;
    }

    s_ready = true;
    return ESP_OK;
}

void sensor_mag_start_task(void)
{
    if (!s_ready) {
        return;
    }
    xTaskCreate(mag_task, "mag", APP_STACK_MAG, NULL, APP_PRIO_MAG, NULL);
}
