/*
 * sensor_imu.c - BMI160 acquisition.
 *
 * The sensor is configured for the full-scale ranges and ODR selected in
 * menuconfig. Its data-ready signal is routed to INT1 (GPIO0); the GPIO ISR
 * captures an esp_timer timestamp and hands it to a high-priority task which
 * performs the I2C burst read. Timestamping inside the ISR is what keeps the
 * IMU samples usable for tightly-coupled INS/GNSS fusion later on.
 */
#include "sensor_imu.h"

#include <string.h>

#include "app_config.h"
#include "board_pins.h"
#include "data_types.h"

#include "bmi160.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "imu";

/* ------------------------------------------------- Kconfig -> driver enums */

#if defined(CONFIG_APP_BMI160_ACC_RANGE_2G)
#define IMU_ACC_RANGE   BMI160_ACC_RANGE_2G
#elif defined(CONFIG_APP_BMI160_ACC_RANGE_4G)
#define IMU_ACC_RANGE   BMI160_ACC_RANGE_4G
#elif defined(CONFIG_APP_BMI160_ACC_RANGE_16G)
#define IMU_ACC_RANGE   BMI160_ACC_RANGE_16G
#else
#define IMU_ACC_RANGE   BMI160_ACC_RANGE_8G
#endif

#if defined(CONFIG_APP_BMI160_GYR_RANGE_125DPS)
#define IMU_GYR_RANGE   BMI160_GYR_RANGE_125DPS
#elif defined(CONFIG_APP_BMI160_GYR_RANGE_250DPS)
#define IMU_GYR_RANGE   BMI160_GYR_RANGE_250DPS
#elif defined(CONFIG_APP_BMI160_GYR_RANGE_500DPS)
#define IMU_GYR_RANGE   BMI160_GYR_RANGE_500DPS
#elif defined(CONFIG_APP_BMI160_GYR_RANGE_2000DPS)
#define IMU_GYR_RANGE   BMI160_GYR_RANGE_2000DPS
#else
#define IMU_GYR_RANGE   BMI160_GYR_RANGE_1000DPS
#endif

/* The BMI160 offers one ODR per power of two; pick the nearest one that is
 * at least as fast as the requested rate. */
#if APP_IMU_ODR_HZ <= 25
#define IMU_ACC_ODR     BMI160_ACC_ODR_25HZ
#define IMU_GYR_ODR     BMI160_GYR_ODR_25HZ
#elif APP_IMU_ODR_HZ <= 50
#define IMU_ACC_ODR     BMI160_ACC_ODR_50HZ
#define IMU_GYR_ODR     BMI160_GYR_ODR_50HZ
#elif APP_IMU_ODR_HZ <= 100
#define IMU_ACC_ODR     BMI160_ACC_ODR_100HZ
#define IMU_GYR_ODR     BMI160_GYR_ODR_100HZ
#elif APP_IMU_ODR_HZ <= 200
#define IMU_ACC_ODR     BMI160_ACC_ODR_200HZ
#define IMU_GYR_ODR     BMI160_GYR_ODR_200HZ
#elif APP_IMU_ODR_HZ <= 400
#define IMU_ACC_ODR     BMI160_ACC_ODR_400HZ
#define IMU_GYR_ODR     BMI160_GYR_ODR_400HZ
#elif APP_IMU_ODR_HZ <= 800
#define IMU_ACC_ODR     BMI160_ACC_ODR_800HZ
#define IMU_GYR_ODR     BMI160_GYR_ODR_800HZ
#else
#define IMU_ACC_ODR     BMI160_ACC_ODR_1600HZ
#define IMU_GYR_ODR     BMI160_GYR_ODR_1600HZ
#endif

#if defined(CONFIG_APP_BMI160_ACC_AVG_1)
#define IMU_ACC_AVG     BMI160_ACC_LP_AVG_1
#elif defined(CONFIG_APP_BMI160_ACC_AVG_4)
#define IMU_ACC_AVG     BMI160_ACC_LP_AVG_4
#elif defined(CONFIG_APP_BMI160_ACC_AVG_8)
#define IMU_ACC_AVG     BMI160_ACC_LP_AVG_8
#elif defined(CONFIG_APP_BMI160_ACC_AVG_16)
#define IMU_ACC_AVG     BMI160_ACC_LP_AVG_16
#elif defined(CONFIG_APP_BMI160_ACC_AVG_32)
#define IMU_ACC_AVG     BMI160_ACC_LP_AVG_32
#elif defined(CONFIG_APP_BMI160_ACC_AVG_64)
#define IMU_ACC_AVG     BMI160_ACC_LP_AVG_64
#elif defined(CONFIG_APP_BMI160_ACC_AVG_128)
#define IMU_ACC_AVG     BMI160_ACC_LP_AVG_128
#else
#define IMU_ACC_AVG     BMI160_ACC_LP_AVG_2
#endif

/* ------------------------------------------------------------------ state */

static bmi160_t        s_dev;
static volatile bool   s_ready;
static QueueHandle_t   s_drdy_q;      /* int64_t timestamps from the INT1 ISR */

/*
 * If no data-ready edge arrives within this window, the INT1 line is treated as
 * unusable and the task falls back to polling, so a missing INT1 connection
 * costs timestamps but not the data itself.
 */
#define IMU_DRDY_TIMEOUT_MS  500

#if APP_BMI160_USE_INT1
static void IRAM_ATTR imu_drdy_isr(void *arg)
{
    const int64_t t = esp_timer_get_time();
    BaseType_t hp = pdFALSE;

    if (xQueueSendFromISR(s_drdy_q, &t, &hp) != pdTRUE) {
        /* Queue full: the sample is lost, but the count stays honest. */
        g_stats.imu_drops++;
    }
    if (hp) {
        portYIELD_FROM_ISR();
    }
}
#endif

/* -------------------------------------------------------------------- task */

static void imu_task(void *arg)
{
    (void)arg;

    const TickType_t period = pdMS_TO_TICKS(1000 / APP_IMU_ODR_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    bool use_int1 = APP_BMI160_USE_INT1;

    for (;;) {
        int64_t ts;

        if (use_int1) {
            if (xQueueReceive(s_drdy_q, &ts, pdMS_TO_TICKS(IMU_DRDY_TIMEOUT_MS)) != pdTRUE) {
                ESP_LOGW(TAG, "no INT1 data-ready for %d ms, falling back to polling; "
                         "the BMI160 side is configured correctly, so check the "
                         "INT1 -> GPIO%d connection (or set the INT1 option off)",
                         IMU_DRDY_TIMEOUT_MS, PIN_BMI160_INT1);
                use_int1 = false;
                last_wake = xTaskGetTickCount();
                continue;
            }
            /* If the task fell behind, only the newest pending edge describes
             * the data we are about to read; older edges are stale. */
            int64_t newer;
            while (xQueueReceive(s_drdy_q, &newer, 0) == pdTRUE) {
                ts = newer;
            }
        } else {
            vTaskDelayUntil(&last_wake, period ? period : 1);
            ts = esp_timer_get_time();
        }

        bmi160_result_t r;
        imu_sample_t s = {
            .t_us = ts,
            .valid = false,
        };

        if (bmi160_read_data(&s_dev, &r) == ESP_OK) {
            s.ax = r.accX;
            s.ay = r.accY;
            s.az = r.accZ;
            s.gx = r.gyroX;
            s.gy = r.gyroY;
            s.gz = r.gyroZ;
            s.valid = true;
        } else {
            g_stats.imu_errors++;
        }

        if (xQueueSend(q_imu, &s, 0) == pdPASS) {
            g_stats.imu_samples++;
        } else {
            g_stats.imu_drops++;
        }
    }
}

/* -------------------------------------------------------------------- init */

/*
 * Dump the registers that decide whether an interrupt can possibly appear.
 * PMU_STATUS says whether the accelerometer and gyroscope are actually running
 * (1 = normal), and STATUS exposes the data-ready flags. Together they separate
 * "the sensor is idle" from "the sensor is fine but INT1 is not connected".
 */
static void imu_dump_int_registers(void)
{
    uint8_t pmu = 0, out = 0, en1 = 0, map1 = 0, st = 0;

    bmi160_read_reg(&s_dev, BMI160_PMU_STATUS, &pmu);
    bmi160_read_reg(&s_dev, BMI160_INT_OUT_CTRL, &out);
    bmi160_read_reg(&s_dev, BMI160_INT_EN_1, &en1);
    bmi160_read_reg(&s_dev, BMI160_INT_MAP_1, &map1);
    bmi160_read_reg(&s_dev, BMI160_STATUS, &st);

    /* PMU_STATUS layout used by the driver: acc = bits[5:4], gyr = bits[3:2],
     * mag = bits[1:0]; 1 means normal for accel/gyro. */
    ESP_LOGI(TAG, "regs: PMU_STATUS=0x%02X (acc=%u gyr=%u mag=%u, 1=normal) "
             "INT_OUT_CTRL=0x%02X INT_EN_1=0x%02X INT_MAP_1=0x%02X "
             "STATUS=0x%02X (drdy_acc=%u drdy_gyr=%u)",
             pmu,
             (unsigned)((pmu & 0x30u) >> 4),
             (unsigned)((pmu & 0x0Cu) >> 2),
             (unsigned)(pmu & 0x03u),
             out, en1, map1, st,
             (unsigned)((st >> 7) & 1u), (unsigned)((st >> 6) & 1u));
}

/* Poll STATUS for a bounded time to confirm the sensor emits data-ready. */
static bool imu_wait_data_ready(uint32_t timeout_ms)
{
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while ((int32_t)(xTaskGetTickCount() - deadline) < 0) {
        uint8_t st = 0;
        if (bmi160_read_reg(&s_dev, BMI160_STATUS, &st) == ESP_OK && (st & 0xC0u)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

esp_err_t sensor_imu_init(void)
{
    esp_err_t err;

    err = bmi160_init(&s_dev, APP_BMI160_I2C_ADDR,
                      (i2c_port_t)APP_I2C_PORT_NUM, PIN_I2C_SDA, PIN_I2C_SCL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bmi160_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Run the whole bus at one predictable speed (QMC5883L tops out at
     * 400 kHz). i2cdev creates the per-device handle lazily, so setting this
     * before the first I2C transaction takes effect. */
    s_dev.i2c_dev.cfg.master.clk_speed = APP_I2C_FREQ_HZ;

    uint8_t id = 0;
    err = bmi160_read_reg(&s_dev, BMI160_CHIP_ID, &id);
    if (err != ESP_OK || id != BMI160_CHIP_ID_DEFAULT_VALUE) {
        ESP_LOGE(TAG, "BMI160 not found (chip id 0x%02X, err %s)",
                 id, esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BMI160 detected (chip id 0x%02X), I2C %d Hz", id, APP_I2C_FREQ_HZ);

    const bmi160_conf_t conf = {
        .accRange = IMU_ACC_RANGE,
        .accOdr   = IMU_ACC_ODR,
        .accMode  = BMI160_PMU_ACC_NORMAL,
        .accAvg   = IMU_ACC_AVG,
        .accUs    = BMI160_ACC_US_OFF,
        .gyrRange = IMU_GYR_RANGE,
        .gyrOdr   = IMU_GYR_ODR,
        .gyrMode  = BMI160_PMU_GYR_NORMAL,
    };
    err = bmi160_start(&s_dev, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bmi160_start failed: %s", esp_err_to_name(err));
        return err;
    }

#if APP_BMI160_USE_INT1
    s_drdy_q = xQueueCreate(8, sizeof(int64_t));
    if (!s_drdy_q) {
        ESP_LOGE(TAG, "failed to create data-ready queue");
        return ESP_ERR_NO_MEM;
    }

    /* The GPIO ISR service is installed once by app_main (it is shared with the
     * PPS input); here we only configure the pin and attach its handler. */
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_BMI160_INT1,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INT1 gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_isr_handler_add(PIN_BMI160_INT1, imu_drdy_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INT1 isr handler add failed: %s", esp_err_to_name(err));
        return err;
    }

    const bmi160_int_out_conf_t ic = {
        .intPin    = BMI160_PIN_INT1,
        .intEnable = BMI160_INT_ENABLE,
        .intOd     = BMI160_INT_PUSH_PULL,
        .intLevel  = BMI160_INT_ACTIVE_HIGH,
    };
    err = bmi160_enable_int_new_data(&s_dev, &ic);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "enable data-ready INT1 failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "data-ready routed to INT1 (GPIO%d)", PIN_BMI160_INT1);
    imu_dump_int_registers();

    if (imu_wait_data_ready(200)) {
        ESP_LOGI(TAG, "data-ready confirmed (polling fallback armed)");
    } else {
        ESP_LOGW(TAG, "BMI160 reports no data-ready in 200 ms; check the PMU_STATUS "
                 "and configuration shown above");
    }
#else
    ESP_LOGW(TAG, "INT1 data-ready disabled, polling at %d Hz", APP_IMU_ODR_HZ);
#endif

    s_ready = true;
    return ESP_OK;
}

void sensor_imu_start_task(void)
{
    if (!s_ready) {
        return;
    }
    xTaskCreate(imu_task, "imu", APP_STACK_IMU, NULL, APP_PRIO_IMU, NULL);
}
