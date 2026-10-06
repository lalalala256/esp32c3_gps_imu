/*
 * app_main.c - startup sequence.
 *
 * Everything is created in dependency order:
 *   1. shared queues and mutexes
 *   2. I2C subsystem
 *   3. sensors (so a missing device is reported before any task starts)
 *   4. GNSS UART and PPS capture
 *   5. logger (opens the run files, then receives the queued boot events)
 *   6. tasks
 *
 * A missing sensor or a missing TF card is never fatal: the corresponding
 * source is skipped, the event is recorded, and the rest of the pipeline keeps
 * running. That matters in the field, where a loose card should not cost the
 * IMU data.
 */
#include <stdio.h>

#include "app_config.h"
#include "board_pins.h"
#include "data_types.h"
#include "gnss_uart.h"
#include "heartbeat.h"
#include "i2c_diag.h"
#include "keys.h"
#include "logger_sd.h"
#include "log_format.h"
#include "pps_capture.h"
#include "sensor_imu.h"
#include "sensor_mag.h"
#include "status.h"
#include "web_server.h"
#include "wifi_ap.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2cdev.h"

static const char *TAG = "app";

static void create_shared_objects(void)
{
    g_gnss_mutex = xSemaphoreCreateMutex();
    g_sensor_mutex = xSemaphoreCreateMutex();

    q_imu  = xQueueCreate(APP_QUEUE_LEN_IMU,  sizeof(imu_sample_t));
    q_mag  = xQueueCreate(APP_QUEUE_LEN_MAG,  sizeof(mag_sample_t));
    q_gnss = xQueueCreate(APP_QUEUE_LEN_GNSS, sizeof(gnss_sample_t));
    q_key  = xQueueCreate(APP_QUEUE_LEN_KEY,  sizeof(key_event_t));
    q_pps  = xQueueCreate(APP_QUEUE_LEN_PPS,  sizeof(pps_event_t));
}

static void log_banner(void)
{
    ESP_LOGI(TAG, "======================================================");
    ESP_LOGI(TAG, " ESP32-C3 GPS/IMU acquisition (INS preparation build)");
    ESP_LOGI(TAG, "   TF  SPI : SCK=%d MISO=%d MOSI=%d CS=%d",
             PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    ESP_LOGI(TAG, "   I2C     : SDA=%d SCL=%d @%d Hz (BMI160 0x%02X, QMC5883L 0x%02X)",
             PIN_I2C_SDA, PIN_I2C_SCL, APP_I2C_FREQ_HZ,
             APP_BMI160_I2C_ADDR, APP_QMC5883L_I2C_ADDR);
    ESP_LOGI(TAG, "   BMI160  : INT1=%d INT2=%d (data-ready routed to INT1)",
             PIN_BMI160_INT1, PIN_BMI160_INT2);
    ESP_LOGI(TAG, "   GNSS    : UART%d RX=%d TX=%d @%d baud, PPS=%d",
             APP_GNSS_UART_PORT, PIN_GNSS_UART_RX, PIN_GNSS_UART_TX,
             APP_GNSS_BAUD, PIN_GNSS_PPS);
    ESP_LOGI(TAG, "   Keys    : KEY1=%d (log start/stop) KEY2=%d (mark)",
             PIN_KEY1, PIN_KEY2);
    ESP_LOGI(TAG, "   Console : USB Serial/JTAG (UART0 pins left to the GNSS)");
    ESP_LOGI(TAG, "======================================================");
}

/*
 * Probe every 7-bit I2C address and report what answers, with a register dump
 * for each so that a clone module can be identified. This is the fastest way to
 * tell apart a missing magnetometer, a mis-wired one, and a board sold as a
 * QMC5883L that is really an HMC5883L (0x1E) or a QMC5883P (0x2C).
 *
 * The full report goes to the console once at boot; /i2c serves the same report
 * on demand, which is what makes rewiring testable without a reflash.
 */
static unsigned i2c_bus_scan(void)
{
    static char report[1400];

    const unsigned count = i2c_diag_report(report, sizeof(report));
    ESP_LOGI(TAG, "I2C scan report:\n%s", report);
    return count;
}

void app_main(void)
{
    create_shared_objects();
    if (!g_gnss_mutex || !g_sensor_mutex || !q_imu || !q_mag || !q_gnss || !q_key || !q_pps) {
        ESP_LOGE(TAG, "shared queue/mutex allocation failed, aborting");
        return;
    }

    log_banner();

    /*
     * Runs before i2cdev touches anything, while SDA/SCL are still ordinary
     * GPIOs: this is the only moment the lines can be inspected without the I2C
     * peripheral driving them, and the only moment a stuck bus can be made
     * healthy again by hand. It is not fatal either way - the sensors that do
     * work keep working.
     */
    i2c_diag_lines_pre_init();

    esp_err_t err = i2cdev_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        /* Not fatal on its own: each sensor init reports its own failure. */
        ESP_LOGE(TAG, "i2cdev_init failed: %s", esp_err_to_name(err));
    }

    /* The GPIO ISR service is process-wide, so it is installed once here before
     * any driver attaches a handler. Both the BMI160 data-ready line and the
     * GNSS PPS input need it; installing it twice just logs a spurious error. */
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
    }

    /* Just configures the GPIO; the self-test blinks run later in the LED task
     * so they cannot delay the sensor bring-up below. */
    heartbeat_init();

    const bool imu_ok = (sensor_imu_init() == ESP_OK);
    const bool mag_ok = (sensor_mag_init() == ESP_OK);

    /* Registering an I2C device is what creates the bus, so the scan runs after
     * the sensors were probed and reports what is actually on the bus. */
    const unsigned i2c_devices = i2c_bus_scan();

    if (!imu_ok) {
        ESP_LOGE(TAG, "BMI160 unavailable, IMU stream disabled");
        app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SENSOR_MISSING);
    }
    if (!mag_ok) {
        if (i2c_devices <= 1) {
            ESP_LOGE(TAG, "QMC5883L did not answer at ANY I2C address, so this is not "
                     "an address mismatch: check its VCC/GND/SDA/SCL and pull-ups");
        } else {
            ESP_LOGE(TAG, "QMC5883L did not answer at 0x%02X while other devices did; "
                     "the module may be a clone (HMC5883L 0x1E, QMC5883P 0x2C)",
                     APP_QMC5883L_I2C_ADDR);
        }
        app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SENSOR_MISSING);
    }

    const bool gnss_ok = (gnss_uart_init() == ESP_OK);
    if (!gnss_ok) {
        ESP_LOGE(TAG, "GNSS UART unavailable, GNSS stream disabled");
        app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SENSOR_MISSING);
    }

    const bool pps_ok = (pps_capture_init() == ESP_OK);
    if (!pps_ok) {
        ESP_LOGE(TAG, "PPS capture unavailable");
    }
    if (gnss_ok && !pps_ok) {
        ESP_LOGW(TAG, "GNSS without PPS: UTC epochs will be logged unpaired");
    }

    if (keys_init() != ESP_OK) {
        ESP_LOGW(TAG, "keys unavailable, logging will stay at its boot state");
    }

    status_init();
    logger_init();

    /* Producers, then the logger, then the reporter. */
    if (imu_ok) {
        sensor_imu_start_task();
    }
    if (mag_ok) {
        sensor_mag_start_task();
    }
    if (gnss_ok) {
        gnss_uart_start_task();
    }
    keys_start_task();
    logger_start_task();
    status_start_task();
    heartbeat_start_task();

#if APP_WIFI_ENABLE
    /* Started last on purpose: the acquisition pipeline is already running, and
     * WiFi bring-up costs a second or two plus a sizeable slice of RAM. */
    if (wifi_ap_start() == ESP_OK) {
        ESP_LOGI(TAG, "browse the TF card at http://%s/", wifi_ap_ip_str());
        if (web_server_start() != ESP_OK) {
            ESP_LOGE(TAG, "web file browser failed to start");
        }
    } else {
        ESP_LOGE(TAG, "WiFi access point failed to start");
    }
#else
    ESP_LOGI(TAG, "WiFi access point and web UI disabled in menuconfig");
#endif

    ESP_LOGI(TAG, "acquisition running");
}
