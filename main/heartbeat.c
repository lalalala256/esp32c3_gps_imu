/*
 * heartbeat.c - one LED that reports the state of the acquisition pipeline.
 *
 * This exists because the interesting failure of this board is intermittent:
 * the TF card answers the raw SPI probe for a few seconds and then goes quiet
 * for minutes. Watching that on a serial console means leaving a terminal
 * attached, and the act of probing the socket can itself disturb a marginal
 * contact. A blink pattern on the on-board LED lets the board be left alone,
 * powered from any source, while the fault is chased.
 *
 * Patterns, repeating every HEARTBEAT_PERIOD_MS while unmounted:
 *
 *   solid on     card mounted and the CSV files are open
 *   2 blinks     the card answered the raw probe but the mount still failed
 *   1 blink      nothing on MISO at all (no card, no power, no wiring)
 *
 * GPIO8 doubles as a strapping pin and drives the on-board LED on most
 * ESP32-C3 SuperMini clones. It is only configured as an output here, long
 * after the strapping value has been latched at reset, so boot mode is
 * unaffected.
 */
#include "heartbeat.h"

#include "app_config.h"
#include "board_pins.h"
#include "logger_sd.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "led";

#define HEARTBEAT_PERIOD_MS 2000
#define LED_PULSE_MS        60
#define LED_GAP_MS          180
#define LED_SELFTEST_BLINKS 3

static void led_on(void)
{
    gpio_set_level(PIN_STATUS_LED, APP_LED_ACTIVE_LEVEL);
}

static void led_off(void)
{
    gpio_set_level(PIN_STATUS_LED, !APP_LED_ACTIVE_LEVEL);
}

static void blink(unsigned times)
{
    for (unsigned i = 0; i < times; i++) {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(LED_PULSE_MS));
        led_off();
        if (i + 1 < times) {
            vTaskDelay(pdMS_TO_TICKS(LED_GAP_MS));
        }
    }
}

esp_err_t heartbeat_init(void)
{
    if (!APP_LED_HEARTBEAT) {
        return ESP_OK;
    }

    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_STATUS_LED,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    const esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "status LED on GPIO%d unavailable: %s",
                 PIN_STATUS_LED, esp_err_to_name(err));
        return err;
    }

    led_off();

    ESP_LOGI(TAG, "status LED on GPIO%d (%s active): solid=card mounted, "
             "2 blinks=card answers but mount fails, 1 blink=no card",
             PIN_STATUS_LED, APP_LED_ACTIVE_LEVEL ? "high" : "low");

    return ESP_OK;
}

static void heartbeat_task(void *arg)
{
    (void)arg;

    /* Self test runs here rather than in heartbeat_init() so the ~0.7 s of
     * blinking does not delay the sensor bring-up in app_main. If the LED
     * stays dark, or comes on and never goes out, the active level is
     * inverted: flip APP_LED_ACTIVE_LEVEL. */
    blink(LED_SELFTEST_BLINKS);

    for (;;) {
        const TickType_t started = xTaskGetTickCount();

        logger_status_t st;
        logger_get_status(&st);

        if (st.sd_mounted) {
            led_on();
            vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
            continue;
        }

        if (st.probe_hits > 0) {
            /* The card is talking but still will not mount - the state worth
             * watching while chasing a marginal contact or a weak supply. */
            blink(2);
        } else {
            blink(1);
        }

        const TickType_t period = pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS);
        const TickType_t spent = xTaskGetTickCount() - started;
        if (spent < period) {
            vTaskDelay(period - spent);
        }
    }
}

void heartbeat_start_task(void)
{
    if (!APP_LED_HEARTBEAT) {
        return;
    }
    xTaskCreate(heartbeat_task, "heartbeat", APP_STACK_LED, NULL, APP_PRIO_LED, NULL);
}
