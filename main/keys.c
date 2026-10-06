/*
 * keys.c - KEY1 / KEY2 debouncing.
 *
 * Both keys are active-low with the internal pull-up enabled (GPIO2 and GPIO3
 * are strapping pins and must read high at boot). The task only reports raw
 * presses; the logger task decides what a KEY1 press means, so that the
 * "logging started/stopped" rows stay consistent with the actual file state.
 */
#include "keys.h"

#include "app_config.h"
#include "board_pins.h"
#include "data_types.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "keys";

typedef struct {
    gpio_num_t   pin;
    log_source_t src;
    log_event_t  press_event;
    int          raw;            /* last raw level (1 = released, 0 = pressed) */
    int          stable;         /* debounced level */
    TickType_t   last_change;
} key_ctx_t;

static key_ctx_t s_keys[] = {
    { .pin = PIN_KEY1, .src = SRC_KEY1, .press_event = EVT_KEY1_PRESS },
    { .pin = PIN_KEY2, .src = SRC_KEY2, .press_event = EVT_KEY2_PRESS },
};

#define KEY_COUNT (sizeof(s_keys) / sizeof(s_keys[0]))

static void keys_task(void *arg)
{
    (void)arg;

    const TickType_t debounce = pdMS_TO_TICKS(APP_KEY_DEBOUNCE_MS);
    const TickType_t poll = pdMS_TO_TICKS(APP_KEY_POLL_MS);

    for (;;) {
        const TickType_t now = xTaskGetTickCount();

        for (size_t i = 0; i < KEY_COUNT; i++) {
            key_ctx_t *k = &s_keys[i];
            const int level = gpio_get_level(k->pin);

            if (level != k->raw) {
                k->raw = level;
                k->last_change = now;
                continue;
            }

            if (level != k->stable && (now - k->last_change) >= debounce) {
                k->stable = level;
                if (level == 0) {
                    ESP_LOGI(TAG, "KEY%d pressed", (int)(i + 1));
                    app_event_log(esp_timer_get_time(), k->src, k->press_event);
                }
            }
        }

        vTaskDelay(poll ? poll : 1);
    }
}

esp_err_t keys_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_KEY1) | (1ULL << PIN_KEY2),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }

    for (size_t i = 0; i < KEY_COUNT; i++) {
        s_keys[i].raw = gpio_get_level(s_keys[i].pin);
        s_keys[i].stable = s_keys[i].raw;
        s_keys[i].last_change = xTaskGetTickCount();
    }

    ESP_LOGI(TAG, "KEY1=GPIO%d (log start/stop), KEY2=GPIO%d (mark)",
             PIN_KEY1, PIN_KEY2);
    return ESP_OK;
}

void keys_start_task(void)
{
    xTaskCreate(keys_task, "keys", APP_STACK_KEYS, NULL, APP_PRIO_KEYS, NULL);
}
