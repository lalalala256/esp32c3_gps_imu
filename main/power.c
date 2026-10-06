/*
 * power.c - sleep mode: hold KEY1 to sleep, press either key to wake.
 *
 * Deep sleep, not light sleep. On this chip the difference is three orders of
 * magnitude of current (~5 uA against roughly 200 uA before a Wi-Fi connection
 * is accounted for), and light sleep would not actually be simpler here: it
 * suspends the CPU with the SD card's SPI transactions and the Wi-Fi connection
 * frozen mid-flight, so it needs the same teardown as deep sleep to be safe,
 * and then needs all of it put back together afterwards.
 *
 * The price of deep sleep is that waking is a full reset: the wake source has
 * to be armed before sleeping, and on the way out the board starts from
 * scratch. Everything below exists to make both ends of that clean - the card
 * is flushed, closed and unmounted on the way in, and the wake cause is
 * reported on the way out.
 *
 * The wake trigger is a level, not an edge, so a key that is still held down
 * when sleep is entered wakes the chip immediately. Holding KEY1 in particular
 * is worth avoiding across the wake reset: GPIO2 is a strapping pin, and
 * although the C3 boot mode is decided by GPIO9 (high, internal pull-up) and
 * GPIO8 is ignored unless GPIO9 is low, there is nothing to gain by finding out
 * what the ROM does with a strapping pin held during reset.
 */
#include "power.h"

#include "app_config.h"
#include "board_pins.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heartbeat.h"
#include "logger_sd.h"
#include "soc/soc_caps.h"
#include "wifi_ap.h"

static const char *TAG = "power";

/* Either key wakes the board. Both are inside the C3's usable set (GPIO0..5),
 * which is checked in power_init() rather than assumed. */
#define WAKE_PIN_MASK   ((1ULL << PIN_KEY1) | (1ULL << PIN_KEY2))

static TaskHandle_t s_task;
static bool         s_ready;

/* ---------------------------------------------------------------- shutdown */

/*
 * Wait for the key that asked for sleep to be released.
 *
 * Not cosmetic: with a low-level trigger, sleeping while the key is still down
 * means waking up again immediately, which would look like "the sleep button
 * does nothing".
 */
static void wait_for_release(void)
{
    const TickType_t deadline =
        xTaskGetTickCount() + pdMS_TO_TICKS(APP_SLEEP_RELEASE_MS);

    while (!gpio_get_level(PIN_KEY1) || !gpio_get_level(PIN_KEY2)) {
        if ((int32_t)(xTaskGetTickCount() - deadline) >= 0) {
            ESP_LOGW(TAG, "%s still held after %d ms, sleeping anyway",
                     gpio_get_level(PIN_KEY1) ? "KEY2" : "KEY1",
                     APP_SLEEP_RELEASE_MS);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void enter_sleep(void)
{
    if (!s_ready) {
        ESP_LOGW(TAG, "sleep is unavailable on this chip, staying awake");
        return;
    }

    ESP_LOGW(TAG, "sleep requested: closing the log, then deep sleep "
             "(press KEY1 or KEY2 to wake)");

    /*
     * Order matters. The card goes first, so nothing is writing when the radio
     * comes down; then the radio, which is pointless while asleep and would
     * otherwise keep taking turns on the bus; then the LED, which would
     * otherwise stay lit for as long as the chip sleeps.
     */
    logger_shutdown_for_sleep();
    wifi_ap_stop();
    heartbeat_off();

    wait_for_release();

    const esp_err_t err =
        esp_deep_sleep_enable_gpio_wakeup(WAKE_PIN_MASK, ESP_GPIO_WAKEUP_GPIO_LOW);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cannot arm the wake sources (%s); staying awake so the "
                 "board can still be reached", esp_err_to_name(err));
        return;
    }

    ESP_LOGW(TAG, "entering deep sleep now");
    esp_deep_sleep_start();
    /* Not reached. */
}

static void power_task(void *arg)
{
    (void)arg;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        enter_sleep();

        /* Only reached if arming the wake source failed: stay usable and wait
         * for the next request instead of sleeping with no way out. */
        ESP_LOGW(TAG, "sleep aborted, still awake");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* -------------------------------------------------------------------- API */

esp_err_t power_init(void)
{
    const uint64_t valid = SOC_GPIO_DEEP_SLEEP_WAKE_VALID_GPIO_MASK;

    if ((WAKE_PIN_MASK & valid) != WAKE_PIN_MASK) {
        ESP_LOGE(TAG, "KEY1=GPIO%d and KEY2=GPIO%d are not both valid deep-sleep "
                 "wake pins on this chip (valid mask 0x%llX): sleep disabled",
                 PIN_KEY1, PIN_KEY2, (unsigned long long)valid);
        return ESP_ERR_NOT_SUPPORTED;
    }

    s_ready = true;
    ESP_LOGI(TAG, "sleep ready: hold KEY1 (GPIO%d) for %d ms to sleep, press "
             "KEY1 or KEY2 to wake", PIN_KEY1, APP_KEY_LONGPRESS_MS);
    return ESP_OK;
}

void power_start_task(void)
{
    if (!s_ready) {
        return;
    }
    xTaskCreate(power_task, "power", APP_STACK_POWER, NULL, APP_PRIO_POWER, &s_task);
}

void power_request_sleep(void)
{
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

void power_log_wakeup_cause(void)
{
    switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_GPIO:
        ESP_LOGW(TAG, "woke from deep sleep: a key was pressed");
        break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        ESP_LOGI(TAG, "cold boot (not a wake from deep sleep)");
        break;
    default:
        ESP_LOGW(TAG, "woke from deep sleep, cause %d",
                 (int)esp_sleep_get_wakeup_cause());
        break;
    }
}
