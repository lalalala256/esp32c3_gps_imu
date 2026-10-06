/*
 * pps_capture.c - captures the GNSS pulse-per-second edge.
 *
 * The ISR does the absolute minimum: it reads the microsecond timer and pushes
 * the timestamp plus a monotonically increasing sequence number onto q_pps.
 * All statistics and the PPS <-> UTC pairing are done in task context by the
 * GNSS task, which is the only consumer of q_pps.
 */
#include "pps_capture.h"

#include "app_config.h"
#include "board_pins.h"
#include "data_types.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "pps";

/* Only ever written from the ISR. */
static volatile uint32_t s_seq;

static void IRAM_ATTR pps_isr(void *arg)
{
    (void)arg;

    pps_event_t e = {
        .t_us = esp_timer_get_time(),
        .seq  = ++s_seq,
    };

    BaseType_t hp = pdFALSE;
    /* A full queue only means a lost event; the sequence number still shows the
     * gap, which is exactly what the status output reports. */
    xQueueSendFromISR(q_pps, &e, &hp);
    g_stats.pps_edges++;

    if (hp) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t pps_capture_init(void)
{
    /* The GPIO ISR service is installed once by app_main and shared with the
     * BMI160 data-ready input. */
    esp_err_t err;

    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_GNSS_PPS,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };

    err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PPS gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(PIN_GNSS_PPS, pps_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PPS isr handler add failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "PPS capture on GPIO%d (rising edge)", PIN_GNSS_PPS);
    return ESP_OK;
}
