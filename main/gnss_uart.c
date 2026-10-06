/*
 * gnss_uart.c - GNSS acquisition, NMEA parsing and PPS <-> UTC pairing.
 *
 * Pairing rule
 * ------------
 * The GNSS PPS edge is asserted at the very top of a UTC second, while the
 * NMEA sentence describing that second is transmitted *after* the edge. So the
 * pending (most recent, not yet consumed) PPS edge is paired with the next
 * completed UTC epoch. Using the wrong direction here is the classic
 * off-by-one-second bug, so the age of the pairing is checked and reported.
 *
 * The status LED on the receiver usually indicates a fix; without a fix no RMC
 * epoch is produced, so after a quiet period a PPS-only record is emitted to
 * keep the timing information in the log.
 */
#include "gnss_uart.h"

#include <string.h>

#include "app_config.h"
#include "board_pins.h"
#include "data_types.h"
#include "gnss_nmea.h"
#include "log_format.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "gnss";

/* Emit a PPS-only record if no UTC epoch has been produced for this long. */
#define PPS_ONLY_TIMEOUT_US   (1500000)

/* Maximum plausible delay between a PPS edge and the sentence for that second. */
#define PPS_PAIR_MAX_AGE_US   (1500000)

static volatile bool s_ready;

/*
 * Receive-path diagnostics.
 *
 * A GPS module that is not wired, not powered, or transmitting at a different
 * baud rate than we listen at all produce the same status line (zero epochs), so
 * count the raw traffic and keep the first bytes of each reporting window. NMEA
 * shows up as printable '$G...' text; a baud mismatch shows up as mojibake, and
 * silence shows up as nothing at all.
 */
static uint32_t s_rx_window;
static uint32_t s_lines_window;
static uint32_t s_sent_window;
static uint8_t  s_dump[APP_GNSS_REPORT_DUMP_BYTES];
static size_t   s_dump_len;
static int64_t  s_last_report_us;

/* The first RMC of each window. Epochs are only produced by RMC, and RMC only
 * produces one when its time field parses, so keeping the raw sentence is what
 * says whether the receiver is sending no time (no fix) or something the parser
 * cannot read. */
static char     s_rmc[APP_GNSS_LINE_BYTES];
static bool     s_have_rmc;

/* ---------------------------------------------------------------- emitters */

static void enqueue_gnss(const gnss_sample_t *s)
{
    if (xQueueSend(q_gnss, s, 0) == pdPASS) {
        g_stats.gnss_samples++;
    } else {
        g_stats.gnss_drops++;
    }
}

static void emit_epoch(const gnss_nmea_t *st, const pps_event_t *pending,
                       bool have_pps, int64_t *last_emit_us)
{
    gnss_sample_t s;
    gnss_nmea_snapshot(st, &s);
    s.t_us = esp_timer_get_time();

    if (have_pps) {
        s.pps_t_us = pending->t_us;
        s.pps_seq  = pending->seq;

        const int64_t age = s.t_us - s.pps_t_us;
        if (age < 0 || age > PPS_PAIR_MAX_AGE_US) {
            ESP_LOGW(TAG, "PPS/UTC pairing suspicious: age %lld us",
                     (long long)age);
        }
    }

    enqueue_gnss(&s);
    app_state_set_gnss(&s);
    *last_emit_us = s.t_us;

    ESP_LOGD(TAG, "epoch %04u-%02u-%02u %02u:%02u:%02u.%03u fix=%u sats=%u pps=%u",
             (unsigned)s.year, (unsigned)s.month, (unsigned)s.day,
             (unsigned)s.hour, (unsigned)s.minute, (unsigned)s.second,
             (unsigned)s.millisecond, (unsigned)s.fix_type, (unsigned)s.sats,
             (unsigned)s.pps_seq);
}

/*
 * Timing-only record: the receiver is producing PPS but no usable UTC yet
 * (no fix, or the stream carries no RMC). It is logged but not published as
 * "the latest fix", so the status line keeps showing the last real position.
 */
static void emit_pps_only(const pps_event_t *pending, int64_t *last_emit_us)
{
    gnss_sample_t s;
    memset(&s, 0, sizeof(s));
    s.t_us     = esp_timer_get_time();
    s.pps_t_us = pending->t_us;
    s.pps_seq  = pending->seq;

    enqueue_gnss(&s);
    *last_emit_us = s.t_us;
}

/*
 * Status record: the receiver is alive and reporting, but has no UTC time yet
 * because it has no fix. That is what a receiver with no sky view looks like.
 *
 * Logging it matters for two reasons. Without it the GNSS file simply stays
 * empty until the first fix, so "has not acquired yet" and "the link is broken"
 * look identical in the data. And the satellite count is then recorded over
 * time, so moving the antenna somewhere useful is visible in the log itself.
 *
 * Unlike the PPS-only record this one *is* published as the latest GNSS state,
 * so the status line shows a live satellite count; utc_valid and pos_valid are
 * both forced to 0, which is how a consumer tells it from a fix.
 */
static void emit_status(const gnss_nmea_t *st, int64_t *last_emit_us)
{
    gnss_sample_t s;
    gnss_nmea_snapshot(st, &s);
    s.utc_valid = false;
    s.pos_valid = false;
    s.t_us = esp_timer_get_time();

    enqueue_gnss(&s);
    app_state_set_gnss(&s);
    *last_emit_us = s.t_us;
}

/* -------------------------------------------------------------------- task */

static void gnss_task(void *arg)
{
    (void)arg;

    gnss_nmea_t st;
    gnss_nmea_reset(&st);

    pps_stats_t pps = { 0 };
    unsigned intervals = 0;

    pps_event_t pending = { 0 };
    bool have_pending_pps = false;

    char line[APP_GNSS_LINE_BYTES];
    size_t len = 0;
    bool overflow = false;

    int64_t last_emit_us = esp_timer_get_time();

    for (;;) {
        /* 1. Drain PPS edges. Keeping only the newest unpaired edge is both
         *    correct (it belongs to the second we are about to parse) and
         *    self-healing if the task ever falls behind. */
        pps_event_t ev;
        while (xQueueReceive(q_pps, &ev, 0) == pdTRUE) {
            if (pps.last_t_us != 0) {
                const uint32_t dt = (uint32_t)(ev.t_us - pps.last_t_us);
                pps.last_interval_us = dt;
                if (intervals == 0) {
                    pps.min_interval_us = dt;
                    pps.max_interval_us = dt;
                } else {
                    if (dt < pps.min_interval_us) {
                        pps.min_interval_us = dt;
                    }
                    if (dt > pps.max_interval_us) {
                        pps.max_interval_us = dt;
                    }
                }
                intervals++;
            }
            pps.last_t_us = ev.t_us;
            pps.seq = ev.seq;
            pending = ev;
            have_pending_pps = true;
        }

        if (pps.seq != 0) {
            app_state_set_pps(&pps);
        }

        /* 2. Pump the UART and assemble lines. */
        uint8_t chunk[APP_GNSS_READ_CHUNK];
        const int rd = uart_read_bytes(APP_GNSS_UART_PORT, chunk, sizeof(chunk),
                                       pdMS_TO_TICKS(APP_GNSS_READ_TIMEOUT_MS));

        for (int i = 0; i < rd; i++) {
            const char c = (char)chunk[i];

            if (s_dump_len < sizeof(s_dump)) {
                s_dump[s_dump_len++] = chunk[i];
            }

            if (c == '\n') {
                if (!overflow && len > 0) {
                    line[len] = '\0';
                    s_lines_window++;
                    if (line[0] == '$') {
                        if (!s_have_rmc && strstr(line, "RMC") != NULL) {
                            snprintf(s_rmc, sizeof(s_rmc), "%s", line);
                            s_have_rmc = true;
                        }
                        const gnss_nmea_res_t res = gnss_nmea_feed(&st, line);
                        if (res != GNSS_NMEA_INVALID) {
                            s_sent_window++;
                        }
                        if (res == GNSS_NMEA_EPOCH) {
                            emit_epoch(&st, &pending, have_pending_pps,
                                       &last_emit_us);
                            have_pending_pps = false;
                        } else if (res == GNSS_NMEA_STATUS) {
                            emit_status(&st, &last_emit_us);
                        }
                    }
                }
                len = 0;
                overflow = false;
            } else if (c != '\r') {
                if (len + 1 < sizeof(line)) {
                    line[len++] = c;
                } else {
                    /* Overlong line: discard it and resynchronise on the next
                     * newline, then wait for the next '$'. */
                    overflow = true;
                    len = 0;
                }
            }
        }

        if (rd > 0) {
            s_rx_window += (uint32_t)rd;
        }

        /* 3. Without a fix there is no RMC epoch, so log the PPS timing on its
         *    own rather than losing it. */
        const int64_t now = esp_timer_get_time();
        if (have_pending_pps && (now - last_emit_us) > PPS_ONLY_TIMEOUT_US) {
            emit_pps_only(&pending, &last_emit_us);
            have_pending_pps = false;
        }

        /* 4. Receive-path report. */
        if ((now - s_last_report_us) > (int64_t)APP_GNSS_REPORT_MS * 1000) {
            s_last_report_us = now;

            if (s_rx_window == 0) {
                ESP_LOGW(TAG, "no data on UART%d for %d ms: check the GPS TX -> GPIO%d "
                         "wire, that the module is powered, and that %d baud matches it",
                         APP_GNSS_UART_PORT, APP_GNSS_REPORT_MS, PIN_GNSS_UART_RX,
                         APP_GNSS_BAUD);
            } else {
                ESP_LOGW(TAG, "rx=%u bytes, %u lines, %u sentences parsed in %d ms",
                         (unsigned)s_rx_window, (unsigned)s_lines_window,
                         (unsigned)s_sent_window, APP_GNSS_REPORT_MS);
                if (s_have_rmc) {
                    ESP_LOGW(TAG, "  RMC: %s", s_rmc);
                } else {
                    ESP_LOGW(TAG, "  no RMC sentence seen in this window; only RMC produces "
                             "a UTC epoch");
                }
                ESP_LOGW(TAG, "  parser: fix=%u mode=%u sats_used=%u sats_view=%u hdop=%.1f "
                         "utc=%d pos=%d",
                         (unsigned)st.fix_type, (unsigned)st.fix_mode,
                         (unsigned)st.sats_used, (unsigned)st.sats_view,
                         (double)st.hdop, (int)st.utc_valid, (int)st.pos_valid);
                /* The whole window's opening bytes, for when even the RMC line is
                 * not enough to tell what the receiver is sending. */
                ESP_LOG_BUFFER_CHAR_LEVEL(TAG, s_dump, s_dump_len, ESP_LOG_DEBUG);
            }

            s_rx_window = 0;
            s_lines_window = 0;
            s_sent_window = 0;
            s_dump_len = 0;
            s_have_rmc = false;
        }
    }
}

/* -------------------------------------------------------------------- init */

esp_err_t gnss_uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = APP_GNSS_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(APP_GNSS_UART_PORT, APP_GNSS_UART_RX_BUF,
                                        0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(APP_GNSS_UART_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(APP_GNSS_UART_PORT, PIN_GNSS_UART_TX, PIN_GNSS_UART_RX,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_flush_input(APP_GNSS_UART_PORT);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_flush_input failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "GNSS UART%d @ %d baud, ESP RX=GPIO%d TX=GPIO%d",
             APP_GNSS_UART_PORT, APP_GNSS_BAUD, PIN_GNSS_UART_RX, PIN_GNSS_UART_TX);

    s_ready = true;
    s_last_report_us = esp_timer_get_time();
    return ESP_OK;
}

void gnss_uart_start_task(void)
{
    if (!s_ready) {
        return;
    }
    xTaskCreate(gnss_task, "gnss", APP_STACK_GNSS, NULL, APP_PRIO_GNSS, NULL);
}
