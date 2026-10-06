/*
 * gnss_uart.h - GNSS receiver serial interface and epoch assembly.
 */
#pragma once

#include "esp_err.h"

/*
 * Install the GNSS UART (baud rate and port from menuconfig, pins GPIO20/21)
 * and clear any stale input.
 */
esp_err_t gnss_uart_init(void);

/*
 * Create the GNSS task. It pumps the UART, parses NMEA, drains the PPS queue,
 * performs the PPS <-> UTC pairing and emits one gnss_sample_t per epoch.
 */
void gnss_uart_start_task(void);
