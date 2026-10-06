/*
 * heartbeat.h - console-free state indication on the on-board LED.
 */
#pragma once

#include "esp_err.h"

/*
 * Configure the LED GPIO and run a short self test (three quick blinks).
 * Safe to call when the feature is disabled; it then does nothing.
 */
esp_err_t heartbeat_init(void);

/* Create the blinking task (call after logger_init). */
void heartbeat_start_task(void);
