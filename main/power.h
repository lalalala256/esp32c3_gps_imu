/*
 * power.h - sleep mode.
 *
 * Hold KEY1 to put the board into deep sleep; press either key to wake it.
 */
#pragma once

#include "esp_err.h"

/*
 * Validate that this chip can actually wake on both keys before anything
 * offers to sleep. Fails (and leaves sleep unusable) rather than letting the
 * board go to sleep with no way back.
 */
esp_err_t power_init(void);

/* Create the task that performs the shutdown. Call after logger_init(). */
void power_start_task(void);

/*
 * Ask for sleep. Returns immediately - the shutdown itself runs in the power
 * task, which is the only place that closes the card and the radio, so a key
 * handler is never blocked by a slow SD write.
 *
 * Safe to call when sleep is unavailable: it then does nothing.
 */
void power_request_sleep(void);

/* Log why the chip started (cold boot vs wake from sleep). Call from app_main. */
void power_log_wakeup_cause(void);
