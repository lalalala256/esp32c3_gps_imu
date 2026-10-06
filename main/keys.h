/*
 * keys.h - KEY1 / KEY2 debounced input.
 */
#pragma once

#include "esp_err.h"

/* Configure both keys as pulled-up inputs. */
esp_err_t keys_init(void);

/* Create the debounce task. */
void keys_start_task(void);
