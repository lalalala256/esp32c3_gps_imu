/*
 * i2c_diag.h - bring-up diagnostics for the shared I2C bus.
 *
 * Why this exists: the BMI160 and the magnetometer share one bus, and the
 * acquisition code can only ever report "found" or "not found" for one
 * address. When the magnetometer does not answer that leaves the interesting
 * question open - is the module absent, mis-wired, powered off, dead, or simply
 * answering somewhere else? These two entry points are what turns "not found"
 * into a statement about where the fault actually lives.
 */
#pragma once

#include <stddef.h>

#include "esp_err.h"

/*
 * Run BEFORE any i2cdev / i2c_master_bus creation, while SDA and SCL are still
 * plain GPIOs.
 *
 * It reports whether both lines idle high - which is the only way to check the
 * pull-ups, look for a short, and find a slave that is holding the bus - and if
 * SDA is stuck it clocks the bus free.
 *
 * Returns ESP_OK when the bus looks usable, ESP_ERR_INVALID_STATE when a line
 * is being held low by something that is not going to let go.
 */
esp_err_t i2c_diag_lines_pre_init(void);

/*
 * Multi-line, human-readable scan report: every address that answers, a
 * register dump for each, and a best-effort identification.
 *
 * Safe to call at any time after the bus exists, including concurrently with
 * the sensor tasks: the i2c_master driver serialises transactions on the bus
 * with an internal mutex. The pad input path also stays readable once the I2C
 * peripheral owns the pins, so the line check still works at run time.
 *
 * `out` is always NUL-terminated. Returns the number of devices that answered.
 */
unsigned i2c_diag_report(char *out, size_t out_len);
