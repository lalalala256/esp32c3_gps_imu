/*
 * board_pins.h - physical wiring of the ESP32-C3 SuperMini carrier.
 *
 * All pins are fixed by the hardware and are therefore compile-time constants
 * rather than Kconfig options.
 *
 * Board cautions (ESP32-C3):
 *   - GPIO4..GPIO7 double as MTMS/MTDI/MTCK/MTDO. Using them for SPI means
 *     pin-JTAG is unavailable; USB-Serial/JTAG on GPIO18/19 still works.
 *   - GPIO2, GPIO8 and GPIO9 are strapping pins. They must be HIGH at boot,
 *     which is why the keys use pull-ups and the I2C bus has pull-ups.
 *   - GPIO8 drives the on-board addressable RGB LED on many SuperMini clones.
 *   - GPIO20/GPIO21 are UART0's default pins; the console is moved to
 *     USB-Serial/JTAG so the GNSS module can own them.
 *   - GPIO0/GPIO1 are XTAL_32K_P/N; the RTC keeps using the internal RC clock.
 */
#pragma once

#include "driver/gpio.h"

/* ---- TF card (SPI2, routed through the GPIO matrix) --------------------- */
#define PIN_SD_SCK          GPIO_NUM_4
#define PIN_SD_MISO         GPIO_NUM_5
#define PIN_SD_MOSI         GPIO_NUM_6
#define PIN_SD_CS           GPIO_NUM_7

/* ---- I2C bus: BMI160 + QMC5883L share I2C_NUM_0 ------------------------ */
#define PIN_I2C_SDA         GPIO_NUM_8
#define PIN_I2C_SCL         GPIO_NUM_9

/* ---- BMI160 interrupt lines ------------------------------------------- */
#define PIN_BMI160_INT1     GPIO_NUM_0
#define PIN_BMI160_INT2     GPIO_NUM_1

/* ---- Keys (active low, internal pull-up) ------------------------------ */
#define PIN_KEY1            GPIO_NUM_2
#define PIN_KEY2            GPIO_NUM_3

/* ---- Status LED --------------------------------------------------------
 * GPIO8 is a strapping pin (it must read HIGH at boot) and drives the
 * on-board LED on most SuperMini clones. It is only reconfigured as an
 * output after boot, so the strapping value has already been latched. */
#define PIN_STATUS_LED      GPIO_NUM_8

/* ---- GNSS ------------------------------------------------------------- */
#define PIN_GNSS_PPS        GPIO_NUM_10
#define PIN_GNSS_UART_TX    GPIO_NUM_21   /* ESP32 TX -> GNSS RX */
#define PIN_GNSS_UART_RX    GPIO_NUM_20   /* GNSS TX  -> ESP32 RX */
