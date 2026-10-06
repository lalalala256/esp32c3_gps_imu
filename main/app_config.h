/*
 * app_config.h - maps Kconfig options onto typed constants and collects the
 * fixed tuning constants of the acquisition pipeline.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "driver/uart.h"

/* ---------------------------------------------------------------- helpers */
#ifdef CONFIG_APP_LOG_TO_SD
#define APP_LOG_TO_SD               1
#else
#define APP_LOG_TO_SD               0
#endif

#ifdef CONFIG_APP_LOG_SERIAL_MIRROR
#define APP_SERIAL_MIRROR           1
#else
#define APP_SERIAL_MIRROR           0
#endif

#ifdef CONFIG_APP_LOG_AUTOSTART
#define APP_LOG_AUTOSTART           1
#else
#define APP_LOG_AUTOSTART           0
#endif

#ifdef CONFIG_APP_BMI160_USE_INT1
#define APP_BMI160_USE_INT1         1
#else
#define APP_BMI160_USE_INT1         0
#endif

#ifdef CONFIG_APP_BMI160_I2C_ADDR_VDD
#define APP_BMI160_I2C_ADDR         0x69
#else
#define APP_BMI160_I2C_ADDR         0x68
#endif

/* ------------------------------------------------------------------- I2C */
#define APP_I2C_PORT_NUM            0                       /* I2C_NUM_0 */
#define APP_I2C_FREQ_HZ             CONFIG_APP_I2C_FREQ_HZ
#define APP_QMC5883L_I2C_ADDR       0x0D
#define APP_I2C_SCAN_TIMEOUT_MS     50

/* -------------------------------------------------------------- FreeRTOS */
#define APP_PRIO_IMU                10
#define APP_PRIO_GNSS               8
#define APP_PRIO_LOGGER             6
#define APP_PRIO_MAG                5
#define APP_PRIO_KEYS               4
#define APP_PRIO_STATUS             2
#define APP_PRIO_LED                1

#define APP_STACK_IMU               3072
#define APP_STACK_GNSS              4096
#define APP_STACK_LOGGER            4096
#define APP_STACK_MAG               2560
#define APP_STACK_KEYS              2560
#define APP_STACK_STATUS            3072
#define APP_STACK_LED               2560

/* ----------------------------------------------------------- sensor rates */
#define APP_IMU_ODR_HZ              CONFIG_APP_BMI160_ODR_HZ
#define APP_MAG_RATE_HZ             CONFIG_APP_MAG_RATE_HZ

/* ---------------------------------------------------------------- queues */
#define APP_QUEUE_LEN_IMU           256
#define APP_QUEUE_LEN_MAG           128
#define APP_QUEUE_LEN_GNSS          32
#define APP_QUEUE_LEN_KEY           16
#define APP_QUEUE_LEN_PPS           16

/*
 * The logger drains the queues by polling at this interval rather than using a
 * FreeRTOS queue set: xQueueAddToSet() refuses a queue that is not empty, which
 * would make the logger's startup dependent on whether any event happened to be
 * queued before it started.
 */
#define APP_LOG_POLL_MS             2

/* --------------------------------------------------------------- logging */
#define APP_LOG_MOUNT_POINT         CONFIG_APP_SD_MOUNT_POINT
#define APP_LOG_FLUSH_MS            CONFIG_APP_LOG_FLUSH_MS
#define APP_LOG_FILE_BUF_BYTES      (16 * 1024)
#define APP_LOG_LINE_BYTES          192
#define APP_LOG_MAX_CONSEC_ERR      5
#define APP_LOG_RUN_INDEX_MAX       9999
#define APP_LOG_BATCH_FLUSH_BYTES   (32 * 1024)

#define APP_SD_SPI_HOST             SPI2_HOST
#define APP_SD_SPI_FREQ_KHZ         CONFIG_APP_SD_SPI_FREQ_KHZ
#define APP_SD_MAX_TRANSFER_SZ      4000
#define APP_SD_RETRY_MS             CONFIG_APP_SD_RETRY_MS

#define APP_SERIAL_MIRROR_DECIM     CONFIG_APP_SERIAL_MIRROR_DECIM

/* ------------------------------------------------------------------ GNSS */
#if CONFIG_APP_GNSS_UART_PORT == 0
#define APP_GNSS_UART_PORT          UART_NUM_0
#else
#define APP_GNSS_UART_PORT          UART_NUM_1
#endif
#define APP_GNSS_BAUD               CONFIG_APP_GNSS_BAUD
#define APP_GNSS_UART_RX_BUF        2048
#define APP_GNSS_LINE_BYTES         128
#define APP_GNSS_READ_CHUNK         64
#define APP_GNSS_READ_TIMEOUT_MS    20

/*
 * How often the GNSS task reports on raw UART traffic. Without it a silent wire
 * and a wire carrying bytes the parser rejects are indistinguishable: both show
 * up as zero epochs in the status line.
 */
#define APP_GNSS_REPORT_MS          5000
#define APP_GNSS_REPORT_DUMP_BYTES  48

/* ------------------------------------------------------------------ keys */
#define APP_KEY_DEBOUNCE_MS         CONFIG_APP_KEY_DEBOUNCE_MS
#define APP_KEY_POLL_MS             10

/* --------------------------------------------------------------- status */
#define APP_STATUS_PERIOD_S         CONFIG_APP_STATUS_PERIOD_S

/* ------------------------------------------------------------ status LED */
#ifdef CONFIG_APP_LED_HEARTBEAT
#define APP_LED_HEARTBEAT           1
#else
#define APP_LED_HEARTBEAT           0
#endif

/* Level that lights the LED. Measured active high on this board: with the
 * active-low assumption the LED sat lit while the card was *not* mounted, which
 * is only possible if driving the pin high lights it. */
#ifdef CONFIG_APP_LED_ACTIVE_LOW
#define APP_LED_ACTIVE_LEVEL        0
#else
#define APP_LED_ACTIVE_LEVEL        1
#endif

/* -------------------------------------------------------- WiFi + web UI */
#ifdef CONFIG_APP_WIFI_ENABLE
#define APP_WIFI_ENABLE             1
#else
#define APP_WIFI_ENABLE             0
#endif

#define APP_WIFI_SSID               CONFIG_APP_WIFI_SSID
#define APP_WIFI_PASSWORD           CONFIG_APP_WIFI_PASSWORD
#define APP_WIFI_CHANNEL            CONFIG_APP_WIFI_CHANNEL
#define APP_WIFI_MAX_STA            CONFIG_APP_WIFI_MAX_STA
#define APP_WIFI_TX_POWER_DBM       CONFIG_APP_WIFI_TX_POWER_DBM

#define APP_WEB_PORT                CONFIG_APP_WEB_PORT
#define APP_WEB_PREVIEW_LINES       CONFIG_APP_WEB_PREVIEW_LINES
