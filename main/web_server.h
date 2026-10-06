/*
 * web_server.h - HTTP file browser for the TF card.
 */
#pragma once

#include "esp_err.h"

/*
 * Start the HTTP server. Requires a network interface to already be up
 * (call after wifi_ap_start()).
 */
esp_err_t web_server_start(void);
