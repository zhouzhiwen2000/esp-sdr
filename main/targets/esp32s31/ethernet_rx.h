#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
typedef void (*s31_net_reply_fn)(const char *fmt, ...);
esp_err_t s31_net_init(void);
void s31_net_poll(void (*command)(const char *), bool serial_available);
bool s31_net_reply(const void *data, size_t size);
bool s31_net_command(const char *line, unsigned frequency_mhz, s31_net_reply_fn reply);
bool s31_net_active(void);
