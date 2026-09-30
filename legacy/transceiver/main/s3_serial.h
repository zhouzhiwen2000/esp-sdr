#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef enum { S3_SERIAL_USB, S3_SERIAL_UART, S3_SERIAL_COUNT } s3_serial_port_t;

void s3_serial_init(void);
/* 1: complete line, -1: overlong line, 0: idle. Each port has its own parser.
 * A complete line selects the port for all subsequent replies and payloads. */
int s3_serial_poll_line(char *line, size_t capacity);
s3_serial_port_t s3_serial_port(void);
unsigned s3_serial_baud(void);
bool s3_serial_send(const void *data, size_t size);
bool s3_serial_receive(void *data, size_t size);
