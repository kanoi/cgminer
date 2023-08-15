#ifndef UART_UTILS_H
#define UART_UTILS_H
#include <stdbool.h>
#include <stdint.h>
#include <termios.h>
#include <errno.h>


#define UART_BUFFER_SIZE 128

typedef struct {
	char        *device;
	int         fd;

	uint8_t     mode;
	uint8_t     bits;
	uint32_t    speed;
	uint16_t    delay;

	uint16_t    datalen;
	uint16_t    size;
	uint8_t     *rx;
	uint8_t     *tx;
} device_t;

typedef enum {
	UART_CHANNEL1 = 1,
	UART_CHANNEL2
} uart_channel_id_t;

extern char *uart1_device_name;
extern char *uart2_device_name;

int8_t uart_init(device_t* attr,
		int8_t mode, uint32_t speed, uint16_t size);
int8_t uart_transfer(device_t *attr);
void uart_release(device_t *attr);

#endif