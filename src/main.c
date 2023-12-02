#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

/**
 * This program reads data over uart1, then prints the receved data along with debug info
 * to uart0 with.
 */

const struct device *gps_uart = DEVICE_DT_GET(DT_NODELABEL(uart1));  // uart1: TX = P0.04, RX = P0.05.    TO BE USED FOR UART COMMS WITH GPS

#define BUFF_SIZE 50  // Note: UART_RX_RDY event only occurs when RX buffer is full.
static char* rx_buf;  // Buffer that is used internally by UART callback


void handle_uart_rx_data(struct uart_event *evt) {
	/**
	 * Parse string received over UART and store important info. 
  	 */
	for (int i=0; i < evt->data.rx.len; i++) {
		printk("%c", evt->data.rx.buf[evt->data.rx.offset + i]);
	}
}


static void gps_uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {

	int ret;

	switch (evt->type) {
		case UART_RX_RDY:
			handle_uart_rx_data(evt);
			break;
		case UART_RX_BUF_REQUEST:
			rx_buf = k_malloc(BUFF_SIZE * sizeof(uint8_t));
			if (rx_buf) {
				ret = uart_rx_buf_rsp(gps_uart, rx_buf, BUFF_SIZE);
				if (ret) { 
					printk("uart_rx_buf_rsp with ret=%d\n", ret);
				}
			} else {
				printk("Not able to allocate UART receive buffer.");
			}
			break;
		case UART_RX_BUF_RELEASED:
			k_free(rx_buf);
			break;
		case UART_RX_DISABLED:
			ret = uart_rx_enable(gps_uart, rx_buf, BUFF_SIZE, SYS_FOREVER_US);
			if (ret) { 
				printk("uart_rx_enable with ret=%d\n", ret);
			}
			break;
		default:
			break;
	}
}


int main(void) {

	k_msleep(1000);
	int ret;

	/////////////////////////////////// GPS UART SETUP ///////////////////////////////////
	if (!device_is_ready(gps_uart)) {
		printk("uart not ready. returning.\n");
		return -1;
	}

	ret = uart_callback_set(gps_uart, gps_uart_cb, NULL);
	if (ret) {
		printk("uart_callback_set failed. returning.\n");
		return ret;
	}

	k_msleep(1000);
	ret = uart_rx_enable(gps_uart, rx_buf, BUFF_SIZE, SYS_FOREVER_US);
	if (ret) {
		printk("uart_rx_enable failed with ret=%d. returning.\n", ret);
		return ret;
	}

	/////////////////////////////////// MAIN LOOP ///////////////////////////////////
	int ctr = 0;
	while (1) {
		ctr++;
		k_msleep(500);
	}

	return 0;
}
