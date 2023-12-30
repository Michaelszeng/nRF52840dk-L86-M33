#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

/**
 * This program reads data over uart1, then prints the receved data along with debug info
 * to uart0.
 */

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define DEBUG_PRINT_ENABLED 1

const struct device *gps_uart = DEVICE_DT_GET(DT_NODELABEL(uart1));  // uart1: TX = P0.04, RX = P0.05.    TO BE USED FOR UART COMMS WITH GPS
const struct device *dbg_uart = DEVICE_DT_GET(DT_NODELABEL(uart0));  // uart0: TX = P0.28, RX = P0.30.    TO BE USED FOR UART DEBUGGING

// Buffer size that is large enough to fully capture a single NMEA sentence without causing printing to drop messages
#define BUFF_SIZE 75  // Note: UART_RX_RDY event only occurs when RX buffer is full.
static char* rx_buf;  // Buffer that is used internally by UART callback

// Buffer to hold newest NMEA data
static char combined_nmea_output[2*BUFF_SIZE] = {0};
static char prev_uart_str[BUFF_SIZE] = {0};

uint8_t new_data_flag = 0;


static void dbg_print(char* buf, int len) {
	/**
	 * Helper function to print things over debug uart
 	 */
	if (DEBUG_PRINT_ENABLED) {
		uart_tx(dbg_uart, buf, len, SYS_FOREVER_US);
	}
}


void handle_uart_rx_data(struct uart_event *evt) {
	/**
	 * Parse string received over UART and store important info. 
	 * 
	 * Called every time rx_buf is full
  	 */

	// Merge this UART buffer with the last one so we don't miss messages that might be split between the two
	memcpy(combined_nmea_output, prev_uart_str, BUFF_SIZE);
	memcpy(combined_nmea_output+BUFF_SIZE, &(evt->data.rx.buf[evt->data.rx.offset]), BUFF_SIZE);

	// Printing
	printk("\n==============================\n");
	combined_nmea_output[2*BUFF_SIZE-1] = '\0';
	printk("%s", combined_nmea_output);
	printk("\n==============================\n\n");

	new_data_flag = 1;  // set flag to 1 to indicate to other threads that new data is ready

	// // Search for $GPRMC
	// char* GGA_msg_start = strstr(combined_nmea_output, "$GPRMC");
	// char* GGA_msg_end = strchr(GGA_msg_start+1, '\n');
	
	// if (GGA_msg_start != NULL && GGA_msg_end !=NULL) {
	// 	// combined_nmea_output contains a full GPRMC message
	// }

	// 	char type[7];
	// 	int time;
	// 	float latitude, longitude, altitude;
	// 	char latDirection, longDirection, altitudeUnit;
	// 	int fixQuality, numSatellites;
	// 	float HDOP;

	// 	// Use sscanf to parse the NMEA string
	// 	sscanf(nmea_output, "%6[^,],%d,%f,%c,%f,%c,%d,%d,%f,%f,%c",
	// 		type, &time, &latitude, &latDirection, &longitude, &longDirection,
	// 		&fixQuality, &numSatellites, &HDOP, &altitude, &altitudeUnit, );

	// Save the current UART string to use next cycle
	memcpy(prev_uart_str, &(evt->data.rx.buf[evt->data.rx.offset]), BUFF_SIZE);
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
					dbg_print("uart_rx_buf_rsp failed.", sizeof("uart_rx_buf_rsp failed."));
				}
			} else {
				dbg_print("Not able to allocate UART receive buffer.", sizeof("Not able to allocate UART receive buffer."));
			}
			break;
		case UART_RX_BUF_RELEASED:
			k_free(rx_buf);
			break;
		case UART_RX_DISABLED:
			ret = uart_rx_enable(gps_uart, rx_buf, BUFF_SIZE, SYS_FOREVER_US);
			if (ret) { 
				dbg_print("uart_rx_enable failed.", sizeof("uart_rx_enable failed."));
			}
			break;
		default:
			break;
	}
}


int main(void) {

	k_msleep(2000);
	int ret;

	/////////////////////////////////// DEBUG UART SETUP ///////////////////////////////////
	if (!device_is_ready(dbg_uart)) {
		return -1;
	}

	/////////////////////////////////// GPS UART SETUP ///////////////////////////////////
	if (!device_is_ready(gps_uart)) {
		dbg_print("uart not ready. returning.\n", sizeof("uart not ready. returning.\n"));
		return -1;
	}

	ret = uart_callback_set(gps_uart, gps_uart_cb, NULL);
	if (ret) {
		dbg_print("gps_uart_cb set failed.", sizeof("gps_uart_cb set failed."));
		return ret;
	}

	k_msleep(1000);
	ret = uart_rx_enable(gps_uart, rx_buf, BUFF_SIZE, SYS_FOREVER_US);
	if (ret) {
		dbg_print("gps_uart uart_rx_enable failed.", sizeof("gps_uart uart_rx_enable failed."));
		return ret;
	}

	/////////////////////////////////// MAIN LOOP ///////////////////////////////////
	int ctr = 0;
	while (1) {
		ctr++;
		k_msleep(1);
		
	}

	return 0;
}
