#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
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
	
	new_data_flag = 1;  // set flag to 1 to indicate to other threads that new data is ready

	// Search for $XXGSA (GNSS DOP and Active Satellites; XX can be either GP, GL, GA, BD, QZ, or GN depending on constellation being used)
	char* GSA_msg_start_GN = strstr(combined_nmea_output, "$GNGSA");
	char* GSA_msg_start_GP = strstr(combined_nmea_output, "$GPGSA");
	char* GSA_msg_start_GL = strstr(combined_nmea_output, "$GLGSA");
	char* GSA_msg_start_GA = strstr(combined_nmea_output, "$GAGSA");
	char* GSA_msg_start_BD = strstr(combined_nmea_output, "$BDGSA");
	char* GSA_msg_start_QZ = strstr(combined_nmea_output, "$QZGSA");
	char* GSA_msg_start = NULL;
	
	// Pick any constellation that is sending GSA messages
	if (GSA_msg_start_GN != NULL) GSA_msg_start = GSA_msg_start_GN;  // Check GN (multi-constellation) first
	else if (GSA_msg_start_GP != NULL) GSA_msg_start = GSA_msg_start_GP;
	else if (GSA_msg_start_GL != NULL) GSA_msg_start = GSA_msg_start_GL;
	else if (GSA_msg_start_GA != NULL) GSA_msg_start = GSA_msg_start_GA;
	else if (GSA_msg_start_BD != NULL) GSA_msg_start = GSA_msg_start_BD;
	else if (GSA_msg_start_QZ != NULL) GSA_msg_start = GSA_msg_start_QZ;

	char* GSA_msg_end = strchr(GSA_msg_start+1, '\n');

	if (GSA_msg_start != NULL && GSA_msg_end != NULL) {  // combined_nmea_output contains a full $XXGSA message
		printk("\nFOUND GSA MESSAGE\n");

		char type[7];
		char mode;
		int fix_type;
		int prn[12];  // up to 12 satellites
		float pdop, hdop, vdop;
		int gnss_id;
		int checksum;

		// Parse the NMEA sentence
		int parsed = sscanf(GSA_msg_start, "%6s,%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%d*%x",
							type, &mode, &fix_type,
							&prn[0], &prn[1], &prn[2], &prn[3], &prn[4], &prn[5],
							&prn[6], &prn[7], &prn[8], &prn[9], &prn[10], &prn[11],
							&pdop, &hdop, &vdop, &gnss_id, &checksum);

		// Check if parsing succeeded
		if (parsed >= 20) { // Ensure mandatory fields are parsed
			printk("Parsed successfully:\n");
			printk("Type: %s\n", type);
			printk("Mode: %c\n", mode);
			printk("Fix Type: %d\n", fix_type);
			for (int i = 0; i < 12; i++) {
				if (prn[i] != 0) { // Print only if satellite PRN is available
					printk("Satellite PRN %d: %d\n", i + 1, prn[i]);
				}
			}
			printk("PDOP: %.2f\n", pdop);
			printk("HDOP: %.2f\n", hdop);
			printk("VDOP: %.2f\n", vdop);
			printk("Checksum: %X\n", checksum);
		} else {
			printk("Parsing failed. Parsed %d fields.\n", parsed);
		}
	}


	// Search for $XXRMC (Recommended Minimum Data; XX can be either GP, GL, GA, BD, QZ, or GN depending on constellation being used)
	char* RMC_msg_start_GN = strstr(combined_nmea_output, "$GNRMC");
	char* RMC_msg_start_GP = strstr(combined_nmea_output, "$GPRMC");
	char* RMC_msg_start_GL = strstr(combined_nmea_output, "$GLRMC");
	char* RMC_msg_start_GA = strstr(combined_nmea_output, "$GARMC");
	char* RMC_msg_start_BD = strstr(combined_nmea_output, "$BDRMC");
	char* RMC_msg_start_QZ = strstr(combined_nmea_output, "$QZRMC");
	char* RMC_msg_start = NULL;

	// Pick any constellation that has a fix
	if (RMC_msg_start_GN != NULL) RMC_msg_start = RMC_msg_start_GN;  // Check GN (multi-constellation) first
	else if (RMC_msg_start_GP != NULL) RMC_msg_start = RMC_msg_start_GP;
	else if (RMC_msg_start_GL != NULL) RMC_msg_start = RMC_msg_start_GL;
	else if (RMC_msg_start_GA != NULL) RMC_msg_start = RMC_msg_start_GA;
	else if (RMC_msg_start_BD != NULL) RMC_msg_start = RMC_msg_start_BD;
	else if (RMC_msg_start_QZ != NULL) RMC_msg_start = RMC_msg_start_QZ;

	char* RMC_msg_end = strchr(RMC_msg_start+1, '\n');
	
	if (RMC_msg_start != NULL && RMC_msg_end != NULL) {  // combined_nmea_output contains a full $XXRMC message
		printk("\nFOUND RMC MESSAGE\n");

		char type[7];
		float time, speed, true_course;
		char status, ns, ew, mode, faa;
		char date[7];
		float latitude, longitude;
		int checksum;

		// Parse the NMEA sentence
		int parsed = sscanf(RMC_msg_start, "%6s,%f,%c,%f,%c,%f,%c,%f,%f,%6s,,,%c,%c*%x",
							type, &time, &status, &latitude, &ns, &longitude, &ew,
							&speed, &true_course, date, &mode, &faa, &checksum);

		// Check if parsing succeeded
		if (parsed >= 13) {
			printk("Parsed successfully:\n");
			printk("Type: %s\n", type);
			printk("Time: %.3f\n", time);
			printk("Status: %c\n", status);
			printk("Latitude: %.4f %c\n", latitude, ns);
			printk("Longitude: %.4f %c\n", longitude, ew);
			printk("Speed: %.2f knots\n", speed);
			printk("True Course: %.2f degrees\n", true_course);
			printk("Date: %s\n", date);
			printk("Mode: %c\n", mode);
			printk("FAA mode: %c\n", faa);
			printk("Checksum: %X\n", checksum);
		} else {
			printk("Parsing failed. Parsed %d fields.\n", parsed);
    	}
	}
	printk("\n==============================\n\n");

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
