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
		// Initialize variables to default "null" values
		char type[7] = {0};
		char mode = '\0';
		int fix_type = -1;  // 2 or 3 means successful fix
		int prn[12];
		for (int i = 0; i < 12; i++) prn[i] = -1;  // Initialize all PRNs to -1 (null)
		float pdop = -1.0f, hdop = -1.0f, vdop = -1.0f;
		int gnss_id = -1;
		int checksum = -1;

		const char* current = GSA_msg_start;
		char field[20];  // Adjust size as needed for the largest expected field
		int fieldIndex = 0;
		int prnIndex = 0;

		while (*current && *current != '*') {  // Stop at the end of the string or the checksum indicator
			const char* next = current;
			while (*next && *next != ',' && *next != '*') next++;  // Find the end of the current field

			// Copy the current field into 'field'
			strncpy(field, current, next - current);
			field[next - current] = '\0';

			// Parse the current field based on its position
			switch (fieldIndex) {
				case 0: sscanf(field, "%6s", type); break;
				case 1: sscanf(field, "%c", &mode); break;
				case 2: sscanf(field, "%d", &fix_type); break;
				// Handle PRNs (satellite numbers)
				case 3: case 4: case 5: case 6: case 7: case 8: case 9: case 10: case 11: case 12: case 13: case 14:
					if (*field) sscanf(field, "%d", &prn[prnIndex++]);
					break;
				case 15: sscanf(field, "%f", &pdop); break;
				case 16: sscanf(field, "%f", &hdop); break;
				case 17: sscanf(field, "%f", &vdop); break;
				case 18: sscanf(field, "%d", &gnss_id); break;
			}

			current = next + 1;  // Move to the start of the next field
			fieldIndex++;
		}

		// Parse the checksum
		sscanf(strchr(GSA_msg_start+1, '*')+1, "%x", &checksum);
		
		// Debug output to verify parsing results
		LOG_INF("\n\nType: %s", type);
		// LOG_INF("Mode: %c", mode);
		LOG_INF("Fix Type (2 or 3 = fix acquired): %d", fix_type);
		// for (int i = 0; i < 12; i++) {
		// 	LOG_INF("PRN[%d]: %d", i, prn[i]);
		// }
		// LOG_INF("PDOP: %f", pdop);
		// LOG_INF("HDOP: %f", hdop);
		// LOG_INF("VDOP: %f", vdop);
		// LOG_INF("GNSS ID: %d", gnss_id);
		LOG_INF("Checksum: %02X", checksum);
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
		char type[7] = "";
		float time = 0.0f, speed = 0.0f, true_course = 0.0f;
		char status = '\0', ns = '\0', ew = '\0', mode = '\0', faa = '\0';  // mode and faa mode probably not useful
		char date[7] = "";
		float latitude = 0.0f, longitude = 0.0f;
		int checksum = -1;

		const char* current = RMC_msg_start;
		char field[20];  // Adjust size as needed for the largest expected field
		int fieldIndex = 0;

		while (*current && *current != '*') {  // Stop at the end of the string or the checksum indicator
			const char* next = current;
			while (*next && *next != ',' && *next != '*') next++;  // Find the end of the current field

			// Copy the current field into 'field'
			strncpy(field, current, next - current);
			field[next - current] = '\0';

			// Parse the current field based on its position
			switch (fieldIndex) {
				case 0: sscanf(field, "%6s", type); break;
				case 1: sscanf(field, "%f", &time); break;
				case 2: sscanf(field, "%c", &status); break;
				case 3: sscanf(field, "%f", &latitude); break;
				case 4: sscanf(field, "%c", &ns); break;
				case 5: sscanf(field, "%f", &longitude); break;
				case 6: sscanf(field, "%c", &ew); break;
				case 7: sscanf(field, "%f", &speed); break;
				case 8: sscanf(field, "%f", &true_course); break;
				case 9: sscanf(field, "%6s", date); break;
				case 10: sscanf(field, "%c", &mode); break;
				case 11: sscanf(field, "%c", &faa); break;
			}

			current = next + 1;  // Move to the start of the next field
			fieldIndex++;
		}
		
		// Parse the checksum
		sscanf(strchr(RMC_msg_start+1, '*')+1, "%x", &checksum);

		LOG_INF("\n\nType: %s", type);
		LOG_INF("Time (hhmmss.sss UTC): %.3f", time);
		LOG_INF("Status (A = data valid): %c", status);
		LOG_INF("Latitude: %.4f %c", latitude, ns);
		LOG_INF("Longitude: %.4f %c", longitude, ew);
		LOG_INF("Speed: %.2f knots", speed);
		LOG_INF("True Course: %.2f degrees", true_course);
		LOG_INF("Date: %s", date);
		// LOG_INF("Mode: %c\n", mode);
		// LOG_INF("FAA mode: %c\n", faa);
		LOG_INF("Checksum: %X\n", checksum);
	}


	// Search for $XXGGA (Global Positioning System Fix Data; XX can be either GP, GL, GA, BD, QZ, or GN depending on constellation being used)
	// GGA message is needed bc it's the only message containing altitude data
	char* GGA_msg_start_GN = strstr(combined_nmea_output, "$GNGGA");
	char* GGA_msg_start_GP = strstr(combined_nmea_output, "$GPGGA");
	char* GGA_msg_start_GL = strstr(combined_nmea_output, "$GLGGA");
	char* GGA_msg_start_GA = strstr(combined_nmea_output, "$GAGGA");
	char* GGA_msg_start_BD = strstr(combined_nmea_output, "$BDGGA");
	char* GGA_msg_start_QZ = strstr(combined_nmea_output, "$QZGGA");
	char* GGA_msg_start = NULL;

	// Pick any constellation that has a fix
	if (GGA_msg_start_GN != NULL) GGA_msg_start = GGA_msg_start_GN;  // Check GN (multi-constellation) first
	else if (GGA_msg_start_GP != NULL) GGA_msg_start = GGA_msg_start_GP;
	else if (GGA_msg_start_GL != NULL) GGA_msg_start = GGA_msg_start_GL;
	else if (GGA_msg_start_GA != NULL) GGA_msg_start = GGA_msg_start_GA;
	else if (GGA_msg_start_BD != NULL) GGA_msg_start = GGA_msg_start_BD;
	else if (GGA_msg_start_QZ != NULL) GGA_msg_start = GGA_msg_start_QZ;

	char* GGA_msg_end = strchr(GGA_msg_start+1, '\n');
	
	if (GGA_msg_start != NULL && GGA_msg_end != NULL) {  // combined_nmea_output contains a full $XXGGA message
		char type[7] = "";
		float time = 0.0f;
		int num_satellites = -1;
		float latitude = 0.0f, longitude = 0.0f;
		char ns = '\0', ew = '\0';
		int fix_quality = -1;
		float hdop = -1.0f, altitude = 0.0f, geoid_separation = 0.0f;
		char altitude_unit = '\0', geoid_separation_unit = '\0';
		int checksum = -1;

		const char* current = GGA_msg_start;
		char field[20];  // Adjust size as needed for the largest expected field
		int fieldIndex = 0;

		while (*current && *current != '*') {  // Stop at the end of the string or the checksum indicator
			const char* next = current;
			while (*next && *next != ',' && *next != '*') next++;  // Find the end of the current field

			// Copy the current field into 'field'
			strncpy(field, current, next - current);
			field[next - current] = '\0';

			// Parse the current field based on its position
			switch (fieldIndex) {
				case 0: sscanf(field, "%6s", type); break;
				case 1: sscanf(field, "%f", &time); break;
				case 2: sscanf(field, "%f", &latitude); break;
				case 3: sscanf(field, "%c", &ns); break;
				case 4: sscanf(field, "%f", &longitude); break;
				case 5: sscanf(field, "%c", &ew); break;
				case 6: sscanf(field, "%d", &fix_quality); break;
				case 7: sscanf(field, "%d", &num_satellites); break;
				case 8: sscanf(field, "%f", &hdop); break;
				case 9: sscanf(field, "%f", &altitude); break;
				case 10: sscanf(field, "%c", &altitude_unit); break;
				case 11: sscanf(field, "%f", &geoid_separation); break;
				case 12: sscanf(field, "%c", &geoid_separation_unit); break;
			}

			current = next + 1;  // Move to the start of the next field
			fieldIndex++;

			// Parse the checksum
			sscanf(strchr(GGA_msg_start+1, '*')+1, "%x", &checksum);
		}

		LOG_INF("\n\nType: %s", type);
		LOG_INF("Time (hhmmss.sss UTC): %.3f", time);
		LOG_INF("Num Satellites: %d", num_satellites);
		LOG_INF("Latitude: %.4f %c", latitude, ns);
		LOG_INF("Longitude: %.4f %c", longitude, ew);
		LOG_INF("Fix Quality (1 = fix acquired): %d", fix_quality);
		// LOG_INF("HDOP: %.2f", hdop);
		LOG_INF("Altitude: %.2f meters", altitude);
		// LOG_INF("Geoid Separation: %.2f meters", geoid_separation);
		LOG_INF("Checksum: %X", checksum);
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
