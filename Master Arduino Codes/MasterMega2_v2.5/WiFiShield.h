/*
 * WiFiShield.h
 *
 *  Created on: May 17, 2012
 *      Author: yuncong
 */

#ifndef WIFISHIELD_H_
#define WIFISHIELD_H_

#include <WiShield.h>

#define WIRELESS_MODE_INFRA	1
#define WIRELESS_MODE_ADHOC	2

// Wireless configuration parameters ----------------------------------------
unsigned char local_ip[] = { 192, 168, 1, 2 }; // IP address of WiShield
unsigned char gateway_ip[] = { 192, 168, 1, 1 }; // router or gateway IP address
unsigned char subnet_mask[] = { 255, 255, 255, 0 }; // subnet mask for the local network
const prog_char ssid[] PROGMEM = { "ASYNCLABS" }; // max 32 bytes

unsigned char security_type = 0; // 0 - open; 1 - WEP; 2 - WPA; 3 - WPA2

// WPA/WPA2 passphrase
const prog_char security_passphrase[] PROGMEM = { "12345678" }; // max 64 characters

// WEP 128-bit keys
// sample HEX keys
prog_uchar wep_keys[] PROGMEM = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c,
		0x0d, // Key 0
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, // Key 1
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, // Key 2
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00 // Key 3
		};

// setup the wireless mode
// infrastructure - connect to AP
// adhoc - connect to another WiFi device
unsigned char wireless_mode = WIRELESS_MODE_ADHOC;
unsigned char ssid_len;
unsigned char security_passphrase_len;

extern WiShield WiFi;

uint16_t datalen = 0;
bool done = false;
bool pc_connected = false;
const unsigned char* dataptr;
bool get_or_send = 1;

#define wifi_init() WiFi.init()
#define wifi_connect() do {WiFi.run();} while (!pc_connected)

void wifi_send(char* str, uint16_t len) {
	done = false;
	get_or_send = 1;
	datalen = len;
	dataptr = (const unsigned char*)str;
	while (!done) {
		WiFi.run();
	};
}

void wifi_get(char* str, uint16_t len) {
	done = false;
	get_or_send = 0;
	while (!done) {
		WiFi.run();
	};
}

#endif /* WIFISHIELD_H_ */
