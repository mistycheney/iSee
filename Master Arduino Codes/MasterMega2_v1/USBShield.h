/*
 * USBShield.h
 *
 *  Created on: May 13, 2012
 *      Author: yuncong
 */

#ifndef USBSHIELD_H_
#define USBSHIELD_H_


#include <stdint.h>

void usb_setup();

void usb_send(const char* data, uint16_t len);
void usb_receive(char* buf, uint16_t* rcvd);

void hokuyo_on();
void hokuyo_off();
void hokuyo_capture_send();

#endif /* USBSHIELD_H_ */
