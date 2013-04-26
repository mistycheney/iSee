#include "USBShield.h"

#include <avrpins.h>
#include <max3421e.h>
#include <usbhost.h>
#include <usb_ch9.h>
#include <Usb.h>
#include <usbhub.h>
#include <avr/pgmspace.h>
#include <address.h>

#include <cdcacm.h>

#include <printhex.h>
#include <message.h>
#include <hexdump.h>
#include <parsetools.h>

#include "pgmstrings.h"

class ACMAsyncOper: public CDCAsyncOper {
public:
	virtual uint8_t OnInit(ACM *pacm);
};

USB Usb;
//USBHub     Hub(&Usb);
ACMAsyncOper AsyncOper;
ACM Acm(&Usb, &AsyncOper);

extern void wifi_send(char* str, uint16_t len);

extern char* hokuyo_data;
uint16_t usblen;

uint8_t ACMAsyncOper::OnInit(ACM *pacm) {
	uint8_t rcode;
	// Set DTR = 1 RTS=1
	rcode = pacm->SetControlLineState(3);

	if (rcode) {
		ErrorMessage<uint8_t>(PSTR("SetControlLineState"), rcode);
		return rcode;
	}

	LINE_CODING lc;
	lc.dwDTERate = 115200;
	lc.bCharFormat = 0;
	lc.bParityType = 0;
	lc.bDataBits = 8;

	rcode = pacm->SetLineCoding(&lc);

	if (rcode)
		ErrorMessage<uint8_t>(PSTR("SetLineCoding"), rcode);

	return rcode;
}

void usb_setup() {
	if (Usb.Init() == -1)
		Serial.println(PSTR("OSCOKIRQ failed to assert"));

	delay(200);
}

void usb_send(const char* data, uint16_t len) {
	while (!Acm.isReady()) {
		Usb.Task();
		delay(20);
	};

	uint8_t rcode;
	if (Acm.isReady()) {
//		Serial.print("send ");
//		Serial.println(data);
		delay(100);
		rcode = Acm.SndData(len, (uint8_t*) data);
		if (rcode)
			ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
		delay(100);
	}
}

void usb_receive(char* buf, uint16_t* rcvd) {
	while (!Acm.isReady()) {
		Usb.Task();
		delay(20);
	};

	uint8_t rcode;
	if (Acm.isReady()) {
		rcode = Acm.RcvData(rcvd, (uint8_t*) buf);
		if (rcode && rcode != hrNAK)
			ErrorMessage<uint8_t>(PSTR("Ret"), rcode);

//		Serial.print("received ");
//		Serial.println(*rcvd);
//		for (uint16_t i = 0; i < *rcvd; i++) {
//			Serial.print((char) buf[i]);
//		}
//		Serial.println("");
	}
}

void hokuyo_on() {
	usb_send("BM\n", 3);
	usblen = 8;
	usb_receive(hokuyo_data, &usblen);
}

void hokuyo_off() {
	usb_send("QT\n", 3);
	usblen = 8;
	usb_receive(hokuyo_data, &usblen);
}

//#define GD_SEGMENT_LEN 64*34 //must not be larger than HOKUYO_DATA_LENGTH
#define GD_SEGMENT_LEN 64*34 //must not be larger than HOKUYO_DATA_LENGTH
void hokuyo_capture_send() {
	bool reach_end = false;
	usb_send("GD0044072501\n", 13);

	do {
		usblen = GD_SEGMENT_LEN;
		usb_receive(hokuyo_data, &usblen);

		if (usblen == 0) {
			continue;
		} else if (usblen < GD_SEGMENT_LEN) {
			reach_end = true;
		}

//		dataptr = (const unsigned char*) hokuyo_data;
//		wifi_run(1);

		wifi_send(hokuyo_data, usblen);
		delay(100);
	} while (!reach_end);

//	memset(hokuyo_data, 0, 2200);

}
