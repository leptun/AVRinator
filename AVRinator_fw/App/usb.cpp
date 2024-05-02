#include "usb.hpp"
#include <tinyusb.hpp>
#include "config.hpp"
#include "pavr2.hpp"
#include "pavr2_protocol.h"

namespace usb {

void Setup() {
	tinyusb::Setup();
}

extern "C"
void tud_cdc_rx_cb(uint8_t itf) {
	switch (itf) {
	case config::cdc_itf_isp:
		break;
	case config::cdc_itf_uart:
		break;
	default:
		Error_Handler();
	}
}

extern "C"
void tud_cdc_tx_complete_cb(uint8_t itf) {
	switch (itf) {
	case config::cdc_itf_isp:
		break;
	case config::cdc_itf_uart:
		break;
	default:
		Error_Handler();
	}
}

extern "C"
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
	switch (itf) {
	case config::cdc_itf_isp:
		break;
	case config::cdc_itf_uart:
		break;
	default:
		Error_Handler();
	}
}

extern "C"
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) {
	switch (request->bRequest) {
	case PAVR2_REQUEST_GET_SETTING: {
		if (stage == CONTROL_STAGE_SETUP) {
			TU_VERIFY(request->bmRequestType == 0xC0);
			TU_VERIFY(request->wValue == 0);
			TU_VERIFY(request->wLength == 1);

			uint8_t response = pavr2::getSetting(request->wIndex);

			TU_VERIFY(tud_control_xfer(rhport, request, &response, sizeof(response)));
		}
	} break;
	case PAVR2_REQUEST_SET_SETTING: {
		if (stage == CONTROL_STAGE_SETUP) {
			TU_VERIFY(tud_control_status(rhport, request));
		}
		else if (stage == CONTROL_STAGE_ACK) {
			TU_VERIFY(request->bmRequestType == 0x40);
			TU_VERIFY(request->wLength == 0);

			pavr2::setSetting(request->wIndex, request->wValue);
		}
	} break;
	case PAVR2_REQUEST_GET_VARIABLE: {
		if (stage == CONTROL_STAGE_SETUP) {
			TU_VERIFY(request->bmRequestType == 0xC0);
			TU_VERIFY(request->wValue == 0);
			TU_VERIFY(request->wLength == 1);

			uint8_t response = pavr2::getVariable(request->wIndex);

			TU_VERIFY(tud_control_xfer(rhport, request, &response, sizeof(response)));
		}
	} break;
	case PAVR2_REQUEST_DIGITAL_READ: {
		if (stage == CONTROL_STAGE_SETUP) {
			TU_VERIFY(request->bmRequestType == 0xC0);
			TU_VERIFY(request->wValue == 0);
			TU_VERIFY(request->wIndex == 0);
			TU_VERIFY(request->wLength == 3);

			pavr2::ProgrammerDigitalReadings response = pavr2::digitalRead();

			TU_VERIFY(tud_control_xfer(rhport, request, &response, sizeof(response)));
		}
	} break;
	default:
		return false;
	}
	return true;
}

}