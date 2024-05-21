#include "usb.hpp"
#include <tinyusb.hpp>
#include "config.hpp"
#include "pavr2.hpp"
#include "AppMain.hpp"

namespace usb {

void Setup() {
	tinyusb::Setup();
}

int cdc_read(uint8_t itf, uint8_t *buf, size_t count, TickType_t xTicksToWait) {
	TimeOut_t xTimeOut;
	vTaskSetTimeOutState(&xTimeOut);
	size_t read = 0;
	while (true) {
		if (!tud_ready()) {
			return -1;
		}
		read += tud_cdc_n_read(itf, buf + read, count - read);
		if (read == count) {
			return 0;
		} else {
			if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) {
				return -2;
			}
			if (!cdc_awaitRx(itf, xTicksToWait)) {
				return -2; //timeout
			}
		}
	}
}

int cdc_read_any(uint8_t itf, uint8_t *buf, size_t maxcount) {
	if (!tud_ready()) {
		return -1;
	}
	size_t read = 0;
	while (tud_cdc_n_available(itf) && read < maxcount) {
		if (!tud_ready()) {
			return -1;
		}
		read += tud_cdc_n_read(itf, buf + read, maxcount - read);
	}
	return read;
}

bool cdc_awaitRx(uint8_t itf, TickType_t xTicksToWait) {
	TimeOut_t xTimeOut;
	vTaskSetTimeOutState(&xTimeOut);
	if (!tud_ready()) {
		return false;
	}
	while (!tud_cdc_n_available(itf)) {
		if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) {
			return false;
		}
		uint32_t ulNotificationValue;
		util::xTaskNotifyWaitBitsAnyIndexed(1, 0, FLAG_COMM_RX, &ulNotificationValue, xTicksToWait);
	}
	return true;
}

int cdc_write(uint8_t itf, const uint8_t *buf, size_t count, TickType_t xTicksToWait) {
	TimeOut_t xTimeOut;
	vTaskSetTimeOutState(&xTimeOut);
	size_t written = 0;
	while (true) {
		if (!tud_ready()) {
			return -1;
		}
		written += tud_cdc_n_write(itf, buf + written, count - written);
		if (written == count) {
			return 0;
		} else {
			if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE) {
				return -2;
			}
			if (util::xTaskNotifyWaitBitsAnyIndexed(1, 0, FLAG_COMM_TX, NULL, xTicksToWait) != pdTRUE) {
				return -2;
			}
		}
	}
}

uint32_t cdc_write_push(uint8_t itf) {
	if (!tud_ready()) {
		return 0;
	}
	tud_cdc_n_write_flush(itf);
	return CFG_TUD_CDC_TX_BUFSIZE - tud_cdc_n_write_available(itf);
}

bool cdc_write_flush(uint8_t itf) {
	if (!tud_ready()) {
		return false;
	}
	while (tud_cdc_n_write_available(itf) != CFG_TUD_CDC_TX_BUFSIZE) {
		if (!tud_ready()) {
			return -1;
		}
		tud_cdc_n_write_flush(itf);
		portYIELD();
	}
	return true;
}

extern "C"
void tud_cdc_rx_cb(uint8_t itf) {
	AppMain::Notify(itf, FLAG_COMM_RX);
}

extern "C"
void tud_cdc_tx_complete_cb(uint8_t itf) {
	AppMain::Notify(itf, FLAG_COMM_TX);
}

extern "C"
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding) {
	if (itf == config::cdc_itf_ttl) {
		config::resources::ttl_usart.setBaud(p_line_coding->bit_rate);
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
