#include "usb.hpp"
#include <tinyusb.hpp>
#include "config.hpp"
#include "pavr2/pavr2.hpp"
#include "AppMain.hpp"

namespace usb {

void Setup() {
	tinyusb::Setup();
}

int cdc_read(uint8_t itf, uint8_t *buf, size_t count) {
	size_t read = 0;
	while (true) {
		if (!(tud_ready() && tud_cdc_n_get_line_state(itf) & 0x02)) {
			return -1;
		}
		read += tud_cdc_n_read(itf, buf + read, count - read);
		if (read == count) {
			return 0;
		} else {
			cdc_awaitRx(itf);
		}
	}
}

int cdc_read_any(uint8_t itf, uint8_t *buf, size_t maxcount) {
	if (!tud_ready()) {
		return -1;
	}
	size_t read = 0;
	while (tud_cdc_n_available(itf) && read < maxcount) {
		if (!(tud_ready() && tud_cdc_n_get_line_state(itf) & 0x02)) {
			return -1;
		}
		read += tud_cdc_n_read(itf, buf + read, maxcount - read);
	}
	return read;
}

void cdc_awaitRx(uint8_t itf) {
	while (!tud_ready()) {
		vTaskDelay(1);
	}
	while (!tud_cdc_n_available(itf)) {
		uint32_t ulNotificationValue;
		util::xTaskNotifyWaitBitsAnyIndexed(1, 0, FLAG_COMM_RX | FLAG_COMM_LINE_STATE, &ulNotificationValue, portMAX_DELAY);
		if (ulNotificationValue & FLAG_COMM_LINE_STATE) {
			return;
		}
	}
}

int cdc_write(uint8_t itf, const uint8_t *buf, size_t count) {
	size_t written = 0;
	while (true) {
		if (!(tud_ready() && tud_cdc_n_get_line_state(itf) & 0x02)) {
			return -1;
		}
		written += tud_cdc_n_write(itf, buf + written, count - written);
		if (written == count) {
			return 0;
		} else {
			util::xTaskNotifyWaitBitsAnyIndexed(1, 0, FLAG_COMM_TX | FLAG_COMM_LINE_STATE, NULL, portMAX_DELAY);
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

void cdc_write_flush(uint8_t itf) {
	while (!tud_ready()) {
		vTaskDelay(1);
	}
	while (tud_cdc_n_write_available(itf) != CFG_TUD_CDC_TX_BUFSIZE) {
		tud_cdc_n_write_flush(itf);
		portYIELD();
	}
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
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
	AppMain::Notify(itf, FLAG_COMM_LINE_STATE);
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
