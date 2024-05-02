#include "AppMain.hpp"
#include <main.h>
#include <FreeRTOS.h>
#include <task.h>
#include <config.hpp>
#include "eeprom.hpp"
#include "usb.hpp"

namespace AppMain {

uint32_t rcc_csr_initial;

static TaskHandle_t taskHandleISP;
static TaskHandle_t taskHandleTTLrx;
static TaskHandle_t taskHandleTTLtx;

static int cdc_read(uint8_t itf, uint8_t *buf, size_t count) {
	size_t read = 0;
	while (true) {
		if (!(tud_ready() && tud_cdc_n_get_line_state(itf) & 0x02)) {
			return -1;
		}
		read += tud_cdc_n_read(itf, buf + read, count - read);
		if (read == count) {
			return 0;
		} else {
			util::xTaskNotifyWaitBitsAnyIndexed(1, 0, FLAG_COMM_RX | FLAG_COMM_LINE_STATE, NULL, portMAX_DELAY);
		}
	}
}

static int cdc_read_any(uint8_t itf, uint8_t *buf, size_t maxcount) {
	size_t read = 0;
	while (true) {
		if (!(tud_ready() && tud_cdc_n_get_line_state(itf) & 0x02)) {
			return -1;
		}
		read += tud_cdc_n_read(itf, buf + read, maxcount - read);
		if (read != 0) {
			return read;
		} else {
			util::xTaskNotifyWaitBitsAnyIndexed(1, 0, FLAG_COMM_RX | FLAG_COMM_LINE_STATE, NULL, portMAX_DELAY);
		}
	}
}

static int cdc_write(uint8_t itf, const uint8_t *buf, size_t count) {
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

static void cdc_write_flush(uint8_t itf) {
	while (tud_cdc_n_write_available(itf) != CFG_TUD_CDC_TX_BUFSIZE) {
		tud_cdc_n_write_flush(itf);
	}
}

static void taskISP(void *pvParameters) {
	for (;;) {
		vTaskDelay(1000);
	}
}

static void taskTTLrx(void *pvParameters) {
	for (;;) {
		uint8_t buf[64];
		int rx = config::resources::ttl_usart->receiveAny(buf, sizeof(buf));
		cdc_write(config::cdc_itf_ttl, buf, rx);
		cdc_write_flush(config::cdc_itf_ttl);
	}
}

static void taskTTLtx(void *pvParameters) {
	for (;;) {
		uint8_t buf[64];
		int rx = cdc_read_any(config::cdc_itf_ttl, buf, sizeof(buf));
		if (rx > 0) {
			config::resources::ttl_usart->send(buf, rx);
		}
		else {
			portYIELD();
		}

	}
}

void Setup() {
	rcc_csr_initial = RCC->CSR;
	LL_RCC_ClearResetFlags();

	eeprom::Setup();
	config::resources::isp_usart->Setup();
	config::resources::ttl_usart->Setup();
	usb::Setup();

	if (xTaskCreate(taskISP, "ISP", config::resources::ISP_stack_depth, NULL, 0, &taskHandleISP) != pdPASS) {
		Error_Handler();
	}

	if (xTaskCreate(taskTTLrx, "TTLrx", config::resources::TTL_stack_depth, NULL, 0, &taskHandleTTLrx) != pdPASS) {
		Error_Handler();
	}

	if (xTaskCreate(taskTTLtx, "TTLtx", config::resources::TTL_stack_depth, NULL, 0, &taskHandleTTLtx) != pdPASS) {
		Error_Handler();
	}
}

void Notify(uint8_t itf, uint32_t flags) {
	switch (itf) {
	case config::cdc_itf_isp:
		if (taskHandleISP && xTaskNotifyIndexed(taskHandleISP, 1, flags, eSetBits) != pdPASS) {
			Error_Handler();
		}
		break;
	case config::cdc_itf_ttl:
		if (flags & (FLAG_COMM_TX | FLAG_COMM_LINE_STATE)) {
			if (taskHandleTTLrx && xTaskNotifyIndexed(taskHandleTTLtx, 1, flags, eSetBits) != pdPASS) {
				Error_Handler();
			}
		}
		else if (flags & (FLAG_COMM_RX | FLAG_COMM_LINE_STATE)) {
			if (taskHandleTTLtx && xTaskNotifyIndexed(taskHandleTTLtx, 1, flags, eSetBits) != pdPASS) {
				Error_Handler();
			}
		}
		break;
	default:
		Error_Handler();
	}
}

}
