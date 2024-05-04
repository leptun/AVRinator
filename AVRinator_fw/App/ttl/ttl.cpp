#include "ttl.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <config.hpp>
#include <usb.hpp>


namespace ttl {

static TaskHandle_t taskHandleTTLrx;
static TaskHandle_t taskHandleTTLtx;

static void taskTTLrx(void *pvParameters) {
	for (;;) {
		uint8_t buf[64];
		int rx = config::resources::ttl_usart->receiveAny(buf, sizeof(buf));
		if (rx > 0) {
			usb::cdc_write(config::cdc_itf_ttl, buf, rx);
		}
		else {
			usb::cdc_write_flush(config::cdc_itf_ttl);
			config::resources::ttl_usart->awaitRx();
		}
	}
}

static void taskTTLtx(void *pvParameters) {
	for (;;) {
		uint8_t buf[64];
		int rx = usb::cdc_read_any(config::cdc_itf_ttl, buf, sizeof(buf));
		if (rx > 0) {
			config::resources::ttl_usart->send(buf, rx);
		}
	}
}

void Setup() {
	config::resources::ttl_usart->Setup();

	if (xTaskCreate(taskTTLrx, "TTLrx", config::resources::TTL_stack_depth, NULL, 1, &taskHandleTTLrx) != pdPASS) {
		Error_Handler();
	}

	if (xTaskCreate(taskTTLtx, "TTLtx", config::resources::TTL_stack_depth, NULL, 1, &taskHandleTTLtx) != pdPASS) {
		Error_Handler();
	}
}

void Notify(uint32_t flags) {
	if (flags & (usb::FLAG_COMM_TX | usb::FLAG_COMM_LINE_STATE)) {
		if (taskHandleTTLrx && xTaskNotifyIndexed(taskHandleTTLrx, 1, flags, eSetBits) != pdPASS) {
			Error_Handler();
		}
	}
	else if (flags & (usb::FLAG_COMM_RX | usb::FLAG_COMM_LINE_STATE)) {
		if (taskHandleTTLtx && xTaskNotifyIndexed(taskHandleTTLtx, 1, flags, eSetBits) != pdPASS) {
			Error_Handler();
		}
	}
}

}