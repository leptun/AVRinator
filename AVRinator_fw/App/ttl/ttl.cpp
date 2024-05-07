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
		static uint8_t buf[config::resources::ttl_rxtransfer_size];
		int rx = config::resources::ttl_usart.receiveAny(buf, sizeof(buf));
		if (rx > 0) {
			usb::cdc_write(config::cdc_itf_ttl, buf, rx);
		}
		else if (!usb::cdc_write_push(config::cdc_itf_ttl)) {
			// nothing left to push, wait for more uart rx to happen
			config::resources::ttl_usart.awaitRx();
		}
	}
}
StackType_t TTLrx_stack[config::resources::usbd_stack_depth];
StaticTask_t TTLrx_taskdef;

static void taskTTLtx(void *pvParameters) {
	for (;;) {
		uint8_t buf[config::resources::ttl_txtransfer_size];
		int rx = usb::cdc_read_any(config::cdc_itf_ttl, buf, sizeof(buf));
		if (rx > 0) {
			config::resources::ttl_usart.send(buf, rx);
		} else {
			usb::cdc_awaitRx(config::cdc_itf_ttl);
		}
	}
}
StackType_t TTLtx_stack[config::resources::usbd_stack_depth];
StaticTask_t TTLtx_taskdef;

void Setup() {
	config::resources::ttl_usart.Setup();

	if (!(taskHandleTTLrx = xTaskCreateStatic(
			taskTTLrx,
			"TTLrx",
			config::resources::TTL_stack_depth,
			NULL,
			config::task_priorities::TTLrx,
			TTLrx_stack,
			&TTLrx_taskdef
	))) {
		Error_Handler();
	}

	if (!(taskHandleTTLtx = xTaskCreateStatic(
			taskTTLtx,
			"TTLtx",
			config::resources::TTL_stack_depth,
			NULL,
			config::task_priorities::TTLtx,
			TTLtx_stack,
			&TTLtx_taskdef
	))) {
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
