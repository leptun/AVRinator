#include "ttl.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <config.hpp>
#include <usb.hpp>


namespace ttl {

static void taskTTLrx(void *pvParameters) {
	for (;;) {
		static uint8_t rxbuf[config::resources::ttl_rxtransfer_size] __attribute__((section(".buffers")));
		int rx = config::resources::ttl_usart.receiveAny(rxbuf, sizeof(rxbuf));
		if (rx > 0) {
			usb::cdc_write(config::cdc_itf_ttl, rxbuf, rx, portMAX_DELAY);
		}
		else if (!usb::cdc_write_push(config::cdc_itf_ttl)) {
			// nothing left to push, wait for more uart rx to happen
			config::resources::ttl_usart.awaitRx();
		}
		taskYIELD();
	}
}
static TaskHandle_t taskHandleTTLrx;
StackType_t TTLrx_stack[config::resources::TTL_stack_depth] __attribute__((section(".stack")));
StaticTask_t TTLrx_taskdef;

static void taskTTLtx(void *pvParameters) {
	for (;;) {
		static uint8_t txbuf[config::resources::ttl_txtransfer_size] __attribute__((section(".buffers")));
		int rx = usb::cdc_read_any(config::cdc_itf_ttl, txbuf, sizeof(txbuf));
		if (rx > 0) {
			config::resources::ttl_usart.send(txbuf, rx);
		} else {
			usb::cdc_awaitRx(config::cdc_itf_ttl);
		}
		taskYIELD();
	}
}
static TaskHandle_t taskHandleTTLtx;
StackType_t TTLtx_stack[config::resources::TTL_stack_depth] __attribute__((section(".stack")));
StaticTask_t TTLtx_taskdef;

void Setup() {
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
	if (flags & usb::FLAG_COMM_TX) {
		if (taskHandleTTLrx && xTaskNotifyIndexed(taskHandleTTLrx, 1, flags, eSetBits) != pdPASS) {
			Error_Handler();
		}
	}
	else if (flags & usb::FLAG_COMM_RX) {
		if (taskHandleTTLtx && xTaskNotifyIndexed(taskHandleTTLtx, 1, flags, eSetBits) != pdPASS) {
			Error_Handler();
		}
	}
}

}
