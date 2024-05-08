#include "isp.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <config.hpp>
#include <usb.hpp>


namespace isp {

static uint8_t stk500_hwversion;
static uint8_t stk500_swVersionMajor;
static uint8_t stk500_swVersionMinor;

static bool programmingEnabled;
static uint8_t error;
static uint8_t dataBuf[config::resources::isp_dataBuf_size] __attribute__((section(".buffers")));


static void taskISP(void *pvParameters) {
	for (;;) {
		int rx = usb::cdc_read_any(config::cdc_itf_isp, dataBuf, sizeof(dataBuf));
		if (rx > 0) {
			usb::cdc_write(config::cdc_itf_isp, dataBuf, rx);
		} else if (!usb::cdc_write_push(config::cdc_itf_isp)) {
			// nothing left to push, wait for more cdc rx to happen
			usb::cdc_awaitRx(config::cdc_itf_isp);
		}
		taskYIELD();
	}
}
static TaskHandle_t taskHandleISP;
StackType_t ISP_stack[config::resources::ISP_stack_depth] __attribute__((section(".stack")));
StaticTask_t ISP_taskdef;

void Setup() {
	config::resources::isp_usart.Setup();

	if (!(taskHandleISP = xTaskCreateStatic(
			taskISP,
			"ISP",
			config::resources::ISP_stack_depth,
			NULL,
			config::task_priorities::ISP,
			ISP_stack,
			&ISP_taskdef
	))) {
		Error_Handler();
	}
}

void Notify(uint32_t flags) {
	if (taskHandleISP && xTaskNotifyIndexed(taskHandleISP, 1, flags, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

uint8_t getError() {
	return error;
}

bool getProgrammingEnabled() {
	return programmingEnabled;
}


}
