#include "isp.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <config.hpp>
#include <usb.hpp>


namespace isp {

static TaskHandle_t taskHandleISP;

static void taskISP(void *pvParameters) {
	for (;;) {
		uint8_t buf[64];
		int rx = usb::cdc_read_any(config::cdc_itf_isp, buf, sizeof(buf));
		if (rx > 0) {
			usb::cdc_write(config::cdc_itf_isp, buf, rx);
		} else {
			if (!usb::cdc_write_push(config::cdc_itf_isp)) {
				// nothing left to push, wait for more cdc rx to happen
				usb::cdc_awaitRx(config::cdc_itf_isp);
			}
		}
	}
}

void Setup() {
	config::resources::isp_usart->Setup();

	if (xTaskCreate(
			taskISP,
			"ISP",
			config::resources::ISP_stack_depth,
			NULL,
			config::task_priorities::ISP,
			&taskHandleISP
	) != pdPASS) {
		Error_Handler();
	}
}

void Notify(uint32_t flags) {
	if (taskHandleISP && xTaskNotifyIndexed(taskHandleISP, 1, flags, eSetBits) != pdPASS) {
		Error_Handler();
	}
}


}
