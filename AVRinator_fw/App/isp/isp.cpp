#include "isp.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <config.hpp>
#include <usb.hpp>


namespace isp {

static TaskHandle_t taskHandleISP;

static void taskISP(void *pvParameters) {
	for (;;) {
		vTaskDelay(1000);
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
