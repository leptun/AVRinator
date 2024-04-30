#include "AppMain.hpp"
#include <main.h>
#include <FreeRTOS.h>
#include <task.h>
#include <config.hpp>
#include <usb.hpp>

namespace AppMain {

TaskHandle_t pxTaskHandle;

static void taskAppMain(void *pvParameters) {
	usb::Setup();

	for (;;) {
		vTaskDelay(1000);
	}
}

void Setup() {
	if (xTaskCreate(taskAppMain, "AppMain", config::resources::appMain_stack_depth, NULL, 0, &pxTaskHandle) != pdPASS) {
		Error_Handler();
	}

//	LL_DBGMCU_APB1_GRP1_FreezePeriph(
//			LL_DBGMCU_APB1_GRP1_IWDG_STOP |
//	);
//	LL_DBGMCU_APB2_GRP1_FreezePeriph(
//			LL_DBGMCU_APB2_GRP1_TIM9_STOP |
//			LL_DBGMCU_APB2_GRP1_TIM11_STOP
//	);
}

}
