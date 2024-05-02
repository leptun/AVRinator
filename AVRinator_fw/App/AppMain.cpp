#include "AppMain.hpp"
#include <main.h>
#include <FreeRTOS.h>
#include <task.h>
#include <config.hpp>
#include "eeprom.hpp"
#include "usb.hpp"

namespace AppMain {

uint32_t rcc_csr_initial;

TaskHandle_t taskHandleISP;
TaskHandle_t taskHandleTTL;


static void taskISP(void *pvParameters) {
	for (;;) {
		vTaskDelay(1000);
	}
}

static void taskTTL(void *pvParameters) {
	for (;;) {
		vTaskDelay(1000);
	}
}

void Setup() {
	rcc_csr_initial = RCC->CSR;
	LL_RCC_ClearResetFlags();

	eeprom::Setup();
	usb::Setup();

	if (xTaskCreate(taskISP, "ISP", config::resources::ISP_stack_depth, NULL, 0, &taskHandleISP) != pdPASS) {
		Error_Handler();
	}

	if (xTaskCreate(taskTTL, "TTL", config::resources::TTL_stack_depth, NULL, 0, &taskHandleTTL) != pdPASS) {
		Error_Handler();
	}
}

}
