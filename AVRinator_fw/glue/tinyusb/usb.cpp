#include "usb.hpp"
#include <tusb.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <util.hpp>
#include <config.hpp>

namespace usb {

static TaskHandle_t pxTaskHandle;

static void usb_device_task(void *pvParameters) {
	(void) pvParameters;

	// init device stack on configured roothub port
	// This should be called after scheduler/kernel is started.
	// Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.

	// RTOS forever loop
	while (1) {
		// put this thread to waiting state until there is new events
		tud_task();
	}
}

extern "C"
void tud_cdc_rx_cb(uint8_t itf) {
	switch (itf) {
	case config::cdc_itf_uart:
		break;
	case config::cdc_itf_isp:
		break;
	default:
		Error_Handler();
	}
}

extern "C"
void tud_cdc_tx_complete_cb(uint8_t itf) {
	switch (itf) {
	case config::cdc_itf_uart:
		break;
	case config::cdc_itf_isp:
		break;
	default:
		Error_Handler();
	}
}

extern "C"
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
	switch (itf) {
	case config::cdc_itf_uart:
		break;
	case config::cdc_itf_isp:
		break;
	default:
		Error_Handler();
	}
}

extern "C"
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) {
	return true;
}

void Setup() {
    /* Peripheral clock enable */
    __HAL_RCC_USB_CLK_ENABLE();
    /* USB interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_HP_IRQn);
    HAL_NVIC_SetPriority(USB_LP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_IRQn);
    HAL_NVIC_SetPriority(USBWakeUp_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USBWakeUp_IRQn);

	tud_init(BOARD_TUD_RHPORT);

	// Create a task for tinyusb device stack
	if (xTaskCreate(usb_device_task, "usbd", config::resources::usbd_stack_depth, NULL, (configMAX_PRIORITIES-1), &pxTaskHandle) != pdPASS) {
		Error_Handler();
	}
}

static size_t board_get_unique_id(uint8_t id[], size_t max_len) {
	(void) max_len;
	uint32_t* id32 = (uint32_t*) (uintptr_t) id;
	uint8_t const len = 12;

	id32[0] = HAL_GetUIDw0();
	id32[1] = HAL_GetUIDw1();
	id32[2] = HAL_GetUIDw2();

	return len;
}

// Get USB Serial number string from unique ID if available. Return number of character.
// Input is string descriptor from index 1 (index 0 is type + len)
extern "C" size_t board_usb_get_serial(uint16_t desc_str1[], size_t max_chars) {
	uint8_t uid[16] TU_ATTR_ALIGNED(4);
	size_t uid_len;

	uid_len = board_get_unique_id(uid, sizeof(uid));

	if ( uid_len > max_chars / 2 ) uid_len = max_chars / 2;

	for ( size_t i = 0; i < uid_len; i++ ) {
		for ( size_t j = 0; j < 2; j++ ) {
			const char nibble_to_hex[16] = {
				'0', '1', '2', '3', '4', '5', '6', '7',
				'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
			};
			uint8_t const nibble = (uid[i] >> (j * 4)) & 0xf;
			desc_str1[i * 2 + (1 - j)] = nibble_to_hex[nibble]; // UTF-16-LE
		}
	}

	return 2 * uid_len;
}

extern "C"
void USB_HP_IRQHandler(void) {
	tud_int_handler(0);
}

extern "C"
void USB_LP_IRQHandler(void) {
	tud_int_handler(0);
}

extern "C"
void USBWakeUp_IRQHandler(void) {
  tud_int_handler(0);
}

}
