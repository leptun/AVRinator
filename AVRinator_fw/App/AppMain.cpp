#include "AppMain.hpp"
#include <main.h>
#include <FreeRTOS.h>
#include <task.h>
#include <config.hpp>
#include "eeprom.hpp"
#include "usb.hpp"
#include <ttl/ttl.hpp>
#include <isp/isp.hpp>

namespace AppMain {

uint32_t rcc_csr_initial;

void Setup() {
	rcc_csr_initial = RCC->CSR;
	LL_RCC_ClearResetFlags();

	eeprom::Setup();
	usb::Setup();
	isp::Setup();
	ttl::Setup();
}

void Notify(uint8_t itf, uint32_t flags) {
	switch (itf) {
	case config::cdc_itf_isp:
		isp::Notify(flags);
		break;
	case config::cdc_itf_ttl:
		ttl::Notify(flags);
		break;
	default:
		Error_Handler();
	}
}

}
