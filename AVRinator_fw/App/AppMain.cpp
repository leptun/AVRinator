#include "AppMain.hpp"
#include <main.h>
#include <FreeRTOS.h>
#include <task.h>
#include "config.hpp"
#include "eeprom.hpp"
#include "power.hpp"
#include "usb.hpp"
#include "ttl/ttl.hpp"
#include "isp/isp.hpp"

namespace AppMain {

uint32_t rcc_csr_initial;

void Setup() {
	rcc_csr_initial = RCC->CSR;
	LL_RCC_ClearResetFlags();

	eeprom::Setup();
	power::Setup();
	usb::Setup();
	isp::Setup();
	ttl::Setup();
	applySettings(eeprom::getSettings());
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

void applySettings(eeprom::Settings *settings) {
	uint32_t spi_freq = (12000000ul / settings->isp_fastest_period) / (settings->sck_duration + 1ul);
	config::resources::isp_usart.setBaud(spi_freq);

	if (settings->vcc_output_enabled) {
		power::setTargetVoltage(settings->regulator_mode);
		power::setPowerOutput(true);
	}
	else {
		power::setPowerOutput(false);
		power::setTargetVoltage(settings->regulator_mode);
	}
}

}
