#include "pavr2.hpp"
#include "pavr2_protocol.h"
#include "eeprom.hpp"
#include <main.h>
#include "AppMain.hpp"

namespace pavr2 {

uint8_t getSetting(uint8_t id) {
	eeprom::Settings *settings = eeprom::getSettings();

	switch (id) {
	case PAVR2_SETTING_SCK_DURATION:
	case PAVR2_SETTING_ISP_FASTEST_PERIOD:
	case PAVR2_SETTING_REGULATOR_MODE:
	case PAVR2_SETTING_VCC_OUTPUT_ENABLED:
	case PAVR2_SETTING_VCC_OUTPUT_INDICATOR:
	case PAVR2_SETTING_LINE_A_FUNCTION:
	case PAVR2_SETTING_LINE_B_FUNCTION:
	case PAVR2_SETTING_SOFTWARE_VERSION_MAJOR:
	case PAVR2_SETTING_SOFTWARE_VERSION_MINOR:
	case PAVR2_SETTING_HARDWARE_VERSION:
	case PAVR2_SETTING_RESET_POLARITY:
	case PAVR2_SETTING_VCC_VDD_MAX_RANGE:
	case PAVR2_SETTING_VCC_3V3_MIN:
	case PAVR2_SETTING_VCC_3V3_MAX:
	case PAVR2_SETTING_VCC_5V_MIN:
	case PAVR2_SETTING_VCC_5V_MAX:
		return settings->raw[id - 1];
	case PAVR2_SETTING_NOT_INITIALIZED:
	default:
		return 0;
	}
}

void setSetting(uint8_t id, uint8_t value) {
	eeprom::Settings *settings = eeprom::getSettings();

	switch (id) {
	case PAVR2_SETTING_SCK_DURATION:
	case PAVR2_SETTING_ISP_FASTEST_PERIOD:
	case PAVR2_SETTING_REGULATOR_MODE:
	case PAVR2_SETTING_VCC_OUTPUT_ENABLED:
	case PAVR2_SETTING_VCC_OUTPUT_INDICATOR:
	case PAVR2_SETTING_LINE_A_FUNCTION:
	case PAVR2_SETTING_LINE_B_FUNCTION:
	case PAVR2_SETTING_SOFTWARE_VERSION_MAJOR:
	case PAVR2_SETTING_SOFTWARE_VERSION_MINOR:
	case PAVR2_SETTING_HARDWARE_VERSION:
	case PAVR2_SETTING_RESET_POLARITY:
	case PAVR2_SETTING_VCC_VDD_MAX_RANGE:
	case PAVR2_SETTING_VCC_3V3_MIN:
	case PAVR2_SETTING_VCC_3V3_MAX:
	case PAVR2_SETTING_VCC_5V_MIN:
	case PAVR2_SETTING_VCC_5V_MAX:
		settings->raw[id - 1] = value;
		//todo start apply timer
		break;
	case PAVR2_SETTING_NOT_INITIALIZED:
		eeprom::resetToDefaults();
		break;
	default:
		break;
	}
}

uint8_t getVariable(uint8_t id) {
	switch (id) {
	case PAVR2_VARIABLE_LAST_DEVICE_RESET:
		if (AppMain::rcc_csr_initial & RCC_CSR_BORRSTF) {
			return PAVR2_RESET_BROWNOUT;
		}
		else if (AppMain::rcc_csr_initial & RCC_CSR_PINRSTF) {
			return PAVR2_RESET_RESET_LINE;
		}
		else if (AppMain::rcc_csr_initial & (RCC_CSR_WWDGRSTF | RCC_CSR_IWDGRSTF)) {
			return PAVR2_RESET_WATCHDOG;
		}
		else if (AppMain::rcc_csr_initial & RCC_CSR_SFTRSTF) {
			return PAVR2_RESET_SOFTWARE;
		}
		else {
			return PAVR2_RESET_POWER_UP;
		}
	case PAVR2_VARIABLE_PROGRAMMING_ERROR:
	case PAVR2_VARIABLE_TARGET_VCC_MEASURED_MIN:
	case PAVR2_VARIABLE_TARGET_VCC_MEASURED_MAX:
	case PAVR2_VARIABLE_PROGRAMMER_VDD_MEASURED_MIN:
	case PAVR2_VARIABLE_PROGRAMMER_VDD_MEASURED_MAX:
	case PAVR2_VARIABLE_TARGET_VCC:
	case PAVR2_VARIABLE_PROGRAMMER_VDD:
	case PAVR2_VARIABLE_REGULATOR_LEVEL:
	case PAVR2_VARIABLE_IN_PROGRAMMING_MODE:
	default:
		return 0;
	}
}

ProgrammerDigitalReadings digitalRead() {
	ProgrammerDigitalReadings ret = {};
	return ret;
}

}
