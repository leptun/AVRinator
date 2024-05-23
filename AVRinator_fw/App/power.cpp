#include "power.hpp"
#include "pins.hpp"
#include <inttypes.h>
#include <FreeRTOS.h>
#include <task.h>
#include "adc.hpp"

namespace power {

static bool powerOutputEnabled;
static uint8_t target_voltage;

void Setup() {
	adc::Setup();
	setPowerOutput(false);
}

void setTargetVoltage(uint8_t val) {
	bool powerOutputInitial = powerOutputEnabled;
	if (val != target_voltage) {
		setPowerOutput(false);
	}
	target_voltage = val;
	setPowerOutput(powerOutputInitial);
}

uint8_t getTargetVoltage() {
	return target_voltage;
}

void setPowerOutput(bool enabled) {
	if (powerOutputEnabled) {
		pins::POWER::T_3V3_EN.Write(0);
		pins::POWER::T_5V_EN.Write(0);
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	powerOutputEnabled = enabled;
	if (enabled) {
		switch (target_voltage) {
		case PAVR2_REGULATOR_MODE_3V3:
			pins::POWER::T_3V3_EN.Write(1);
			break;
		case PAVR2_REGULATOR_MODE_5V:
			pins::POWER::T_5V_EN.Write(1);
			break;
		case PAVR2_REGULATOR_MODE_AUTO:
		default:
			break;
		}
	}
}

void setLevelShifter(bool enabled) {
	pins::POWER::LS_OE.Write(enabled);
}

}

