#pragma once
#include <inttypes.h>
#include "pavr2_protocol.h"

namespace pavr2 {

struct ProgrammerDigitalReadings
{
    uint8_t portA;
    uint8_t portB;
    uint8_t portC;
};

inline uint8_t mvToRaw(uint16_t mv) {
	return mv / PAVR2_VOLTAGE_UNITS;
}
inline uint16_t rawToMv(uint8_t raw) {
	return raw * PAVR2_VOLTAGE_UNITS;
}

uint8_t getSetting(uint8_t id);
void setSetting(uint8_t id, uint8_t value);
uint8_t getVariable(uint8_t id);
ProgrammerDigitalReadings digitalRead();

}
