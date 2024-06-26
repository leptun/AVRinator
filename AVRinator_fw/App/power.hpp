#pragma once
#include <inttypes.h>
#include "defs/pavr2_protocol.h"

namespace power {

void Setup();

void setTargetVoltage(uint8_t val);
uint8_t getTargetVoltage();
void setPowerOutput(bool enabled);
void setLevelShifter(bool enabled);

}
