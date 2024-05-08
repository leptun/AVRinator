#pragma once
#include <FreeRTOS.h>
#include <task.h>
#include "eeprom.hpp"

namespace AppMain {

extern uint32_t rcc_csr_initial;

extern void Setup();
void Notify(uint8_t itf, uint32_t flags);
void applySettings(eeprom::Settings *settings);

}
