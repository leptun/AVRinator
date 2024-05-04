#pragma once
#include <FreeRTOS.h>
#include <task.h>

namespace AppMain {

extern uint32_t rcc_csr_initial;

extern void Setup();
void Notify(uint8_t itf, uint32_t flags);

}
