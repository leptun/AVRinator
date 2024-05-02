#pragma once
#include <FreeRTOS.h>
#include <task.h>

namespace AppMain {

enum Flags {
//	FLAG_MODULES = 0x000001,
};

extern uint32_t rcc_csr_initial;

extern TaskHandle_t taskHandleISP;
extern TaskHandle_t taskHandleTTL;

extern void Setup();

}
