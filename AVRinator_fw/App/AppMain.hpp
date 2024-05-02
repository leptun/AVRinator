#pragma once
#include <FreeRTOS.h>
#include <task.h>

namespace AppMain {

enum Flags {
	FLAG_COMM_RX = 0x000001,
	FLAG_COMM_TX = 0x000002,
	FLAG_COMM_LINE_STATE = 0x000004,
};

extern uint32_t rcc_csr_initial;

extern void Setup();
void Notify(uint8_t itf, uint32_t flags);

}
