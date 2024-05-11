#pragma once
#include <inttypes.h>
#include <stddef.h>
#include <FreeRTOS.h>

namespace usb {

enum Flags {
	FLAG_COMM_RX = 0x000001,
	FLAG_COMM_TX = 0x000002,
};

void Setup();

int cdc_read(uint8_t itf, uint8_t *buf, size_t count, TickType_t xTicksToWait = portMAX_DELAY);
int cdc_read_any(uint8_t itf, uint8_t *buf, size_t maxcount);
bool cdc_awaitRx(uint8_t itf, TickType_t xTicksToWait = portMAX_DELAY);
int cdc_write(uint8_t itf, const uint8_t *buf, size_t count, TickType_t xTicksToWait = portMAX_DELAY);
uint32_t cdc_write_push(uint8_t itf);
bool cdc_write_flush(uint8_t itf);


}
