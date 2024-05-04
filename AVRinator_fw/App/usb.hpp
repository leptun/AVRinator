#pragma once
#include <inttypes.h>
#include <stddef.h>

namespace usb {

enum Flags {
	FLAG_COMM_RX = 0x000001,
	FLAG_COMM_TX = 0x000002,
	FLAG_COMM_LINE_STATE = 0x000004,
};

void Setup();

int cdc_read(uint8_t itf, uint8_t *buf, size_t count);
int cdc_read_any(uint8_t itf, uint8_t *buf, size_t maxcount);
int cdc_write(uint8_t itf, const uint8_t *buf, size_t count);
uint32_t cdc_write_push(uint8_t itf);
void cdc_write_flush(uint8_t itf);


}
