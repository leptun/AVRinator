#include "avr_isp.hpp"
#include <usart.hpp>
#include <config.hpp>

namespace isp {
namespace avr_isp {

static constexpr usart::SyncUSART& com = config::resources::isp_usart;

bool programmingEnable() {
	uint8_t seq[4] = { 0xac, 0x53 };
	com.txrx(seq, seq, sizeof(seq));
	return seq[2] == 0x53;
}

void chipErase() {
	uint8_t seq[4] = { 0xac, 0x80 };
	com.txrx(seq, seq, sizeof(seq));
}

void loadExtendedAddressByte(uint8_t addr) {
	uint8_t seq[4] = { 0x4d, 0x00, addr };
	com.txrx(seq, seq, sizeof(seq));
}

uint8_t readProgramMemory(uint16_t addr, bool hiLow) {
	uint8_t seq[4] = { (uint8_t)(hiLow ? 0x28 : 0x20), (uint8_t)(addr >> 8), (uint8_t)addr };
	com.txrx(seq, seq, sizeof(seq));
	return seq[3];
}

void loadProgramMemoryPage(uint16_t addr, bool hiLow, uint8_t data) {
	uint8_t seq[4] = { (uint8_t)(hiLow ? 0x48 : 0x40), (uint8_t)(addr >> 8), (uint8_t)addr, data };
	com.txrx(seq, seq, sizeof(seq));
}

void writeProgramMemoryPage(uint16_t addr) {
	uint8_t seq[4] = { 0x4c, (uint8_t)(addr >> 8), (uint8_t)addr };
	com.txrx(seq, seq, sizeof(seq));
}

uint8_t readEEPROMMemory(uint16_t addr) {
	uint8_t seq[4] = { 0xa0, (uint8_t)(addr >> 8), (uint8_t)addr };
	com.txrx(seq, seq, sizeof(seq));
	return seq[3];
}

void writeEEPROMMemory(uint16_t addr, uint8_t data) {
	uint8_t seq[4] = { 0xc0, (uint8_t)(addr >> 8), (uint8_t)addr, data };
	com.txrx(seq, seq, sizeof(seq));
}

void loadEEPROMMemoryPage(uint16_t addr, uint8_t data) {
	uint8_t seq[4] = { 0xc1, (uint8_t)(addr >> 8), (uint8_t)addr, data };
	com.txrx(seq, seq, sizeof(seq));
}

void writeEEPROMMemoryPage(uint16_t addr) {
	uint8_t seq[4] = { 0xc2, (uint8_t)(addr >> 8), (uint8_t)addr };
	com.txrx(seq, seq, sizeof(seq));
}

uint8_t readLockBits() {
	uint8_t seq[4] = { 0x58 };
	com.txrx(seq, seq, sizeof(seq));
	return seq[3];
}

void writeLockBits(uint8_t data) {
	uint8_t seq[4] = { 0xac, 0xe0, 0x00, data };
	com.txrx(seq, seq, sizeof(seq));
}

uint8_t readSignatureByte(uint16_t addr) {
	uint8_t seq[4] = { 0x30, (uint8_t)(addr >> 8), (uint8_t)addr };
	com.txrx(seq, seq, sizeof(seq));
	return seq[3];
}

void writeFuseBits(uint8_t data) {
	uint8_t seq[4] = { 0xac, 0xa0, 0x00, data };
	com.txrx(seq, seq, sizeof(seq));
}

void writeFuseHighBits(uint8_t data) {
	uint8_t seq[4] = { 0xac, 0xa8, 0x00, data };
	com.txrx(seq, seq, sizeof(seq));
}

void writeExtendedFuseBits(uint8_t data) {
	uint8_t seq[4] = { 0xac, 0xa4, 0x00, data };
	com.txrx(seq, seq, sizeof(seq));
}

uint8_t readFuseBits() {
	uint8_t seq[4] = { 0x50, 0x00 };
	com.txrx(seq, seq, sizeof(seq));
	return seq[3];
}

uint8_t readFuseHighBits() {
	uint8_t seq[4] = { 0x58, 0x08 };
	com.txrx(seq, seq, sizeof(seq));
	return seq[3];
}

uint8_t readExtendedFuseBits() {
	uint8_t seq[4] = { 0x50, 0x00 };
	com.txrx(seq, seq, sizeof(seq));
	return seq[3];
}

uint8_t readCalibrationByte() {
	uint8_t seq[4] = { 0x38 };
	com.txrx(seq, seq, sizeof(seq));
	return seq[3];
}

bool pollBSY() {
	uint8_t seq[4] = { 0xf0 };
	com.txrx(seq, seq, sizeof(seq));
	return seq[3] & 0x01;
}

}
}
