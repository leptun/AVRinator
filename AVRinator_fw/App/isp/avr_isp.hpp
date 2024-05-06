#pragma once
#include <inttypes.h>

namespace isp {
namespace avr_isp {

bool programmingEnable();
void chipErase();
void loadExtendedAddressByte(uint8_t addr);
uint8_t readProgramMemory(uint16_t addr, bool hiLow);
void loadProgramMemoryPage(uint16_t addr, bool hiLow, uint8_t data);
void writeProgramMemoryPage(uint16_t addr);
uint8_t readEEPROMMemory(uint16_t addr);
void writeEEPROMMemory(uint16_t addr, uint8_t data);
void loadEEPROMMemoryPage(uint16_t addr, uint8_t data);
void writeEEPROMMemoryPage(uint16_t addr);
uint8_t readLockBits();
void writeLockBits(uint8_t data);
uint8_t readSignatureByte(uint16_t addr);
void writeFuseBits(uint8_t data);
void writeFuseHighBits(uint8_t data);
void writeExtendedFuseBits(uint8_t data);
uint8_t readFuseBits();
uint8_t readFuseHighBits();
uint8_t readExtendedFuseBits();
uint8_t readCalibrationByte();
bool pollRDY_BSY();

}
}
