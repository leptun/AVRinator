#pragma once
#include <inttypes.h>

namespace isp {

void Setup();
void Notify(uint32_t flags);
uint8_t getError();
bool getProgrammingEnabled();

}
