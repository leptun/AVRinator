#pragma once
#include <inttypes.h>
#include "util.hpp"

namespace adc {

void Setup();

util::MinMax<uint16_t> getMinMaxVDD();
util::MinMax<uint16_t> getMinMaxTargetVCC();

}
