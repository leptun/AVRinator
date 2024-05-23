#include "pins.hpp"

namespace pins {

void Setup() {
	setIspOutput(false);
	pins::POWER::LS_OE.Write(true);
}

void setIspOutput(bool val) {
	if (val) {
		pins::ISP::T_MISO.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::ISP::T_MOSI.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::ISP::T_SCK.SetMode(LL_GPIO_MODE_ALTERNATE);
		pins::ISP::T_NRESET.SetPinOutputType(LL_GPIO_OUTPUT_PUSHPULL);
	}
	else {
		pins::ISP::T_MISO.SetMode(LL_GPIO_MODE_INPUT);
		pins::ISP::T_MOSI.SetMode(LL_GPIO_MODE_INPUT);
		pins::ISP::T_SCK.SetMode(LL_GPIO_MODE_INPUT);
		pins::ISP::T_NRESET.SetPinOutputType(LL_GPIO_OUTPUT_OPENDRAIN);
	}
}

}
