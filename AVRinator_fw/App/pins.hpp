#pragma once
#include <main.h>

namespace pins {

struct GPIO {
	GPIO_TypeDef *port;
	uint32_t pin;

	void Write(bool val) const {
		if (val) {
			LL_GPIO_SetOutputPin(port, pin);
		}
		else {
			LL_GPIO_ResetOutputPin(port, pin);
		}
	}
	bool Read() const {
		return LL_GPIO_IsInputPinSet(port, pin);
	}
	void SetMode(uint32_t mode) const {
		LL_GPIO_SetPinMode(port, pin, mode);
	}
	void SetPinOutputType(uint32_t OutputType) const {
		LL_GPIO_SetPinOutputType(port, pin, OutputType);
	}
	void SetAF(uint32_t Alternate) const {
		if (pin & 0xFF) {
			LL_GPIO_SetAFPin_0_7(port, pin, Alternate);
		}
		else {
			LL_GPIO_SetAFPin_8_15(port, pin, Alternate);
		}
	}
};

namespace ISP {
inline constexpr GPIO T_MISO = { T_MISO_GPIO_Port, T_MISO_Pin };
inline constexpr GPIO T_MOSI = { T_MOSI_GPIO_Port, T_MOSI_Pin };
inline constexpr GPIO T_SCK = { T_SCK_GPIO_Port, T_SCK_Pin };
inline constexpr GPIO T_NRESET = { T_NRESET_GPIO_Port, T_NRESET_Pin };
}

namespace TTL {
inline constexpr GPIO RX = { VCOM_RX_GPIO_Port, VCOM_RX_Pin };
inline constexpr GPIO TX = { VCOM_TX_GPIO_Port, VCOM_TX_Pin };
inline constexpr GPIO A = { A_GPIO_Port, A_Pin };
inline constexpr GPIO B = { B_GPIO_Port, B_Pin };
}

namespace USER {
inline constexpr GPIO SW = { SW_GPIO_Port, SW_Pin };
inline constexpr GPIO LED = { LED_GPIO_Port, LED_Pin };
}

namespace POWER {
inline constexpr GPIO LS_OE = { LS_OE_GPIO_Port, LS_OE_Pin };
inline constexpr GPIO T_5V_EN = { T_5V_EN_GPIO_Port, T_5V_EN_Pin };
inline constexpr GPIO T_3V3_EN = { T_3V3_EN_GPIO_Port, T_3V3_EN_Pin };
inline constexpr GPIO NFAULT = { NFAULT_GPIO_Port, NFAULT_Pin };
}

void Setup();
void setIspOutput(bool val);

}
