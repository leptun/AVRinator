#pragma once
#include <inttypes.h>

namespace eeprom {
union Settings {
	struct __attribute__((packed)) __attribute__((aligned(8))) {
		uint8_t sck_duration;
		uint8_t isp_fastest_period;
		uint8_t regulator_mode;
		uint8_t vcc_output_enabled;
		uint8_t vcc_output_indicator;
		uint8_t line_a_function;
		uint8_t line_b_function;
		uint8_t software_version_major;
		uint8_t software_version_minor;
		uint8_t hardware_version;
		uint8_t reset_polarity;
		uint8_t vcc_vdd_max_range;
		uint8_t vcc_3v3_min;
		uint8_t vcc_3v3_max;
		uint8_t vcc_5v_min;
		uint8_t vcc_5v_max;
	};
	uint8_t raw[0];

	inline bool isEmpty() const {
		for (const uint8_t *v = (const uint8_t*)this; v < (const uint8_t*)(this + 1); v++) {
			if (*v != 0xFF) {
				return false;
			}
		}
		return true;
	}
};

static_assert(sizeof(Settings) % 8 == 0, "Settings not a multiple of double word");

void Setup();
Settings *getSettings();
void applySettings();
void resetToDefaults();

}
