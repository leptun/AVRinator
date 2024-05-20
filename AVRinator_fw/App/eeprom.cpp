#include "eeprom.hpp"
#include <main.h>
#include "pavr2.hpp"
#include "config.hpp"
#include <FreeRTOS.h>
#include <timers.h>

extern const eeprom::Settings _eeprom[];
extern const eeprom::Settings _eeprom_end[];

namespace eeprom {

static const Settings *lastSettings = nullptr;
static Settings settingsCache;

static TimerHandle_t eeprom_commit_timerHandle;
static StaticTimer_t eeprom_commit_staticBuf;

static void commitSettings(TimerHandle_t xTimer) {
	if (lastSettings) {
		for (lastSettings++; lastSettings < _eeprom_end; lastSettings++) {
			if (lastSettings->isEmpty()) {
				break;
			}
		}
	}
	else {
		lastSettings = _eeprom;
	}

	if (HAL_FLASH_Unlock() != HAL_OK) { Error_Handler(); }

	if (lastSettings >= _eeprom_end) {
		// ran out of eeprom cells. erase entire eeprom
		FLASH_EraseInitTypeDef pEraseInit = {
				FLASH_TYPEERASE_PAGES,
				FLASH_BANK_1,
				((uint32_t)(_eeprom) - FLASH_BASE) / FLASH_PAGE_SIZE,
				1,
		};
		uint32_t PageError;
		if (HAL_FLASHEx_Erase(&pEraseInit, &PageError) != HAL_OK) { Error_Handler(); }
		if (PageError != 0xFFFFFFFFU) { Error_Handler(); }

		lastSettings = _eeprom;
	}

	for (uint64_t *eedw = (uint64_t*)lastSettings, *settingsdw = (uint64_t*)&settingsCache; eedw < (uint64_t*)(lastSettings + 1); eedw++, settingsdw++) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)eedw, *settingsdw) != HAL_OK) { Error_Handler(); }
	}

	if (HAL_FLASH_Lock() != HAL_OK) { Error_Handler(); }
}

void Setup() {
	eeprom_commit_timerHandle = xTimerCreateStatic("eeprom_commit", pdMS_TO_TICKS(config::eeprom_apply_delay), false, 0, commitSettings, &eeprom_commit_staticBuf);

	// find last valid settings
	for (const Settings *settings = _eeprom; settings < _eeprom_end; settings++) {
		if (!settings->isEmpty()) {
			lastSettings = settings;
		}
		else {
			break;
		}
	}

	if (!lastSettings) {
		resetToDefaults();
	}
	else {
		settingsCache = *lastSettings;
	}
}

Settings *getSettings() {
	return &settingsCache;
}

void saveSettings(bool instant) {
	if (instant) {
		commitSettings(eeprom_commit_timerHandle);
	}
	else {
		if (xTimerReset(eeprom_commit_timerHandle, 0) != pdPASS) { Error_Handler(); }
	}
}

void resetToDefaults() {
	settingsCache.sck_duration = 0;
	settingsCache.isp_fastest_period = PAVR2_ISP_FASTEST_PERIOD_MIN;
	settingsCache.regulator_mode = PAVR2_REGULATOR_MODE_AUTO;
	settingsCache.vcc_output_enabled = 0;
	settingsCache.vcc_output_indicator = PAVR2_VCC_OUTPUT_INDICATOR_BLINKING;
	settingsCache.line_a_function = PAVR2_LINE_IS_NOTHING;
	settingsCache.line_b_function = PAVR2_LINE_IS_DTR;
	settingsCache.software_version_major = 0x2;
	settingsCache.software_version_minor = 0xa;
	settingsCache.hardware_version = 0xf;
	settingsCache.reset_polarity = 0;
	settingsCache.vcc_vdd_max_range = pavr2::mvToRaw(896);
	settingsCache.vcc_3v3_min = pavr2::mvToRaw(2720);
	settingsCache.vcc_3v3_max = pavr2::mvToRaw(3872);
	settingsCache.vcc_5v_min = pavr2::mvToRaw(4128);
	settingsCache.vcc_5v_max = pavr2::mvToRaw(5856);
	saveSettings(true);
}

}
