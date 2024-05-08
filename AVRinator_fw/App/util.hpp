#pragma once
#include "main.h"
#include <inttypes.h>
#include <FreeRTOS.h>
#include <pins.hpp>
#include <limits>

extern uint8_t null_ptr;

namespace util {

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))
#define ALIGN_CACHE __attribute__ ((aligned(32)))

template <typename T>
struct MinMax {
	T latest;
	T min;
	T max;

	constexpr MinMax() : min(std::numeric_limits<T>::max()), max(std::numeric_limits<T>::lowest()) {}

	void update(T val) {
		latest = val;
		if (val < min) {
			min = val;
		}
		if (val > max) {
			max = val;
		}
	}
};

class TIM_CHAN_PAIR {
public:
	TIM_TypeDef *tim;
	uint32_t chan;
	void SetCompare(uint32_t val) const {
		switch (chan) {
		case LL_TIM_CHANNEL_CH1:
		case LL_TIM_CHANNEL_CH1N:
			LL_TIM_OC_SetCompareCH1(tim, val);
			break;
		case LL_TIM_CHANNEL_CH2:
		case LL_TIM_CHANNEL_CH2N:
			LL_TIM_OC_SetCompareCH2(tim, val);
			break;
		case LL_TIM_CHANNEL_CH3:
		case LL_TIM_CHANNEL_CH3N:
			LL_TIM_OC_SetCompareCH3(tim, val);
			break;
		case LL_TIM_CHANNEL_CH4:
			LL_TIM_OC_SetCompareCH4(tim, val);
			break;
		default:
			Error_Handler();
		}
	}
	uint32_t MaxVal() const {
		return IS_TIM_32B_COUNTER_INSTANCE(tim) ? 0xFFFFFFFFul : 0xFFFFul;
	}
};

struct LL_DMA_CHANNEL {
	DMA_TypeDef * const DMAx;
	const uint32_t Channel;
	volatile uint32_t * const ISR;
	volatile uint32_t * const IFCR;
	const uint8_t bitOffset;

public:
	constexpr LL_DMA_CHANNEL() :
			DMAx(0),
			Channel(0),
			ISR(0),
			IFCR(0),
			bitOffset(0) {}
	constexpr LL_DMA_CHANNEL(DMA_TypeDef *DMAx, uint32_t Channel) :
			DMAx(DMAx),
			Channel(Channel),
			ISR(&DMAx->ISR),
			IFCR(&DMAx->IFCR),
			bitOffset(4 * Channel) {}

	uint32_t irq_handler() const {
		uint32_t mask =
				(LL_DMA_IsEnabledIT_TE(DMAx, Channel) ? DMA_ISR_TEIF1 : 0) |
				(LL_DMA_IsEnabledIT_HT(DMAx, Channel) ? DMA_ISR_HTIF1 : 0) |
				(LL_DMA_IsEnabledIT_TC(DMAx, Channel) ? DMA_ISR_TCIF1 : 0);
		uint32_t flags = *ISR & (mask << bitOffset);
		*IFCR = flags; //clear enabled flags
		return flags >> bitOffset;
	}

	void clearIRQ(uint32_t mask) const {
		*IFCR = mask << bitOffset;
	}
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
template<typename T>
constexpr T * ioCast(unsigned long addr) {
	return static_cast<T *>(static_cast<void *>(&null_ptr + addr));
}
#pragma GCC diagnostic pop

constexpr uint32_t getMPURegionSizeSettingCxpr(uint32_t ulActualSizeInBytes)
{
    uint32_t ulRegionSize, ulReturnValue = 4;

    /* 32 is the smallest region size, 31 is the largest valid value for
     * ulReturnValue. */
    for(ulRegionSize = 32UL; ulReturnValue < 31UL; (ulRegionSize <<= 1UL)) {
        if( ulActualSizeInBytes <= ulRegionSize ) {
            break;
        }
        else {
            ulReturnValue++;
        }
    }

    /* Shift the code by one before returning so it can be written directly
     * into the the correct bit position of the attribute register. */
    return(ulReturnValue << 1UL);
}

class AtomicFlags {
	uint32_t flags_thread;
	uint32_t flags_handler;
public:
	constexpr AtomicFlags() : flags_thread(0), flags_handler(0) {}

	uint32_t Get() const {
		return flags_thread ^ flags_handler;
	}
	void Set(uint32_t flags) {
		flags &= ~Get();
		flags_thread ^= flags;
	}
	void SetFromISR(uint32_t flags) {
		flags &= ~Get();
		flags_handler ^= flags;
	}
	void Clear(uint32_t flags) {
		flags &= Get();
		flags_thread ^= flags;
	}
	void ClearFromISR(uint32_t flags) {
		flags &= Get();
		flags_handler ^= flags;
	}
};

BaseType_t xTaskNotifyWaitBitsAllIndexed(UBaseType_t uxIndexToWaitOn, uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait);
BaseType_t xTaskNotifyWaitBitsAnyIndexed(UBaseType_t uxIndexToWaitOn, uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait);

}

