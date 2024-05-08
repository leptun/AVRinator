#include "adc.hpp"
#include <main.h>
#include "config.hpp"
#include <FreeRTOS.h>
#include <timers.h>

namespace adc {


struct RegularConversion {
	uint16_t cc2;
	uint16_t cc1;
	uint16_t vrefint1;
	uint16_t vsense_lv;
	uint16_t vrefint2;
	uint16_t isense_out;
} __attribute__((packed)) __attribute__((aligned(4)));

static constexpr util::LL_DMA_CHANNEL adcDMA = util::LL_DMA_CHANNEL(util::ioCast<DMA_TypeDef>(DMA1_BASE), LL_DMA_CHANNEL_3);

static RegularConversion measurementBuf __attribute__((section(".buffers")));

static util::MinMax<uint16_t> VDD;
static util::MinMax<uint16_t> targetVCC;
static util::MinMax<uint16_t> targetISenseAmplified;

static TimerHandle_t adc_regular_timerHandle;
static StaticTimer_t adc_regular_staticBuf;

static void processRegularMeasurement() {
	VDD.update(__LL_ADC_CALC_VREFANALOG_VOLTAGE(measurementBuf.vrefint1, config::adc::resolution));
	targetVCC.update(__LL_ADC_CALC_DATA_TO_VOLTAGE(measurementBuf.vrefint1, measurementBuf.vsense_lv, config::adc::resolution));
	targetISenseAmplified.update(__LL_ADC_CALC_DATA_TO_VOLTAGE(measurementBuf.vrefint2, measurementBuf.isense_out, config::adc::resolution));
}

extern "C"
void DMA1_Channel3_IRQHandler(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t flags = adcDMA.irq_handler();

	if (flags & (DMA_ISR_TCIF1)) {
		LL_DMA_DisableChannel(adcDMA.DMAx, adcDMA.Channel);
		processRegularMeasurement();
	}
	else {
		Error_Handler();
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

extern "C"
void ADC1_2_IRQHandler(void) {

}

static void startRegularMeasurement(TimerHandle_t xTimer) {
	LL_DMA_SetMemoryAddress(adcDMA.DMAx, adcDMA.Channel, (uint32_t)&measurementBuf);
	LL_DMA_SetDataLength(adcDMA.DMAx, adcDMA.Channel, sizeof(measurementBuf));
	LL_DMA_EnableChannel(adc::adcDMA.DMAx, adcDMA.Channel);

	LL_ADC_REG_StartConversion(ADC1);
}

void Setup() {
	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	while (LL_ADC_IsCalibrationOnGoing(ADC1)) { vTaskDelay(1); }
	LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
	while (LL_ADC_IsCalibrationOnGoing(ADC2)) { vTaskDelay(1); }

	LL_DMA_SetPeriphAddress(adcDMA.DMAx, adcDMA.Channel, (uint32_t)&ADC12_COMMON->CDR);
	adcDMA.clearIRQ(DMA_ISR_TEIF1 | DMA_ISR_TCIF1);
	LL_DMA_EnableIT_TE(adcDMA.DMAx, adcDMA.Channel);
	LL_DMA_EnableIT_TC(adcDMA.DMAx, adcDMA.Channel);

	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);

	adc_regular_timerHandle = xTimerCreateStatic("adc_regular", pdMS_TO_TICKS(config::adc::adc_regular_period), true, 0, startRegularMeasurement, &adc_regular_staticBuf);
	xTimerStart(adc_regular_timerHandle, 100);
}

util::MinMax<uint16_t> getMinMaxVDD() {
	return VDD;
}

util::MinMax<uint16_t> getMinMaxTargetVCC() {
	return targetVCC;
}

}
