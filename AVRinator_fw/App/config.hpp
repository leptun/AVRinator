#pragma once
#include "util.hpp"
#include <inttypes.h>
#include <stddef.h>
#include <tusb.h>
#include "usart.hpp"
#include <FreeRTOS.h>

namespace config {

static constexpr uint32_t system_clock_frequency = 170000000;

namespace clocks {
static constexpr uint32_t hclk = system_clock_frequency;
static constexpr uint32_t pclk1 = hclk / 1;
static constexpr uint32_t pclk2 = hclk / 1;
static constexpr uint32_t hsi = HSI_VALUE;
static constexpr uint32_t hse = HSE_VALUE;
static constexpr uint32_t lsi = LSI_VALUE;
static constexpr uint32_t lse = LSE_VALUE;
static constexpr uint32_t pll48clk = 48000000;
}


static constexpr uint8_t cdc_itf_isp = 0;
static constexpr uint8_t cdc_itf_ttl = 1;

namespace resources {
static constexpr configSTACK_DEPTH_TYPE ISP_stack_depth = configMINIMAL_STACK_SIZE;
static constexpr configSTACK_DEPTH_TYPE TTL_stack_depth = configMINIMAL_STACK_SIZE;
static constexpr configSTACK_DEPTH_TYPE usbd_stack_depth = 256 * (CFG_TUSB_DEBUG ? 2 : 1);

static constexpr usart::SyncUSART& isp_usart = usart::usart1;

static constexpr usart::AsyncUSART& ttl_usart = usart::usart2;
static constexpr size_t ttl_rxbuf_size = 2048;
static constexpr size_t ttl_rxtransfer_size = 512;
static constexpr size_t ttl_txtransfer_size = 512;
}

namespace task_priorities {
static constexpr UBaseType_t usbd = configMAX_PRIORITIES - 1;
static constexpr UBaseType_t ISP = 4;
static constexpr UBaseType_t TTLrx = 2;
static constexpr UBaseType_t TTLtx = 3;
}

namespace adc {
static constexpr uint32_t resolution = LL_ADC_RESOLUTION_12B;
static constexpr uint32_t adc_regular_period = 10;
}

static constexpr uint32_t eeprom_apply_delay = 200;

}
