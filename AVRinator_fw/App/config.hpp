#pragma once
#include "util.hpp"
#include <inttypes.h>
#include <stddef.h>
#include <tusb.h>
#include "usart.hpp"

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
static constexpr size_t ISP_stack_depth = 256;
static constexpr size_t TTL_stack_depth = 256;
static constexpr size_t usbd_stack_depth = 256 * (CFG_TUSB_DEBUG ? 2 : 1);

static usart::USART * const isp_usart = &usart::usart1;
static constexpr size_t isp_rxbuf_size = 256;

static usart::USART * const ttl_usart = &usart::usart2;
static constexpr size_t ttl_rxbuf_size = 256;
}

static constexpr uint32_t eeprom_apply_delay = 1000;

}
