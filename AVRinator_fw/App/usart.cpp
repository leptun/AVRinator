#include "usart.hpp"
#include <assert.h>
#include <stdio.h>
#include <config.hpp>
#include <util.hpp>

namespace usart {

void USART::Setup() {
	hwdef->MX_USARTx_Init();
	flags = xEventGroupCreate();
	assert(flags);
	if (hwdef->rxBuf) {
		if (hwdef->rxDMA.DMAx) {
			// receive using DMA circular with interrupts for half, complete and UART idle
			LL_USART_EnableDMAReq_RX(hwdef->USARTx);
			LL_DMA_SetPeriphAddress(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel, LL_USART_DMA_GetRegAddr(hwdef->USARTx, LL_USART_DMA_REG_DATA_RECEIVE));
			LL_DMA_SetMemoryAddress(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel, (uint32_t)hwdef->rxBuf);
			LL_DMA_SetDataLength(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel, hwdef->rxBufSize);
			hwdef->rxDMA.clearIRQ(DMA_ISR_TEIF1 | DMA_ISR_HTIF1 | DMA_ISR_TCIF1);
			LL_DMA_EnableIT_TE(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel);
			LL_DMA_EnableIT_HT(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel);
			LL_DMA_EnableIT_TC(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel);
			LL_DMA_EnableChannel(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel);
			LL_USART_ClearFlag_IDLE(hwdef->USARTx);
			LL_USART_EnableIT_IDLE(hwdef->USARTx);
		} else {
			// receive using UART rxne
			LL_USART_EnableIT_RXNE(hwdef->USARTx);
		}
		LL_USART_EnableDirectionRx(hwdef->USARTx);
	}
	if (hwdef->txDMA.DMAx) {
		// transmit using DMA normal with interrupts for complete
		LL_USART_EnableDMAReq_TX(hwdef->USARTx);
		LL_DMA_SetPeriphAddress(hwdef->txDMA.DMAx, hwdef->txDMA.Channel, LL_USART_DMA_GetRegAddr(hwdef->USARTx, LL_USART_DMA_REG_DATA_TRANSMIT));
		hwdef->rxDMA.clearIRQ(DMA_ISR_TEIF1 | DMA_ISR_HTIF1 | DMA_ISR_TCIF1);
		LL_DMA_EnableIT_TE(hwdef->txDMA.DMAx, hwdef->txDMA.Channel);
		LL_DMA_EnableIT_TC(hwdef->txDMA.DMAx, hwdef->txDMA.Channel);
	}
}

size_t USART::receiveAny(uint8_t *buf, size_t maxlen) {
	while (rxHead == rxTail) {
		xEventGroupWaitBits(flags, FLAG_RX_AVAILABLE, pdTRUE, pdTRUE, portMAX_DELAY);
	}
	uint32_t pushed = 0;
	while (rxHead != rxTail && maxlen-- > 0) {
		uint32_t newTail = rxTail + 1;
		*(buf++) = hwdef->rxBuf[rxTail];
		if (newTail > hwdef->rxBufSize) {
			newTail = 0;
		}
		rxTail = newTail;
		pushed++;
	}
	return pushed;
}

void USART::receive(uint8_t *buf, size_t len) {
	while (len-- > 0) {
		while (rxHead == rxTail) {
			xEventGroupWaitBits(flags, FLAG_RX_AVAILABLE, pdTRUE, pdTRUE, portMAX_DELAY);
		}
		uint32_t newTail = rxTail + 1;
		*(buf++) = hwdef->rxBuf[rxTail];
		if (newTail > hwdef->rxBufSize) {
			newTail = 0;
		}
		rxTail = newTail;
	}
}

void USART::send(const uint8_t *buf, size_t len) {
	if (len <= 0) {
		return;
	}
	if (hwdef->txDMA.DMAx) {
		// transmit using DMA
		LL_DMA_SetMemoryAddress(hwdef->txDMA.DMAx, hwdef->txDMA.Channel, (uint32_t)buf);
		LL_DMA_SetDataLength(hwdef->txDMA.DMAx, hwdef->txDMA.Channel, len);
		LL_USART_EnableDirectionTx(hwdef->USARTx);
		LL_DMA_EnableChannel(hwdef->txDMA.DMAx, hwdef->txDMA.Channel);
	}
	else {
		// transmit using interrupts
		txbuf = buf;
		txndtr = len;
		LL_USART_EnableDirectionTx(hwdef->USARTx);
		LL_USART_EnableIT_TXE(hwdef->USARTx);
	}

	xEventGroupWaitBits(flags, FLAG_TX_COMPLETE, pdTRUE, pdTRUE, portMAX_DELAY);
}

void USART::setBaud(uint32_t baud) {
	LL_USART_DisableDirectionRx(hwdef->USARTx);
	if (hwdef->rxDMA.DMAx) {
		LL_USART_DisableDMAReq_RX(hwdef->USARTx);
	}
	if (hwdef->txDMA.DMAx) {
		LL_USART_DisableDMAReq_TX(hwdef->USARTx);
	}
	LL_USART_Disable(hwdef->USARTx);

	LL_USART_SetBaudRate(hwdef->USARTx, hwdef->periphclk, LL_USART_GetPrescaler(hwdef->USARTx), LL_USART_GetOverSampling(hwdef->USARTx), baud);

	LL_USART_Enable(hwdef->USARTx);
	if (hwdef->txDMA.DMAx) {
		LL_USART_EnableDMAReq_TX(hwdef->USARTx);
	}
	if (hwdef->rxDMA.DMAx) {
		LL_USART_EnableDMAReq_RX(hwdef->USARTx);
	}
	LL_USART_EnableDirectionRx(hwdef->USARTx);
}

BaseType_t USART::rx_push() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	uint32_t newHead = hwdef->rxBufSize - LL_DMA_GetDataLength(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel);
	if (newHead != rxHead && xEventGroupSetBitsFromISR(flags, FLAG_RX_AVAILABLE, &xHigherPriorityTaskWoken) != pdPASS) {
		Error_Handler();
	}
	rxHead = newHead;

	return xHigherPriorityTaskWoken;
}

void USART::irq_usart() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (LL_USART_IsActiveFlag_IDLE(hwdef->USARTx) && LL_USART_IsEnabledIT_IDLE(hwdef->USARTx)) {
		LL_USART_ClearFlag_IDLE(hwdef->USARTx);
		xHigherPriorityTaskWoken |= rx_push();
	}
	if (LL_USART_IsActiveFlag_RXNE(hwdef->USARTx) && LL_USART_IsEnabledIT_RXNE(hwdef->USARTx)) {
		uint32_t newHead = rxHead;
		hwdef->rxBuf[newHead++] = LL_USART_ReceiveData8(hwdef->USARTx);
		if (newHead >= hwdef->rxBufSize) {
			newHead = 0;
		}
		rxHead = newHead;
		if (xEventGroupSetBitsFromISR(flags, FLAG_RX_AVAILABLE, &xHigherPriorityTaskWoken) != pdPASS) {
			Error_Handler();
		}
	}
	if (LL_USART_IsActiveFlag_ORE(hwdef->USARTx) && LL_USART_IsEnabledIT_RXNE(hwdef->USARTx)) {
		LL_USART_ClearFlag_ORE(hwdef->USARTx);
//		puts("usart: overrun");
	}
	if (LL_USART_IsActiveFlag_TXE(hwdef->USARTx) && LL_USART_IsEnabledIT_TXE(hwdef->USARTx)) {
		LL_USART_TransmitData8(hwdef->USARTx, *(txbuf++));
		if (--txndtr == 0) {
			LL_USART_DisableIT_TXE(hwdef->USARTx);
			LL_USART_ClearFlag_TC(hwdef->USARTx);
			LL_USART_EnableIT_TC(hwdef->USARTx);
		}
	}
	if (LL_USART_IsActiveFlag_TC(hwdef->USARTx) && LL_USART_IsEnabledIT_TC(hwdef->USARTx)) {
		LL_USART_DisableIT_TC(hwdef->USARTx);
		LL_USART_DisableDirectionTx(hwdef->USARTx);

		if (xEventGroupSetBitsFromISR(flags, FLAG_TX_COMPLETE, &xHigherPriorityTaskWoken) != pdPASS) {
			Error_Handler();
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void USART::irq_dma_rx() {
	assert(hwdef->rxDMA.DMAx);
	uint32_t flags = hwdef->rxDMA.irq_handler();
	if (flags & (DMA_ISR_HTIF1 | DMA_ISR_TCIF1)) {
		rx_push();
	}
	else {
		Error_Handler();
	}
}

void USART::irq_dma_tx() {
	assert(hwdef->txDMA.DMAx);
	uint32_t flags = hwdef->txDMA.irq_handler();
	if (flags & (DMA_ISR_TCIF1)) {
		LL_DMA_DisableChannel(hwdef->txDMA.DMAx, hwdef->txDMA.Channel);
		LL_USART_EnableIT_TC(hwdef->USARTx);
	}
	else {
		Error_Handler();
	}
}

static uint8_t usart1_rxbuf[config::resources::isp_rxbuf_size];
static constexpr USART_Def usart1_def = {
	MX_USART1_Init,
	util::ioCast<USART_TypeDef>(USART1_BASE),
	config::clocks::pclk2,
	usart1_rxbuf,
	sizeof(usart1_rxbuf),
	util::LL_DMA_CHANNEL(util::ioCast<DMA_TypeDef>(DMA1_BASE), LL_DMA_CHANNEL_1), //rx
	util::LL_DMA_CHANNEL(util::ioCast<DMA_TypeDef>(DMA1_BASE), LL_DMA_CHANNEL_2), //tx
};
USART usart1(&usart1_def);
extern "C" void USART1_IRQHandler(void) { usart1.irq_usart(); }
extern "C" void DMA1_Channel1_IRQHandler(void) { usart1.irq_dma_rx(); }
extern "C" void DMA1_Channel2_IRQHandler(void) { usart1.irq_dma_tx(); }

static uint8_t usart2_rxbuf[config::resources::ttl_rxbuf_size];
static constexpr USART_Def usart2_def = {
	MX_USART2_UART_Init,
	util::ioCast<USART_TypeDef>(USART2_BASE),
	config::clocks::pclk1,
	usart2_rxbuf,
	sizeof(usart2_rxbuf),
	util::LL_DMA_CHANNEL(util::ioCast<DMA_TypeDef>(DMA2_BASE), LL_DMA_CHANNEL_1), //rx
	util::LL_DMA_CHANNEL(util::ioCast<DMA_TypeDef>(DMA2_BASE), LL_DMA_CHANNEL_2), //tx
};
USART usart2(&usart2_def);
extern "C" void USART2_IRQHandler(void) { usart2.irq_usart(); }
extern "C" void DMA2_Channel1_IRQHandler(void) { usart2.irq_dma_rx(); }
extern "C" void DMA2_Channel2_IRQHandler(void) { usart2.irq_dma_tx(); }

}
