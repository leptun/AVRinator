#include "usart.hpp"
#include <stdio.h>
#include <config.hpp>
#include <util.hpp>

namespace usart {

static constexpr UBaseType_t notifyIndex = tskDEFAULT_INDEX_TO_NOTIFY;

void USART::lock() {
	if (xSemaphoreTakeRecursive(mutex, portMAX_DELAY) != pdTRUE) {
		Error_Handler();
	}
}
void USART::unlock() {
	if (xSemaphoreGiveRecursive(mutex) != pdTRUE) {
		Error_Handler();
	}
}

void USART::setupRx() {
	lock();
	if (hwdef->rxDMA.DMAx) {
		// receive using DMA
		LL_USART_EnableDMAReq_RX(hwdef->USARTx);
		LL_DMA_SetPeriphAddress(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel, LL_USART_DMA_GetRegAddr(hwdef->USARTx, LL_USART_DMA_REG_DATA_RECEIVE));
		hwdef->rxDMA.clearIRQ(DMA_ISR_TEIF1 | DMA_ISR_HTIF1 | DMA_ISR_TCIF1);
		LL_DMA_EnableIT_TE(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel);
		if (LL_DMA_GetMode(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel) == LL_DMA_MODE_CIRCULAR) {
			LL_DMA_EnableIT_HT(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel);
		}
		LL_DMA_EnableIT_TC(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel);
	}
	unlock();
}

void USART::setupTx() {
	lock();
	if (hwdef->txDMA.DMAx) {
		// transmit using DMA normal with interrupts for complete
		LL_USART_EnableDMAReq_TX(hwdef->USARTx);
		LL_DMA_SetPeriphAddress(hwdef->txDMA.DMAx, hwdef->txDMA.Channel, LL_USART_DMA_GetRegAddr(hwdef->USARTx, LL_USART_DMA_REG_DATA_TRANSMIT));
		hwdef->rxDMA.clearIRQ(DMA_ISR_TEIF1 | DMA_ISR_HTIF1 | DMA_ISR_TCIF1);
		LL_DMA_EnableIT_TE(hwdef->txDMA.DMAx, hwdef->txDMA.Channel);
		LL_DMA_EnableIT_TC(hwdef->txDMA.DMAx, hwdef->txDMA.Channel);
	}
	unlock();
}

void USART::startRx(uint8_t *buf, uint16_t len) {
	lock();
	rxbuf = buf;
	if (hwdef->rxDMA.DMAx) {
		LL_DMA_SetMemoryAddress(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel, (uint32_t)buf);
		LL_DMA_SetDataLength(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel, len);
		LL_DMA_EnableChannel(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel);
	}
	else {
		// receive using UART rxne
		LL_USART_EnableIT_RXNE(hwdef->USARTx);
	}
	LL_USART_EnableDirectionRx(hwdef->USARTx);
	unlock();
}

void USART::startTx(const uint8_t *buf, uint16_t len) {
	lock();
	txbuf = buf;
	txndtr = len;
	if (hwdef->txDMA.DMAx) {
		// transmit using DMA
		LL_DMA_SetMemoryAddress(hwdef->txDMA.DMAx, hwdef->txDMA.Channel, (uint32_t)buf);
		LL_DMA_SetDataLength(hwdef->txDMA.DMAx, hwdef->txDMA.Channel, len);
		LL_USART_EnableDirectionTx(hwdef->USARTx);
		LL_DMA_EnableChannel(hwdef->txDMA.DMAx, hwdef->txDMA.Channel);
	}
	else {
		// transmit using interrupts
		LL_USART_EnableDirectionTx(hwdef->USARTx);
		LL_USART_EnableIT_TXE(hwdef->USARTx);
	}
	unlock();
}

void USART::startIdle() {
	lock();
	LL_USART_ClearFlag_IDLE(hwdef->USARTx);
	LL_USART_EnableIT_IDLE(hwdef->USARTx);
	unlock();
}

void USART::Setup() {
	mutex = xSemaphoreCreateRecursiveMutexStatic(&mutex_staticBuf);
	if (!mutex) {
		Error_Handler();
	}

	lock();
	hwdef->MX_USARTx_Init();
	unlock();

	setupRx();
	setupTx();
}

void USART::setBaud(uint32_t baud) {
	lock();
	bool wasEnabled = LL_USART_IsEnabled(hwdef->USARTx);
	LL_USART_Disable(hwdef->USARTx);
	LL_USART_SetBaudRate(hwdef->USARTx, hwdef->periphclk, LL_USART_GetPrescaler(hwdef->USARTx), LL_USART_GetOverSampling(hwdef->USARTx), baud);
	if (wasEnabled) {
		LL_USART_Enable(hwdef->USARTx);
	}
	unlock();
}

void USART::irq_usart() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (LL_USART_IsActiveFlag_IDLE(hwdef->USARTx) && LL_USART_IsEnabledIT_IDLE(hwdef->USARTx)) {
		LL_USART_ClearFlag_IDLE(hwdef->USARTx);
		idle_handler(xHigherPriorityTaskWoken);
	}
	if (LL_USART_IsActiveFlag_RXNE(hwdef->USARTx) && LL_USART_IsEnabledIT_RXNE(hwdef->USARTx)) {
		rxne_handler(xHigherPriorityTaskWoken);
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
		tc_handler(xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void USART::irq_dma_rx() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (!hwdef->rxDMA.DMAx) {
		Error_Handler();
	}
	uint32_t flags = hwdef->rxDMA.irq_handler();
	dmarx_handler(flags, xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void USART::irq_dma_tx() {
	if (!hwdef->txDMA.DMAx) {
		Error_Handler();
	}
	uint32_t flags = hwdef->txDMA.irq_handler();
	if (flags & (DMA_ISR_TCIF1)) {
		LL_DMA_DisableChannel(hwdef->txDMA.DMAx, hwdef->txDMA.Channel);
		LL_USART_EnableIT_TC(hwdef->USARTx);
	}
	else {
		Error_Handler();
	}
}

void AsyncUSART::idle_handler(BaseType_t &xHigherPriorityTaskWoken) {
	if (hwdef->rxDMA.DMAx) {
		rx_push(xHigherPriorityTaskWoken);
	}
	if (taskRx && xTaskNotifyIndexedFromISR(taskRx, notifyIndex, FLAG_RX_IDLE, eSetBits, &xHigherPriorityTaskWoken) != pdPASS) {
		Error_Handler();
	}
}

void AsyncUSART::rxne_handler(BaseType_t &xHigherPriorityTaskWoken) {
	uint32_t newHead = rxHead;
	rxbuf[newHead++] = LL_USART_ReceiveData8(hwdef->USARTx);
	if (newHead >= rxBufSize) {
		newHead = 0;
	}
	rxHead = newHead;
	if (taskRx && xTaskNotifyIndexedFromISR(taskRx, notifyIndex, FLAG_RX_AVAILABLE, eSetBits, &xHigherPriorityTaskWoken) != pdPASS) {
		Error_Handler();
	}
}

void AsyncUSART::tc_handler(BaseType_t &xHigherPriorityTaskWoken) {
	if (taskTx && xTaskNotifyIndexedFromISR(taskTx, notifyIndex, FLAG_TX_COMPLETE, eSetBits, &xHigherPriorityTaskWoken) != pdPASS) {
		Error_Handler();
	}
}

void AsyncUSART::dmarx_handler(uint32_t flags, BaseType_t &xHigherPriorityTaskWoken) {
	if (flags & (DMA_ISR_HTIF1 | DMA_ISR_TCIF1)) {
		rx_push(xHigherPriorityTaskWoken);
	}
	else {
		Error_Handler();
	}
}

void AsyncUSART::rx_push(BaseType_t &xHigherPriorityTaskWoken) {
	if (!hwdef->rxDMA.DMAx) {
		Error_Handler();
	}
	uint32_t newHead = rxBufSize - LL_DMA_GetDataLength(hwdef->rxDMA.DMAx, hwdef->rxDMA.Channel);
	if (newHead != rxHead && taskRx && xTaskNotifyIndexedFromISR(taskRx, notifyIndex, FLAG_RX_AVAILABLE, eSetBits, &xHigherPriorityTaskWoken) != pdPASS) {
		Error_Handler();
	}
	rxHead = newHead;
}

void AsyncUSART::Setup() {
	USART::Setup();
	if (rxbuf) {
		startRx(rxbuf, rxBufSize);
		startIdle();
	}
}

void AsyncUSART::receive(uint8_t *buf, size_t len) {
	while (len-- > 0) {
		awaitRx();
		lock();
		uint32_t newTail = rxTail + 1;
		*(buf++) = rxbuf[rxTail];
		if (newTail >= rxBufSize) {
			newTail = 0;
		}
		rxTail = newTail;
		unlock();
	}


}

void AsyncUSART::awaitRx() {
	lock();
	if (taskRx) {
		Error_Handler();
	}
	taskRx = xTaskGetCurrentTaskHandle();
	while (rxHead == rxTail) {
		unlock();
		util::xTaskNotifyWaitBitsAnyIndexed(notifyIndex, 0, FLAG_RX_AVAILABLE, NULL, portMAX_DELAY);
		lock();
	}
	taskRx = nullptr;
	unlock();
}

size_t AsyncUSART::receiveAny(uint8_t *buf, size_t maxlen) {
	lock();
	uint32_t pushed = 0;
	while (rxHead != rxTail && maxlen-- > 0) {
		uint32_t newTail = rxTail + 1;
		*(buf++) = rxbuf[rxTail];
		if (newTail >= rxBufSize) {
			newTail = 0;
		}
		rxTail = newTail;
		pushed++;
	}
	unlock();

	return pushed;
}

void AsyncUSART::send(const uint8_t *buf, size_t len) {
	if (!len) {
		return;
	}

	lock();
	if (taskTx) {
		// a task is already transmitting on this usart
		Error_Handler();
	}
	taskTx = xTaskGetCurrentTaskHandle();
	startTx(buf, len);
	unlock();

	util::xTaskNotifyWaitBitsAnyIndexed(notifyIndex, 0, FLAG_TX_COMPLETE, NULL, portMAX_DELAY);

	lock();
	txndtr = 0;
	taskTx = nullptr;
	unlock();
}

void SyncUSART::Setup() {
	USART::Setup();
}

void SyncUSART::rxne_handler(BaseType_t &xHigherPriorityTaskWoken) {
	*(rxbuf++) = LL_USART_ReceiveData8(hwdef->USARTx);
	if (--rxndtr == 0) {
		LL_USART_DisableDirectionRx(hwdef->USARTx);
		if (task && xTaskNotifyIndexedFromISR(task, notifyIndex, FLAG_RX_COMPLETE, eSetBits, &xHigherPriorityTaskWoken) != pdPASS) {
			Error_Handler();
		}
	}
}

void SyncUSART::tc_handler(BaseType_t &xHigherPriorityTaskWoken) {
	if (task && xTaskNotifyIndexedFromISR(task, notifyIndex, FLAG_TX_COMPLETE, eSetBits, &xHigherPriorityTaskWoken) != pdPASS) {
		Error_Handler();
	}
}

void SyncUSART::dmarx_handler(uint32_t flags, BaseType_t &xHigherPriorityTaskWoken) {
	if (flags & DMA_ISR_TCIF1) {
		rxndtr = 0;
		if (task && xTaskNotifyIndexedFromISR(task, notifyIndex, FLAG_RX_COMPLETE, eSetBits, &xHigherPriorityTaskWoken) != pdPASS) {
			Error_Handler();
		}
	}
	else {
		Error_Handler();
	}
}

void SyncUSART::txrx(uint8_t *rxbuf, const uint8_t *txbuf, size_t len) {
	if (!len) {
		return;
	}

	lock();
	task = xTaskGetCurrentTaskHandle();
	if (rxbuf) {
		rxndtr = len;
		startRx(rxbuf, len);
	}
	startTx(txbuf, len);

	util::xTaskNotifyWaitBitsAllIndexed(notifyIndex, 0, FLAG_TX_COMPLETE | (rxbuf ? FLAG_RX_COMPLETE : 0), NULL, portMAX_DELAY);
	txndtr = 0;

	unlock();
}



static constexpr USART_Def usart1_def = {
	MX_USART1_Init,
	util::ioCast<USART_TypeDef>(USART1_BASE),
	config::clocks::pclk2,
	util::LL_DMA_CHANNEL(util::ioCast<DMA_TypeDef>(DMA1_BASE), LL_DMA_CHANNEL_1), //rx
	util::LL_DMA_CHANNEL(util::ioCast<DMA_TypeDef>(DMA1_BASE), LL_DMA_CHANNEL_2), //tx
};
SyncUSART usart1(&usart1_def);
extern "C" void USART1_IRQHandler(void) { usart1.irq_usart(); }
extern "C" void DMA1_Channel1_IRQHandler(void) { usart1.irq_dma_rx(); }
extern "C" void DMA1_Channel2_IRQHandler(void) { usart1.irq_dma_tx(); }

static uint8_t usart2_rxbuf[config::resources::ttl_rxbuf_size] __attribute__((section(".buffers")));
static constexpr USART_Def usart2_def = {
	MX_USART2_UART_Init,
	util::ioCast<USART_TypeDef>(USART2_BASE),
	config::clocks::pclk1,

	util::LL_DMA_CHANNEL(util::ioCast<DMA_TypeDef>(DMA2_BASE), LL_DMA_CHANNEL_1), //rx
	util::LL_DMA_CHANNEL(util::ioCast<DMA_TypeDef>(DMA2_BASE), LL_DMA_CHANNEL_2), //tx
};
AsyncUSART usart2(&usart2_def, usart2_rxbuf, sizeof(usart2_rxbuf));
extern "C" void USART2_IRQHandler(void) { usart2.irq_usart(); }
extern "C" void DMA2_Channel1_IRQHandler(void) { usart2.irq_dma_rx(); }
extern "C" void DMA2_Channel2_IRQHandler(void) { usart2.irq_dma_tx(); }

}
