#pragma once
#include <inttypes.h>
#include <main.h>
#include "util.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <event_groups.h>
#include <semphr.h>

namespace usart {

struct USART_Def {
	void (* const MX_USARTx_Init)(void);
	USART_TypeDef * const USARTx; // peripheral
	const uint32_t periphclk;
	const util::LL_DMA_CHANNEL rxDMA;
	const util::LL_DMA_CHANNEL txDMA;
};

class USART {
protected:
	// constant definition of the peripheral
	const USART_Def *hwdef;

	// rx buffer handling
	uint8_t *rxbuf;

	// tx buffer handling
	const uint8_t *txbuf;
	size_t txndtr;

	SemaphoreHandle_t mutex;
	StaticSemaphore_t mutex_staticBuf;
	void lock();
	void unlock();

	void setupRx();
	void setupTx();
	void startRx(uint8_t *buf, uint16_t len);
	void startTx(const uint8_t *buf, uint16_t len);
	void startIdle();

	virtual void idle_handler(BaseType_t &xHigherPriorityTaskWoken) = 0;
	virtual void rxne_handler(BaseType_t &xHigherPriorityTaskWoken) = 0;
	virtual void tc_handler(BaseType_t &xHigherPriorityTaskWoken) = 0;
	virtual void dmarx_handler(uint32_t flags, BaseType_t &xHigherPriorityTaskWoken) = 0;

public:
	USART(const USART_Def *hwdef) : hwdef(hwdef) {}
	USART(const USART_Def *hwdef, uint8_t *rxbuf) : hwdef(hwdef), rxbuf(rxbuf) {}
	void Setup();

	void setBaud(uint32_t baud);

	void irq_usart();
	void irq_dma_rx();
	void irq_dma_tx();
};

class AsyncUSART : public USART {
	enum EventFlags {
		FLAG_RX_AVAILABLE = 0x01,
		FLAG_RX_IDLE = 0x02,
		FLAG_TX_COMPLETE = 0x04,
	};

	const uint16_t rxBufSize;
	// tasks that need to be notified when events happens
	TaskHandle_t taskRx;
	TaskHandle_t taskTx;

	// rx circular buffer handling
	uint32_t rxHead;
	uint32_t rxTail;

	void idle_handler(BaseType_t &xHigherPriorityTaskWoken) override;
	void rxne_handler(BaseType_t &xHigherPriorityTaskWoken) override;
	void tc_handler(BaseType_t &xHigherPriorityTaskWoken) override;
	void dmarx_handler(uint32_t flags, BaseType_t &xHigherPriorityTaskWoken) override;

	inline bool rx_empty() {
		return rxHead == rxTail;
	}

	uint32_t rx_pending() {
		return (rxHead + rxBufSize - rxTail) % rxBufSize;
	}

	void rx_push(BaseType_t &xHigherPriorityTaskWoken);

public:
	AsyncUSART(const USART_Def *hwdef, uint8_t *rxbuf, uint16_t rxBufSize) : USART(hwdef, rxbuf), rxBufSize(rxBufSize) {}
	void Setup();

	void receive(uint8_t *buf, size_t len);
	void awaitRx();
	size_t receiveAny(uint8_t *buf, size_t maxlen);
	void send(const uint8_t *buf, size_t len);
};

class SyncUSART : public USART {
	enum EventFlags {
		FLAG_TX_COMPLETE = 0x01,
		FLAG_RX_COMPLETE = 0x02,
	};

	// task that needs to be notified when events happens
	TaskHandle_t task;

	size_t rxndtr;

	void idle_handler(BaseType_t &xHigherPriorityTaskWoken) override { Error_Handler(); }
	void rxne_handler(BaseType_t &xHigherPriorityTaskWoken) override;
	void tc_handler(BaseType_t &xHigherPriorityTaskWoken) override;
	void dmarx_handler(uint32_t flags, BaseType_t &xHigherPriorityTaskWoken) override;

public:
	SyncUSART(const USART_Def *hwdef) : USART(hwdef) {}
	void Setup();

	void txrx(uint8_t *rxbuf, const uint8_t *txbuf, size_t len);
};

extern SyncUSART usart1;
extern AsyncUSART usart2;

}
