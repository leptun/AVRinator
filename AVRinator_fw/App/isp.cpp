#include "isp.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include "config.hpp"
#include "usb.hpp"
#include "defs/command.h"


namespace isp {

namespace param {
static constexpr uint8_t signature[] = "AVRISP_2";
static constexpr uint16_t buildNumber = 0;
static uint8_t hwVersion;
static uint8_t swVersionMajor;
static uint8_t swVersionMinor;
static uint8_t vTarget;
}

namespace state {
static bool programmingEnabled;
static uint8_t error;
}

class CommandParser {
	enum class State {
		start,
		seq_number,
		message_size,
		token,
		data,
		checksum,
	} state;

	static constexpr TickType_t parserTimeout = pdMS_TO_TICKS(2000);

	struct __attribute__((packed)) Header {
		uint8_t start;
		uint8_t seqNum;
		uint16_t dataLen;
		uint8_t token;

		void swapDataLen() {
			dataLen = (dataLen >> 8) | (dataLen << 8);
		}
	} header;
	uint8_t dataBuf[512];

	static uint8_t updateChecksum(uint8_t checksum, const void *data, size_t len) {
		const uint8_t *data8_t = (const uint8_t *)data;
		while (len-- > 0) {
			checksum ^= *(data8_t++);
		}
		return checksum;
	}

	void restartSequenceHandler() {
		// todo reset programmer to default state
//		programmingEnabled = false;
	}

	void errorHandler() {
		state = State::start;
	}

	void processCommand() {
		switch(dataBuf[0]) {
		case CMD_SIGN_ON: {
			dataBuf[2] = sizeof(param::signature) - 1;
			memcpy(&dataBuf[3], param::signature, sizeof(param::signature) - 1);
			header.dataLen = 3 + (sizeof(param::signature) - 1);
			dataBuf[1] = STATUS_CMD_OK;
		} break;
		case CMD_GET_PARAMETER: {
			switch(dataBuf[1]) {
			case PARAM_BUILD_NUMBER_LOW:
				dataBuf[2] = (uint8_t)(param::buildNumber);
				goto success;
			case PARAM_BUILD_NUMBER_HIGH:
				dataBuf[2] = (uint8_t)(param::buildNumber >> 8);
				goto success;
			case PARAM_HW_VER:
				dataBuf[2] = param::hwVersion;
				goto success;
			case PARAM_SW_MAJOR:
				dataBuf[2] = param::swVersionMajor;
				goto success;
			case PARAM_SW_MINOR:
				dataBuf[2] = param::swVersionMinor;
				goto success;
			case PARAM_VTARGET:
				dataBuf[2] = param::vTarget;
				goto success;
			success:
				dataBuf[1] = STATUS_CMD_OK;
				header.dataLen = 3;
				break;
			default:
				dataBuf[1] = STATUS_CMD_FAILED;
				header.dataLen = 2;
			}

		} break;
		default:
			header.dataLen = 2;
			dataBuf[1] = STATUS_CMD_UNKNOWN;
		}

		// Send response
		header.swapDataLen();
		if (usb::cdc_write(config::cdc_itf_isp, (uint8_t*)&header, sizeof(header), parserTimeout) < 0) {
			errorHandler();
			return;
		}
		header.swapDataLen();
		if (usb::cdc_write(config::cdc_itf_isp, (uint8_t*)&dataBuf, header.dataLen, parserTimeout) < 0) {
			errorHandler();
			return;
		}
		uint8_t checksum = updateChecksum(0, &header, sizeof(header));
		checksum = updateChecksum(checksum, dataBuf, header.dataLen);
		if (usb::cdc_write(config::cdc_itf_isp, &checksum, 1, parserTimeout) < 0) {
			errorHandler();
			return;
		}
		if (!usb::cdc_write_flush(config::cdc_itf_isp)) {
			errorHandler();
			return;
		}
	}

public:
	bool step() {
		switch (state) {
		case State::start: {
			if (usb::cdc_read(config::cdc_itf_isp, &header.start, 1, parserTimeout) < 0) {
				errorHandler();
			}
			else if (header.start == MESSAGE_START) {
				state = State::seq_number;
			}
			else {
				goto error;
			}
		} break;
		case State::seq_number: {
			uint8_t lastSeqNum = header.seqNum;
			if (usb::cdc_read(config::cdc_itf_isp, &header.seqNum, 1, parserTimeout) < 0) {
				goto error;
			}
			else if (lastSeqNum + 1 == header.seqNum) {
				state = State::message_size;
			}
			else if (header.seqNum == 1) {
				restartSequenceHandler();
				state = State::message_size;
			}
			else {
				goto error;
			}
		} break;
		case State::message_size: {
			if (usb::cdc_read(config::cdc_itf_isp, (uint8_t*)&header.dataLen, 2, parserTimeout) < 0) {
				goto error;
			}
			else {
				header.swapDataLen();
				if (header.dataLen > sizeof(dataBuf) || header.dataLen == 0) {
					goto error;
				}
				else {
					state = State::token;
				}
			}
		} break;
		case State::token: {
			if (usb::cdc_read(config::cdc_itf_isp, &header.token, 1, parserTimeout) < 0) {
				goto error;
			}
			else if (header.token == TOKEN) {
				state = State::data;
			}
			else {
				goto error;
			}
		} break;
		case State::data: {
			if (usb::cdc_read(config::cdc_itf_isp, dataBuf, header.dataLen, parserTimeout) < 0) {
				goto error;
			}
			else {
				state = State::checksum;
			}
		} break;
		case State::checksum: {
			uint8_t expectedChecksum = updateChecksum(0, &header, sizeof(header));
			expectedChecksum = updateChecksum(expectedChecksum, dataBuf, header.dataLen);
			uint8_t c;
			if (usb::cdc_read(config::cdc_itf_isp, &c, 1, parserTimeout) < 0) {
				goto error;
			}
			else if (c == expectedChecksum) {
				processCommand();
				state = State::start;
			}
			else {
				goto error;
			}
		} break;
		error:
			errorHandler();
			return false;
		default:
			Error_Handler();
			return false;
		}
		return true;
	}
} static parser;


static void taskISP(void *pvParameters) {
	for (;;) {
		if (!parser.step()) {
			vTaskDelay(10);
		}
//		int rx = usb::cdc_read_any(config::cdc_itf_isp, dataBuf, sizeof(dataBuf));
//		if (rx > 0) {
//			usb::cdc_write(config::cdc_itf_isp, dataBuf, rx);
//		} else if (!usb::cdc_write_push(config::cdc_itf_isp)) {
//			// nothing left to push, wait for more cdc rx to happen
//			usb::cdc_awaitRx(config::cdc_itf_isp);
//		}
	}
}
static TaskHandle_t taskHandleISP;
StackType_t ISP_stack[config::resources::ISP_stack_depth] __attribute__((section(".stack")));
StaticTask_t ISP_taskdef;

void Setup() {
	if (!(taskHandleISP = xTaskCreateStatic(
			taskISP,
			"ISP",
			config::resources::ISP_stack_depth,
			NULL,
			config::task_priorities::ISP,
			ISP_stack,
			&ISP_taskdef
	))) {
		Error_Handler();
	}
}

void Notify(uint32_t flags) {
	if (taskHandleISP && xTaskNotifyIndexed(taskHandleISP, 1, flags, eSetBits) != pdPASS) {
		Error_Handler();
	}
}

uint8_t getError() {
	return state::error;
}

bool getProgrammingEnabled() {
	return state::programmingEnabled;
}


}
