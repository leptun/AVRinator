#include "isp.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include "config.hpp"
#include "usb.hpp"
#include "defs/command.h"
#include "pavr2.hpp"
#include <timing_precise.hpp>


namespace isp {

static constexpr usart::SyncUSART& com = config::resources::isp_usart;

namespace param {
static constexpr uint8_t signature[] = "AVRISP_2";
static constexpr uint16_t buildNumber = 0;
static constexpr uint8_t vTarget = 50; //5.0V
static uint8_t param_controller_init;
}

namespace state {
static bool programmingEnabled;
static uint8_t error;
static uint32_t address;
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
	static constexpr TickType_t sendTimeout = pdMS_TO_TICKS(500);
	static constexpr TickType_t bsyRdyTimeout = pdMS_TO_TICKS(500);
	static constexpr uint32_t maxPageSize = 256; //bytes

	struct __attribute__((packed)) Header {
		uint8_t start;
		uint8_t seqNum;
		uint16_t dataLen;
		uint8_t token;

		void swapDataLen() {
			dataLen = (dataLen >> 8) | (dataLen << 8);
		}
	} header;
	uint8_t dataBuf[275];

	struct __attribute__((packed)) IspCommand {
		uint8_t val[4];

		uint8_t& operator[] (std::size_t idx) { return val[idx]; }
		const uint8_t& operator[] (std::size_t idx) const { return val[idx]; }

		uint8_t *raw() { return val; }
		const uint8_t *raw() const { return val; }
	};

	// used for sending and receiving data in bursts
	IspCommand cmdBuf[maxPageSize];

	static uint8_t updateChecksum(uint8_t checksum, const void *data, size_t len) {
		const uint8_t *data8_t = (const uint8_t *)data;
		while (len-- > 0) {
			checksum ^= *(data8_t++);
		}
		return checksum;
	}

	void targetEnterISP() {
		taskENTER_CRITICAL();

		pins::ISP::T_NRESET.Write(false); //enter reset mode with undefined SCK pin, possibly high
		delay_us_precise(1);
		pins::setIspOutput(true); //start outputing the SPI signals, this sets the SCK to be low

		// give reset a positive pulse to reset the serial programming state machine
		pins::ISP::T_NRESET.Write(true);
		delay_us_precise(1);
		pins::ISP::T_NRESET.Write(false);

		taskEXIT_CRITICAL();
	}

	void targetReleaseISP() {
		pins::setIspOutput(false);
		pins::ISP::T_NRESET.Write(true);
	}

	void targetForceRelease() {
		if (state::programmingEnabled) {
			state::programmingEnabled = false;
			targetReleaseISP();
			vTaskDelay(pdMS_TO_TICKS(20));
		}
		state::address = 0;
	}

	bool waitRdyBsy() {
		TimeOut_t xTimeOut;
		vTaskSetTimeOutState(&xTimeOut);
		TickType_t xTicksToWait = bsyRdyTimeout;

		while (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE) {
			IspCommand cmd = { 0xf0 };
			com.txrx(cmd.raw(), cmd.raw(), sizeof(cmd));
			if (!(cmd[3] & 0x01)) {
				return true;
			}

			vTaskDelay(pdMS_TO_TICKS(1));
		}

		//timeout
		return false;
	}

	bool waitValuePolling(uint8_t cmd3, uint8_t expectedVal, uint8_t delay) {
		TimeOut_t xTimeOut;
		vTaskSetTimeOutState(&xTimeOut);
		TickType_t xTicksToWait = pdMS_TO_TICKS(delay);

		while (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE) {
			IspCommand cmd = { cmd3, (uint8_t)(state::address >> 8), (uint8_t)state::address, 0x00 };
			IspCommand response;
			com.txrx(response.raw(), cmd.raw(), sizeof(response));
			if (response[3] == expectedVal) {
				return true;
			}

			vTaskDelay(pdMS_TO_TICKS(1));
		}

		//timeout
		return false;
	}

	void cmdSignOn() {
		dataBuf[2] = sizeof(param::signature) - 1;
		memcpy(&dataBuf[3], param::signature, sizeof(param::signature) - 1);
		dataBuf[1] = STATUS_CMD_OK;
		header.dataLen = 3 + (sizeof(param::signature) - 1);
	}

	void cmdSetParameter() {
		switch(dataBuf[1]) {
		case PARAM_CONTROLLER_INIT:
			param::param_controller_init = dataBuf[2];
			goto success;
		success:
			dataBuf[1] = STATUS_CMD_OK;
			header.dataLen = 2;
			break;
		default:
			dataBuf[1] = STATUS_CMD_FAILED;
			header.dataLen = 2;
		}
	}

	void cmdGetParameter() {
		switch(dataBuf[1]) {
		case PARAM_BUILD_NUMBER_LOW:
			dataBuf[2] = (uint8_t)(param::buildNumber);
			goto success;
		case PARAM_BUILD_NUMBER_HIGH:
			dataBuf[2] = (uint8_t)(param::buildNumber >> 8);
			goto success;
		case PARAM_HW_VER:
			dataBuf[2] = pavr2::getSetting(PAVR2_SETTING_HARDWARE_VERSION);
			goto success;
		case PARAM_SW_MAJOR:
			dataBuf[2] = pavr2::getSetting(PAVR2_SETTING_SOFTWARE_VERSION_MAJOR);
			goto success;
		case PARAM_SW_MINOR:
			dataBuf[2] = pavr2::getSetting(PAVR2_SETTING_SOFTWARE_VERSION_MINOR);
			goto success;
		case PARAM_VTARGET:
			dataBuf[2] = param::vTarget;
			goto success;
		case PARAM_CONTROLLER_INIT:
			dataBuf[2] = param::param_controller_init;
			goto success;
		success:
			dataBuf[1] = STATUS_CMD_OK;
			header.dataLen = 3;
			break;
		default:
			dataBuf[1] = STATUS_CMD_FAILED;
			header.dataLen = 2;
		}
	}

	void cmdLoadAddress() {
		state::address = (
				((uint32_t)dataBuf[1] << 24) |
				((uint32_t)dataBuf[2] << 16) |
				((uint32_t)dataBuf[3] << 8) |
				((uint32_t)dataBuf[4])
		);
		dataBuf[1] = STATUS_CMD_OK;
		header.dataLen = 2;
	}

	void cmdEnterProgmode() {
//		const uint8_t timeout = dataBuf[1];
		const uint8_t stabDelay = dataBuf[2];
		const uint8_t cmdexeDelay = dataBuf[3];
		const uint8_t synchLoops = dataBuf[4];
		const uint8_t byteDelay = dataBuf[5];
		const uint8_t pollValue = dataBuf[6];
		const uint8_t pollIndex = dataBuf[7];

		const IspCommand cmd = {
				dataBuf[8],
				dataBuf[9],
				dataBuf[10],
				dataBuf[11],
		};

		if (pollIndex > 4) {
			goto failed;
		}

		if (state::programmingEnabled) {
			targetReleaseISP();
			vTaskDelay(pdMS_TO_TICKS(stabDelay));
		}

		targetEnterISP();
		vTaskDelay(pdMS_TO_TICKS(stabDelay));

		for (uint32_t i = 0; i < synchLoops; i++) {
			IspCommand response;
			for (uint32_t i = 0; i < sizeof(cmd); i++) {
				com.txrx(response.raw() + i, cmd.raw() + i, 1);
				vTaskDelay(pdMS_TO_TICKS(byteDelay));
			}

			if (response[pollIndex - 1] != pollValue) {
				goto failed;
			}
		}

		vTaskDelay(pdMS_TO_TICKS(cmdexeDelay));

		// success
		state::programmingEnabled = true;
		dataBuf[1] = STATUS_CMD_OK;
		header.dataLen = 2;
		return;
failed:
		targetReleaseISP();
		vTaskDelay(pdMS_TO_TICKS(stabDelay));

		dataBuf[1] = STATUS_CMD_FAILED;
		header.dataLen = 2;
		return;
	}

	void cmdLeaveProgmode() {
		vTaskDelay(pdMS_TO_TICKS(dataBuf[1]));
		targetReleaseISP();
		vTaskDelay(pdMS_TO_TICKS(dataBuf[2]));

		state::programmingEnabled = false;
		dataBuf[1] = STATUS_CMD_OK;
		header.dataLen = 2;
	}

	void cmdChipErase() {
		if (!state::programmingEnabled) {
			dataBuf[1] = STATUS_CMD_FAILED;
			header.dataLen = 2;
			return;
		}

		// send command
		com.txrx(&dataBuf[3], &dataBuf[3], 4);

		if (dataBuf[2]) {
			dataBuf[1] = waitRdyBsy() ? STATUS_CMD_OK : STATUS_RDY_BSY_TOUT;
		}
		else {
			vTaskDelay(pdMS_TO_TICKS(dataBuf[1]));
			dataBuf[1] = STATUS_CMD_OK;

		}
		header.dataLen = 2;
	}

	void cmdProgramFlashEEPROM(bool flashAccess) {
		const uint16_t NumBytes = ((uint16_t)dataBuf[1] << 8) | dataBuf[2];
		const uint8_t mode = dataBuf[3];
		const uint8_t delay = dataBuf[4];
		const uint8_t cmd1 = dataBuf[5];
		const uint8_t cmd2 = dataBuf[6];
		const uint8_t cmd3 = dataBuf[7];
		const uint8_t poll1 = dataBuf[8];
		const uint8_t poll2 = dataBuf[9];
		uint8_t *txData = &dataBuf[10];
		IspCommand *currentCmd = cmdBuf;

		if (!state::programmingEnabled || NumBytes > maxPageSize) {
			goto failed;
		}

		// if a request to cross the 64KWord boundary was sent previously, do it now
		if (flashAccess && (state::address & 0x80000000)) {
			// load extended address
			IspCommand cmd = { 0x4d, 0x00, (uint8_t)(state::address >> 16), 0x00 };
			com.txrx(cmd.raw(), cmd.raw(), sizeof(cmd));
			// mark the load extended address as sent
			state::address &= ~0x80000000;
		}

		if (mode & 0x1) { // page mode
			// construct the commands
			for (uint16_t i = 0; i < NumBytes; i += flashAccess ? 2 : 1, state::address++) {
				if (flashAccess) {
					*(currentCmd++) = { cmd1, (uint8_t)(state::address >> 8), (uint8_t)state::address, *(txData++) };
					*(currentCmd++) = { (uint8_t)(cmd1 | 0x08), (uint8_t)(state::address >> 8), (uint8_t)state::address, *(txData++) };
				}
				else {
					*(currentCmd++) = { cmd1, (uint8_t)(state::address >> 8), (uint8_t)state::address, *(txData++) };
				}
			}

			// send the commands
			com.txrx(cmdBuf[0].raw(), cmdBuf[0].raw(), NumBytes * sizeof(*cmdBuf));

			// commit page if requested
			if (mode & 0x80) {
				uint8_t checkByte = dataBuf[10];
				uint32_t pageAddress = state::address - (flashAccess ? NumBytes >> 1 : NumBytes);

				IspCommand cmd = { cmd2, (uint8_t)(pageAddress >> 8), (uint8_t)pageAddress, 0x00 };
				com.txrx(cmd.raw(), cmd.raw(), sizeof(cmd));

				if (mode & 0x40) {
					// page mode ready busy polling
					if (!waitRdyBsy()) {
						goto rdyBsyTimeout;
					}
				}
				else if (mode & 0x20 && !((flashAccess && checkByte == poll1) || (!flashAccess && (checkByte == poll1 || checkByte == poll2)))) {
					// page mode value polling (if possible)
					if (!waitValuePolling(cmd3, checkByte, delay)) {
						goto timeout;
					}
				}
				else if (mode & 0x10) {
					// word mode timed delay
					vTaskDelay(pdMS_TO_TICKS(delay));
				}
				else {
					// no mode provided?
					goto failed;
				}
			}
		}
		else { // word mode
			for (uint16_t i = 0; i < NumBytes; i += flashAccess ? 2 : 1, state::address++) {
				IspCommand cmd = { cmd1, (uint8_t)(state::address >> 8), (uint8_t)state::address, *(txData++) };
				uint8_t checkByte = cmd[3];
				com.txrx(cmd.raw(), cmd.raw(), sizeof(cmd));
				if (mode & 0x08) {
					// word mode ready busy polling
					if (!waitRdyBsy()) {
						goto rdyBsyTimeout;
					}
				}
				else if (mode & 0x04 && !((flashAccess && checkByte == poll1) || (!flashAccess && (checkByte == poll1 || checkByte == poll2)))) {
					// word mode value polling (if possible)
					if (!waitValuePolling(cmd3, checkByte, delay)) {
						goto timeout;
					}
				}
				else if (mode & 0x02) {
					// word mode timed delay
					vTaskDelay(pdMS_TO_TICKS(delay));
				}
				else {
					// no mode provided?
					goto failed;
				}
			}
		}

		// success
		dataBuf[1] = STATUS_CMD_OK;
		header.dataLen = 2;
		return;
failed:
		dataBuf[1] = STATUS_CMD_FAILED;
		header.dataLen = 2;
		return;
timeout:
		dataBuf[1] = STATUS_CMD_TOUT;
		header.dataLen = 2;
		return;
rdyBsyTimeout:
		dataBuf[1] = STATUS_RDY_BSY_TOUT;
		header.dataLen = 2;
	}

	void cmdReadFlashEEPROM(bool flashAccess) {
		const uint16_t NumBytes = ((uint16_t)dataBuf[1] << 8) | dataBuf[2];
		const uint8_t cmd1 = dataBuf[3];
		uint8_t *rxData = &dataBuf[2];
		IspCommand *currentCmd = cmdBuf;

		if (!state::programmingEnabled || NumBytes > maxPageSize) {
			dataBuf[1] = STATUS_CMD_FAILED;
			header.dataLen = 2;
			return;
		}

		// if a request to cross the 64KWord boundary was sent previously, do it now
		if (flashAccess && (state::address & 0x80000000)) {
			// load extended address
			IspCommand cmd = { 0x4d, 0x00, (uint8_t)(state::address >> 16), 0x00 };
			com.txrx(cmd.raw(), cmd.raw(), sizeof(cmd));
			// mark the load extended address as sent
			state::address &= ~0x80000000;
		}

		// construct the commands
		for (uint16_t i = 0; i < NumBytes; i += flashAccess ? 2 : 1, state::address++) {
			if (flashAccess) {
				*(currentCmd++) = { cmd1, (uint8_t)(state::address >> 8), (uint8_t)state::address, 0x00 };
				*(currentCmd++) = { (uint8_t)(cmd1 | 0x08), (uint8_t)(state::address >> 8), (uint8_t)state::address, 0x00 };
			}
			else {
				*(currentCmd++) = { cmd1, (uint8_t)(state::address >> 8), (uint8_t)state::address, 0x00 };
			}
		}

		// send the commands
		com.txrx(cmdBuf[0].raw(), cmdBuf[0].raw(), NumBytes * sizeof(*cmdBuf));

		// process the responses
		for (uint16_t i = 0; i < NumBytes; i++) {
			*(rxData++) = cmdBuf[i][3];
		}

		dataBuf[1] = STATUS_CMD_OK; //Status1
		*(rxData++) = STATUS_CMD_OK; //Status2
		header.dataLen = 3 + NumBytes;
	}

	void cmdProgramFuse() {
		if (!state::programmingEnabled) {
			dataBuf[1] = STATUS_CMD_FAILED;
			header.dataLen = 2;
			return;
		}

		// send command
		com.txrx(&dataBuf[1], &dataBuf[1], 4);

		// wait for fuses to be written to flash
		if (waitRdyBsy()) {
			dataBuf[1] = STATUS_CMD_OK;
			dataBuf[2] = STATUS_CMD_OK;
			header.dataLen = 3;
		}
		else {
			dataBuf[1] = STATUS_CMD_FAILED;
			header.dataLen = 2;
		}
	}

	void cmdReadFuse() {
		const uint8_t RetAddr = dataBuf[1];
		const IspCommand cmd = {
				dataBuf[2],
				dataBuf[3],
				dataBuf[4],
				dataBuf[5],
		};

		if (!state::programmingEnabled || RetAddr > 4) {
			dataBuf[1] = STATUS_CMD_FAILED;
			header.dataLen = 2;
			return;
		}

		// send command
		IspCommand response;
		com.txrx(response.raw(), cmd.raw(), sizeof(response));

		// success
		dataBuf[1] = STATUS_CMD_OK;
		dataBuf[2] = response[RetAddr - 1];
		dataBuf[3] = STATUS_CMD_OK;
		header.dataLen = 4;
	}

	void cmdSPIMulti() {
		if (!state::programmingEnabled) {
			dataBuf[1] = STATUS_CMD_FAILED;
			header.dataLen = 2;
			return;
		}

		uint8_t numTx = dataBuf[1];
		uint8_t numRx = dataBuf[2];
		const uint8_t rxStartAddr = dataBuf[3];
		uint8_t *txData = &dataBuf[4];
		uint8_t *rxData = &dataBuf[2];
		if (rxStartAddr > numRx) {
			dataBuf[1] = STATUS_CMD_FAILED;
			header.dataLen = 2;
			return;
		}
		if (numRx > numTx) {
			numTx = numRx;
			memset(txData + numTx, 0, numRx - numTx);
		}

		// part where we transmit only and discard any received data by reading it to the txbuf
		com.txrx(txData, txData, rxStartAddr);
		numTx -= rxStartAddr;
		numRx -= rxStartAddr;

		// part where we both transmit and receive data.
		// Excess rx data will just be ignored in the response packet
		com.txrx(rxData, txData, numTx);

		dataBuf[1] = STATUS_CMD_OK;
		dataBuf[numRx + 2] = STATUS_CMD_OK;
		header.dataLen = numRx + 3;
	}

	bool sendResponse() {
		const uint16_t dataLen = header.dataLen;
		header.swapDataLen();
		uint8_t checksum = updateChecksum(0, &header, sizeof(header));
		checksum = updateChecksum(checksum, dataBuf, dataLen);
		if (usb::cdc_write(config::cdc_itf_isp, (uint8_t*)&header, sizeof(header), sendTimeout) < 0) {
			goto timeout;
		}
		if (usb::cdc_write(config::cdc_itf_isp, (uint8_t*)&dataBuf, dataLen, sendTimeout) < 0) {
			goto timeout;
		}
		if (usb::cdc_write(config::cdc_itf_isp, &checksum, 1, sendTimeout) < 0) {
			goto timeout;
		}
		if (!usb::cdc_write_flush(config::cdc_itf_isp)) {
			goto timeout;
		}
		return true;
timeout:
		return false;
	}

	bool processCommand() {
		uint8_t command = dataBuf[0];
		switch(command) {
		case CMD_SIGN_ON:
			cmdSignOn();
			break;
		case CMD_SET_PARAMETER:
			cmdSetParameter();
			break;
		case CMD_GET_PARAMETER:
			cmdGetParameter();
			break;
		case CMD_LOAD_ADDRESS:
			cmdLoadAddress();
			break;
		case CMD_ENTER_PROGMODE_ISP:
			cmdEnterProgmode();
			break;
		case CMD_LEAVE_PROGMODE_ISP:
			cmdLeaveProgmode();
			break;
		case CMD_CHIP_ERASE_ISP:
			cmdChipErase();
			break;
		case CMD_PROGRAM_FLASH_ISP:
			cmdProgramFlashEEPROM(true);
			break;
		case CMD_PROGRAM_EEPROM_ISP:
			cmdProgramFlashEEPROM(false);
			break;
		case CMD_READ_FLASH_ISP:
			cmdReadFlashEEPROM(true);
			break;
		case CMD_READ_EEPROM_ISP:
			cmdReadFlashEEPROM(false);
			break;
		case CMD_PROGRAM_FUSE_ISP:
		case CMD_PROGRAM_LOCK_ISP:
			cmdProgramFuse();
			break;
		case CMD_READ_FUSE_ISP:
		case CMD_READ_LOCK_ISP:
		case CMD_READ_SIGNATURE_ISP:
		case CMD_READ_OSCCAL_ISP:
			cmdReadFuse();
			break;
		case CMD_SPI_MULTI:
			cmdSPIMulti();
			break;
		default:
			dataBuf[1] = STATUS_CMD_UNKNOWN;
			header.dataLen = 2;
		}

		return sendResponse();
	}

public:
	bool step() {
		switch (state) {
		case State::start: {
			if (usb::cdc_read(config::cdc_itf_isp, &header.start, 1, parserTimeout) < 0) {
				goto timeout;
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
				goto timeout;
			}
			else if (lastSeqNum + 1 == header.seqNum) {
				state = State::message_size;
			}
			else if (header.seqNum == 1) {
				// restart of sequence. assume that a new session was started
				targetForceRelease();
				state = State::message_size;
			}
			else {
				goto error;
			}
		} break;
		case State::message_size: {
			if (usb::cdc_read(config::cdc_itf_isp, (uint8_t*)&header.dataLen, 2, parserTimeout) < 0) {
				goto timeout;
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
				goto timeout;
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
				goto timeout;
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
				goto timeout;
			}
			else if (c == expectedChecksum) {
				processCommand();
				state = State::start;
			}
			else {
				dataBuf[0] = STATUS_CKSUM_ERROR;
				dataBuf[1] = STATUS_CKSUM_ERROR;
				header.dataLen = 2;
				return sendResponse();
			}
		} break;
		timeout:
			if (state::programmingEnabled) {
				//todo pavr2 idle too long
			}
			targetForceRelease();
			state = State::start;
			return false;
		error:
			state = State::start;
			return false;
		default:
			// unhandled state;
			Error_Handler();
			return false;
		}
		return true;
	}
} static parser;


static void taskISP(void *pvParameters) {
	for (;;) {
		if (!parser.step()) {
			vTaskDelay(pdMS_TO_TICKS(10));
		}
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
