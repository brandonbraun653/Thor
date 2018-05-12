#pragma once
#ifndef UART_THOR_TRANSPORT_H
#define UART_THOR_TRANSPORT_H

/* eRPC Includes */
#include "framed_transport.h"
#include <stdlib.h>

/* Thor Includes */
#include <Thor/include/thor.h>
#include <Thor/include/serial.h>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "semphr.h"


extern SemaphoreHandle_t serverWakeup;

namespace erpc
{
	class UartTransport : public FramedTransport
	{
	public:
		UartTransport(int channel);
		virtual ~UartTransport();

		/*!
		 * @brief Initialize Thor UART peripheral configuration structure with values specified in UartTransport
		 * constructor.
		 *
		 * @retval kErpcStatus_InitFailed When UART init function failed.
		 * @retval kErpcStatus_Success When UART init function was executed successfully.
		 */
		virtual erpc_status_t init();
		
	private:
		/*!
		 * @brief Receive data from UART peripheral.
		 *
		 * @param[inout] data Preallocated buffer for receiving data.
		 * @param[in] data Size of data to read.
		 *
		 * @retval kErpcStatus_ReceiveFailed UART failed to receive data.
		 * @retval kErpcStatus_Success Successfully received all data.
		 */
		virtual erpc_status_t underlyingReceive(uint8_t *data, uint32_t size);

		/*!
		 * @brief Write data to UART peripheral.
		 *
		 * @param[in] data Buffer to send.
		 * @param[in] data Size of data to send.
		 *
		 * @retval kErpcStatus_Success Always returns success status.
		 */
		virtual erpc_status_t underlyingSend(const uint8_t *data, uint32_t size);
		
		int serial_channel;
		Thor::Peripheral::Serial::SerialClass_sPtr serial;
	};
}

#endif