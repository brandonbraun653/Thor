#include "serial_thor_transport.h"
#include <cassert>
#include <cstdio>

using namespace erpc;
using namespace Thor::Peripheral::Serial;
using namespace Thor::Definitions::Serial;

SemaphoreHandle_t serverWakeup = xSemaphoreCreateBinary();

SerialTransport::SerialTransport(int channel)
{
	serial_channel = channel;
}

erpc_status_t SerialTransport::init()
{
	serial = boost::make_shared<SerialClass>(serial_channel);
	
	serial->begin(SERIAL_BAUD_115200);
	serial->setMode(TX, BLOCKING);		//Transmit to PC
	serial->setMode(RX, INTERRUPT);		//Receive transmissions from PC
	
	serial->attachThreadTrigger(RX_COMPLETE, &serverWakeup);

	return kErpcStatus_Success;
}

erpc_status_t SerialTransport::underlyingReceive(uint8_t *data, uint32_t size)
{
	erpc_status_t errorCode = kErpcStatus_Success;
	
	if (serial->availablePackets())
	{
		if (serial->readPacket(data, (size_t)size) != PERIPH_OK)
		{
			errorCode = kErpcStatus_ReceiveFailed;
		}
	}
	else
		errorCode = kErpcStatus_ReceiveFailed;

	return errorCode;
}

erpc_status_t SerialTransport::underlyingSend(const uint8_t *data, uint32_t size)
{
	serial->write((uint8_t*)data, (size_t)size);
	return kErpcStatus_Success; //Supposed to always return success 
}