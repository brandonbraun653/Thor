#include "uart_thor_transport.h"
#include <cassert>
#include <cstdio>

#include <Thor/include/definitions.h>

using namespace erpc;
using namespace Thor::Peripheral::UART;
using namespace Thor::Definitions::Serial;

UartTransport::UartTransport(int channel)
{
	uart_channel = channel;
}

UartTransport::~UartTransport()
{
	
}

erpc_status_t UartTransport::init()
{
	uart = UARTClass::create(uart_channel);
	uart->begin(SERIAL_BAUD_115200);
	uart->setDMAMode(SubPeripheral::TX);	//Transmit to PC	
	uart->setDMAMode(SubPeripheral::RX);	//Receive transmissions from PC
	
	return kErpcStatus_Success;
}

erpc_status_t UartTransport::underlyingReceive(uint8_t *data, uint32_t size)
{
	if (uart->readPacket(data, (size_t)size) != PERIPH_OK)
		return kErpcStatus_ReceiveFailed;
	return kErpcStatus_Success;
}

erpc_status_t UartTransport::underlyingSend(const uint8_t *data, uint32_t size)
{
	uart->write((uint8_t*)data, (size_t)size);
	return kErpcStatus_Success;
}
