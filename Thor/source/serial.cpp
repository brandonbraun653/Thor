/* Boost Includes */
#include <boost/container/static_vector.hpp>

/* Project Includes */
#include <Thor/include/serial.h>




using namespace Thor::Definitions::Serial;
using namespace Thor::Peripheral::UART;
using namespace Thor::Peripheral::USART;

namespace Thor
{
	namespace Peripheral
	{
		namespace Serial
		{
			/* This maps various UART and USART peripherals into the higher level Serial class. The purpose
			 * is to allow the user to only call 'Serial(1)' or 'Serial(5)' and not have to worry about 
			 * whether or not it is a UART or USART type. This mapping will however need to be updated for
			 * each type of chip.*/
			boost::container::static_vector<HardwareClassMapping, MAX_SERIAL_CHANNELS + 1> serialPeripheralMap =
			{ 
				#if defined(STM32F767xx)
				{false, 0},		/* Not actually a UART instance */
				{false, 1},		/* USART 1	*/
				{false, 2},		/* USART 2	*/
				{false, 3},		/* USART 3	*/
				{true,  4},		/* UART  4	*/
				{true,  5},		/* UART  5	*/
				{false, 6},		/* USART 6	*/
				{true,  7},		/* UART  7	*/
				{true,	8}		/* UART	 8	*/
				#endif 
			};
			

			SerialClass::SerialClass(const int& channel)
			{
				//TODO: Add an assert here for checking the channel boundary...exceptions?
				this->serial_channel = channel;
				if (serialPeripheralMap[channel].ON_UART)
				{
					uart = UARTClass::create(serialPeripheralMap[channel].peripheral_number);
				}
				else
				{
					usart = USARTClass::create(serialPeripheralMap[channel].peripheral_number);
				}
			}

			Status SerialClass::begin()
			{
				if (uart)
				{
					uart->begin();
				}
				else
				{
					usart->begin();
				}
			}

			Status SerialClass::begin(const BaudRate& baud)
			{
				if (uart)
				{
					uart->begin(baud);
				}
				else
				{
					usart->begin(baud);
				}
			}
			
			Status SerialClass::begin(const BaudRate& baud, const Modes& tx_mode, const Modes& rx_mode)
			{
				if (uart)
				{
					uart->begin(baud, tx_mode, rx_mode);
				}
				else
				{
					usart->begin(baud, tx_mode, rx_mode);
				}
			}
			
		}
	}
}