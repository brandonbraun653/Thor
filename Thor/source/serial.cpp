/* Boost Includes */
#include <boost/container/static_vector.hpp>

/* Project Includes */
#include <Thor/include/serial.h>

using namespace Thor::Definitions::Serial;
using namespace Thor::Peripheral::UART;
using namespace Thor::Peripheral::USART;

using ThorStatus = Thor::Definitions::Serial::Status;
using ThorBaud = Thor::Definitions::Serial::BaudRate;
using ThorMode = Thor::Definitions::Serial::Modes;
using ThorSPer = Thor::Definitions::Serial::SubPeripheral;

#if defined(USING_CHIMERA)
using ChimStatus = Chimera::Serial::Status;
using ChimBaud = Chimera::Serial::BaudRate;
using ChimMode = Chimera::Serial::Modes;
using ChimSPer = Chimera::Serial::SubPeripheral;
#endif 

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
					return uart->begin();
				}
				else
				{
					return usart->begin();
				}
			}

			Status SerialClass::begin(const BaudRate& baud)
			{
				if (uart)
				{
					return uart->begin(baud);
				}
				else
				{
					return usart->begin(baud);
				}
			}
			
			Status SerialClass::begin(const BaudRate& baud, const Modes& tx_mode, const Modes& rx_mode)
			{
				if (uart)
				{
					return uart->begin(baud, tx_mode, rx_mode);
				}
				else
				{
					return usart->begin(baud, tx_mode, rx_mode);
				}
			}
			
			Status SerialClass::write(uint8_t* val, size_t length)
			{
				if (uart)
				{
					return uart->write(val, length);
				}
				else
				{
					return usart->write(val, length);
				}
			}
			
			Status SerialClass::write(char* string, size_t length)
			{
				if (uart)
				{
					return uart->write(string, length);
				}
				else
				{
					return usart->write(string, length);
				}
			}
			
			Status SerialClass::write(const char* string)
			{
				if (uart)
				{
					return uart->write(string);
				}
				else
				{
					return usart->write(string);
				}
			}
			
			Status SerialClass::write(const char* string, size_t length)
			{
				if (uart)
				{
					return uart->write(string, length);
				}
				else
				{
					return usart->write(string, length);
				}
			}
			
			Status SerialClass::readPacket(uint8_t* buff, size_t buff_length)
			{
				if (uart)
				{
					return uart->readPacket(buff, buff_length);
				}
				else
				{
					return usart->readPacket(buff, buff_length);
				}
			}
	
			int SerialClass::availablePackets()
			{
				if (uart)
				{
					return uart->availablePackets();
				}
				else
				{
					return usart->availablePackets();
				}
			}
			
			size_t SerialClass::nextPacketSize()
			{
				if (uart)
				{
					return uart->nextPacketSize();
				}
				else
				{
					return usart->nextPacketSize();
				}
			}
			
			void SerialClass::flush()
			{
				if (uart)
				{
					uart->flush();
				}
				else
				{
					usart->flush();
				}
			}
			
			void SerialClass::end()
			{
				if (uart)
				{
					uart->end();
				}
				else
				{
					usart->end();
				}
			}
			
//			void SerialClass::setBlockMode(const SubPeripheral& periph)
//			{
//				if (uart)
//				{
//					uart->setBlockMode(periph);
//				}
//				else
//				{
//					usart->setBlockMode(periph);
//				}
//			}
//			
//			void SerialClass::setITMode(const SubPeripheral& periph)
//			{
//				if (uart)
//				{
//					uart->setITMode(periph);
//				}
//				else
//				{
//					usart->setITMode(periph);
//				}
//			}
//			
//			void SerialClass::setDMAMode(const SubPeripheral& periph)
//			{
//				if (uart)
//				{
//					uart->setDMAMode(periph);
//				}
//				else
//				{
//					usart->setDMAMode(periph);
//				}
//			}
			

			#if defined(USING_CHIMERA)
			ChimStatus SerialClass::begin(ChimBaud baud, ChimMode tx_mode, ChimMode rx_mode)
			{
				auto chimera_error = ChimStatus::SERIAL_OK;
				auto thor_error = begin(static_cast<ThorBaud>(baud), static_cast<ThorMode>(tx_mode), static_cast<ThorMode>(rx_mode));

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = ChimStatus::SERIAL_ERROR;
				}

				return chimera_error;
			}

			ChimStatus SerialClass::setMode(ChimSPer sp, ChimMode mode)
			{	
				switch (mode)
				{
				case ChimMode::TX_MODE_BLOCKING:
				case ChimMode::RX_MODE_BLOCKING:
					setBlockMode(static_cast<ThorSPer>(sp));
					break;

				case ChimMode::TX_MODE_INTERRUPT:
				case ChimMode::RX_MODE_INTERRUPT:
					setITMode(static_cast<ThorSPer>(sp));
					break;

				case ChimMode::TX_MODE_DMA:
				case ChimMode::RX_MODE_DMA:
					setDMAMode(static_cast<ThorSPer>(sp));
					break;

				default: break;
				}


				return ChimStatus::SERIAL_OK;
			}

			ChimStatus SerialClass::cwrite(uint8_t* val, size_t length)
			{
				auto chimera_error = ChimStatus::SERIAL_OK;
				auto thor_error = write(val, length);

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = convertStatus(thor_error);
				}

				return chimera_error;
			}
			
			ChimStatus SerialClass::cwrite(char* string, size_t length)
			{
				auto chimera_error = ChimStatus::SERIAL_OK;
				auto thor_error = write(string, length);

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = convertStatus(thor_error);
				}

				return chimera_error;
			}
			
			ChimStatus SerialClass::cwrite(const char* string)
			{
				auto chimera_error = ChimStatus::SERIAL_OK;
				auto thor_error = write(string);

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = convertStatus(thor_error);
				}

				return chimera_error;
			}
			
			ChimStatus SerialClass::cwrite(const char* string, size_t length)
			{
				auto chimera_error = ChimStatus::SERIAL_OK;
				auto thor_error = write(string, length);

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = convertStatus(thor_error);
				}

				return chimera_error;
			}
			
			ChimStatus SerialClass::creadPacket(uint8_t* buff, size_t buff_length)
			{
				auto chimera_error = ChimStatus::SERIAL_OK;
				auto thor_error = readPacket(buff, buff_length);

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = convertStatus(thor_error);
				}

				return chimera_error;
			}

			Chimera::Serial::Status SerialClass::convertStatus(Thor::Definitions::Serial::Status status)
			{
				switch (status)
				{
				case PERIPH_OK:
					return ChimStatus::SERIAL_OK;
					break;

				case PERIPH_LOCKED:
					return ChimStatus::SERIAL_LOCKED;
					break;

				case PERIPH_NOT_INITIALIZED:
					return ChimStatus::SERIAL_NOT_INITIALIZED;
					break;

				case PERIPH_ERROR:
					return ChimStatus::SERIAL_ERROR;
					break;

				case PERIPH_NOT_READY:
					return ChimStatus::SERIAL_NOT_READY;
					break;

				case TX_IN_PROGRESS:
					return ChimStatus::SERIAL_TX_IN_PROGRESS;
					break;

				case PACKET_TOO_LARGE_FOR_BUFFER:
					return ChimStatus::SERIAL_PACKET_TOO_LARGE_FOR_BUFFER;
					break;

				default: return ChimStatus::SERIAL_UNKNOWN_ERROR;
				}
			}

			#endif 
		}
	}
}