/* Boost Includes */
#include <boost/container/static_vector.hpp>
#include <boost/make_shared.hpp>

/* Project Includes */
#include <Thor/include/serial.hpp>

using namespace Thor::Definitions::Serial;
using namespace Thor::Peripheral::UART;
using namespace Thor::Peripheral::USART;

using ThorStatus = Thor::Definitions::Serial::Status;
using ThorBaud = Thor::Definitions::Serial::BaudRate;
using ThorMode = Thor::Definitions::Serial::Modes;
using ThorSubPeriph = Thor::Definitions::Serial::SubPeripheral;

#if defined(USING_CHIMERA)
using ChimStatus = Chimera::Serial::Status;
using ChimBaud = Chimera::Serial::BaudRate;
using ChimMode = Chimera::Serial::Modes;
using ChimSubPeriph = Chimera::Serial::SubPeripheral;
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
					auto tmp = UARTClass::create(serialPeripheralMap[channel].peripheral_number);
					serial = boost::dynamic_pointer_cast<SerialBase, UARTClass>(tmp);
				}
				else
				{
					auto tmp = USARTClass::create(serialPeripheralMap[channel].peripheral_number);
					serial = boost::dynamic_pointer_cast<SerialBase, USARTClass>(tmp);
				}
			}

			ThorStatus SerialClass::begin(const BaudRate& baud, const Modes& tx_mode, const Modes& rx_mode)
			{
				return serial->begin(baud, tx_mode, rx_mode);
			}
			
			Status SerialClass::setMode(const SubPeripheral& periph, const Modes& mode)
			{
				return serial->setMode(periph, mode);
			}
			
			Status SerialClass::write(uint8_t* val, size_t length)
			{
				return serial->write(val, length);
			}
			
			Status SerialClass::write(char* string, size_t length)
			{
				return serial->write(string, length);
			}
			
			Status SerialClass::write(const char* string)
			{
				return serial->write(string);
			}
			
			Status SerialClass::write(const char* string, size_t length)
			{
				return serial->write(string, length);
			}
			
			Status SerialClass::readSync(uint8_t* buff, size_t length)
			{
				return serial->readSync(buff, length);
			}
			
			Status SerialClass::readPacket(uint8_t* buff, size_t buff_length)
			{
				return serial->readPacket(buff, buff_length);
			}
	
			uint32_t SerialClass::availablePackets()
			{
				return serial->availablePackets();
			}
			
			size_t SerialClass::nextPacketSize()
			{
				return serial->nextPacketSize();
			}
			
			void SerialClass::end()
			{
				serial->end();
			}
			
			#if defined(USING_FREERTOS)
			void SerialClass::attachThreadTrigger(Trigger trig, SemaphoreHandle_t* semphr)
			{
				serial->attachThreadTrigger(trig, semphr);
			}
			
			void SerialClass::removeThreadTrigger(Trigger trig)
			{
				serial->removeThreadTrigger(trig);
			}
			#endif 

			#if defined(USING_CHIMERA)
			ChimStatus SerialClass::cbegin(ChimBaud baud, ChimMode tx_mode, ChimMode rx_mode)
			{
				auto chimera_error = ChimStatus::SERIAL_OK;
				auto thor_error = begin(static_cast<ThorBaud>(baud), static_cast<ThorMode>(tx_mode), static_cast<ThorMode>(rx_mode));

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = ChimStatus::SERIAL_ERROR;
				}

				return chimera_error;
			}

			ChimStatus SerialClass::csetMode(ChimSubPeriph periph, ChimMode mode)
			{					
				if (periph == ChimSubPeriph::TX)
				{
					switch (mode)
					{
					case Chimera::Serial::Modes::BLOCKING:
						serial->setMode(TX, BLOCKING);
						break;
						
					case Chimera::Serial::Modes::INTERRUPT:
						serial->setMode(TX, INTERRUPT);
						break;
					
					case Chimera::Serial::Modes::DMA:
						serial->setMode(TX, DMA);
						break;
						
					default: break;
					}
				}
				else if (periph == ChimSubPeriph::RX)
				{
					switch (mode)
					{
					case Chimera::Serial::Modes::BLOCKING:
						serial->setMode(RX, BLOCKING);
						break;
						
					case Chimera::Serial::Modes::INTERRUPT:
						serial->setMode(RX, INTERRUPT);
						break;
					
					case Chimera::Serial::Modes::DMA:
						serial->setMode(RX, DMA);
						break;
						
					default: break;
					}
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

				case PERIPH_TX_IN_PROGRESS:
					return ChimStatus::SERIAL_TX_IN_PROGRESS;
					break;
				
				case PERIPH_RX_IN_PROGRESS:
					return ChimStatus::SERIAL_RX_IN_PROGRESS;
					break;

				case PERIPH_PACKET_TOO_LARGE_FOR_BUFFER:
					return ChimStatus::SERIAL_PACKET_TOO_LARGE_FOR_BUFFER;
					break;
					
				case PERIPH_TIMEOUT:
					return ChimStatus::SERIAL_TIMEOUT;
					break;

				default: return ChimStatus::SERIAL_UNKNOWN_ERROR;
				}
			}

			#endif 
		}
	}
}