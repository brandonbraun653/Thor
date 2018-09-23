/* Boost Includes */
#include <boost/container/static_vector.hpp>
#include <boost/make_shared.hpp>

/* Project Includes */
#include <Thor/include/serial.hpp>

using namespace Thor::Definitions;
using namespace Thor::Definitions::Serial;
using namespace Thor::Peripheral::UART;
using namespace Thor::Peripheral::USART;

using ThorStatus = Thor::Definitions::Status;
using ThorMode = Thor::Definitions::Modes;
using ThorSubPeriph = Thor::Definitions::SubPeripheral;
using ThorBaud = Thor::Definitions::Serial::BaudRate;

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
			#if defined(USING_CHIMERA)
			Chimera::Serial::Status SerialClass::convertStatus(ThorStatus status)
			{
				switch (status)
				{
				case ThorStatus::PERIPH_OK:								return ChimStatus::SERIAL_OK;
				case ThorStatus::PERIPH_LOCKED:							return ChimStatus::SERIAL_LOCKED;
				case ThorStatus::PERIPH_NOT_INITIALIZED:				return ChimStatus::SERIAL_NOT_INITIALIZED;
				case ThorStatus::PERIPH_ERROR:							return ChimStatus::SERIAL_ERROR;
				case ThorStatus::PERIPH_NOT_READY:						return ChimStatus::SERIAL_NOT_READY;
				case ThorStatus::PERIPH_TX_IN_PROGRESS:					return ChimStatus::SERIAL_TX_IN_PROGRESS;
				case ThorStatus::PERIPH_RX_IN_PROGRESS:					return ChimStatus::SERIAL_RX_IN_PROGRESS;
				case ThorStatus::PERIPH_PACKET_TOO_LARGE_FOR_BUFFER:	return ChimStatus::SERIAL_PACKET_TOO_LARGE_FOR_BUFFER;
				case ThorStatus::PERIPH_TIMEOUT:						return ChimStatus::SERIAL_TIMEOUT;
				default:												return ChimStatus::SERIAL_UNKNOWN_ERROR;
				}
			}

			ThorBaud convertBaud(ChimBaud baud)
			{
				switch (baud)
				{
				case ChimBaud::SERIAL_BAUD_110:			return ThorBaud::SERIAL_BAUD_110;
				case ChimBaud::SERIAL_BAUD_150:			return ThorBaud::SERIAL_BAUD_150;
				case ChimBaud::SERIAL_BAUD_300:			return ThorBaud::SERIAL_BAUD_300;
				case ChimBaud::SERIAL_BAUD_1200:		return ThorBaud::SERIAL_BAUD_1200;
				case ChimBaud::SERIAL_BAUD_2400:		return ThorBaud::SERIAL_BAUD_2400;
				case ChimBaud::SERIAL_BAUD_4800:		return ThorBaud::SERIAL_BAUD_4800;
				case ChimBaud::SERIAL_BAUD_9600:		return ThorBaud::SERIAL_BAUD_9600;
				case ChimBaud::SERIAL_BAUD_19200:		return ThorBaud::SERIAL_BAUD_19200;
				case ChimBaud::SERIAL_BAUD_38400:		return ThorBaud::SERIAL_BAUD_38400;
				case ChimBaud::SERIAL_BAUD_57600:		return ThorBaud::SERIAL_BAUD_57600;
				case ChimBaud::SERIAL_BAUD_115200:		return ThorBaud::SERIAL_BAUD_115200;
				case ChimBaud::SERIAL_BAUD_230400:		return ThorBaud::SERIAL_BAUD_230400;
				case ChimBaud::SERIAL_BAUD_460800:		return ThorBaud::SERIAL_BAUD_460800;
				case ChimBaud::SERIAL_BAUD_921600:		return ThorBaud::SERIAL_BAUD_921600;
				default:								return ThorBaud::SERIAL_BAUD_9600;
				}
			}

			ChimStatus SerialClass::cbegin(uint32_t baud, ChimMode tx_mode, ChimMode rx_mode)
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
						serialObject->setMode(ThorSubPeriph::TX, ThorMode::BLOCKING);
						break;

					case Chimera::Serial::Modes::INTERRUPT:
						serialObject->setMode(ThorSubPeriph::TX, ThorMode::INTERRUPT);
						break;

					case Chimera::Serial::Modes::DMA:
						serialObject->setMode(ThorSubPeriph::TX, ThorMode::DMA);
						break;

					default: break;
					}
				}
				else if (periph == ChimSubPeriph::RX)
				{
					switch (mode)
					{
					case Chimera::Serial::Modes::BLOCKING:
						serialObject->setMode(ThorSubPeriph::RX, ThorMode::BLOCKING);
						break;

					case Chimera::Serial::Modes::INTERRUPT:
						serialObject->setMode(ThorSubPeriph::RX, ThorMode::INTERRUPT);
						break;

					case Chimera::Serial::Modes::DMA:
						serialObject->setMode(ThorSubPeriph::RX, ThorMode::DMA);
						break;

					default: break;
					}
				}

				return ChimStatus::SERIAL_OK;
			}

			Chimera::Serial::Status SerialClass::csetBaud(uint32_t baud)
			{
				auto chimera_error = ChimStatus::SERIAL_OK;

				//TODO: Need to convert between Thor/Chimera bauds...or just allow numbers??
				auto thor_error = this->serialObject->setBaud(baud);

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = convertStatus(thor_error);
				}

				return chimera_error;
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

			#endif

			/* This maps various UART and USART peripherals into the higher level Serial class. The purpose
			 * is to allow the user to only call 'Serial(1)' or 'Serial(5)' and not have to worry about 
			 * whether or not it is a UART or USART type. This mapping will however need to be updated for
			 * each type of chip.*/
			boost::container::static_vector<HardwareClassMapping, MAX_SERIAL_CHANNELS + 1> serialPeripheralMap =
			{ 
				#if defined(STM32F767xx) || defined(STM32F446xx)
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
			
			SerialClass::SerialClass(const int& channel, SerialPins* config)
			{
				//TODO: Add an assert here for checking the channel boundary...exceptions?
				this->serialChannel = channel;
				if (serialPeripheralMap[channel].ON_UART)
				{
					auto tmp = UARTClass::create(serialPeripheralMap[channel].peripheral_number, config);
					this->serialObject = boost::dynamic_pointer_cast<SerialBase, UARTClass>(tmp);
				}
				else
				{
					auto tmp = USARTClass::create(serialPeripheralMap[channel].peripheral_number, config);
					this->serialObject = boost::dynamic_pointer_cast<SerialBase, USARTClass>(tmp);
				}
			}

			ThorStatus SerialClass::begin(const BaudRate& baud, const Modes& tx_mode, const Modes& rx_mode)
			{
				return this->serialObject->begin(baud, tx_mode, rx_mode);
			}
			
			Status SerialClass::setMode(const SubPeripheral& periph, const Modes& mode)
			{
				return this->serialObject->setMode(periph, mode);
			}
			
			Status SerialClass::setBaud(const BaudRate& baud)
			{
				return this->serialObject->setBaud(baud);
			}

			Status SerialClass::write(uint8_t* val, size_t length)
			{
				return this->serialObject->write(val, length);
			}
			
			Status SerialClass::write(char* string, size_t length)
			{
				return this->serialObject->write(string, length);
			}
			
			Status SerialClass::write(const char* string)
			{
				return this->serialObject->write(string);
			}
			
			Status SerialClass::write(const char* string, size_t length)
			{
				return this->serialObject->write(string, length);
			}
			
			Status SerialClass::readSync(uint8_t* buff, size_t length)
			{
				return this->serialObject->readSync(buff, length);
			}
			
			Status SerialClass::readPacket(uint8_t* buff, size_t buff_length)
			{
				return this->serialObject->readPacket(buff, buff_length);
			}
	
			uint32_t SerialClass::availablePackets()
			{
				return this->serialObject->availablePackets();
			}
			
			size_t SerialClass::nextPacketSize()
			{
				return this->serialObject->nextPacketSize();
			}
			
			void SerialClass::end()
			{
				this->serialObject->end();
			}
			
			#if defined(USING_FREERTOS)
			void SerialClass::attachThreadTrigger(Trigger trig, SemaphoreHandle_t* semphr)
			{
				this->serialObject->attachThreadTrigger(trig, semphr);
			}
			
			void SerialClass::removeThreadTrigger(Trigger trig)
			{
				this->serialObject->removeThreadTrigger(trig);
			}
			#endif 

			 
		}
	}
}