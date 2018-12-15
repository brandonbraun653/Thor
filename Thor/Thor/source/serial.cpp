

/* Project Includes */
#include <Thor/include/serial.hpp>
#include <Thor/include/utilities.hpp>

using namespace Thor::Definitions;
using namespace Thor::Definitions::Serial;
using namespace Thor::Peripheral::UART;
using namespace Thor::Peripheral::USART;

//using ThorStatus = Thor::Definitions::Status;
//using ThorMode = Thor::Definitions::Modes;
//using ThorSubPeriph = Thor::Definitions::SubPeripheral;
//using ThorBaud = Thor::Definitions::Serial::BaudRate;

#if defined(USING_CHIMERA)
//using ChimStatus = Chimera::Serial::Status;
//using ChimBaud = Chimera::Serial::BaudRate;
//using ChimMode = Chimera::Serial::Modes;
//using ChimSubPeriph = Chimera::Serial::SubPeripheral;
#endif

namespace Thor
{
	namespace Peripheral
	{
		namespace Serial
		{
			#if 0
			Chimera::Serial::Status SerialClass::convertStatus(ThorStatus status)
			{
				switch (status)
				{
				case ThorStatus::PERIPH_OK:								return ChimStatus::OK;
				case ThorStatus::PERIPH_LOCKED:							return ChimStatus::LOCKED;
				case ThorStatus::PERIPH_NOT_INITIALIZED:				return ChimStatus::NOT_INITIALIZED;
				case ThorStatus::PERIPH_ERROR:							return ChimStatus::ERROR;
				case ThorStatus::PERIPH_BUSY:						    return ChimStatus::NOT_READY;
				case ThorStatus::PERIPH_TX_IN_PROGRESS:					return ChimStatus::TX_IN_PROGRESS;
				case ThorStatus::PERIPH_RX_IN_PROGRESS:					return ChimStatus::RX_IN_PROGRESS;
				case ThorStatus::PERIPH_PACKET_TOO_LARGE_FOR_BUFFER:	return ChimStatus::PACKET_TOO_LARGE_FOR_BUFFER;
				case ThorStatus::PERIPH_TIMEOUT:						return ChimStatus::TIMEOUT;
				default:												return ChimStatus::UNKNOWN_ERROR;
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
				auto chimera_error = ChimStatus::OK;
				auto thor_error = begin(static_cast<ThorBaud>(baud), static_cast<ThorMode>(tx_mode), static_cast<ThorMode>(rx_mode));

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = ChimStatus::ERROR;
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

				return ChimStatus::OK;
			}

			Chimera::Serial::Status SerialClass::csetBaud(uint32_t baud)
			{
				auto chimera_error = ChimStatus::OK;

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
				auto chimera_error = ChimStatus::OK;
				auto thor_error = write(val, length);

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = convertStatus(thor_error);
				}

				return chimera_error;
			}

			ChimStatus SerialClass::cwrite(char* string, size_t length)
			{
				auto chimera_error = ChimStatus::OK;
				auto thor_error = write(string, length);

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = convertStatus(thor_error);
				}

				return chimera_error;
			}

			ChimStatus SerialClass::cwrite(const char* string)
			{
				auto chimera_error = ChimStatus::OK;
				auto thor_error = write(string);

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = convertStatus(thor_error);
				}

				return chimera_error;
			}

			ChimStatus SerialClass::cwrite(const char* string, size_t length)
			{
				auto chimera_error = ChimStatus::OK;
				auto thor_error = write(string, length);

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = convertStatus(thor_error);
				}

				return chimera_error;
			}

			ChimStatus SerialClass::creadPacket(uint8_t* buff, size_t buff_length)
			{
				auto chimera_error = ChimStatus::OK;
				auto thor_error = readPacket(buff, buff_length);

				if (thor_error != ThorStatus::PERIPH_OK)
				{
					chimera_error = convertStatus(thor_error);
				}

				return chimera_error;
			}

			#endif


			static HardwareClassMapping serialPeripheralMap[MAX_SERIAL_CHANNELS + 1] =
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
            static_assert(COUNT_OF_ARRAY(serialPeripheralMap) == (MAX_SERIAL_CHANNELS + 1), "Invalid array size");

            SerialClass::SerialClass(const uint8_t channel)
            {

            }

            SerialClass::SerialClass(const uint8_t channel, const SerialPins &config)
			{
                if (channel && (channel < MAX_SERIAL_CHANNELS))
                {
                    serialChannel = channel;

                    /*------------------------------------------------
                    Decide which instance to create
                    -------------------------------------------------*/
                    if (serialPeripheralMap[channel].ON_UART)
                    {
                        auto tmp = UARTClass::create(serialPeripheralMap[channel].peripheral_number, config);
                        serialObject = std::static_pointer_cast<SerialInterface, UARTClass>(tmp);
                    }
                    else
                    {
                        auto tmp = USARTClass::create(serialPeripheralMap[channel].peripheral_number, config);
                        serialObject = std::static_pointer_cast<SerialInterface, USARTClass>(tmp);
                    }
                }
                else
                {
                    serialObject = nullptr;
                }
            }

			Status SerialClass::begin(const BaudRate baud, const Modes tx_mode, const Modes rx_mode)
			{
                assert(serialObject);
                return serialObject->begin(baud, tx_mode, rx_mode);
			}

			Status SerialClass::setMode(const SubPeripheral periph, const Modes mode)
			{
                assert(serialObject);
				return serialObject->setMode(periph, mode);
			}

            Status SerialClass::setBaud(const uint32_t baud)
			{
                assert(serialObject);
				return serialObject->setBaud(baud);
			}

			Status SerialClass::setBaud(const BaudRate baud)
			{
                assert(serialObject);
				return serialObject->setBaud(baud);
			}

			Status SerialClass::write(const uint8_t *const val, const size_t length)
			{
                assert(serialObject);
				return serialObject->write(val, length);
			}

			Status SerialClass::write(char *const string, const size_t length)
			{
                assert(serialObject);
				return serialObject->write(string, length);
			}

			Status SerialClass::write(const char *const string)
			{
                assert(serialObject);
				return serialObject->write(string);
			}

			Status SerialClass::write(const char *const string, const size_t length)
			{
                assert(serialObject);
				return serialObject->write(string, length);
			}

			Status SerialClass::read(uint8_t *const buffer, size_t length)
			{
                assert(serialObject);
				return serialObject->read(buffer, length);
			}

			void SerialClass::end()
			{
                assert(serialObject);
				serialObject->end();
			}

			#if defined(USING_FREERTOS)
			void SerialClass::attachThreadTrigger(const Trigger trig, SemaphoreHandle_t *const semphr)
			{
                assert(serialObject);
				serialObject->attachThreadTrigger(trig, semphr);
			}

			void SerialClass::removeThreadTrigger(const Trigger trig)
			{
                assert(serialObject);
				serialObject->removeThreadTrigger(trig);
			}
			#endif


		}
	}
}
