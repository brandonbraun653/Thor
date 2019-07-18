/********************************************************************************
 *   File Name:
 *    hw_usart_driver_stm32f4.cpp
 *
 *   Description:
 *    STM32F4 specific driver implementation for the UART/USART driver. Both drivers
 *    are merged into one as the datasheet does not make a distinction between the
 *    two. In practice with the STM32HAL this was also found to be true.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */

/* Chimera Includes */
#include <Chimera/chimera.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>
#include <Thor/drivers/f4/usart/hw_usart_driver.hpp>
#include <Thor/drivers/f4/usart/hw_usart_mapping.hpp>
#include <Thor/drivers/f4/usart/hw_usart_prj.hpp>
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )


static std::array<Thor::Driver::USART::Driver *, Thor::Driver::USART::NUM_USART_PERIPHS> usartObjects;

namespace Thor::Driver::USART
{
  /**
   *  Number of elements to allocate by default for any vectors used in the Driver.
   *
   *  The goal is to size this such that a dynamic allocation isn't likely to occur
   *  unless there suddenly are a lot of parties interested in being notified that
   *  events are occuring.
   */
  static constexpr size_t DFLT_VECTOR_SIZE = 5;

  bool isUSART( const std::uintptr_t address )
  {
    bool result = false;

    for ( auto &val : periphAddressList )
    {
      if ( val == address )
      {
        result = true;
      }
    }

    return result;
  }

  Driver::Driver( RegisterMap *const peripheral ) : periph( peripheral )
  {
    auto address   = reinterpret_cast<std::uintptr_t>( peripheral );
    peripheralType = Chimera::Peripheral::Type::PERIPH_USART;
    resourceIndex  = Thor::Driver::USART::InstanceToResourceIndex.find( address )->second;
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::init( const Thor::Driver::Serial::Config &cfg )
  {
    /*------------------------------------------------
    First de-initialize the driver so we know we are
    starting from a clean slate. There are no guarantees
    on what state the system is in when this is called.
    ------------------------------------------------*/
    if ( deinit() != Chimera::CommonStatusCodes::OK )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    /*------------------------------------------------
    Initialize driver memory
    ------------------------------------------------*/
    enterCriticalSection();
    
    //initialize all isr variables

    exitCriticalSection();

    /*------------------------------------------------
    Ensure the clock is enabled otherwise the hardware is "dead"
    ------------------------------------------------*/
    auto rccPeriph = Thor::Driver::RCC::PeripheralController::get();
    rccPeriph->enableClock( peripheralType, resourceIndex );

    /*------------------------------------------------
    Follow the initialization sequence as defined in RM0390 pg.801 
    ------------------------------------------------*/
    /* Enable the USART by writing the UE bit to 1 */
    CR1::UE::set( periph, CR1_UE );

    /* Program the M bit to define the word length */
    CR1::M::set( periph, cfg.WordLength );

    /* Program the number of stop bits */
    CR2::STOP::set( periph, cfg.StopBits );

    /* Select the desired baud rate */
    BRR::set( periph, calculateBRR( cfg.BaudRate ) );

    /* Turn on the Transmitter */
    CR1::TE::set( periph, CR1_TE );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::deinit()
  {
    auto rcc = Thor::Driver::RCC::PeripheralController::get();
    rcc->enableClock( peripheralType, resourceIndex );
    rcc->reset( peripheralType, resourceIndex );
    rcc->disableClock( peripheralType, resourceIndex );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::reset()
  {
    /*------------------------------------------------
    Reset the hardware registers
    ------------------------------------------------*/
    deinit();

    /*------------------------------------------------
    Erases pointers to the listeners, not the listeners themselves
    ------------------------------------------------*/
    rxCompleteActors.clear();
    txCompleteActors.clear();

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::transmit( const uint8_t *const data, const size_t size, const size_t timeout )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    size_t startTime = Chimera::millis();

    /*------------------------------------------------
    Wait for the driver to signal it finished the last transfer
    ------------------------------------------------*/
    if ( waitUntilSet( Configuration::Flags::FLAG_TC, timeout ) )
    {
      return Chimera::CommonStatusCodes::TIMEOUT;
    }


    for ( size_t x = 0; x < size; x++ )
    {
      /*------------------------------------------------
      Wait for hardware to signal data has transfered from
      the TDR into the shift register.
      ------------------------------------------------*/
      startTime = Chimera::millis();

      if ( waitUntilSet( Configuration::Flags::FLAG_TXE, timeout ) )
      {
        return Chimera::CommonStatusCodes::TIMEOUT;
      }

      /*------------------------------------------------
      Write new data to the TDR
      ------------------------------------------------*/
      periph->DR = data[ x ];
    }

    /*------------------------------------------------
    Wait for the driver to signal it finished the last transfer
    ------------------------------------------------*/
    if ( waitUntilSet( Configuration::Flags::FLAG_TC, timeout ) )
    {
      return Chimera::CommonStatusCodes::TIMEOUT;
    }

    return result;
  }

  Chimera::Status_t Driver::receive( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::enableIT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::disableIT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::transmitIT( uint8_t *const data, const size_t size, const size_t timeout )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;



    return result;
  }

  Chimera::Status_t Driver::receiveIT( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::initDMA()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::deinitDMA()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::enableDMA_IT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::disableDMA_IT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::transmitDMA( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::receiveDMA( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::enableSignal( const InterruptSignal_t sig )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::disableSignal( const InterruptSignal_t sig )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::registerEventListener( const Chimera::Event::Trigger event, SemaphoreHandle_t *const listener )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::removeEventListener( const Chimera::Event::Trigger event, SemaphoreHandle_t *const listener )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Hardware::Status Driver::pollTransferStatus()
  {
    return Chimera::Hardware::Status::PERIPHERAL_FREE;
  }

  void Driver::IRQHandler()
  {
    using namespace Configuration::Flags;

    const uint32_t statusRegister = SR::get( periph );
    const uint32_t txFlags        = statusRegister & ( FLAG_CTS | FLAG_TC | FLAG_TXE );
    const uint32_t rxFlags        = statusRegister & ( FLAG_RXNE | FLAG_IDLE );
    const uint32_t errorFlags     = statusRegister & ( FLAG_ORE | FLAG_PE | FLAG_NF | FLAG_FE );

    /*------------------------------------------------
    TX Related Handler
    ------------------------------------------------*/
    if ( txFlags ) {}

    // Notify the user
    // Clean up?? I'm not sure there is anything.

    /*------------------------------------------------
    RX Related Handler
    ------------------------------------------------*/
    if ( rxFlags ) {}

    // Handle RX related stuff
    //  Single reception character by character, up to a certain size
    //  Need a transfer control block structure that is volatile/protected when accessed user side.

    /*------------------------------------------------
    Error Related Handler
    ------------------------------------------------*/
    if ( errorFlags ) {}

    // Set flags for the user
    // Acknowledge flags and go to a safe state.
  }


  bool Driver::waitUntilSet( const uint32_t flag, const size_t timeout )
  {
    uint32_t srVal   = SR::get( periph );
    size_t startTime = Chimera::millis();

    while ( !( srVal & flag ) )
    {
      srVal = SR::get( periph );

      if ( ( Chimera::millis() - startTime ) > timeout )
      {
        return true;
      }
    } 

    return false;
  }

  uint32_t Driver::calculateBRR( const size_t desiredBaud )
  {
    size_t periphClock = 0u;
    size_t calculatedBRR = 0u;
    auto periphAddress = reinterpret_cast<std::uintptr_t>( periph );

    /*------------------------------------------------
    Figure out the frequency of the clock that drives the USART
    ------------------------------------------------*/
    auto rccSys = Thor::Driver::RCC::SystemClock::get();
    rccSys->getPeriphClock( Chimera::Peripheral::Type::PERIPH_USART, periphAddress, &periphClock );

    /*------------------------------------------------
    Protect from fault conditions in the math below
    ------------------------------------------------*/
    if ( !desiredBaud || !periphClock )
    {
      return 0u;
    }

    /*------------------------------------------------
    Calculate the BRR value. Mostly this was taken directly from
    the STM32 HAL Macros.
    ------------------------------------------------*/
    uint32_t over8Compensator = 2u;

    if ( CR1::OVER8::get( periph ) )
    {
      over8Compensator = 1u;
    }

    auto divisor          = ( 25u * periphClock ) / ( 2u * over8Compensator * desiredBaud );
    auto mantissa_divisor = divisor / 100u;
    auto fraction_divisor = ( ( divisor - ( mantissa_divisor * 100u ) ) * 16u + 50u ) / 100u;
    calculatedBRR         = ( mantissa_divisor << BRR_DIV_Mantissa_Pos ) | ( fraction_divisor & BRR_DIV_Fraction );

    return calculatedBRR;
  }

}    // namespace Thor::Driver::USART

void USART1_IRQHandler( void )
{
  static constexpr size_t index = 0;

  if ( usartObjects[ index ] )
  {
    usartObjects[ index ]->IRQHandler();
  }
}

void USART2_IRQHandler( void )
{
  static constexpr size_t index = 1;

  if ( usartObjects[ index ] )
  {
    usartObjects[ index ]->IRQHandler();
  }
}

void USART3_IRQHandler( void )
{
  static constexpr size_t index = 2;

  if ( usartObjects[ index ] )
  {
    usartObjects[ index ]->IRQHandler();
  }
}

void USART6_IRQHandler( void )
{
  static constexpr size_t index = 3;

  if ( usartObjects[ index ] )
  {
    usartObjects[ index ]->IRQHandler();
  }
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */