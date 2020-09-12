/********************************************************************************
 *  File Name:
 *    usart_sim_driver.cpp
 *
 *  Description:
 *    Simulator driver for USART
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/usart/usart_prv_data.hpp>
#include <Thor/lld/interface/usart/usart_intf.hpp>
#include <Thor/lld/interface/usart/usart_types.hpp>
#include <Thor/lld/interface/usart/usart_detail.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_USART )

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static Driver s_usart_drivers[ NUM_USART_PERIPHS ];

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();

    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
    if ( attachDriverInstances( s_usart_drivers, ARRAY_COUNT( s_usart_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  Driver_rPtr getDriver( const Chimera::Serial::Channel channel )
  {
    if ( auto idx = getResourceIndex( channel ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_usart_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/

  Driver::Driver()
  {
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::init( const Thor::LLD::Serial::Config &cfg )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::deinit()
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::reset()
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transmit( const uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::receive( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::enableIT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::disableIT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transmitIT( const uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::receiveIT( uint8_t *const data, const size_t size, const size_t timeout )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::initDMA()
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::deinitDMA()
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::enableDMA_IT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::disableDMA_IT( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::transmitDMA( const void *const data, const size_t size, const size_t timeout )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::receiveDMA( void *const data, const size_t size, const size_t timeout )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::txTransferStatus()
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::rxTransferStatus()
  {
    return Chimera::Status::OK;
  }


  uint32_t Driver::getFlags()
  {
    return 0;
  }

  void Driver::clearFlags( const uint32_t flagBits )
  {
  }


  void Driver::killTransmit()
  {
  }


  void Driver::killReceive()
  {
  }


  void Driver::attachISRWakeup( Chimera::Threading::BinarySemaphore *const wakeup )
  {
  }


  Thor::LLD::Serial::CDTCB Driver::getTCB_TX()
  {
    return Thor::LLD::Serial::CDTCB();
  }


  Thor::LLD::Serial::MDTCB Driver::getTCB_RX()
  {
    return Thor::LLD::Serial::MDTCB();
  }


  Thor::LLD::Serial::Config Driver::getConfiguration()
  {
    return Thor::LLD::Serial::Config();
  }

}    // namespace Thor::LLD::USART
#endif