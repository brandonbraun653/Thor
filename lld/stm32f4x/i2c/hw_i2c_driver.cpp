/******************************************************************************
 *  File Name:
 *    hw_i2c_driver.cpp
 *
 *  Description:
 *    I2C driver for STM32F4
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/i2c>
#include <Thor/cfg>
#include <Thor/i2c>
#include <Thor/lld/interface/inc/i2c>

#if defined( THOR_LLD_I2C )
namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr ), mCfg( {} )
  {
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Driver::enableClock()
  {
  }


  void Driver::disableClock()
  {
  }


  Chimera::Status_t Driver::configure( const Chimera::I2C::DriverConfig &cfg )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::enableISRSignal( const Chimera::I2C::Interrupt signal )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Driver::disableISRSignal( const Chimera::I2C::Interrupt signal )
  {
  }


  Chimera::Status_t Driver::read( const uint16_t address, void *const data, const size_t length )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::write( const uint16_t address, const void *const data, const size_t length )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::transfer( const uint16_t address, const void *const tx_data, void *const rx_data,
                                      const size_t length )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Driver::enterCriticalSection()
  {
  }


  void Driver::exitCriticalSection()
  {
  }


  void Driver::IRQErrorHandler()
  {
  }


  void Driver::IRQEventHandler()
  {
  }


  void Driver::onDMAComplete( const Chimera::DMA::TransferStats &stats )
  {
  }

}    // namespace Thor::LLD::I2C
#endif /* THOR_LLD_I2C */
