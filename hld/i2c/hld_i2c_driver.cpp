/******************************************************************************
 *  File Name:
 *    hld_i2c_driver.cpp
 *
 *  Description:
 *    HLD I2C driver
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Chimera/event>
#include <Chimera/i2c>
#include <Chimera/thread>
#include <Thor/cfg>
#include <Thor/i2c>
#include <Thor/lld/interface/i2c/i2c_detail.hpp>
#include <Thor/lld/interface/i2c/i2c_intf.hpp>
#include <Thor/lld/interface/i2c/i2c_types.hpp>
#include <array>
#include <cstring>
#include <limits>


#if defined( THOR_HLD_I2C )
/*-----------------------------------------------------------------------------
Aliases
-----------------------------------------------------------------------------*/
namespace HLD = ::Thor::I2C;
namespace LLD = ::Thor::LLD::I2C;

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = LLD::NUM_I2C_PERIPHS;

/*-----------------------------------------------------------------------------
Variables
-----------------------------------------------------------------------------*/
static size_t s_driver_initialized;           /**< Tracks the module level initialization state */
static HLD::Driver hld_driver[ NUM_DRIVERS ]; /**< Driver objects */

namespace Thor::I2C
{
  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Common thread for handling I2C interrupts in userspace
   *
   * @param arg   Unused
   */
  static void I2CxISRUserThread( void *arg )
  {
    using namespace Chimera::Thread;
    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Wait for something to wake this thread directly
      -----------------------------------------------------------------------*/
      if ( !this_thread::pendTaskMsg( ITCMsg::TSK_MSG_ISR_HANDLER ) )
      {
        continue;
      }

      /*-----------------------------------------------------------------------
      Handle every ISR. Don't know which triggered this.
      -----------------------------------------------------------------------*/
      for ( size_t index = 0; index < NUM_DRIVERS; index++ )
      {
        hld_driver[ index ].postISRProcessing();
      }
    }
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t reset()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Driver_rPtr getDriver( const Chimera::I2C::Channel channel )
  {
    return nullptr;
  }


  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::open( const Chimera::I2C::DriverConfig &cfg )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::close()
  {
    return Chimera::Status::NOT_SUPPORTED;
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


  Chimera::Status_t Driver::stop()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::start()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Thread::BinarySemaphore &notifier,
                                   const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Driver::postISRProcessing()
  {
  }

}    // namespace Thor::I2C

#endif /* THOR_HLD_I2C */
