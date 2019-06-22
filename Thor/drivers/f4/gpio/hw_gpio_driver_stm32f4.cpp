/********************************************************************************
 *   File Name:
 *    hw_gpio_driver_stm32f4.cpp
 *
 *   Description:
 *    Implements the low level driver for the GPIO peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */

/* Driver Includes */
#include <Thor/drivers/common/cortex-m4/utilities.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_driver.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_prj.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_types.hpp>

namespace Thor::Driver::GPIO
{
  Driver::Driver( RegisterMap *const peripheral ) : periph( peripheral )
  {
    mutex = Chimera::Threading::createRecursiveMutex();
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::threadedDriveSet( const Chimera::GPIO::Drive drive )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::threadedSpeedSet( const Thor::Driver::GPIO::Speed speed )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  Chimera::Status_t Driver::threadedPullSet( const Chimera::GPIO::Pull pull )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  size_t Driver::threadedRead()
  {
    return 0;
  };

  Chimera::Status_t Driver::threadedWrite( const size_t val )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  Chimera::Status_t Driver::threadedAlternateFunctionSet( const size_t val )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  size_t Driver::threadedAlternateFunctionGet()
  {
    return 0;
  };

  Chimera::Status_t Driver::atomicDriveSet( const Chimera::GPIO::Drive drive )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  Chimera::Status_t Driver::atomicSpeedSet( const Thor::Driver::GPIO::Speed speed )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  Chimera::Status_t Driver::atomicPullSet( const Chimera::GPIO::Pull pull )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  Chimera::Status_t Driver::atomicWrite( const size_t val )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  size_t Driver::atomicRead()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  Chimera::Status_t Driver::atomicAlternateFunctionSet( const size_t val )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  size_t Driver::atomicAlternateFunctionGet()
  {
    return 0;
  };

  Chimera::Status_t Driver::driveSet( const Chimera::GPIO::Drive drive )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  Chimera::Status_t Driver::speedSet( const Thor::Driver::GPIO::Speed speed )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  Chimera::Status_t Driver::pullSet( const Chimera::GPIO::Pull pull )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  size_t Driver::read()
  {
    return 0;
  };

  Chimera::Status_t Driver::write( const size_t val )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  Chimera::Status_t Driver::alternateFunctionSet( const size_t val )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  };

  size_t Driver::alternateFunctionGet()
  {
    return 0;
  };

  Chimera::Status_t Driver::enableSignal( const InterruptSignal_t sig )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::disableSignal( const InterruptSignal_t sig )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}    // namespace Thor::Driver::GPIO
