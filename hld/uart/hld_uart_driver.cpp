/********************************************************************************
 *  File Name:
 *    hld_uart_driver.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor UART interface.
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Chimera/assert>
#include <Chimera/common>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/uart>

//#if defined( THOR_UART )
namespace Chimera::UART
{
  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::open( const Chimera::Serial::Config &config )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::close()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  int Driver::write( const void *const buffer, const size_t length )
  {
    return 0;
  }


  int Driver::read( void *const buffer, const size_t length )
  {
    return 0;
  }
}


namespace Chimera::UART::Backend
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t registerDriver( Chimera::UART::Backend::DriverConfig &registry )
  {
    registry.isSupported   = false;
    registry.getDriver     = nullptr;
    registry.initialize    = nullptr;
    registry.reset         = nullptr;
    registry.isChannelUART = nullptr;
    return Chimera::Status::NOT_SUPPORTED;
  }
}
// #endif /* THOR_UART */
