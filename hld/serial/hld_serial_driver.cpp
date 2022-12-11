/******************************************************************************
 *  File Name:
 *    hld_serial_driver.cpp
 *
 *  Description:
 *    Serial (UART) driver interface for STM32 UART/USART peripherals
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/serial>

namespace Chimera::Serial
{
  /*---------------------------------------------------------------------------
  Driver Class Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }

  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::open( const Chimera::Serial::Config &config )
  {
  }


  Chimera::Status_t Driver::close()
  {
  }


  int Driver::write( const void *const buffer, const size_t length )
  {
  }


  int Driver::read( void *const buffer, const size_t length )
  {
  }


  size_t Driver::available()
  {
  }

}    // namespace Chimera::Serial
