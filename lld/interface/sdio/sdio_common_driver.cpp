/******************************************************************************
 *  File Name:
 *    sdio_common_driver.cpp
 *
 *  Description:
 *    Common driver for the STM32 SDIO peripheral
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/sdio>

#if defined( THOR_SDIO ) && defined( TARGET_STM32F4 )
namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Driver s_sdio_drivers[ NUM_SDIO_PERIPHS ];

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr ), resourceIndex( INVALID_RESOURCE_INDEX )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    /*-------------------------------------------------------------------------
    Get peripheral descriptor settings
    -------------------------------------------------------------------------*/
    mPeriph       = peripheral;
    resourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*-------------------------------------------------------------------------
    Handle the ISR configuration
    -------------------------------------------------------------------------*/
    INT::disableIRQ( Resource::IRQSignals[ resourceIndex ] );
    INT::clearPendingIRQ( Resource::IRQSignals[ resourceIndex ] );
    INT::setPriority( Resource::IRQSignals[ resourceIndex ], INT::SPI_IT_PREEMPT_PRIORITY, 0u );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::reset()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Driver::clockEnable()
  {
  }


  void Driver::clockDisable()
  {
  }


  void Driver::IRQHandler()
  {
  }


  void Driver::enterCriticalSection()
  {
  }


  void Driver::exitCriticalSection()
  {
  }

}    // namespace Thor::LLD::SDIO
#endif /* THOR_SDIO && TARGET_STM32F4 */
