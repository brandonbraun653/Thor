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
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Calculates the HW clock divider setting for a given target bus frequency
   *
   * @param in_freq    Input clock frequency in Hz
   * @param out_freq   Target bus frequency in Hz
   * @return uint32_t HW register clock divider setting
   */
  static uint32_t calcClockDivider( const uint32_t in_freq, const uint32_t out_freq )
  {
    // TODO: Fill me in!
    return 0;
  }


  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr ), mResourceIdx( INVALID_RESOURCE_INDEX )
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
    mPeriph      = peripheral;
    mResourceIdx = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*-------------------------------------------------------------------------
    Handle the ISR configuration
    -------------------------------------------------------------------------*/
    INT::disableIRQ( Resource::IRQSignals[ mResourceIdx ] );
    INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIdx ] );
    INT::setPriority( Resource::IRQSignals[ mResourceIdx ], INT::SDIO_IT_PREEMPT_PRIORITY, 0u );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::reset()
  {
    auto rcc = RCC::getPeriphClockCtrl();
    return rcc->reset( Chimera::Peripheral::Type::PERIPH_SDIO, mResourceIdx );
  }


  void Driver::clockEnable()
  {
    auto rcc = RCC::getPeriphClockCtrl();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_SDIO, mResourceIdx );
  }


  void Driver::clockDisable()
  {
    auto rcc = RCC::getPeriphClockCtrl();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_SDIO, mResourceIdx );
  }


  void Driver::enterCriticalSection()
  {
    INT::disableIRQ( Resource::IRQSignals[ mResourceIdx ] );
  }


  void Driver::exitCriticalSection()
  {
    INT::enableIRQ( Resource::IRQSignals[ mResourceIdx ] );
  }


  Chimera::Status_t Driver::init()
  {
    return Chimera::Status::FAIL;
  }


  uint32_t Driver::getBusFrequency()
  {
    return 0;
  }

  void Driver::IRQHandler()
  {
  }
}    // namespace Thor::LLD::SDIO
#endif /* THOR_SDIO && TARGET_STM32F4 */
