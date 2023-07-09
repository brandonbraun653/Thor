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
   * @note Written using the formula from the STM32F4xx reference manual
   *
   * @param in_freq    Input clock frequency in Hz
   * @param out_freq   Target bus frequency in Hz
   * @return uint32_t HW register clock divider setting
   */
  static inline uint32_t calcClockDivider( const uint32_t in_freq, const uint32_t out_freq )
  {
    // Input constraints
    RT_DBG_ASSERT( in_freq > out_freq );
    RT_DBG_ASSERT( out_freq > 0 );

    // Calculate the divider and ensure it fits in the register field
    const uint32_t div = ( in_freq / out_freq ) - 2u;
    RT_DBG_ASSERT( ( div & CLKCR_CLKDIV_Msk ) == div );

    return div;
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


  void Driver::busClockEnable()
  {
    CLKEN::set( mPeriph, CLKCR_CLKEN );
  }


  void Driver::busClockDisable()
  {
    CLKEN::clear( mPeriph, CLKCR_CLKEN );
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
    /*-------------------------------------------------------------------------
    Reset the peripheral
    -------------------------------------------------------------------------*/
    reset();

    /*-------------------------------------------------------------------------
    Configure the clock divider to a default of 300KHz
    -------------------------------------------------------------------------*/
    auto           rcc  = RCC::getCoreClockCtrl();
    const uint32_t pClk = rcc->getPeriphClock( Chimera::Peripheral::Type::PERIPH_SDIO, reinterpret_cast<uintptr_t>( mPeriph ) );
    const uint32_t clkDiv = calcClockDivider( pClk, 300000u );

    CLKDIV::set( mPeriph, clkDiv << CLKCR_CLKDIV_Pos );

    /*-------------------------------------------------------------------------
    Set remainder of the clock control register
    -------------------------------------------------------------------------*/
    HWFCEN::clear( mPeriph, CLKCR_HWFC_EN );     // Disable HW flow control
    NEGEDGE::clear( mPeriph, CLKCR_NEGEDGE );    // Data is valid on the falling edge
    WIDBUS::set( mPeriph, 0 );                   // 1-bit bus width
    BYPASS::clear( mPeriph, CLKCR_BYPASS );      // Disable clock bypass
    PWRSAV::clear( mPeriph, CLKCR_PWRSAV );      // Disable power saving mode
    CLKEN::clear( mPeriph, CLKCR_CLKEN );        // Disable the clock output

    return Chimera::Status::OK;
  }


  uint32_t Driver::getBusFrequency()
  {
    auto           rcc  = RCC::getCoreClockCtrl();
    const uint32_t pClk = rcc->getPeriphClock( Chimera::Peripheral::Type::PERIPH_SDIO, reinterpret_cast<uintptr_t>( mPeriph ) );
    const uint32_t clkDiv = CLKDIV::get( mPeriph ) >> CLKCR_CLKDIV_Pos;

    return pClk / ( clkDiv + 2 );
  }


  void Driver::IRQHandler()
  {
  }
}    // namespace Thor::LLD::SDIO
#endif /* THOR_SDIO && TARGET_STM32F4 */
