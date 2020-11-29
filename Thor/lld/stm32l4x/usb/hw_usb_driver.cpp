/********************************************************************************
 *  File Name:
 *    hw_usb_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series USB hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <cstring>
#include <limits>

/* Chimera Includes */
#include <Chimera/algorithm>
#include <Chimera/common>
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/interrupt/hld_interrupt_definitions.hpp>

#include <Thor/lld/interface/usb/usb_prv_data.hpp>
#include <Thor/lld/interface/usb/usb_intf.hpp>
#include <Thor/lld/interface/usb/usb_types.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_driver.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_types.hpp>
#include <Thor/lld/stm32l4x/usb/hw_usb_prj.hpp>
#include <Thor/lld/stm32l4x/usb/hw_usb_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_USB )

namespace Thor::LLD::USB
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static Driver s_usb_drivers[ NUM_USB_PERIPHS ];


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
    if ( attachDriverInstances( s_usb_drivers, ARRAY_COUNT( s_usb_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  Driver_rPtr getDriver( const Chimera::USB::Channel channel )
  {
    if ( auto idx = getResourceIndex( channel ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_usb_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr ), mResourceIndex( std::numeric_limits<size_t>::max() )
  {
  }

  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    /*------------------------------------------------
    Get peripheral descriptor settings
    ------------------------------------------------*/
    mPeriph        = peripheral;
    mResourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*------------------------------------------------
    Handle the ISR configuration
    ------------------------------------------------*/
    Thor::LLD::IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Thor::LLD::IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Thor::LLD::IT::setPriority( Resource::IRQSignals[ mResourceIndex ], Thor::Interrupt::USB_IT_PREEMPT_PRIORITY, 0u );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::initialize( const Chimera::USB::PeriphConfig &cfg )
  {
    /*-------------------------------------------------
    Local constants
    -------------------------------------------------*/
    constexpr size_t USB_CLOCK_FREQ = 48000000;    // Standard 48MHz
    constexpr size_t USB_SYNC_FREQ  = 1000;        // Periodic 1ms SOF from host
    constexpr size_t USB_TRIM_STEP  = 14;          // Default for the CRS periph

    /*-------------------------------------------------
    Enable the peripheral clock
    -------------------------------------------------*/
    RCC::getCoreClock()->enableClock( Chimera::Clock::Bus::RC48 );
    clockEnable();

    /*-------------------------------------------------
    Configure the CRS driver to sync the RC48 bus on
    the USB SOF signal. This helps tighten the clock.
    -------------------------------------------------*/
    auto crsResult = Chimera::Status::OK;

    crsResult |= CRS::initialize();
    crsResult |= CRS::selectSyncSource( CRS::SyncSource::USB_SOF );
    crsResult |= CRS::configureDiv( CRS::SyncDiv::SYNC_DIV1 );
    crsResult |= CRS::configureHWAlgo( USB_CLOCK_FREQ, USB_SYNC_FREQ, USB_TRIM_STEP );

    CRS::toggleAutoTrim( true );
    CRS::toggleFEQ( true );

    if ( crsResult != Chimera::Status::OK )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------
    Mask all interrupts and clear pending
    -------------------------------------------------*/

    /*-------------------------------------------------
    Reset the USB peripheral
    -------------------------------------------------*/

    /*-------------------------------------------------
    Configure Interrupts
    -------------------------------------------------*/

    /*-------------------------------------------------
    Enable/disable LPM support
    -------------------------------------------------*/

    /*-------------------------------------------------
    Enable/disable Battery Charging Support
    -------------------------------------------------*/
  }


  Chimera::Status_t Driver::reset()
  {
    /*-------------------------------------------------
    Power down the peripheral
    -------------------------------------------------*/
    // TODO: implement the power down sequence

    /*-------------------------------------------------
    Disable the peripheral clock. Don't touch the RC48
    clock as it might be in use by the RNG driver.
    -------------------------------------------------*/
    clockDisable();

    /*-------------------------------------------------
    Disable interrupts
    -------------------------------------------------*/
    Thor::LLD::IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );

    return Chimera::Status::OK;
  }


  void Driver::clockEnable()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_USB, mResourceIndex );
  }


  void Driver::clockDisable()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_USB, mResourceIndex );
  }


  inline void Driver::enterCriticalSection()
  {
    Thor::LLD::IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }


  inline void Driver::exitCriticalSection()
  {
    Thor::LLD::IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }


  void Driver::IRQHandler()
  {
    // dispatch to individual handlers
  }


  void Driver::IRQH_CorrectTransfer()
  {
  }


  void Driver::IRQH_PMA()
  {
  }


  void Driver::IRQH_Error()
  {
  }


  void Driver::IRQH_Wakeup()
  {
  }


  void Driver::IRQH_Suspend()
  {
  }


  void Driver::IRQH_Reset()
  {
  }


  void Driver::IRQH_FrameStart()
  {
  }


  void Driver::IRQH_LostFrameStart()
  {
  }


  void Driver::IRQH_LowPowerModeRequest()
  {
  }

}    // namespace Thor::LLD::USB


#if defined( STM32_USB1_PERIPH_AVAILABLE )
void OTG_FS_IRQHandler()
{
  using namespace Thor::LLD::USB;
  s_usb_drivers[ USB1_RESOURCE_INDEX ].IRQHandler();
}
#endif

#endif /* TARGET_STM32L4 && THOR_DRIVER_USB */
