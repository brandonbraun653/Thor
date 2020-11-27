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
  Driver::Driver() : mPeriph( nullptr ), resourceIndex( std::numeric_limits<size_t>::max() )
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
    mPeriph       = peripheral;
    resourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*------------------------------------------------
    Handle the ISR configuration
    ------------------------------------------------*/
    Thor::LLD::IT::disableIRQ( Resource::IRQSignals[ resourceIndex ] );
    Thor::LLD::IT::clearPendingIRQ( Resource::IRQSignals[ resourceIndex ] );
    Thor::LLD::IT::setPriority( Resource::IRQSignals[ resourceIndex ], Thor::Interrupt::USB_IT_PREEMPT_PRIORITY, 0u );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::reset()
  {
    return Chimera::Status::OK;
  }


  void Driver::clockEnable()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_USB, resourceIndex );
  }


  void Driver::clockDisable()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_USB, resourceIndex );
  }


  inline void Driver::enterCriticalSection()
  {
    Thor::LLD::IT::disableIRQ( Resource::IRQSignals[ resourceIndex ] );
  }


  inline void Driver::exitCriticalSection()
  {
    Thor::LLD::IT::enableIRQ( Resource::IRQSignals[ resourceIndex ] );
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
