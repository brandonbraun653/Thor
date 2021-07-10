/********************************************************************************
 *  File Name:
 *    hw_usb_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32F4 series USB hardware.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <cstring>
#include <limits>

/* Chimera Includes */
#include <Chimera/algorithm>
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/usb>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_USB )

namespace Thor::LLD::USB
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    const IRQn_Type usb_irq[ 6 ] = { OTG_FS_WKUP_IRQn,   OTG_FS_IRQn,      OTG_HS_EP1_OUT_IRQn,
                                     OTG_HS_EP1_IN_IRQn, OTG_HS_WKUP_IRQn, OTG_HS_IRQn };

    /*-------------------------------------------------
    Only enable clocks. Using 3rd party driver.
    -------------------------------------------------*/
    RCC::getPeriphClockCtrl()->enableClock( Chimera::Peripheral::Type::PERIPH_USB, USB_OTG_FS_RESOURCE_INDEX );
    RCC::getPeriphClockCtrl()->enableClock( Chimera::Peripheral::Type::PERIPH_USB, USB_OTG_HS_RESOURCE_INDEX );

    /*-------------------------------------------------
    Enable system interrupts
    -------------------------------------------------*/
    for( size_t x=0; x < ARRAY_COUNT( usb_irq ); x++ )
    {
      Thor::LLD::INT::disableIRQ( usb_irq[ x ] );
      Thor::LLD::INT::clearPendingIRQ( usb_irq[ x ]);
      Thor::LLD::INT::setPriority( usb_irq[ x ], Thor::LLD::INT::USB_IT_PREEMPT_PRIORITY, 0u );
      Thor::LLD::INT::enableIRQ( usb_irq[ x ] );
    }

    return Chimera::Status::OK;
  }
}    // namespace Thor::LLD::USB
#endif /* TARGET_STM32F4 && THOR_DRIVER_USB */
