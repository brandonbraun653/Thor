/********************************************************************************
 *  File Name:
 *    hw_usb_register_stm32l432kc.cpp
 *
 *  Description:
 *    USB register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/usb>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/interface/usb/usb_intf.hpp>
#include <Thor/lld/interface/usb/usb_detail.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32l4x/usb/hw_usb_types.hpp>
#include <Thor/lld/stm32l4x/usb/variant/hw_usb_register_stm32l4xxxx.hpp>

#if defined( STM32L432xx ) && defined( THOR_LLD_USB )

namespace Thor::LLD::USB
{
}    // namespace Thor::LLD::USB

namespace Thor::LLD::RCC::LookupTables
{
  RegisterConfig USB_ClockConfig[ Thor::LLD::USB::NUM_USB_PERIPHS ];
  RegisterConfig USB_ResetConfig[ Thor::LLD::USB::NUM_USB_PERIPHS ];
  Chimera::Clock::Bus USB_SourceClock[ Thor::LLD::USB::NUM_USB_PERIPHS ];

  PCC USBLookup = { USB_ClockConfig,
                    nullptr,
                    USB_ResetConfig,
                    USB_SourceClock,
                    Thor::LLD::USB::NUM_USB_PERIPHS,
                    Thor::LLD::USB::getResourceIndex };

  void USBInit()
  {
    using namespace Thor::LLD::USB;

/*------------------------------------------------
USB clock enable register access lookup table
------------------------------------------------*/
#if defined( STM32_USB1_PERIPH_AVAILABLE )
    USB_ClockConfig[ USB1_RESOURCE_INDEX ].mask = APB1ENR1_USBFSEN;
    USB_ClockConfig[ USB1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;
#endif

/*------------------------------------------------
USB reset register access lookup table
------------------------------------------------*/
#if defined( STM32_USB1_PERIPH_AVAILABLE )
    USB_ResetConfig[ USB1_RESOURCE_INDEX ].mask = APB1RSTR1_USBFSRST;
    USB_ResetConfig[ USB1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;
#endif

/*------------------------------------------------
USB clocking bus source identifier
------------------------------------------------*/
#if defined( STM32_USB1_PERIPH_AVAILABLE )
    USB_SourceClock[ USB1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
#endif
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_USB */
