/********************************************************************************
 *  File Name:
 *    hw_rcc_data.cpp
 *
 *  Description:
 *    Implements the required LLD data interface
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/rcc/rcc_detail.hpp>
#include <Thor/lld/interface/rcc/rcc_prv_data.hpp>
#include <Thor/lld/interface/watchdog/watchdog_intf.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_prj.hpp>
#include <Thor/lld/stm32f4x/flash/hw_flash_prj.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_prj.hpp>
#include <Thor/lld/stm32f4x/power/hw_power_prj.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32f4x/system/sys_memory_map_prj.hpp>
#include <Thor/lld/stm32f4x/uart/hw_uart_prj.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_prj.hpp>
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_prj.hpp>


#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_RCC )

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
#if defined( THOR_LLD_WWDG )
  static const RegisterConfig WWDG_ClockConfig[ NUM_WWDG_PERIPHS ]      = { { .mask = RCC::APB1ENR_WWDGEN,
                                                                         .reg  = &RCC::RCC1_PERIPH->APB1ENR } };
  static const RegisterConfig WWDG_ResetConfig[ NUM_WWDG_PERIPHS ]      = { { .mask = 0, .reg = nullptr } };
  static const Chimera::Clock::Bus WWDG_SourceClock[ NUM_WWDG_PERIPHS ] = { Chimera::Clock::Bus::APB1 };

#endif /* THOR_LLD_WWDG */

  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  const PCC sReg[] = { /* clang-format off */

    /* WWDG */
    #if defined( THOR_LLD_WWDG )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_WWDG ),
      .elements         = WWDG::NUM_WWDG_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = &WWDG_clockConfig,
      .clockLP          = nullptr,
      .reset            = &WWDG_ResetConfig,
      .clockSource      = &WWDG_SourceClock,
      .getResourceIndex = Watchdog::getResourceIndex
    }
    #endif  /* THOR_LLD_WWDG */
  }; /* clang-format on */

  /*-------------------------------------------------------------------------------
  Public Data
  -------------------------------------------------------------------------------*/
#if defined( STM32_RCC1_PERIPH_AVAILABLE )
  RegisterMap *RCC1_PERIPH = reinterpret_cast<RegisterMap *>( RCC1_BASE_ADDR );


  PCC LLD_CONST *PeripheralControlRegistry[ static_cast<size_t>( Chimera::Peripheral::Type::NUM_OPTIONS ) ];

#endif

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initializeRegistry()
  {
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------
    First initialize the registry to empty
    -------------------------------------------------*/
    for ( size_t x = 0; x < ARRAY_COUNT( PeripheralControlRegistry ); x++ )
    {
      PeripheralControlRegistry[ x ] = nullptr;
    }

    /*-------------------------------------------------
    Fill in the defined data where possible
    -------------------------------------------------*/
    for ( size_t x = 0; x < ARRAY_COUNT( sReg ); x++ )
    {
      Type p = static_cast<Type>( sReg[ x ].pType );
      if ( p < Type::NUM_OPTIONS )
      {
        PeripheralControlRegistry[ sReg[ x ].pType ] = &sReg[ x ];
      }
    }
  }


}    // namespace Thor::LLD::RCC

#endif /* TARGET_STM32F4 && THOR_LLD_RCC */
