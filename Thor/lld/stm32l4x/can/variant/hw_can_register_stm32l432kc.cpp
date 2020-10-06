/********************************************************************************
 *  File Name:
 *    hw_can_register_stm32l432kc.cpp
 *
 *  Description:
 *    CAN register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/can>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/interface/can/can_intf.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32l4x/can/hw_can_types.hpp>
#include <Thor/lld/stm32l4x/can/hw_can_prj.hpp>
#include <Thor/lld/stm32l4x/can/variant/hw_can_register_stm32l4xxxx.hpp>


namespace Thor::LLD::CAN
{
  void initializeRegisters()
  {
  }
}  // namespace Thor::LLD::can


namespace Thor::LLD::RCC::LookupTables
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  RegisterConfig CAN_ClockConfig[ Thor::LLD::CAN::NUM_CAN_PERIPHS ];
  RegisterConfig CAN_ResetConfig[ Thor::LLD::CAN::NUM_CAN_PERIPHS ];
  Chimera::Clock::Bus CAN_SourceClock[ Thor::LLD::CAN::NUM_CAN_PERIPHS ];

  PCC CANLookup = { CAN_ClockConfig,
                    nullptr,
                    CAN_ResetConfig,
                    CAN_SourceClock,
                    Thor::LLD::CAN::NUM_CAN_PERIPHS,
                    Thor::LLD::CAN::getResourceIndex };

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void CANInit()
  {
    using namespace Thor::LLD::CAN;

    /*------------------------------------------------
    CAN clock enable register access lookup table
    ------------------------------------------------*/
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
    CAN_ClockConfig[ CAN1_RESOURCE_INDEX ].mask = APB1ENR1_CAN1EN;
    CAN_ClockConfig[ CAN1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;
#endif

    /*------------------------------------------------
    CAN reset register access lookup table
    ------------------------------------------------*/
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
    CAN_ResetConfig[ CAN1_RESOURCE_INDEX ].mask = APB1RSTR1_CAN1RST;
    CAN_ResetConfig[ CAN1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR;
#endif

    /*------------------------------------------------
    CAN clocking bus source identifier
    ------------------------------------------------*/
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
    CAN_SourceClock[ CAN1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
#endif

  };
}