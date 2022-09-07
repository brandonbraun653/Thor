/********************************************************************************
 *  File Name:
 *    bx_can_data.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <limits>
#include <Chimera/can>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/can>

#if defined( THOR_CAN ) && ( defined( TARGET_STM32L4 ) || defined( TARGET_STM32F4 ) )
namespace Thor::LLD::CAN
{
  /*---------------------------------------------------------------------------
  Peripheral Memory Maps
  ---------------------------------------------------------------------------*/
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
  RegisterMap *CAN1_PERIPH = reinterpret_cast<RegisterMap *>( CAN1_BASE_ADDR );
#endif
#if defined( STM32_CAN2_PERIPH_AVAILABLE )
  RegisterMap *CAN2_PERIPH = reinterpret_cast<RegisterMap *>( CAN2_BASE_ADDR );
#endif

  /*---------------------------------------------------------------------------
  Configuration Maps
  ---------------------------------------------------------------------------*/
  namespace ConfigMap
  { /* clang-format off */
    // Definitions correspond with the CAN_BTR register
    LLD_CONST Reg32_t DebugMode[ static_cast<size_t>( Chimera::CAN::DebugMode::NUM_OPTIONS ) ] =
    {
      BTR_SILM,               // Silent Mode
      BTR_LBKM,               // Loopback Mode
      ( BTR_SILM | BTR_LBKM ) // Silent + Loopback
    };

    // Definitions correspond with the CAN_TIxR & CAN_RIxR registers
    LLD_CONST Reg32_t IdentifierMode[ static_cast<size_t>( Chimera::CAN::IdType::NUM_OPTIONS ) ] = {
      0,        // Standard
      TIxR_IDE  // Extended
    };

    // Definitions correspond with the CAN_TIxR & CAN_RIxR registers
    LLD_CONST Reg32_t FrameType[ static_cast<size_t>( Chimera::CAN::FrameType::NUM_OPTIONS ) ] = {
      0,        // Data Frame
      TIxR_RTR  // Remote Frame
    };

  } /* clang-format on */

  /*---------------------------------------------------------------------------
  Peripheral Resources
  ---------------------------------------------------------------------------*/
  namespace Resource
  { /* clang-format off */
    LLD_CONST IRQn_Type IRQSignals[ NUM_CAN_PERIPHS ][ NUM_CAN_IRQ_HANDLERS ] = {
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
      { CAN1_TX_IRQn, CAN1_RX0_IRQn, CAN1_RX1_IRQn, CAN1_SCE_IRQn },
#endif
#if defined( STM32_CAN2_PERIPH_AVAILABLE )
      { CAN2_TX_IRQn, CAN2_RX0_IRQn, CAN2_RX1_IRQn, CAN2_SCE_IRQn },
#endif
    };
  } /* clang-format on */
}    // namespace Thor::LLD::CAN

#endif /* TARGET_STM32L4 & THOR_LLD_CAN */
