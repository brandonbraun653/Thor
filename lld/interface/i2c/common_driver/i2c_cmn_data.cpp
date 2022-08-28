/******************************************************************************
 *  File Name:
 *    i2c_cmn_data.cpp
 *
 *  Description:
 *    Shared data for the I2C LLD interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/i2c>
#include <Chimera/utility>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/i2c>

#if defined( THOR_I2C )
namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Peripheral Memory Maps
  ---------------------------------------------------------------------------*/
#if defined( STM32_I2C1_PERIPH_AVAILABLE )
  RegisterMap *I2C1_PERIPH = reinterpret_cast<RegisterMap *>( I2C1_BASE_ADDR );
#endif
#if defined( STM32_I2C2_PERIPH_AVAILABLE )
  RegisterMap *I2C2_PERIPH = reinterpret_cast<RegisterMap *>( I2C2_BASE_ADDR );
#endif
#if defined( STM32_I2C3_PERIPH_AVAILABLE )
  RegisterMap *I2C3_PERIPH = reinterpret_cast<RegisterMap *>( I2C3_BASE_ADDR );
#endif

  /*---------------------------------------------------------------------------
  Configuration Maps
  ---------------------------------------------------------------------------*/
  namespace ConfigMap
  { /* clang-format off */

  } /* clang-format on */

  /*---------------------------------------------------------------------------
  Peripheral Resources
  ---------------------------------------------------------------------------*/
  namespace Resource
  { /* clang-format off */
    LLD_CONST Thor::LLD::DMA::Source DMASignals[ NUM_I2C_PERIPHS ][ DMA_SIG_PER_PERIPH ] = {
      #if defined( STM32_I2C1_PERIPH_AVAILABLE )
      { Thor::LLD::DMA::Source::I2C1_TX, Thor::LLD::DMA::Source::I2C1_RX },
      #endif
      #if defined( STM32_I2C2_PERIPH_AVAILABLE )
      { Thor::LLD::DMA::Source::I2C2_TX, Thor::LLD::DMA::Source::I2C2_RX },
      #endif
      #if defined( STM32_I2C3_PERIPH_AVAILABLE )
      { Thor::LLD::DMA::Source::I2C3_TX, Thor::LLD::DMA::Source::I2C3_RX },
      #endif
    };
    static_assert( static_cast<size_t>( DMADirection::TX ) == 0 );


    LLD_CONST IRQn_Type IRQSignals[ NUM_I2C_PERIPHS ][ ISR_VEC_PER_PERIPH ] = {
      #if defined( STM32_I2C1_PERIPH_AVAILABLE )
      { I2C1_EV_IRQn, I2C1_ER_IRQn },
      #endif
      #if defined( STM32_I2C2_PERIPH_AVAILABLE )
      { I2C2_EV_IRQn, I2C2_ER_IRQn },
      #endif
      #if defined( STM32_I2C3_PERIPH_AVAILABLE )
      { I2C3_EV_IRQn, I2C3_ER_IRQn },
      #endif
    };
    static_assert( static_cast<size_t>( IRQHandlerIndex::EVENT ) == 0 );
  } /* clang-format on */


}  // namespace Thor::LLD::I2C
#endif  /* THOR_LLD_I2C */
