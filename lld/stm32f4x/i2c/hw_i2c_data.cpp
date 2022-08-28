/******************************************************************************
 *  File Name:
 *    hw_i2c_data.cpp
 *
 *  Description:
 *    Provides implementation details for private I2C data
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/i2c>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/i2c>
#include <limits>

#if defined( TARGET_STM32F4 ) && defined( THOR_I2C )
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

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Driver s_i2c_drivers[ NUM_I2C_PERIPHS ];

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    if ( attachDriverInstances( s_i2c_drivers, ARRAY_COUNT( s_i2c_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  bool isChannelSupported( const Chimera::I2C::Channel channel )
  {
    if ( channel < Chimera::I2C::Channel::NUM_OPTIONS )
    {
      return ( getResourceIndex( channel ) != INVALID_RESOURCE_INDEX );
    }
    else
    {
      return false;
    }
  }


  Driver_rPtr getDriver( const Chimera::I2C::Channel channel )
  {
    if ( isChannelSupported( channel ) )
    {
      return &s_i2c_drivers[ getResourceIndex( channel ) ];
    }

    return nullptr;
  }
}    // namespace Thor::LLD::I2C
#endif /* TARGET_STM32F4 && THOR_LLD_I2C */


/*-----------------------------------------------------------------------------
IRQ Handlers
-----------------------------------------------------------------------------*/
#if defined( STM32_I2C1_PERIPH_AVAILABLE )
void I2C1_EV_IRQHandler()
{
  using namespace Thor::LLD::I2C;
  s_i2c_drivers[ I2C1_RESOURCE_INDEX ].IRQEventHandler();
}


void I2C1_ER_IRQHandler()
{
  using namespace Thor::LLD::I2C;
  s_i2c_drivers[ I2C1_RESOURCE_INDEX ].IRQErrorHandler();
}
#endif /* STM32_I2C1_PERIPH_AVAILABLE */

#if defined( STM32_I2C2_PERIPH_AVAILABLE )
void I2C2_EV_IRQHandler()
{
  using namespace Thor::LLD::I2C;
  s_i2c_drivers[ I2C2_RESOURCE_INDEX ].IRQEventHandler();
}


void I2C2_ER_IRQHandler()
{
  using namespace Thor::LLD::I2C;
  s_i2c_drivers[ I2C2_RESOURCE_INDEX ].IRQErrorHandler();
}
#endif /* STM32_I2C2_PERIPH_AVAILABLE */


#if defined( STM32_I2C3_PERIPH_AVAILABLE )
void I2C3_EV_IRQHandler()
{
  using namespace Thor::LLD::I2C;
  s_i2c_drivers[ I2C3_RESOURCE_INDEX ].IRQEventHandler();
}


void I2C3_ER_IRQHandler()
{
  using namespace Thor::LLD::I2C;
  s_i2c_drivers[ I2C3_RESOURCE_INDEX ].IRQErrorHandler();
}
#endif /* STM32_I2C3_PERIPH_AVAILABLE */