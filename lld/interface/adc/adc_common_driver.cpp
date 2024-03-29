/******************************************************************************
 *  File Name:
 *    adc_common_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the ADC hardware. All functionality is
 *    either completely independent of a specific device or is common to all.
 *
 *  2020-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/algorithm>
#include <Chimera/common>
#include <Chimera/utility>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/adc>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/timer>
#include <cstring>
#include <etl/function.h>
#include <limits>


#if defined( THOR_ADC ) && ( defined( TARGET_STM32F4 ) || defined( TARGET_STM32L4 ) )

namespace Thor::LLD::ADC
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Driver s_adc_drivers[ NUM_ADC_PERIPHS ];

  /*---------------------------------------------------------------------------
  Shared Data
  ---------------------------------------------------------------------------*/
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
  PeriphQueue ADC1_Queue;
#endif
#if defined( STM32_ADC2_PERIPH_AVAILABLE )
  PeriphQueue ADC2_Queue;
#endif
#if defined( STM32_ADC3_PERIPH_AVAILABLE )
  PeriphQueue ADC3_Queue;
#endif

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
  static void lockADC1()
  {
    s_adc_drivers[ ADC1_RESOURCE_INDEX ].disableInterrupts();
  }

  static void unlockADC1()
  {
    s_adc_drivers[ ADC1_RESOURCE_INDEX ].enableInterrupts();
  }

  static etl::function_fv<lockADC1>   adc1Lock;
  static etl::function_fv<unlockADC1> adc1Unlock;
#endif /* STM32_ADC1_PERIPH_AVAILABLE */

#if defined( STM32_ADC2_PERIPH_AVAILABLE )
  static void lockADC2()
  {
    s_adc_drivers[ ADC2_RESOURCE_INDEX ].disableInterrupts();
  }

  static void unlockADC2()
  {
    s_adc_drivers[ ADC2_RESOURCE_INDEX ].enableInterrupts();
  }

  static etl::function_fv<lockADC2>   adc2Lock;
  static etl::function_fv<unlockADC2> adc2Unlock;
#endif /* STM32_ADC2_PERIPH_AVAILABLE */

#if defined( STM32_ADC3_PERIPH_AVAILABLE )
  static void lockADC3()
  {
    s_adc_drivers[ ADC3_RESOURCE_INDEX ].disableInterrupts();
  }

  static void unlockADC3()
  {
    s_adc_drivers[ ADC3_RESOURCE_INDEX ].enableInterrupts();
  }

  static etl::function_fv<lockADC3>   adc3Lock;
  static etl::function_fv<unlockADC3> adc3Unlock;
#endif /* STM32_ADC3_PERIPH_AVAILABLE */

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------------------------------*/
    if ( !attachDriverInstances( s_adc_drivers, ARRAY_COUNT( s_adc_drivers ) ) )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------------------------------
    Initialize ADC channel queues
    -------------------------------------------------------------------------*/
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
    for ( size_t ch = 0; ch < ARRAY_COUNT( ADC1_Queue ); ch++ )
    {
      ADC1_Queue[ ch ] = new ChannelQueue<CHANNEL_QUEUE_SAMPLE_DEPTH>( adc1Lock, adc1Unlock );
    }
#endif
#if defined( STM32_ADC2_PERIPH_AVAILABLE )
    for ( size_t ch = 0; ch < ARRAY_COUNT( ADC2_Queue ); ch++ )
    {
      ADC2_Queue[ ch ] = new ChannelQueue<CHANNEL_QUEUE_SAMPLE_DEPTH>( adc2Lock, adc2Unlock );
    }
#endif
#if defined( STM32_ADC3_PERIPH_AVAILABLE )
    for ( size_t ch = 0; ch < ARRAY_COUNT( ADC3_Queue ); ch++ )
    {
      ADC3_Queue[ ch ] = new ChannelQueue<CHANNEL_QUEUE_SAMPLE_DEPTH>( adc3Lock, adc3Unlock );
    }
#endif

    return Chimera::Status::OK;
  }


  Driver_rPtr getDriver( const Chimera::ADC::Peripheral periph )
  {
    if ( auto idx = getResourceIndex( periph ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_adc_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  bool featureSupported( const Chimera::ADC::Peripheral periph, const Chimera::ADC::Feature feature )
  {
    return true;
  }


  /*---------------------------------------------------------------------------
  Low Level Driver Methods
  ---------------------------------------------------------------------------*/
  Chimera::Status_t Driver::reset()
  {
    /*-------------------------------------------------------------------------
    Use the RCC peripheral to invoke the reset. The clock must be enabled first
    or else this won't work.
    -------------------------------------------------------------------------*/
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    return rcc->reset( Chimera::Peripheral::Type::PERIPH_ADC, mResourceIndex );
  }


  void Driver::clockEnable()
  {
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_ADC, mResourceIndex );
  }


  void Driver::clockDisable()
  {
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_ADC, mResourceIndex );
  }


  void Driver::disableInterrupts()
  {
    Thor::LLD::INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }


  void Driver::enableInterrupts()
  {
    Thor::LLD::INT::enableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }


  void Driver::onInterrupt( const Chimera::ADC::Interrupt signal, Chimera::ADC::ISRCallback cb )
  {
    if ( signal < Chimera::ADC::Interrupt::NUM_OPTIONS )
    {
      /*-----------------------------------------------------------------------
      Globally disable interrupts while updating this structure. ADC ISR
      handlers can span across many DMA/ADC interrupt events and it's not
      practical to deduce which one is currently active.
      -----------------------------------------------------------------------*/
      auto mask                         = Thor::LLD::INT::disableInterrupts();
      mCallbacks[ EnumValue( signal ) ] = cb;
      Thor::LLD::INT::enableInterrupts( mask );
    }
  }

}    // namespace Thor::LLD::ADC


void ADC_IRQHandler()
{
  using namespace Thor::LLD::ADC;

#if defined( STM32_ADC1_PERIPH_AVAILABLE )
  s_adc_drivers[ ADC1_RESOURCE_INDEX ].ISRHandler();
#endif
#if defined( STM32_ADC2_PERIPH_AVAILABLE )
  s_adc_drivers[ ADC2_RESOURCE_INDEX ].ISRHandler();
#endif
#if defined( STM32_ADC3_PERIPH_AVAILABLE )
  s_adc_drivers[ ADC3_RESOURCE_INDEX ].ISRHandler();
#endif
}

#endif /* THOR_ADC */
