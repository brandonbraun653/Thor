/********************************************************************************
 *  File Name:
 *    hw_adc_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the ADC hardware. All functionality here is
 *    either completely independent of a specific device or is common to all.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <cstring>
#include <limits>

/* ETL Includes */
#include <etl/function.h>

/* Chimera Includes */
#include <Chimera/algorithm>
#include <Chimera/common>
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/adc>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/timer>


#if defined( THOR_LLD_ADC ) && ( defined( TARGET_STM32F4 ) || defined( TARGET_STM32L4 ) )

namespace Thor::LLD::ADC
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static Driver s_adc_drivers[ NUM_ADC_PERIPHS ];

  /*-------------------------------------------------------------------------------
  Shared Data
  -------------------------------------------------------------------------------*/
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
  PeriphQueue ADC1_Queue;
#endif /* STM32_ADC1_PERIPH_AVAILABLE */

  /*-------------------------------------------------------------------------------
  Static Functions
  -------------------------------------------------------------------------------*/
  static void lockADC1()
  {
    s_adc_drivers[ ADC1_RESOURCE_INDEX ].disableInterrupts();
  }

  static void unlockADC1()
  {
    s_adc_drivers[ ADC1_RESOURCE_INDEX ].enableInterrupts();
  }

  static etl::function_fv<lockADC1> adc1Lock;
  static etl::function_fv<unlockADC1> adc1Unlock;

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
    if ( !attachDriverInstances( s_adc_drivers, ARRAY_COUNT( s_adc_drivers ) ) )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------
    Initialize ADC1 channel queues
    -------------------------------------------------*/
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
    for ( size_t ch = 0; ch < ARRAY_COUNT( ADC1_Queue ); ch++ )
    {
      ADC1_Queue[ ch ] = std::make_shared<ChannelQueue<CHANNEL_QUEUE_SAMPLE_DEPTH>>( adc1Lock, adc1Unlock );
    }
#endif /* STM32_ADC1_PERIPH_AVAILABLE */

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


  /*-------------------------------------------------------------------------------
  Low Level Driver Methods
  -------------------------------------------------------------------------------*/
  Chimera::Status_t Driver::reset()
  {
    /*-------------------------------------------------
    Use the RCC peripheral to invoke the reset. The
    clock must be enabled first or else this won't work.
    -------------------------------------------------*/
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


  inline void Driver::disableInterrupts()
  {
    Thor::LLD::INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }


  inline void Driver::enableInterrupts()
  {
    Thor::LLD::INT::enableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }

}    // namespace Thor::LLD::ADC


#if defined( STM32_ADC1_PERIPH_AVAILABLE )
void ADC_IRQHandler()
{
  using namespace Thor::LLD::ADC;
  s_adc_drivers[ ADC1_RESOURCE_INDEX ].IRQHandler();
}
#endif

#endif /* TARGET_STM32L4 && THOR_DRIVER_ADC */
