/********************************************************************************
 *  File Name:
 *    hw_adc_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the ADC hardware.
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
#include <Thor/lld/interface/adc/adc_prv_data.hpp>
#include <Thor/lld/interface/adc/adc_intf.hpp>
#include <Thor/lld/interface/timer/timer_intf.hpp>
#include <Thor/lld/stm32l4x/adc/hw_adc_prj.hpp>
#include <Thor/lld/stm32l4x/adc/hw_adc_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_ADC )

namespace Thor::LLD::ADC
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static Driver s_adc_drivers[ NUM_ADC_PERIPHS ];

  /*-------------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------------*/

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
    if ( attachDriverInstances( s_adc_drivers, ARRAY_COUNT( s_adc_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  Driver_rPtr getDriver( const Chimera::ADC::Converter periph )
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


  bool featureSupported( const Chimera::ADC::Converter periph, const Chimera::ADC::Feature feature )
  {
    return true;
  }


  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver()
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
    mPeriph        = peripheral;
    mResourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*------------------------------------------------
    Handle the ISR configuration
    ------------------------------------------------*/
    Thor::LLD::IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Thor::LLD::IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Thor::LLD::IT::setPriority( Resource::IRQSignals[ mResourceIndex ], Thor::Interrupt::ADC_IT_PREEMPT_PRIORITY, 0u );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::initialize( const Chimera::ADC::DriverConfig &cfg )
  {
    using namespace Thor::LLD::RCC;

    /*-------------------------------------------------
    Configure the clock
    -------------------------------------------------*/
    if ( cfg.clockSource != Chimera::Clock::Bus::PCLK2 )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    // Select the clock source
    ADCSEL::set( RCC1_PERIPH, Config::CCIPR::ADCSEL_SYS_CLOCK );

    // Turn on the core peripheral clock
    this->clockEnable();
    this->reset();

    // Set the clock mode to be asynchronous
    CKMODE::set( ADC_COMMON, 0 );

    // Set the prescaler
    switch ( cfg.clockPrescale )
    {
      case Chimera::ADC::Prescaler::DIV_1:
        PRESC::set( ADC_COMMON, 0 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_2:
        PRESC::set( ADC_COMMON, 1 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_4:
        PRESC::set( ADC_COMMON, 2 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_6:
        PRESC::set( ADC_COMMON, 3 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_8:
        PRESC::set( ADC_COMMON, 4 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_10:
        PRESC::set( ADC_COMMON, 5 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_12:
        PRESC::set( ADC_COMMON, 6 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_16:
        PRESC::set( ADC_COMMON, 7 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_32:
        PRESC::set( ADC_COMMON, 8 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_64:
        PRESC::set( ADC_COMMON, 9 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_128:
        PRESC::set( ADC_COMMON, 10 << CCR_PRESC_Pos );
        break;

      case Chimera::ADC::Prescaler::DIV_256:
        PRESC::set( ADC_COMMON, 11 << CCR_PRESC_Pos );
        break;

      default:
        // Prescaler not supported
        break;
    };

    /*-------------------------------------------------
    Enable the sensor monitors
    -------------------------------------------------*/
    VBATEN::set( ADC_COMMON, CCR_VBATEN );    // External battery sensor
    TSEN::set( ADC_COMMON, CCR_TSEN );        // Internal temperature sensor
    VREFEN::set( ADC_COMMON, CCR_VREFEN );    // Internal bandgap vref sensor

    /*-------------------------------------------------
    Bring the ADC out of deep power down
    -------------------------------------------------*/
    DEEPPWD::clear( mPeriph, CR_DEEPPWD );    // Disable deep power down
    ADVREGEN::set( mPeriph, CR_ADVREGEN );    // Enable the voltage regulator

    // Allow the analog voltage regulator time to boot
    Chimera::delayMilliseconds( 250 );

    /*-------------------------------------------------
    Assign the ADC resolution
    -------------------------------------------------*/
    switch ( cfg.resolution )
    {
      case Chimera::ADC::Resolution::BIT_12:
        RES::set( mPeriph, 0 << CFGR_RES_Pos );
        break;

      case Chimera::ADC::Resolution::BIT_10:
        RES::set( mPeriph, 1 << CFGR_RES_Pos );
        break;

      case Chimera::ADC::Resolution::BIT_8:
        RES::set( mPeriph, 2 << CFGR_RES_Pos );
        break;

      case Chimera::ADC::Resolution::BIT_6:
      default:
        RES::set( mPeriph, 3 << CFGR_RES_Pos );
        break;
    };

    /*-------------------------------------------------
    Calibrate the ADC
    -------------------------------------------------*/
    // Default to single ended inputs for all channels
    DIFSEL::clear( mPeriph, DIFSEL_DIFSEL );

    // Start calibration for single ended inputs
    ADCALDIF::clear( mPeriph, CR_ADCALDIF );
    ADCAL::set( mPeriph, CR_ADCAL );

    // Calibration complete when bit is cleared
    while ( ADCAL::get( mPeriph ) )
    {
      continue;
    }

    /*-------------------------------------------------
    Enable ISRs
    -------------------------------------------------*/
    EOCIE::set( mPeriph, IER_EOCIE );
    JEOCIE::set( mPeriph, IER_JEOCIE );

    /*-------------------------------------------------
    Enable the ADC
    -------------------------------------------------*/
    ADEN::set( mPeriph, CR_ADEN );

    // Wait for ADC to signal it's ready, then ACK it via write 1.
    while ( !ADRDY::get( mPeriph ) )
    {
      continue;
    }
    ADRDY::set( mPeriph, ISR_ADRDY );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::reset()
  {
    /*-------------------------------------------------
    Use the RCC peripheral to invoke the reset. The
    clock must be enabled first or else this won't work.
    -------------------------------------------------*/
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    return rcc->reset( Chimera::Peripheral::Type::PERIPH_ADC, mResourceIndex );
  }


  void Driver::clockEnable()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_ADC, mResourceIndex );
  }


  void Driver::clockDisable()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_ADC, mResourceIndex );
  }


  inline void Driver::enterCriticalSection()
  {
    Thor::LLD::IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }


  inline void Driver::exitCriticalSection()
  {
    Thor::LLD::IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }


  void Driver::IRQHandler()
  {
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
