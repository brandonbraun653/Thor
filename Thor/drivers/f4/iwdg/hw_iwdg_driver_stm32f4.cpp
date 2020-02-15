/********************************************************************************
 *   File Name:
 *    hw_iwdg_driver_stm32f4.cpp
 *
 *   Description:
 *    Independent watchdog driver for the STM32F4 series family
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cmath>

/* Chimera Includes */
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/drivers/f4/iwdg/hw_iwdg_driver.hpp>
#include <Thor/drivers/f4/iwdg/hw_iwdg_mapping.hpp>
#include <Thor/drivers/f4/iwdg/hw_iwdg_prj.hpp>
#include <Thor/drivers/f4/iwdg/hw_iwdg_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_IWDG == 1 )

namespace Thor::Driver::IWDG
{
  static constexpr float hwCounterMax = static_cast<float>( RLR_MAX ); /**< Max value the hardware counter can store */
  static constexpr float hwCounterMin = static_cast<float>( RLR_MIN ); /**< Value the hardware counter trips at */
  static constexpr float hwClockFreqHz =
      static_cast<float>( PERIPH_CLOCK_FREQ_HZ ); /**< Clock frequency driving the watchdog */

  /**
   *  The numerical value associated with a register prescaler configuration option
   */
  static const std::vector<uint32_t> prescalerActVals{ 4, 8, 16, 32, 64, 128, 256 };

  /**
   *  Hardware register configuration options
   */
  static const std::vector<uint32_t> prescalerRegVals{ PR::PRESCALE_4,  PR::PRESCALE_8,   PR::PRESCALE_16, PR::PRESCALE_32,
                                                       PR::PRESCALE_64, PR::PRESCALE_128, PR::PRESCALE_256 };


  void initialize()
  {
    initializeRegisters();
    initializeMapping();
  }

  Driver::Driver( RegisterMap *const periph ) : periph( periph )
  {
    resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;
  }

  Driver::~Driver()
  {
  }

  uint32_t Driver::calculatePrescaler( const size_t ms )
  {
    /*------------------------------------------------
    Initialize algorithm variables
    ------------------------------------------------*/
    float clockPeriod_mS = 0.0f;
    float maxTimeout_mS  = 0.0f;
    float minTimeout_mS  = 0.0f;
    float desiredTimeout_mS = static_cast<float>( ms );
    uint32_t prescaler   = PR::PRESCALE_256;

    /*------------------------------------------------
    The desired prescaler is found when the max watchdog
    expiration period drops below the desired timeout.
    ------------------------------------------------*/
    for ( uint8_t i = 0; i < prescalerActVals.size(); i++ )
    {
      clockPeriod_mS = ( 1000.0f / hwClockFreqHz ) * static_cast<float>( prescalerActVals[ i ] );
      maxTimeout_mS  = clockPeriod_mS * hwCounterMax;
      minTimeout_mS  = clockPeriod_mS * hwCounterMin + 1.0f;

      if ( ( minTimeout_mS < desiredTimeout_mS ) || ( desiredTimeout_mS < maxTimeout_mS ) )
      {
        prescaler = prescalerRegVals[ i ];
        break;
      }
    }

    return prescaler;
  }

  uint32_t Driver::calculateReload( const size_t ms, const uint32_t prescaler )
  {
    /*------------------------------------------------
    Find the index of the prescaler register value, which
    correlates to the actual prescaler numerical divisor.
    ------------------------------------------------*/
    auto pair     = Chimera::Utilities::findInVector( prescalerRegVals, prescaler );
    size_t index  = pair.second;
    bool validity = pair.first;

    if ( !validity )
    {
      return static_cast<uint32_t>( hwCounterMax );
    }

    /*------------------------------------------------
    Initialize algorithm variables
    ------------------------------------------------*/
    float lowestError    = std::numeric_limits<float>::max();
    float calcTimeout_mS = std::numeric_limits<float>::max();
    float absError       = std::numeric_limits<float>::max();
    float clockPeriod_mS = ( 1000.0f / hwClockFreqHz ) * prescalerActVals[ index ];
    uint32_t reloadVal   = static_cast<uint32_t>( hwCounterMax );

    /*------------------------------------------------
    Iterate through all counter values to figure out which one
    produces the closest timeout
    ------------------------------------------------*/
    for ( uint32_t testVal = static_cast<uint32_t>( hwCounterMin ); testVal <= hwCounterMax; testVal++ )
    {
      calcTimeout_mS = clockPeriod_mS * ( static_cast<float>( testVal ) + 1.0f );
      absError       = fabs( ms - calcTimeout_mS );

      if ( absError < lowestError )
      {
        lowestError = absError;
        reloadVal   = testVal;
      }
    }

    return reloadVal;
  }

  Chimera::Status_t Driver::setPrescaler( const uint32_t val )
  {
    /*------------------------------------------------
    Wait for SR to indicate no ongoing HW updates.
    This can take at most 125uS (5 clocks @ 40kHz LSI).
    ------------------------------------------------*/
    while ( SR::PVU::get( periph ) ) {}

    /*------------------------------------------------
    Assign the register value with unlock-assign-lock
    ------------------------------------------------*/
    KR::KEY::set( periph, KR::UnlockSequence );
    PR::set( periph, val );
    KR::KEY::set( periph, KR::LockSequence );

    /*------------------------------------------------
    Wait again...
    ------------------------------------------------*/
    while ( SR::PVU::get( periph ) ) {}

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::setReload( const uint32_t val )
  {
    /*------------------------------------------------
    Wait for SR to indicate no ongoing HW updates.
    This can take at most 125uS (5 clocks @ 40kHz LSI).
    ------------------------------------------------*/
    while ( SR::RVU::get( periph ) ) {}

    /*------------------------------------------------
    Assign the register value with unlock-assign-lock
    ------------------------------------------------*/
    KR::KEY::set( periph, KR::UnlockSequence );
    RLR::set( periph, val );
    KR::KEY::set( periph, KR::LockSequence );

    /*------------------------------------------------
    Wait again...
    ------------------------------------------------*/
    while ( SR::RVU::get( periph ) ) {}

    return Chimera::CommonStatusCodes::OK;
  }

  void Driver::start()
  {
    KR::KEY::set( periph, KR::StartSequence );
  }

  void Driver::reload()
  {
    KR::KEY::set( periph, KR::RefreshSequence );
  }

  void Driver::enableClock()
  {
    /**
     *  Do nothing because the LSI clock is automatically started
     *  when the user calls the start() method.
     */
  }

  size_t Driver::getMaxTimeout( const uint32_t prescaler )
  {
    // TODO
    return 0u;
  }

  size_t Driver::getMinTimeout( const uint32_t prescaler )
  {
    // TODO
    return 0u;
  }

  size_t Driver::getTimeout()
  {
    // TODO
    return 0u;
  }

}    // namespace Thor::Driver::IWDG

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
