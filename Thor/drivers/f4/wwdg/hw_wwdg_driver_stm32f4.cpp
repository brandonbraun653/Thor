/********************************************************************************
 *   File Name:
 *    hw_wwdg_driver_stm32f4.cpp
 *
 *   Description:
 *    Window watchdog driver for the STM32F4 series chips
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cmath>

/* Chimera Includes */

/* Driver Includes */
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_driver.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_mapping.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_prj.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_types.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WWDG == 1 )

namespace Thor::Driver::WWDG
{
  static constexpr float hwCounterMax = static_cast<float>( CR_T_MAX );
  static constexpr float hwCounterMin = static_cast<float>( CR_T_MIN );
  static constexpr float hwCounterRng = hwCounterMax - hwCounterMin;

  /**
   *  The numerical value associated with a register prescaler configuration option
   */
  static const std::vector<uint32_t> prescalerActVals{ 1, 2, 4, 8 };

  /**
   *  Hardware register configuration options
   */
  static const std::vector<uint32_t> prescalerRegVals{ CFR::CLK_DIV_1, CFR::CLK_DIV_2, CFR::CLK_DIV_4, CFR::CLK_DIV_8 };


  Driver::Driver( RegisterMap *const periph ) : periph( periph ), reloadValue( 0u )
  {
    resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;
    enableClock();
  }

  Driver::~Driver()
  {
  }

  uint32_t Driver::calculatePrescaler( const size_t ms )
  {
    /*------------------------------------------------
    Initialize algorithm variables
    ------------------------------------------------*/
    const float pclk1div          = static_cast<float>( CFR::PCLK_1_DIV );
    const float desiredTimeout_mS = static_cast<float>( ms );

    size_t pclk1FreqHz = 0u;
    Thor::Driver::RCC::prjGetPCLK1Freq( &pclk1FreqHz );

    float pclk1          = static_cast<float>( pclk1FreqHz );
    float clockPeriod_mS = 0.0f;
    float maxTimeout_mS  = 0.0f;
    float minTimeout_mS  = 0.0f;
    uint32_t prescaler   = CFR::CLK_DIV_MAX;


    for ( uint8_t i = 0; i < prescalerActVals.size(); i++ )
    {
      /*------------------------------------------------
      This is part of the equation found in the datasheet
      ------------------------------------------------*/
      clockPeriod_mS = ( 1000.0f / pclk1 ) * pclk1div * static_cast<float>( prescalerActVals[ i ] );
      maxTimeout_mS  = clockPeriod_mS * hwCounterMax;
      minTimeout_mS  = clockPeriod_mS * hwCounterMin;

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
    int index     = pair.second;
    bool validity = pair.first;

    if ( !validity )
    {
      return hwCounterMax;
    }

    /*------------------------------------------------
    Initialize algorithm variables
    ------------------------------------------------*/
    size_t pclk1FreqHz = 0u;
    Thor::Driver::RCC::prjGetPCLK1Freq( &pclk1FreqHz );

    const float pclk1div          = static_cast<float>( CFR::PCLK_1_DIV );
    const float pclk1 = static_cast<float>( pclk1FreqHz );

    float lowestError    = std::numeric_limits<float>::max();
    float calcTimeout_mS = std::numeric_limits<float>::max();
    float absError       = std::numeric_limits<float>::max();
    float clockPeriod_mS = ( 1000.0f / pclk1 ) * pclk1div * prescalerActVals[ index ];
    uint32_t reloadVal   = hwCounterMax;

    /*------------------------------------------------
    Iterate through all counter values to figure out which one
    produces the closest timeout
    ------------------------------------------------*/
    for ( uint32_t testVal = hwCounterMin; testVal <= hwCounterMax; testVal++ )
    {
      calcTimeout_mS = clockPeriod_mS * static_cast<float>( testVal ) + 1.0f;
      absError       = fabs( ms - calcTimeout_mS );

      if ( absError < lowestError )
      {
        lowestError = absError;
        reloadVal   = testVal;
      }
    }

    return reloadVal;
  }

  uint32_t Driver::calculateWindow( const size_t ms, const uint8_t percent, const uint32_t prescaler )
  {
    /*------------------------------------------------
    Find the index of the prescaler register value, which
    correlates to the actual prescaler numerical divisor.
    ------------------------------------------------*/
    auto pair     = Chimera::Utilities::findInVector( prescalerRegVals, prescaler );
    bool validity = pair.first;

    if ( !validity )
    {
      return hwCounterMax;
    }

    /*------------------------------------------------
    The window is set independent of the reload value, so
    calculate a simple percentage from the counter range.
    ------------------------------------------------*/
    const float percentage = static_cast<float>( percent ) / 100.0f;
    const float percentMagnitude = hwCounterRng * percentage;
    const float windowValue      = hwCounterMin + percentMagnitude;

    return static_cast<uint32_t>( windowValue );
  }

  Chimera::Status_t Driver::setPrescaler( const uint32_t val )
  {
    CFR::WDGTB::set( periph, val );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::setReload( const uint32_t val )
  {
    reloadValue = val;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::setWindow( const uint32_t val )
  {
    CFR::W::set( periph, val );
    return Chimera::CommonStatusCodes::OK;
  }

  void Driver::start()
  {
    CR::WDGA::set( periph, CR_WDGA );
  }

  void Driver::reload()
  {
    CR::T::set( periph, reloadValue );
  }

  void Driver::enableClock()
  {
    auto rcc = Thor::Driver::RCC::PeripheralController::get();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_WWDG, resourceIndex );
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
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
