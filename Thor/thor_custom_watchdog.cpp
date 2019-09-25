/********************************************************************************
 *   File Name:
 *       thor_watchdog.cpp
 *
 *   Description:
 *       Implementation of the hardware watchdog interface
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C/C++ Includes */
#include <array>
#include <cmath>
#include <cassert>
#include <limits>
#include <memory>

/* Thor Includes */
#include <Thor/watchdog.hpp>

namespace Thor::Watchdog
{
    Window::Window()
    {
    }

    Chimera::Status_t Window::initialize( const uint32_t timeout_mS, const uint8_t windowPercent )
    {
//      std::array<float, numPrescalers> error;
//      std::array<uint32_t, numPrescalers> counter;
//      std::array<uint32_t, numPrescalers> prescalerRegVals = { WWDG_PRESCALER_1, WWDG_PRESCALER_2, WWDG_PRESCALER_4,
//                                                                WWDG_PRESCALER_8 };
//      std::array<uint32_t, numPrescalers> prescalerActVals = { 1u, 2u, 4u, 8u };
//
//      /*------------------------------------------------
//      Initialize the algorithm variables
//      ------------------------------------------------*/
//      counter.fill( 0u );
//      error.fill( std::numeric_limits<float>::max() );
//
//      uint32_t wdWindow    = counterMax;
//      uint32_t wdCounter   = counterMax;
//      uint32_t wdPrescaler = WWDG_PRESCALER_8;
//
//      /*------------------------------------------------
//      Calculates the best performing values for each prescaler
//      ------------------------------------------------*/
//      float pclk1 = static_cast<float>( HAL_RCC_GetPCLK1Freq() );
//
//      for ( uint8_t i = 0; i < prescalerActVals.size(); i++ )
//      {
//        /*------------------------------------------------
//        This is part of the equation found in the datasheet
//        ------------------------------------------------*/
//        float clockPeriod_mS = ( 1000.0f / pclk1 ) * 4096.0f * ( 1u << i );
//
//        /*------------------------------------------------
//        Iterate through all counter values to figure out which one
//        produces the closest timeout
//        ------------------------------------------------*/
//        for ( uint8_t testVal = counterMin; testVal <= counterMax; testVal++ )
//        {
//          float calcTimeout_mS = clockPeriod_mS * ( ( testVal & counterMask ) + 1 );
//          float absError       = fabs( timeout_mS - calcTimeout_mS );
//
//          if ( absError < error[ i ] )
//          {
//            error[ i ]   = absError;
//            counter[ i ] = testVal;
//          }
//        }
//      }
//
//      /*------------------------------------------------
//      Use the best performing prescaler and counter vars
//      ------------------------------------------------*/
//      uint8_t bestIdx = 0;
//      float min_err   = std::numeric_limits<float>::max();
//
//      for ( uint8_t i = 0; i < prescalerActVals.size(); i++ )
//      {
//        if ( error[ i ] < min_err )
//        {
//          min_err = error[ i ];
//          bestIdx = i;
//        }
//      }
//
//      wdCounter        = counter[ bestIdx ];
//      wdPrescaler      = prescalerRegVals[ bestIdx ];
//      actualTimeout_mS = calculateTimeout_mS( static_cast<uint32_t>( pclk1 ), bestIdx, wdCounter );
//
//      /*------------------------------------------------
//      Calculate the window in which the watchdog is allowed to be kicked
//      ------------------------------------------------*/
//      auto userPercent = static_cast<float>( std::min<uint8_t>( 100, windowPercent ) ) / 100.0f;
//      auto minOffset   = static_cast<float>( counterRange ) * userPercent;
//      wdWindow         = static_cast<uint32_t>( minOffset ) + counterMin;
//
//      /*------------------------------------------------
//      Initialize the device handle
//      ------------------------------------------------*/
//      handle.Instance       = WWDG;        /** The only instance of WWDG */
//      handle.Init.Prescaler = wdPrescaler; /** Clock counter prescaler */
//      handle.Init.Window    = wdWindow;  /** The counter can only be refreshed if below this value. [Max: 0x7F, Min: 0x40] */
//      handle.Init.Counter   = wdCounter; /** Starting counter value, also the refresh value. [Max: 0x7F, Min: 0x40] */
//      handle.Init.EWIMode   = WWDG_EWI_DISABLE; /** Enable or disable the Early Wakeup Interrupt */
//
//      /*------------------------------------------------
//      Turn on the peripheral clock
//      ------------------------------------------------*/
//      __HAL_RCC_WWDG_CLK_ENABLE();

      return Chimera::CommonStatusCodes::OK;
    }

    Chimera::Status_t Window::start()
    {
      auto status = Chimera::CommonStatusCodes::OK;

      return status;
    }

    Chimera::Status_t Window::stop()
    {
      /*------------------------------------------------
      Once enabled, the watchdog cannot be stopped except by a system reset
      ------------------------------------------------*/
      return Chimera::CommonStatusCodes::LOCKED;
    }

    Chimera::Status_t Window::kick()
    {
      auto status = Chimera::CommonStatusCodes::OK;

      return status;
    }

    size_t Window::getTimeout()
    {
      return 0;
    }

    Chimera::Status_t Window::pauseOnDebugHalt( const bool enable )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

//    uint32_t Window::calculateTimeout_mS( const uint32_t pclk1, const uint8_t prescaler, const uint8_t counter )
//    {
//      assert( pclk1 != 0 );
//      assert( prescaler < numPrescalers );
//      assert( counter < ( counterMax + 1 ) );
//      assert( counter > ( counterMin - 1 ) );
//
//      /*------------------------------------------------
//      Out of sheer laziness, I'm not documenting this function's equation. Look in the initialization function
//      or the device datasheet for a more useful explanation of what is going on.
//      ------------------------------------------------*/
//      float calculatedTimeout = ( 1000.0f / pclk1 ) * 4096.0f * ( 1u << prescaler ) * ( ( counter & counterMask ) + 1 );
//      return static_cast<uint32_t>( calculatedTimeout );
//    }
//
    Independent::Independent() : currentPrescaler(0u)
    {
      hwDriver = std::make_unique<Thor::Driver::IWDG::Driver>( Thor::Driver::IWDG::IWDG_PERIPH );
    }

    Independent::~Independent()
    {
    }

    Chimera::Status_t Independent::initialize( const uint32_t timeout_mS, const uint8_t windowPercent )
    {
      uint32_t prescalerRegVal = hwDriver->calculatePrescaler( timeout_mS );
      uint32_t reloadRegVal    = hwDriver->calculateReload( timeout_mS, prescalerRegVal );

      hwDriver->start();

      if ( ( hwDriver->setPrescaler( prescalerRegVal ) != Chimera::CommonStatusCodes::OK ) ||
           ( hwDriver->setReload( reloadRegVal ) != Chimera::CommonStatusCodes::OK ) )
      {
        return Chimera::CommonStatusCodes::FAIL;
      }

      return Chimera::CommonStatusCodes::OK;
    }

    Chimera::Status_t Independent::start()
    {
      hwDriver->start();
      return Chimera::CommonStatusCodes::OK;
    }

    Chimera::Status_t Independent::stop()
    {
      /*------------------------------------------------
      Once enabled, the watchdog cannot be stopped except by a system reset
      ------------------------------------------------*/
      return Chimera::CommonStatusCodes::LOCKED;
    }

    Chimera::Status_t Independent::kick()
    {
      hwDriver->reload();
      return Chimera::CommonStatusCodes::OK;
    }

    size_t Independent::getTimeout()
    {
      return 0;
    }

    size_t Independent::maxTimeout()
    {
      return hwDriver->maxDelay( Thor::Driver::IWDG::PR::PRESCALE_MAX );
    }

    size_t Independent::minTimeout()
    {
      return hwDriver->minDelay( Thor::Driver::IWDG::PR::PRESCALE_MIN );
    }

    Chimera::Status_t Independent::pauseOnDebugHalt( const bool enable )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }


}    // namespace Thor
