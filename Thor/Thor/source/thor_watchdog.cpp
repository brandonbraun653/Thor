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
#include <limits>
#include <cmath>
#include <cassert>

/* Thor Includes */
#include <Thor/include/watchdog.hpp>

using namespace Thor::Definitions;

namespace Thor
{
  namespace Peripheral
  {
    namespace Watchdog
    {
#if defined( WWDG )
      WindowWatchdog::WindowWatchdog() : actualTimeout_mS( 0 )
      {
      }

      Status WindowWatchdog::initialize( const uint32_t timeout_mS, const uint8_t windowPercent )
      {
        std::array<float, numPrescalers> error;
        std::array<uint32_t, numPrescalers> counter;
        std::array<uint32_t, numPrescalers> prescalerRegVals = { WWDG_PRESCALER_1, WWDG_PRESCALER_2, WWDG_PRESCALER_4,
                                                                 WWDG_PRESCALER_8 };
        std::array<uint32_t, numPrescalers> prescalerActVals = { 1u, 2u, 4u, 8u };

        /*------------------------------------------------
        Initialize the algorithm variables
        ------------------------------------------------*/
        counter.fill( 0u );
        error.fill( std::numeric_limits<float>::max() );

        uint32_t wdWindow    = counterMax;
        uint32_t wdCounter   = counterMax;
        uint32_t wdPrescaler = WWDG_PRESCALER_8;

        /*------------------------------------------------
        Calculates the best performing values for each prescaler
        ------------------------------------------------*/
        float pclk1 = static_cast<float>( HAL_RCC_GetPCLK1Freq() );

        for ( uint8_t i = 0; i < prescalerActVals.size(); i++ )
        {
          /*------------------------------------------------
          This is part of the equation found in the datasheet
          ------------------------------------------------*/
          float clockPeriod_mS = ( 1000.0f / pclk1 ) * 4096.0f * ( 1u << i );

          /*------------------------------------------------
          Iterate through all counter values to figure out which one
          produces the closest timeout
          ------------------------------------------------*/
          for ( uint8_t testVal = counterMin; testVal <= counterMax; testVal++ )
          {
            float calcTimeout_mS = clockPeriod_mS * ( ( testVal & counterMask ) + 1 );
            float absError       = fabs( timeout_mS - calcTimeout_mS );

            if ( absError < error[ i ] )
            {
              error[ i ]   = absError;
              counter[ i ] = testVal;
            }
          }
        }

        /*------------------------------------------------
        Use the best performing prescaler and counter vars
        ------------------------------------------------*/
        uint8_t bestIdx = 0;
        float min_err   = std::numeric_limits<float>::max();

        for ( uint8_t i = 0; i < prescalerActVals.size(); i++ )
        {
          if ( error[ i ] < min_err )
          {
            min_err = error[ i ];
            bestIdx = i;
          }
        }

        wdCounter        = counter[ bestIdx ];
        wdPrescaler      = prescalerRegVals[ bestIdx ];
        actualTimeout_mS = calculateTimeout_mS( pclk1, bestIdx, wdCounter );

        /*------------------------------------------------
        Calculate the window in which the watchdog is allowed to be kicked
        ------------------------------------------------*/
        auto userPercent = static_cast<float>( std::min<uint8_t>( 100, windowPercent ) ) / 100.0f;
        auto minOffset   = static_cast<float>( counterRange ) * userPercent;
        wdWindow         = static_cast<uint32_t>( minOffset ) + counterMin;

        /*------------------------------------------------
        Initialize the device handle
        ------------------------------------------------*/
        handle.Instance       = WWDG;        /** The only instance of WWDG */
        handle.Init.Prescaler = wdPrescaler; /** Clock counter prescaler */
        handle.Init.Window    = wdWindow;  /** The counter can only be refreshed if below this value. [Max: 0x7F, Min: 0x40] */
        handle.Init.Counter   = wdCounter; /** Starting counter value, also the refresh value. [Max: 0x7F, Min: 0x40] */
        handle.Init.EWIMode   = WWDG_EWI_DISABLE; /** Enable or disable the Early Wakeup Interrupt */

        /*------------------------------------------------
        Turn on the peripheral clock
        ------------------------------------------------*/
        __HAL_RCC_WWDG_CLK_ENABLE();

        return Status::PERIPH_OK;
      }

      Status WindowWatchdog::start()
      {
        auto status = Status::PERIPH_OK;

        if ( HAL_WWDG_Init( &handle ) != HAL_OK )
        {
          status = Status::PERIPH_ERROR;
        }

        return status;
      }

      Status WindowWatchdog::stop()
      {
        /*------------------------------------------------
        Once enabled, the watchdog cannot be stopped except by a system reset
        ------------------------------------------------*/
        return Status::PERIPH_LOCKED;
      }

      Status WindowWatchdog::kick()
      {
        auto status = Status::PERIPH_OK;

        if ( HAL_WWDG_Refresh( &handle ) != HAL_OK )
        {
          status = Status::PERIPH_ERROR;
        }

        return status;
      }

      Status WindowWatchdog::getTimeout( uint32_t &timeout_mS )
      {
        timeout_mS = actualTimeout_mS;
        return Status::PERIPH_OK;
      }

      Status WindowWatchdog::pauseOnDebugHalt( const bool enable )
      {
        if ( enable )
        {
          __HAL_DBGMCU_FREEZE_WWDG();
        }
        else
        {
          __HAL_DBGMCU_UNFREEZE_WWDG();
        }

        return Status::PERIPH_OK;
      }

      uint32_t WindowWatchdog::calculateTimeout_mS( const uint32_t pclk1, const uint8_t prescaler, const uint8_t counter )
      {
        assert( pclk1 != 0 );
        assert( prescaler < numPrescalers );
        assert( counter < ( counterMax + 1 ) );
        assert( counter > ( counterMin - 1 ) );

        /*------------------------------------------------
        Out of sheer laziness, I'm not documenting this function's equation. Look in the initialization function
        or the device datasheet for a more useful explanation of what is going on.
        ------------------------------------------------*/
        float calculatedTimeout = ( 1000.0f / pclk1 ) * 4096.0f * ( 1u << prescaler ) * ( ( counter & counterMask ) + 1 );
        return static_cast<uint32_t>( calculatedTimeout );
      }
#endif /* !WWDG */

#if defined( IWDG )
      IndependentWatchdog::IndependentWatchdog() : actualTimeout_mS( 0 )
      {
      }

      Status IndependentWatchdog::initialize( const uint32_t timeout_mS )
      {
        std::array<float, numPrescalers> error;
        std::array<uint32_t, numPrescalers> counter;
        std::array<uint32_t, numPrescalers> prescalerRegVals = { IWDG_PRESCALER_4,  IWDG_PRESCALER_8,  IWDG_PRESCALER_16,
                                                                 IWDG_PRESCALER_32, IWDG_PRESCALER_64, IWDG_PRESCALER_128,
                                                                 IWDG_PRESCALER_256 };
        std::array<uint32_t, numPrescalers> prescalerActVals = { 4, 8, 16, 32, 64, 128, 256 };

        /*------------------------------------------------
        Initialize the algorithm variables
        ------------------------------------------------*/
        counter.fill( 0u );
        error.fill( std::numeric_limits<float>::max() );

        uint32_t wdReload    = counterMax;
        uint32_t wdPrescaler = IWDG_PRESCALER_256;

        /*------------------------------------------------
        Calculates the best performing values for each prescaler
        ------------------------------------------------*/
        for ( uint8_t i = 0; i < prescalerActVals.size(); i++ )
        {
          /*------------------------------------------------
          This is part of the equation found in the datasheet
          ------------------------------------------------*/
          float clockPeriod_mS = ( 1000.0f / clockFreqHz ) * prescalerActVals[ i ];

          /*------------------------------------------------
          Iterate through all counter values to figure out which one
          produces the closest timeout
          ------------------------------------------------*/
          for ( uint16_t testVal = counterMin; testVal <= counterMax; testVal++ )
          {
            float calcTimeout_mS = clockPeriod_mS * testVal;
            float absError       = fabs( timeout_mS - calcTimeout_mS );

            if ( absError < error[ i ] )
            {
              error[ i ]   = absError;
              counter[ i ] = testVal;
            }
          }
        }

        /*------------------------------------------------
        Use the best performing prescaler and counter vars
        ------------------------------------------------*/
        uint8_t bestIdx = 0;
        float min_err   = std::numeric_limits<float>::max();

        for ( uint8_t i = 0; i < prescalerActVals.size(); i++ )
        {
          if ( error[ i ] < min_err )
          {
            min_err = error[ i ];
            bestIdx = i;
          }
        }

        wdReload         = counter[ bestIdx ];
        wdPrescaler      = prescalerRegVals[ bestIdx ];
        actualTimeout_mS = ( ( 1000.0f / clockFreqHz ) * prescalerActVals[ bestIdx ] ) * wdReload;

        /*------------------------------------------------
        Initialize the device handle, allowing the user to reset whenever
        ------------------------------------------------*/
        handle.Instance       = IWDG;
        handle.Init.Prescaler = wdPrescaler;
        handle.Init.Window    = wdReload;
        handle.Init.Reload    = wdReload;

        return Status::PERIPH_OK;
      }

      Status IndependentWatchdog::start()
      {
        auto status = Status::PERIPH_OK;

        if ( HAL_IWDG_Init( &handle ) != HAL_OK )
        {
          status = Status::PERIPH_ERROR;
        }

        return status;
      }

      Status IndependentWatchdog::stop()
      {
        /*------------------------------------------------
        Once enabled, the watchdog cannot be stopped except by a system reset
        ------------------------------------------------*/
        return Status::PERIPH_LOCKED;
      }

      Status IndependentWatchdog::kick()
      {
        auto status = Status::PERIPH_OK;

        if ( HAL_IWDG_Refresh( &handle ) != HAL_OK )
        {
          status = Status::PERIPH_ERROR;
        }

        return status;
      }

      Status IndependentWatchdog::getTimeout( uint32_t &timeout_mS )
      {
        timeout_mS = actualTimeout_mS;
        return Status::PERIPH_OK;
      }

      Status IndependentWatchdog::pauseOnDebugHalt( const bool enable )
      {
        if ( enable )
        {
          __HAL_DBGMCU_FREEZE_IWDG();
        }
        else
        {
          __HAL_DBGMCU_UNFREEZE_IWDG();
        }

        return Status::PERIPH_OK;
      }
#endif /* !IWDG */

#if defined( USING_CHIMERA )

      Chimera::Watchdog::Status ChimeraWatchdog::initialize( const uint32_t timeout_mS )
      {
        auto result = Chimera::Watchdog::Status::OK;

        if ( watchdog.initialize( timeout_mS ) != Status::PERIPH_OK )
        {
          result = Chimera::Watchdog::Status::FAIL;
        }

        return result;
      }

      Chimera::Watchdog::Status ChimeraWatchdog::start()
      {
        auto result = Chimera::Watchdog::Status::OK;

        if ( watchdog.start() != Status::PERIPH_OK )
        {
          result = Chimera::Watchdog::Status::FAIL;
        }

        return result;
      }

      Chimera::Watchdog::Status ChimeraWatchdog::stop()
      {
        auto result = Chimera::Watchdog::Status::OK;

        if ( watchdog.stop() != Status::PERIPH_OK )
        {
          result = Chimera::Watchdog::Status::FAIL;
        }

        return result;
      }

      Chimera::Watchdog::Status ChimeraWatchdog::kick()
      {
        auto result = Chimera::Watchdog::Status::OK;

        if ( watchdog.kick() != Status::PERIPH_OK )
        {
          result = Chimera::Watchdog::Status::FAIL;
        }

        return result;
      }

      Chimera::Watchdog::Status ChimeraWatchdog::getTimeout( uint32_t &timeout )
      {
        auto result = Chimera::Watchdog::Status::OK;

        if ( watchdog.getTimeout( timeout ) != Status::PERIPH_OK )
        {
          result = Chimera::Watchdog::Status::FAIL;
        }

        return result;
      }

      Chimera::Watchdog::Status ChimeraWatchdog::pauseOnDebugHalt( const bool enable )
      {
        auto result = Chimera::Watchdog::Status::OK;

        if ( watchdog.pauseOnDebugHalt( enable ) != Status::PERIPH_OK )
        {
          result = Chimera::Watchdog::Status::FAIL;
        }

        return result;
      }

#endif

    }    // namespace Watchdog
  }      // namespace Peripheral
}    // namespace Thor
