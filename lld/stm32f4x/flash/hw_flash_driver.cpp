/********************************************************************************
 *  File Name:
 *    hw_flash_driver.cpp
 *
 *  Description:
 *    Implements the Flash interface layer
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/interface/inc/flash>
#include <Thor/lld/interface/inc/power>
#include <Thor/lld/interface/inc/rcc>

namespace Thor::LLD::FLASH
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t MIN_LATENCY = 0;
  static constexpr size_t MAX_LATENCY = 15;

  /* clang-format off */
  // Voltage range 2.7V-3.6V
  static const WaitStateDesc Range1[] = {
    { .minClock = 0,    .maxClock = 30,   .waitState = 0 },
    { .minClock = 30,   .maxClock = 60,   .waitState = 1 },
    { .minClock = 60,   .maxClock = 90,   .waitState = 2 },
    { .minClock = 90,   .maxClock = 120,  .waitState = 3 },
    { .minClock = 120,  .maxClock = 150,  .waitState = 4 },
    { .minClock = 150,  .maxClock = 180,  .waitState = 5 },
  };

  // Voltage range 2.4V-2.7V
  static const WaitStateDesc Range2[] = {
    { .minClock = 0,    .maxClock = 24,   .waitState = 0 },
    { .minClock = 24,   .maxClock = 48,   .waitState = 1 },
    { .minClock = 48,   .maxClock = 72,   .waitState = 2 },
    { .minClock = 72,   .maxClock = 96,   .waitState = 3 },
    { .minClock = 96,   .maxClock = 120,  .waitState = 4 },
    { .minClock = 120,  .maxClock = 144,  .waitState = 5 },
    { .minClock = 144,  .maxClock = 168,  .waitState = 6 },
    { .minClock = 168,  .maxClock = 180,  .waitState = 7 },
  };

  // Voltage range 2.1V-2.4V
  static const WaitStateDesc Range3[] = {
    { .minClock = 0,    .maxClock = 22,   .waitState = 0 },
    { .minClock = 22,   .maxClock = 44,   .waitState = 1 },
    { .minClock = 44,   .maxClock = 66,   .waitState = 2 },
    { .minClock = 66,   .maxClock = 88,   .waitState = 3 },
    { .minClock = 88,   .maxClock = 110,  .waitState = 4 },
    { .minClock = 110,  .maxClock = 132,  .waitState = 5 },
    { .minClock = 132,  .maxClock = 154,  .waitState = 6 },
    { .minClock = 154,  .maxClock = 176,  .waitState = 7 },
    { .minClock = 176,  .maxClock = 180,  .waitState = 8 },
  };

  // Voltage range 1.8V-2.1V
  static const WaitStateDesc Range4[] = {
    { .minClock = 0,    .maxClock = 20,   .waitState = 0 },
    { .minClock = 20,   .maxClock = 40,   .waitState = 1 },
    { .minClock = 40,   .maxClock = 60,   .waitState = 2 },
    { .minClock = 60,   .maxClock = 80,   .waitState = 3 },
    { .minClock = 80,   .maxClock = 100,  .waitState = 4 },
    { .minClock = 100,  .maxClock = 120,  .waitState = 5 },
    { .minClock = 120,  .maxClock = 140,  .waitState = 6 },
    { .minClock = 140,  .maxClock = 160,  .waitState = 7 },
    { .minClock = 160,  .maxClock = 168,  .waitState = 8 },
  };
  /* clang-format on */

  static const WaitStateDesc *RangeTable[] = { Range1, Range2, Range3, Range4 };

  /*-------------------------------------------------------------------------------
  Static Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Calculates the optimal wait states given the current system
   *  configuration. This is validating section 3.4.1 of RM0390.
   *
   *  @return size_t
   */
  static size_t optimal_wait_state()
  {
    /*-------------------------------------------------
    Determine the current voltage scale range
    -------------------------------------------------*/
    size_t rangeIdx = 0;
    size_t rangeOpt = 0;
    switch ( PWR::getVoltageScale() )
    {
      case PWR::VoltageScale::SCALE_1:
        rangeIdx = 0;
        rangeOpt = ARRAY_COUNT( Range1 );
        break;

      case PWR::VoltageScale::SCALE_2:
        rangeIdx = 1;
        rangeOpt = ARRAY_COUNT( Range2 );
        break;

      case PWR::VoltageScale::SCALE_3:
        rangeIdx = 2;
        rangeOpt = ARRAY_COUNT( Range3 );
        break;

      default:
        rangeIdx = 3;
        rangeOpt = ARRAY_COUNT( Range4 );
        break;
    };

    /*-------------------------------------------------
    Get the current system clock frequency in MHz
    -------------------------------------------------*/
    size_t clk = RCC::getCoreClockCtrl()->getClockFrequency( Chimera::Clock::Bus::SYSCLK );
    clk /= 1000000;

    /*-------------------------------------------------
    Find the best wait state
    -------------------------------------------------*/
    auto table = RangeTable[ rangeIdx ];
    for ( size_t idx = 0; idx < rangeOpt; idx++ )
    {
      if ( ( table[ idx ].minClock < clk ) && ( clk <= table[ idx ].maxClock ) )
      {
        return table[ idx ].waitState;
      }
    }

    /*-------------------------------------------------
    Reaching here is a problem
    -------------------------------------------------*/
    Chimera::insert_debug_breakpoint();
    return MAX_LATENCY;
  }


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
    if ( address == FLASH_BASE_ADDR )
    {
      return FLASH_RESOURCE_INDEX;
    }
    else
    {
      return INVALID_RESOURCE_INDEX;
    }
  }

  Chimera::Status_t setLatency( const size_t waitStates )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if ( ( waitStates > MAX_LATENCY ) && ( waitStates != LATENCY_AUTO_DETECT ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    size_t latency = waitStates;
    if ( waitStates == LATENCY_AUTO_DETECT )
    {
      latency = optimal_wait_state();
    }

    /*------------------------------------------------
    Validate latency configuration since this is such
    a critical operating parameter.
    ------------------------------------------------*/
    uint32_t regVal = latency << ACR_LATENCY_Pos;
    LATENCY::set( FLASH_PERIPH, regVal );
    while ( LATENCY::get( FLASH_PERIPH ) != regVal )
    {
      continue;
    }

    return Chimera::Status::OK;
  }
}    // namespace Thor::LLD::FLASH
