/******************************************************************************
 *  File Name:
 *    lld_timer_general.cpp
 *
 *  Description:
 *    General timer driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/timer>
#include <Thor/lld/interface/inc/timer>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static GeneralDriver s_timer_driver[ NUM_GENERAL_PERIPHS ];

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initGeneralDriver()
  {
    /*-------------------------------------------------------------------------
    Attach the peripheral instances
    -------------------------------------------------------------------------*/
#if defined( STM32_TIMER2_PERIPH_AVAILABLE )
    s_timer_driver[ TIMER2_TYPE_RESOURCE_INDEX ].attach( TIMER2_PERIPH );
#endif
#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    s_timer_driver[ TIMER3_TYPE_RESOURCE_INDEX ].attach( TIMER3_PERIPH );
#endif
#if defined( STM32_TIMER15_PERIPH_AVAILABLE )
    s_timer_driver[ TIMER15_TYPE_RESOURCE_INDEX ].attach( TIMER15_PERIPH );
#endif
#if defined( STM32_TIMER16_PERIPH_AVAILABLE )
    s_timer_driver[ TIMER16_TYPE_RESOURCE_INDEX ].attach( TIMER16_PERIPH );
#endif
  }


  GeneralDriver_rPtr getGeneralDriver( const RIndex_t typeIndex )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( typeIndex >= NUM_GENERAL_PERIPHS )
    {
      return nullptr;
    }

    /*-------------------------------------------------------------------------
    Grab the driver object
    -------------------------------------------------------------------------*/
    return &s_timer_driver[ typeIndex ];
  }


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  GeneralDriver::GeneralDriver() : ModuleDriver<GeneralDriver>()
  {
  }


  GeneralDriver::~GeneralDriver()
  {
  }

}    // namespace Thor::LLD::TIMER
