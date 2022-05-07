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
#include <Chimera/peripheral>
#include <Chimera/timer>
#include <Thor/lld/interface/inc/timer>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::DeviceManager<GeneralDriver, Chimera::Timer::Instance, NUM_GENERAL_PERIPHS> s_gen_drivers;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  GeneralDriver_rPtr getGeneralDriver( const Chimera::Timer::Instance instance )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( instance >= Chimera::Timer::Instance::NUM_OPTIONS )
    {
      return nullptr;
    }

    /*-------------------------------------------------------------------------
    Grab the driver object and attach the memory mapped registers
    -------------------------------------------------------------------------*/
    RegisterMap       *mmap_reg = PeriphRegisterBlock[ EnumValue( instance ) ];
    GeneralDriver_rPtr driver   = s_gen_drivers.getOrCreate( instance );

    if( driver )
    {
      driver->attach( mmap_reg );
    }

    return driver;
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
