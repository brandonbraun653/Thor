/********************************************************************************
 *  File Name:
 *    hw_power_driver_stm32f4.cpp
 *
 *  Description:
 *    Implements the low level driver for the PWR peripheral
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/clock>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/power>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_prv.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_PWR )

namespace Thor::LLD::PWR
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
    if ( address == PWR_BASE_ADDR )
    {
      return PWR_RESOURCE_INDEX;
    }
    else
    {
      return INVALID_RESOURCE_INDEX;
    }
  }


  void clockEnable()
  {
    auto rcc   = Thor::LLD::RCC::getPeriphClockCtrl();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( PWR_PERIPH ) );

    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_PWR, index );
  }


  Chimera::Status_t setVoltageScaling( const VoltageScale scale )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( scale >= VoltageScale::NUM_OPTIONS )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Decide the register setting to apply
    -------------------------------------------------*/
    uint32_t regVal = 0;
    switch ( scale )
    {
      case VoltageScale::SCALE_1:
        regVal = 3u << CR_VOS_Pos;
        break;

      case VoltageScale::SCALE_2:
        regVal = 2u << CR_VOS_Pos;
        break;

      case VoltageScale::SCALE_3:
        regVal = 1u << CR_VOS_Pos;
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
        break;
    };

    /*-------------------------------------------------
    Apply the scaling. This assumes that all the set up
    steps to perform this action have been performed.
    -------------------------------------------------*/
    VOS::set( PWR_PERIPH, regVal );
    while ( VOS::get( PWR_PERIPH ) != regVal )
    {
      continue;
    }

    return Chimera::Status::OK;
  }


  VoltageScale getVoltageScale()
  {
    uint32_t value = VOS::get( PWR_PERIPH ) >> CR_VOS_Pos;

    switch( value )
    {
      case 0:
      case 1:
        return VoltageScale::SCALE_3;
        break;

      case 2:
        return VoltageScale::SCALE_2;
        break;

      case 3:
        return VoltageScale::SCALE_1;
        break;

      default:
        return VoltageScale::INVALID;
        break;
    };
  }


  void setOverdriveMode( const bool state )
  {
    if ( state )
    {
      /*-------------------------------------------------
      Overdrive can only be enabled if the system clock
      is sourced from HSI/HSE. HSI is guaranteed to exist
      -------------------------------------------------*/
      RCC::enableHSI();
      RCC::select_system_clock_source( Chimera::Clock::Bus::HSI16 );

      /*-------------------------------------------------
      Turn on the main internal regulator output voltage
      and set voltage scaling to achieve max clock.
      -------------------------------------------------*/
      setVoltageScaling( VoltageScale::SCALE_1 );

      /*-------------------------------------------------
      Enable the overdrive hardware
      -------------------------------------------------*/
      ODEN::set( PWR_PERIPH, CR_ODEN );
      while( !ODRDY::get( PWR_PERIPH ) )
      {
        continue;
      }

      /*-------------------------------------------------
      Enable the overdrive switching
      -------------------------------------------------*/
      ODSWEN::set( PWR_PERIPH, CR_ODSWEN );
      while( !ODSWRDY::get( PWR_PERIPH ) )
      {
        continue;
      }
    }
    else
    {
      /*-------------------------------------------------
      Simply clear the two overdrive enable bits
      -------------------------------------------------*/
      ODSWEN::clear( PWR_PERIPH, CR_ODSWEN );
      ODEN::clear( PWR_PERIPH, CR_ODEN );
    }
  }
}    // namespace Thor::LLD::PWR

#endif /* TARGET_STM32F4 && THOR_DRIVER_PWR */
