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

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/power>

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
}    // namespace Thor::LLD::PWR

#endif /* TARGET_STM32F4 && THOR_DRIVER_PWR */
