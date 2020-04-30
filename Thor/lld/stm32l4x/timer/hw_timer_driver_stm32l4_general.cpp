/********************************************************************************
 *  File Name:
 *    hw_timer_driver_stm32l4_general.cpp
 *
 *  Description:
 *    LLD General Timer Driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/timer>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/timer/hw_timer_driver.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_mapping.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_prj.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>

namespace Thor::LLD::TIMER
{
  static std::array<IGeneralDriver_sPtr, NUM_GENERAL_PERIPHS> s_general_drivers;

  /*-------------------------------------------------------------------------------
  LLD Public Free Functions
  -------------------------------------------------------------------------------*/
  IGeneralDriver_sPtr getGeneralDriver( const size_t channel )
  {
    return s_general_drivers[ channel ];
  }

  /*-------------------------------------------------------------------------------
  General Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  GeneralDriverImpl::GeneralDriverImpl() : periph( nullptr )
  {
  }

  GeneralDriverImpl::~GeneralDriverImpl()
  {
  }

  Chimera::Status_t GeneralDriverImpl::attach( RegisterMap *const peripheral )
  {
    return Chimera::CommonStatusCodes::OK;
  }


}  // namespace Thor::LLD::TIMER
