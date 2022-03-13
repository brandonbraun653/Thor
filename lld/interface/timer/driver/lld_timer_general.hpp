/******************************************************************************
 *  File Name:
 *    lld_timer_general.hpp
 *
 *  Description:
 *    General timer driver interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_GENERAL_HPP
#define THOR_LLD_TIMER_GENERAL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/timer/timer_intf.hpp>
#include <Thor/lld/interface/timer/timer_types.hpp>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initialize the general driver module
   */
  void initGeneralDriver();


  /**
   * @brief Get the General Driver object
   *
   * @param channel
   * @return GeneralDriver_rPtr
   */
  GeneralDriver_rPtr getGeneralDriver( const size_t channel );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class GeneralDriver : public CommonHWDriver
  {
  public:

  protected:

  private:

  };
}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_TIMER_GENERAL_HPP */
