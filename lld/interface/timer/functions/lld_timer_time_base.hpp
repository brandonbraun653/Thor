/******************************************************************************
 *  File Name:
 *    lld_timer_time_base.hpp
 *
 *  Description:
 *    Common driver for Advanced, Basic, General timers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_TIME_BASE_DRIVER_HPP
#define THOR_LLD_TIMER_TIME_BASE_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Chimera/common>
#include <Thor/lld/interface/timer/timer_types.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Shared functionality among the General/Basic/Advanced drivers
   */
  template<class Derived>
  class TimeBase
  {
  public:
    TimeBase()
    {
    }

    ~TimeBase()
    {
    }

    /**
     * @brief Sets the core time base for the entire Timer
     * @note The timer is disabled when this function exits
     *
     * @param rollover_period_ms    Period in which the counter rolls over (milliseconds)
     * @return Chimera::Status_t
     */
    Chimera::Status_t configureTimeBase( const float rollover_period_ms )
    {
      /*-------------------------------------------------------------------------
      Local Constants
      -------------------------------------------------------------------------*/
      constexpr HardwareType timerType = static_cast<Derived *>( this )->timerType();

      /*-------------------------------------------------------------------------
      Local Variables
      -------------------------------------------------------------------------*/
      RegisterMap *periph = static_cast<Derived *>( this )->mPeriph;

      /*-------------------------------------------------------------------------
      Calculate configuration values
      -------------------------------------------------------------------------*/
      uint32_t direction     = 0;
      uint32_t reloadValue   = 0;
      uint32_t clockPrescale = 0;

      /*-------------------------------------------------------------------------
      Configure the base timer
      -------------------------------------------------------------------------*/
      static_cast<Derived *>( this )->disableCounter();

      // /* Set the counter direction */
      // if constexpr ( timerType == HardwareType::TIMER_HW_ADVANCED )
      // {
      //   DIR::set( periph, CR1_DIR );
      // }

      // ARPE::set( periph, CR1_ARPE );     /* Buffer the auto reload register updates */
      // ARR::set( periph, reloadValue );   /* Set the reload value */
      // PSC::set( periph, clockPrescale ); /* Adjust the prescaler */
      // CNT::set( periph, reloadValue );   /* Reset counter to indicate a reset */

      return Chimera::Status::OK;
    }
  };

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_TIME_BASE_DRIVER_HPP */
