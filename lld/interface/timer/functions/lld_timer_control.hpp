/******************************************************************************
 *  File Name:
 *    lld_timer_control.hpp
 *
 *  Description:
 *    Common driver for Advanced, Basic, General timers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_CONTROL_UNIT_DRIVER_HPP
#define THOR_LLD_TIMER_CONTROL_UNIT_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/timer/timer_types.hpp>
#include <Thor/lld/interface/timer/registers/lld_timer_intf_register_bits.hpp>
#include <Thor/lld/interface/timer/registers/lld_timer_intf_register_defs.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------------------------------
  Auto-Reload Buffering: CR1_ARPE
  -------------------------------------------------------------------------*/
  enum class ARBehavior
  {
    AUTO_RELOAD_DIRECT,
    AUTO_RELOAD_BUFFER
  };

  inline void setAutoReloadBehavior( Handle_rPtr timer, const ARBehavior &behavior )
  {
    ARPE::set( timer->registers, EnumValue( behavior ) << CR1_ARPE_Pos );
  }

  /*-------------------------------------------------------------------------
  Alignment Mode: CR1_CMS
  -------------------------------------------------------------------------*/
  enum class AlignMode
  {
    EDGE_ALIGNED,
    CENTER_ALIGNED_1,
    CENTER_ALIGNED_2,
    CENTER_ALIGNED_3
  };

  inline void setAlignment( Handle_rPtr timer, const AlignMode &align )
  {
    CMS::set( timer->registers, EnumValue( align ) << CR1_CMS_Pos );
  }

  /*-------------------------------------------------------------------------
  Count Direction: CR1_DIR
  -------------------------------------------------------------------------*/
  enum class CountDir
  {
    COUNT_UP,
    COUNT_DN
  };

  inline void setCountDirection( Handle_rPtr timer, const CountDir &dir )
  {
    DIR::set( timer->registers, EnumValue( dir ) << CR1_DIR_Pos );
  }

  /*-------------------------------------------------------------------------
  Pulse Mode: CR1_OPM
  -------------------------------------------------------------------------*/
  enum class PulseMode
  {
    CONTINUE_ON_UPDATE_EVENT,
    STOP_ON_UPDATE_EVENT
  };

  inline void setPulseMode( Handle_rPtr timer, const PulseMode &mode )
  {
    OPM::set( timer->registers, EnumValue( mode ) << CR1_OPM_Pos );
  }

  /*-------------------------------------------------------------------------
  Counter Enable/Disable: CR1_CEN
  -------------------------------------------------------------------------*/
  inline void enableCounter( Handle_rPtr timer )
  {
    CEN::set( timer->registers, CR1_CEN );
  }

  inline void disableCounter( Handle_rPtr timer )
  {
    CEN::clear( timer->registers, CR1_CEN );
  }

  /*-------------------------------------------------------------------------
  Master Mode Selection: CR2_MMS
  -------------------------------------------------------------------------*/
  enum class MasterMode
  {
    RESET,
    ENABLE,
    UPDATE,
    COMPARE_PULSE,
    COMPARE_OC1REF,
    COMPARE_OC2REF,
    COMPARE_OC3REF,
    COMPARE_OC4REF
  };

  inline void setMasterMode( Handle_rPtr timer, const MasterMode &mode )
  {
    MMS::set( timer->registers, EnumValue( mode ) << CR2_MMS_Pos );
  }

  enum class MasterMode2
  {
    RESET,
    ENABLE,
    UPDATE,
    COMPARE_PULSE,
    COMPARE_OC1REF,
    COMPARE_OC2REF,
    COMPARE_OC3REF,
    COMPARE_OC4REF,
    COMPARE_OC5REF,
    COMPARE_OC6REF,
    COMPARE_PULSE_OC4REF_RF,
    COMPARE_PULSE_OC6REF_RF,
    COMPARE_PULSE_OC4REF_OR_OC6REF_RISING,
    COMPARE_PULSE_OC4REF_RISE_OR_OC6REF_FALL,
    COMPARE_PULSE_OC5REF_OR_OC6REF_RISING,
    COMPARE_PULSE_OC5REF_RISE_OR_OC6REF_FALL,
  };

  inline void setMasterMode2( Handle_rPtr timer, const MasterMode2 &mode )
  {
    MMS2::set( timer->registers, EnumValue( mode ) << CR2_MMS2_Pos );
  }


  /*---------------------------------------------------------------------------
  Main Output Enable: BDTR
  ---------------------------------------------------------------------------*/
  /**
   * @brief Enables all the timer's output channels
   * @note  Individual channel behavior is dependent on CCxE, CCxNE bits
   *
   * @param timer   Which timer to act on
   */
  inline void enableAllOutput( Handle_rPtr timer )
  {
    MOE::set( timer->registers, BDTR_MOE );
  }

  /**
   * @brief Disables all the timer's output channels
   *
   * @param timer   Which timer to act on
   */
  inline void disableAllOutput( Handle_rPtr timer )
  {
    MOE::clear( timer->registers, BDTR_MOE );
  }

  /*---------------------------------------------------------------------------
  Off State Selection: BDTR
  ---------------------------------------------------------------------------*/
  enum class OffStateMode
  {
    HI_Z,
    TIMER_CONTROL
  };

  inline void setRunModeOffState( Handle_rPtr timer, const OffStateMode mode )
  {
    OSSR::set( timer->registers, EnumValue( mode ) << BDTR_OSSR_Pos );
  }

  inline void setIdleModeOffState( Handle_rPtr timer, const OffStateMode mode )
  {
    OSSI::set( timer->registers, EnumValue( mode ) << BDTR_OSSI_Pos );
  }
}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_CONTROL_UNIT_DRIVER_HPP */
