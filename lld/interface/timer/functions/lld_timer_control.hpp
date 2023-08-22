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
  Counter Enable/Disable/Assign: CR1_CEN
  -------------------------------------------------------------------------*/
  inline void enableCounter( Handle_rPtr timer )
  {
    CEN::set( timer->registers, CR1_CEN );
  }

  inline void disableCounter( Handle_rPtr timer )
  {
    CEN::clear( timer->registers, CR1_CEN );
  }

  inline void assignCounter( Handle_rPtr timer, uint32_t value )
  {
    COUNT::set( timer->registers, value );
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
  Master/Slave Synchronization: SMCR_MSM
  ---------------------------------------------------------------------------*/
  enum class MasterSlaveSync
  {
    DISABLED,
    ENABLED
  };

  inline void setMasterSlaveSync( Handle_rPtr timer, const MasterSlaveSync &mode )
  {
    MSM::set( timer->registers, EnumValue( mode ) << SMCR_MSM_Pos );
  }

  /*---------------------------------------------------------------------------
  Trigger Selection: SMCR_TS
  ---------------------------------------------------------------------------*/
  enum class Trigger
  {
    INTERNAL_0,
    INTERNAL_1,
    INTERNAL_2,
    INTERNAL_3,
    TI1_EDGE_DETECT,
    FILTERED_TI1,
    FILTERED_TI2,
    EXTERNAL_TI1
  };

  inline void setSlaveTriggerSource( Handle_rPtr timer, const Trigger &trigger )
  {
    TS::set( timer->registers, EnumValue( trigger ) << SMCR_TS_Pos );
  }

  /*---------------------------------------------------------------------------
  Slave Mode Selection: SMCR_SMS
  ---------------------------------------------------------------------------*/
  enum class SlaveMode
  {
    DISABLED,
    ENCODER_1,
    ENCODER_2,
    ENCODER_3,
    RESET,
    GATED,
    TRIGGER,
    EXTERNAL_CLOCK_MODE_1
  };

  inline void setSlaveMode( Handle_rPtr timer, const SlaveMode &mode )
  {
    SMS::set( timer->registers, EnumValue( mode ) << SMCR_SMS_Pos );
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
