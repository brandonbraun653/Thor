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
#include <Thor/lld/interface/rcc/rcc_intf.hpp>

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /**
   * @brief Shared functionality among the General/Basic/Advanced drivers
   *
   * Supplies methods for interacting with the Control Registers and their
   * configuration.
   */
  template<class Derived>
  class ControlUnit
  {
  public:
    ControlUnit()
    {
    }

    ~ControlUnit()
    {
    }

    /*-------------------------------------------------------------------------
    Auto-Reload Buffering: CR1_ARPE
    -------------------------------------------------------------------------*/
    enum class ARBehavior
    {
      AUTO_RELOAD_DIRECT,
      AUTO_RELOAD_BUFFER
    };

    inline void setAutoReloadBehavior( const ARBehavior &behavior )
    {
      ARPE::set( static_cast<Derived *>( this )->mPeriph, EnumValue( behavior ) << CR1_ARPE_Pos );
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

    inline void setAlignment( const AlignMode &align )
    {
      CMS::set( static_cast<Derived *>( this )->mPeriph, EnumValue( align ) << CR1_CMS_Pos );
    }

    /*-------------------------------------------------------------------------
    Count Direction: CR1_DIR
    -------------------------------------------------------------------------*/
    enum class CountDir
    {
      COUNT_UP,
      COUNT_DN
    };

    inline void setCountDirection( const CountDir &dir )
    {
      DIR::set( static_cast<Derived *>( this )->mPeriph, EnumValue( dir ) << CR1_DIR_Pos );
    }

    /*-------------------------------------------------------------------------
    Pulse Mode: CR1_OPM
    -------------------------------------------------------------------------*/
    enum class PulseMode
    {
      CONTINUE_ON_UPDATE_EVENT,
      STOP_ON_UPDATE_EVENT
    };

    inline void setPulseMode( const PulseMode &mode )
    {
      OPM::set( static_cast<Derived *>( this )->mPeriph, EnumValue( mode ) << CR1_OPM_Pos );
    }

    /*-------------------------------------------------------------------------
    Counter Enable/Disable: CR1_CEN
    -------------------------------------------------------------------------*/
    inline void enableCounter()
    {
      CEN::set( static_cast<Derived *>( this )->mPeriph, CR1_CEN );
    }

    inline void disableCounter()
    {
      CEN::clear( static_cast<Derived *>( this )->mPeriph, CR1_CEN );
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

    inline void setMasterMode( const MasterMode &mode )
    {
      MMS::set( static_cast<Derived *>( this )->mPeriph, EnumValue( mode ) << CR2_MMS );
    }
  };

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_CONTROL_UNIT_DRIVER_HPP */
