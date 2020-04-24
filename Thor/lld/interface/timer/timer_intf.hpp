/********************************************************************************
 *  File Name:
 *    timer_intf.hpp
 *
 *  Description:
 *    LLD Timer Interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef LLD_TIMER_INTERFACE_HPP
#define LLD_TIMER_INTERFACE_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/interface/timer/timer_types.hpp>

namespace Thor::LLD::Timer
{
  Type getTimerType( const size_t channel );

  IAdvancedDriver_sPtr getAdvancedDriver( const size_t channel );

  IBasicDriver_sPtr getBasicDriver( const size_t channel );

  IGeneralDriver_sPtr getGeneralDriver( const size_t channel );

  ILowPowerDriver_sPtr getLowPowerDriver( const size_t channel );


  class ICommonDriver
  {
  public:
    virtual ~ICommonDriver() = default;

    virtual void enableChannel( const size_t channel ) = 0;

    virtual void disableChannel( const size_t channel ) = 0;

    virtual bool hasFunctionality( const Functionality func ) = 0;

    virtual bool isType( const Type type ) = 0;
  };

  class IAdvancedDriver : public virtual ICommonDriver
  {
  public:
    virtual ~IAdvancedDriver() = default;
  };

  class IBasicDriver : public virtual ICommonDriver
  {
  public:
    virtual ~IBasicDriver() = default;
  };

  class IGeneralDriver : public virtual ICommonDriver
  {
  public:
    virtual ~IGeneralDriver() = default;
  };

  class ILowPowerDriver : public virtual ICommonDriver
  {
  public:
    virtual ~ILowPowerDriver() = default;
  };
}    // namespace Thor::LLD::Timer

#endif  /* !LLD_TIMER_INTERFACE_HPP */
