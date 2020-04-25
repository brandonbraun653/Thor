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

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/lld/interface/timer/timer_types.hpp>

namespace Thor::LLD::TIMER
{
  /**
   *  Initializes the low level driver
   */
  extern Chimera::Status_t initialize();

  extern void incrementSystemTick();
  extern size_t millis();
  extern void delayMilliseconds( const size_t ms );
  extern void delayMicroseconds( const size_t us );

  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  bool isChannelSupported( const Chimera::Timer::Channel channel );

  Type getTimerType( const size_t channel );

  IAdvancedDriver_sPtr getAdvancedDriver( const size_t channel );

  IBasicDriver_sPtr getBasicDriver( const size_t channel );

  IGeneralDriver_sPtr getGeneralDriver( const size_t channel );

  ILowPowerDriver_sPtr getLowPowerDriver( const size_t channel );


  class ICommonDriver
  {
  public:
    virtual ~ICommonDriver() = default;

    /**
     *  Resets the hardware registers back to boot-up values
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t reset() = 0;

    /**
     *  Enables the peripheral clock
     *
     *  @return void
     */
    virtual void clockEnable() = 0;

    /**
     *  Disables the peripheral clock
     *
     *  @return void
     */
    virtual void clockDisable() = 0;



    virtual void enableChannel( const size_t channel ) = 0;

    virtual void disableChannel( const size_t channel ) = 0;

    virtual bool hasFunctionality( const Functionality func ) = 0;

    virtual bool isType( const Type type ) = 0;
  };

  class IAdvancedDriver : public virtual ICommonDriver
  {
  public:
    virtual ~IAdvancedDriver() = default;

    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired peripheral
     *  @return void
     */
    virtual Chimera::Status_t attach( RegisterMap *const peripheral ) = 0;
  };

  class IBasicDriver : public virtual ICommonDriver
  {
  public:
    virtual ~IBasicDriver() = default;

    
    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired peripheral
     *  @return void
     */
    virtual Chimera::Status_t attach( RegisterMap *const peripheral ) = 0;
  };

  class IGeneralDriver : public virtual ICommonDriver
  {
  public:
    virtual ~IGeneralDriver() = default;
    
    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired peripheral
     *  @return void
     */
    virtual Chimera::Status_t attach( RegisterMap *const peripheral ) = 0;
  };

  class ILowPowerDriver : public virtual ICommonDriver
  {
  public:
    virtual ~ILowPowerDriver() = default;
    
    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired peripheral
     *  @return void
     */
    virtual Chimera::Status_t attach( LPRegisterMap *const peripheral ) = 0;
  };
}    // namespace Thor::LLD::Timer

#endif  /* !LLD_TIMER_INTERFACE_HPP */
