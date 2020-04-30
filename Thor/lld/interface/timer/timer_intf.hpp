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
#include <Chimera/pwm>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/lld/interface/timer/timer_types.hpp>

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------------------------------------
  HLD->LLD Required Free Functions
  -------------------------------------------------------------------------------*/
  extern void incrementSystemTick();
  extern size_t millis();
  extern void delayMilliseconds( const size_t ms );
  extern void delayMicroseconds( const size_t us );

  /*-------------------------------------------------------------------------------
  LLD Free Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   */
  extern Chimera::Status_t initialize();

  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  bool isChannelSupported( const size_t channel );

  IAdvancedDriver_sPtr getAdvancedDriver( const size_t channel );

  IBasicDriver_sPtr getBasicDriver( const size_t channel );

  IGeneralDriver_sPtr getGeneralDriver( const size_t channel );

  ILowPowerDriver_sPtr getLowPowerDriver( const size_t channel );


  /**
   *  Looks up the LLD resource index associated with a particular
   *  timer channel. There can be multiple channels associated with a
   *  single resource index.
   *
   *  @param[in]  channel     The channel number to look up
   *  @return size_t
   */
  size_t getResourceIndexFromChannel( const size_t channel );


  /**
   *  Gets the peripheral description data associated with the
   *  resource index.
   *
   *  @param[in]  resourceIndex     The index to look up
   *  @return const DeviceDescription *
   */
  const DeviceDescription *getPeripheralDescriptor( const size_t resourceIndex );

  /*-------------------------------------------------------------------------------
  Timer Driver Class Interface Declarations
  -------------------------------------------------------------------------------*/
  template<class T>
  class CRTPParent
  {
  public:
    bool moreFunctions()
    {
      return static_cast<T *>( this )->moreFunctions();
    }
  };

  template<class T>
  class CRTPBaseDriver : public CRTPParent<CRTPBaseDriver<T>>
  {
  public:
    Chimera::Status_t initPeripheral( const Chimera::Timer::DriverConfig &cfg )
    {
      return static_cast<T *>( this )->initPeripheral( cfg );
    }

    bool hasFunction( const Chimera::Timer::Function func )
    {
      return static_cast<T *>( this )->hasFunction( func );
    }

    Chimera::Status_t enable( const Chimera::Timer::Channel channel )
    {
      return static_cast<T *>( this )->enable( channel );
    }

    Chimera::Status_t disable( const Chimera::Timer::Channel channel )
    {
      return static_cast<T *>( this )->disable( channel );
    }

    Chimera::Status_t enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type )
    {
      return static_cast<T *>( this )->enableEvent( channel, type );
    }

    Chimera::Status_t disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type )
    {
      return static_cast<T *>( this )->disableEvent( channel, type );
    }
  };

  class VirtualParent
  {
  public:
    virtual ~VirtualParent() = default;

    virtual bool moreFunctions() = 0;
  };

  class VirtualBaseDriver : public virtual VirtualParent
  {
  public:
    virtual ~VirtualBaseDriver()                                                                                      = default;
    virtual Chimera::Status_t initPeripheral( const Chimera::Timer::DriverConfig &cfg )                               = 0;
    virtual bool hasFunction( const Chimera::Timer::Function func )                                                   = 0;
    virtual Chimera::Status_t enable( const Chimera::Timer::Channel channel )                                         = 0;
    virtual Chimera::Status_t disable( const Chimera::Timer::Channel channel )                                        = 0;
    virtual Chimera::Status_t enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type )  = 0;
    virtual Chimera::Status_t disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type ) = 0;
  };

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
    //
    //    virtual bool hasFunctionality( const Functionality func ) = 0;
    //
    //    virtual bool isType( const Type type ) = 0;
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

#if defined( VIRTUAL_FUNC )
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
#else
  template<class T>
  class IGeneralDriver
  {
  public:
    Chimera::Status_t attach( RegisterMap *const peripheral )
    {
      return static_cast<T *>( this )->attach( peripheral );
    }
  };
#endif

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
}    // namespace Thor::LLD::TIMER

#endif /* !LLD_TIMER_INTERFACE_HPP */
