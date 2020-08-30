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
#include <Thor/hld/common/types.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/timer/timer_types.hpp>

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------------------------------------
  HLD->LLD Required Free Functions
  -------------------------------------------------------------------------------*/
  void incrementSystemTick();
  size_t millis();
  void delayMilliseconds( const size_t ms );
  void delayMicroseconds( const size_t us );

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   */
  Chimera::Status_t initializeModule();

  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  bool isChannelSupported( const size_t channel );

  IAdvancedDriver_sPtr getAdvancedDriver( const Thor::HLD::RIndex channel );

  IBasicDriver_sPtr getBasicDriver( const Thor::HLD::RIndex channel );

  GeneralDriver_rPtr getGeneralDriver( const Thor::HLD::RIndex channel );

  ILowPowerDriver_sPtr getLowPowerDriver( const Thor::HLD::RIndex channel );

  /**
   *  Gets the peripheral description data associated with the
   *  resource index.
   *
   *  @note This data is mapped to a resource index provided by the LLD implementation.
   *        Do not use a HLD resource index to
   *
   *  @param[in]  lldIndex    The look up index (must be from LLD's perspective)
   *  @return const DeviceDescription *
   */
  const DeviceDescription *getPeripheralDescriptor( const Thor::LLD::RIndex lldIndex );

  /**
   *  Looks up a resource index based on a raw peripheral instance
   *
   *  @param[in]  address       The peripheral address
   *  @return RIndexType
   */
  RIndexType getResourceIndex( const std::uintptr_t address );


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
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
