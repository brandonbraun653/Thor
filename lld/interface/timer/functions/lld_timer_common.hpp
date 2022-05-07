/******************************************************************************
 *  File Name:
 *    lld_timer_common.hpp
 *
 *  Description:
 *    Common driver for Advanced, Basic, General timers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_COMMON_DRIVER_HPP
#define THOR_LLD_TIMER_COMMON_DRIVER_HPP

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
   */
  template<class Derived>
  class ModuleDriver
  {
  public:
    ModuleDriver() : mPeriph( nullptr ), mGlobalIndex( 0 ), mTypeIndex( 0 )
    {
    }

    ~ModuleDriver()
    {
    }

    /**
     * @brief Attaches a peripheral instance to the driver
     *
     * @param peripheral    Memory mapped struct of the desired peripheral
     * @return void
     */
    void attach( RegisterMap *const peripheral )
    {
      /*-------------------------------------------------------------------------
      Input Protection
      -------------------------------------------------------------------------*/
      if ( !peripheral )
      {
        return;
      }

      /*-------------------------------------------------------------------------
      Build up the peripheral information
      -------------------------------------------------------------------------*/
      mPeriph      = peripheral;
      mGlobalIndex = getGlobalResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );
      mTypeIndex   = getTypeResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );
    }


    /**
     * @brief Resets the hardware registers back to boot-up values
     * @return Chimera::Status_t
     */
    void reset()
    {
      auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
      rcc->reset( Chimera::Peripheral::Type::PERIPH_TIMER, mGlobalIndex );
    }


    /**
     * @brief Enables the peripheral clock
     * @return void
     */
    void clockEnable()
    {
      auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
      rcc->enableClock( Chimera::Peripheral::Type::PERIPH_TIMER, mGlobalIndex );
    }


    /**
     * @brief Disables the peripheral clock
     * @return void
     */
    void clockDisable()
    {
      auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
      rcc->disableClock( Chimera::Peripheral::Type::PERIPH_TIMER, mGlobalIndex );
    }


    /**
     * @brief Powers up the entire peripheral
     * @return void
     */
    void open()
    {
      /*-------------------------------------------------------------------------
      Power on the clock and reset the hardware to defaults
      -------------------------------------------------------------------------*/
      this->clockEnable();
      this->reset();

      /*-------------------------------------------------------------------------
      Ensure the counter is off
      -------------------------------------------------------------------------*/
      static_cast<Derived *>( this )->disableCounter();
    }


    /**
     * @brief Tear down the peripheral
     * @return void
     */
    void close()
    {
      /*-------------------------------------------------------------------------
      Turn off the peripheral
      -------------------------------------------------------------------------*/
      this->clockDisable();
    }

  protected:
    RegisterMap *mPeriph;  /**< Memory mapped struct to the peripheral driver */
    RIndex_t mGlobalIndex; /**< Global resource index for all timer peripherals */
    RIndex_t mTypeIndex;   /**< Resource index for specific timer type (General, Advanced, etc.) */
  };

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_COMMON_DRIVER_HPP */
