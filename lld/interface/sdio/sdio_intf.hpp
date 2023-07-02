/******************************************************************************
 *  File Name:
 *    sdio_intf.hpp
 *
 *  Description:
 *    SDIO LLD Interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_SDIO_INTF_HPP
#define THOR_LLD_SDIO_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/sdio>
#include <Thor/lld/common/interrupts/sdio_interrupt_vectors.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/sdio/sdio_types.hpp>
#include <Thor/lld/interface/sdio/sdio_detail.hpp>

namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  ---------------------------------------------------------------------------*/
  /**
   * @brief Checks if the given channel is supported by the hardware
   *
   * @param channel    The channel to check
   * @return bool
   */
  bool isSupported( const Chimera::SDIO::Channel channel );

  /**
   * @brief Looks up a resource index based on a raw peripheral instance
   *
   * @param address       The peripheral address
   * @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   * @brief Gets the channel associated with a peripheral address
   *
   * @param address       Memory address the peripheral is mapped to
   * @return Chimera::Serial::Channel
   */
  Chimera::SDIO::Channel getChannel( const std::uintptr_t address );

  /**
   * @brief Initializes the drivers by attaching the appropriate peripheral
   *
   * @param driverList    List of HW driver objects to be initialized
   * @param numDrivers    How many drivers are in driverList
   * @return bool
   */
  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class Driver
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t attach( RegisterMap *const peripheral );
    Chimera::Status_t reset();
    void              clockEnable();
    void              clockDisable();

  protected:
    void IRQHandler();
    void enterCriticalSection();
    void exitCriticalSection();

  private:
    friend void( ::SDIO_IRQHandler )();

    RegisterMap *mPeriph;       /**< Mapped hardware peripheral */
    size_t       resourceIndex; /**< Lookup index for mPeriph */
  };

}    // namespace Thor::LLD::SDIO

#endif /* !THOR_LLD_SDIO_INTF_HPP */
