/******************************************************************************
 *  File Name:
 *    sdio_driver.hpp
 *
 *  Description:
 *    Driver interface for the SDIO peripheral
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_INTF_SDIO_DRIVER_HPP
#define THOR_LLD_INTF_SDIO_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Thor/lld/common/interrupts/sdio_interrupt_vectors.hpp>
#include <Thor/lld/interface/sdio/sdio_detail.hpp>

namespace Thor::LLD::SDIO
{
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
}  // namespace 

#endif  /* !THOR_LLD_INTF_SDIO_DRIVER_HPP */
