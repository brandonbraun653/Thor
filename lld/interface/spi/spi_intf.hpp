/******************************************************************************
 *  File Name:
 *    spi_intf.hpp
 *
 *  Description:
 *    STM32 LLD SPI Interface Spec
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_SPI_DRIVER_INTERFACE_HPP
#define THOR_LLD_SPI_DRIVER_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/spi>
#include <Chimera/thread>
#include <Thor/lld/common/interrupts/spi_interrupt_vectors.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/spi/spi_types.hpp>

namespace Thor::LLD::SPI
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   */
  Chimera::Status_t initialize();

  /**
   *  Gets a shared pointer to the SPI driver for a particular channel
   *
   *  @param[in] channel        The SPI channel to grab (1 indexed)
   *  @return IDriver_rPtr      Instance of the SPI driver for the requested channel
   */
  Driver_rPtr getDriver( const Chimera::SPI::Channel channel );


  /*---------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  ---------------------------------------------------------------------------*/
  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  bool isSupported( const Chimera::SPI::Channel channel );

  /**
   *  Gets the resource index associated with a particular channel. If not
   *  supported, will return INVALID_RESOURCE_INDEX
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const Chimera::SPI::Channel channel );

  /**
   *  Looks up a resource index based on a raw peripheral instance
   *
   *  @param[in]  address       The peripheral address
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   *  Gets the SPI channel associated with a peripheral address
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return Chimera::SPI::Channel
   */
  Chimera::SPI::Channel getChannel( const std::uintptr_t address );

  /**
   *  Initializes the SPI drivers by attaching the appropriate peripheral
   *
   *  @param[in]  driverList    List of driver objects to be initialized
   *  @param[in]  numDrivers    How many drivers are in driverList
   *  @return bool
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
    Chimera::Status_t configure( const Chimera::SPI::HardwareInit &setup );
    Chimera::Status_t registerConfig( Chimera::SPI::HardwareInit *config );
    Chimera::Status_t transfer( const void *txBuffer, void *rxBuffer, const size_t bufferSize );
    Chimera::Status_t transferIT( const void *txBuffer, void *rxBuffer, const size_t bufferSize );
    Chimera::Status_t transferDMA( const void *txBuffer, void *rxBuffer, const size_t bufferSize );

  protected:
    void IRQHandler();
    void enterCriticalSection();
    void exitCriticalSection();

  private:
    friend void( ::SPI1_IRQHandler )();
    friend void( ::SPI2_IRQHandler )();
    friend void( ::SPI3_IRQHandler )();
    friend void( ::SPI4_IRQHandler )();
    friend void( ::SPI5_IRQHandler )();
    friend void( ::SPI6_IRQHandler )();

    RegisterMap                *mPeriph;       /**< Mapped hardware peripheral */
    size_t                      resourceIndex; /**< Lookup index for mPeriph */
    Chimera::SPI::HardwareInit *periphConfig;  /**< Hardware configuration data */
    volatile HWTransfer         txfr;          /**< Control block for transfers */
  };

}    // namespace Thor::LLD::SPI

#endif /* THOR_LLD_SPI_DRIVER_INTERFACE_HPP */
