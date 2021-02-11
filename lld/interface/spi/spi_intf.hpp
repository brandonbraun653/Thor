/********************************************************************************
 *  File Name:
 *    spi_intf.hpp
 *
 *  Description:
 *    STM32 LLD SPI Interface Spec
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_SPI_DRIVER_INTERFACE_HPP
#define THOR_LLD_SPI_DRIVER_INTERFACE_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/spi>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/common/interrupts/spi_interrupt_vectors.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/spi/spi_types.hpp>

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
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


  /*-------------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  -------------------------------------------------------------------------------*/
  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  bool isSupported( const Chimera::SPI::Channel channel );

  /**
   *  Get's the resource index associated with a particular channel. If not
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


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Virtual class that defines the expected interface.
  Useful for mocking purposes.
  -------------------------------------------------*/
  class IDriver
  {
  public:
    virtual ~IDriver() = default;

    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired SPI peripheral
     *  @return void
     */
    virtual Chimera::Status_t attach( RegisterMap *const peripheral ) = 0;

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

    /**
     *  Gets any error flags (bitfield) that might be set
     *
     *  @return ErrorFlags_t
     */
    virtual size_t getErrorFlags() = 0;

    /**
     *  Gets any status flags (bitfield) that might be set
     *
     *  @return StatusFlags_t
     */
    virtual size_t getStatusFlags() = 0;

    virtual Chimera::Status_t configure( const Chimera::SPI::DriverConfig &setup ) = 0;

    virtual Chimera::Status_t registerConfig( Chimera::SPI::DriverConfig *config ) = 0;

    virtual Chimera::Status_t transfer( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) = 0;

    virtual Chimera::Status_t transferIT( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) = 0;

    virtual Chimera::Status_t transferDMA( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) = 0;

    virtual Chimera::Status_t killTransfer() = 0;

    virtual void attachISRWakeup( Chimera::Thread::BinarySemaphore *const wakeup ) = 0;

    virtual HWTransfer getTransferBlock() = 0;
  };


  /*-------------------------------------------------
  Concrete driver declaration. Implements the interface
  of the virtual class, but doesn't inherit due to the
  memory penalties. Definition is done project side.
  -------------------------------------------------*/
  class Driver
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t attach( RegisterMap *const peripheral );
    Chimera::Status_t reset();
    void clockEnable();
    void clockDisable();
    size_t getErrorFlags();
    size_t getStatusFlags();
    Chimera::Status_t configure( const Chimera::SPI::DriverConfig &setup );
    Chimera::Status_t registerConfig( Chimera::SPI::DriverConfig *config );
    Chimera::Status_t transfer( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize );
    Chimera::Status_t transferIT( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize );
    Chimera::Status_t transferDMA( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize );
    Chimera::Status_t killTransfer();
    void attachISRWakeup( Chimera::Thread::BinarySemaphore *const wakeup );
    HWTransfer getTransferBlock();

  protected:
    void IRQHandler();
    void enterCriticalSection();
    void exitCriticalSection();

  private:
    friend void(::SPI1_IRQHandler )();
    friend void(::SPI2_IRQHandler )();
    friend void(::SPI3_IRQHandler )();
    friend void(::SPI4_IRQHandler )();
    friend void(::SPI5_IRQHandler )();
    friend void(::SPI6_IRQHandler )();

    RegisterMap *mPeriph;
    size_t resourceIndex;
    Chimera::SPI::DriverConfig *periphConfig;

    /*------------------------------------------------
    Asynchronous Event Listeners
    ------------------------------------------------*/
    Chimera::Thread::BinarySemaphore *ISRWakeup_external;

    /*------------------------------------------------
    Transfer Control Blocks
    ------------------------------------------------*/
    HWTransfer txfr;
  };

}    // namespace Thor::LLD::SPI

#endif /* THOR_LLD_SPI_DRIVER_INTERFACE_HPP */
