/********************************************************************************
 *   File Name:
 *    spi.hpp
 *
 *   Description:
 *    Thor SPI Interface
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SPI_HPP
#define THOR_SPI_HPP

/* C/C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/event>
#include <Chimera/gpio>
#include <Chimera/spi>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/common/interrupts/spi_interrupt_vectors.hpp>
#include <Thor/drivers/spi.hpp>


#if defined( THOR_CUSTOM_DRIVERS ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::SPI
{
  void initialize();

  class SPIClass : virtual public Chimera::SPI::ISPI,
                   public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Class Specific Functions
    ------------------------------------------------*/
    SPIClass();
    ~SPIClass();

    void postISRProcessing();

    /*------------------------------------------------
    HW Interface
    ------------------------------------------------*/
    Chimera::Status_t init( const Chimera::SPI::DriverConfig &setupStruct ) final override;
    Chimera::SPI::DriverConfig getInit() final override;
    Chimera::Status_t deInit() final override;
    Chimera::Status_t setChipSelect( const Chimera::GPIO::State value ) final override;
    Chimera::Status_t setChipSelectControlMode( const Chimera::SPI::CSMode mode ) final override;
    Chimera::Status_t writeBytes( const void *const txBuffer, const size_t length, const size_t timeoutMS ) final override;
    Chimera::Status_t readBytes( void *const rxBuffer, const size_t length, const size_t timeoutMS ) final override;
    Chimera::Status_t readWriteBytes( const void *const txBuffer, void *const rxBuffer, const size_t length,
                                      const size_t timeoutMS ) final override;
    Chimera::Status_t setPeripheralMode( const Chimera::Hardware::PeripheralMode mode ) final override;
    Chimera::Status_t setClockFrequency( const size_t freq, const size_t tolerance ) final override;
    size_t getClockFrequency() final override;

    /*------------------------------------------------
    Async IO Interface
    ------------------------------------------------*/
    Chimera::Status_t await( const Chimera::Event::Trigger event, const size_t timeout ) final override;
    Chimera::Status_t await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                             const size_t timeout ) final override;

    /*------------------------------------------------
    Listener Interface
    ------------------------------------------------*/
    Chimera::Status_t registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                        size_t &registrationID ) final override;
    Chimera::Status_t removeListener( const size_t registrationID, const size_t timeout ) final override;

  private:
    Chimera::SPI::DriverConfig config;       /**< Configuration used to set up the class */
    Chimera::GPIO::GPIO_uPtr SCK;          /**< SPI clock gpio pin */
    Chimera::GPIO::GPIO_uPtr MOSI;         /**< SPI MOSI gpio pin */
    Chimera::GPIO::GPIO_uPtr MISO;         /**< SPI MISO gpio pin */
    Chimera::GPIO::GPIO_sPtr CS;           /**< SPI Chip Select gpio pin */
    Thor::Driver::SPI::Driver_uPtr driver;   /**< Low level hardware SPI driver */
    
    Chimera::Threading::BinarySemaphore awaitTransferComplete; /**< Internal signal for when the current transfer has completed */
  };

}    // namespace Thor::SPI

#endif /* THOR_CUSTOM_DRIVERS && THOR_DRIVER_SPI */


#endif /* SPI_H_*/
