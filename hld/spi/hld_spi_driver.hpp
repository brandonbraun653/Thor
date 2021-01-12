/********************************************************************************
 *  File Name:
 *    hld_spi_driver.hpp
 *
 *  Description:
 *    Thor SPI high level driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_SPI_HPP
#define THOR_HLD_SPI_HPP

/* C/C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/event>
#include <Chimera/gpio>
#include <Chimera/spi>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/hld/spi/hld_spi_types.hpp>

namespace Thor::SPI
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Chimera::SPI::Channel channel );

  class Driver : public Chimera::Threading::LockableCRTP<Driver>
  {
  public:
    /*------------------------------------------------
    Class Specific Functions
    ------------------------------------------------*/
    Driver();
    ~Driver();

    void postISRProcessing();

    /*------------------------------------------------
    HW Interface
    ------------------------------------------------*/
    Chimera::Status_t init( const Chimera::SPI::DriverConfig &setupStruct );
    Chimera::SPI::DriverConfig getInit();
    Chimera::Status_t deInit();
    Chimera::Status_t setChipSelect( const Chimera::GPIO::State value );
    Chimera::Status_t setChipSelectControlMode( const Chimera::SPI::CSMode mode );
    Chimera::Status_t writeBytes( const void *const txBuffer, const size_t length );
    Chimera::Status_t readBytes( void *const rxBuffer, const size_t length );
    Chimera::Status_t readWriteBytes( const void *const txBuffer, void *const rxBuffer, const size_t length );
    Chimera::Status_t setPeripheralMode( const Chimera::Hardware::PeripheralMode mode );
    Chimera::Status_t setClockFrequency( const size_t freq, const size_t tolerance );
    size_t getClockFrequency();

    /*------------------------------------------------
    Async IO Interface
    ------------------------------------------------*/
    Chimera::Status_t await( const Chimera::Event::Trigger event, const size_t timeout );
    Chimera::Status_t await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                             const size_t timeout );

    /*------------------------------------------------
    Listener Interface
    ------------------------------------------------*/
    Chimera::Status_t registerListener( Chimera::Event::Actionable &listener, const size_t timeout, size_t &registrationID );
    Chimera::Status_t removeListener( const size_t registrationID, const size_t timeout );

  private:
    friend Chimera::Threading::LockableCRTP<Driver>;
    Chimera::Threading::RecursiveTimedMutex mClsMutex;

    Chimera::SPI::DriverConfig config; /**< Configuration used to set up the class */
    Chimera::GPIO::Driver_rPtr SCK;    /**< SPI clock gpio pin */
    Chimera::GPIO::Driver_rPtr MOSI;   /**< SPI MOSI gpio pin */
    Chimera::GPIO::Driver_rPtr MISO;   /**< SPI MISO gpio pin */
    Chimera::GPIO::Driver_rPtr CS;     /**< SPI Chip Select gpio pin */

    Chimera::Event::ActionableList eventListeners;
    Chimera::Threading::BinarySemaphore awaitTransferComplete; /**< Internal signal for current transfer completed */
  };

}    // namespace Thor::SPI

#endif /* THOR_HLD_SPI_HPP */
