/********************************************************************************
 *   File Name:
 *    hw_spi_driver.hpp
 *
 *   Description:
 *    STM32 Driver for the SPI Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_DRIVER_HPP
#define THOR_HW_SPI_DRIVER_HPP

/* C++ Includes */
#include <memory>
#include <vector>

/* Chimera Includes */
#include <Chimera/threading.hpp>
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/peripheral_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/common/interrupts/spi_interrupt_vectors.hpp>
#include <Thor/drivers/f4/spi/hw_spi_types.hpp>
#include <Thor/drivers/model/spi_model.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
  /**
   *  Initializes the low level driver
   *
   *  @return void
   */
  void initialize();


  class Driver : public Thor::Driver::SPI::Model
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t attach( RegisterMap *const peripheral ) final override;
    Chimera::Status_t reset() final override;
    void clockEnable() final override;
    void clockDisable() final override;
    size_t getErrorFlags() override;
    size_t getStatusFlags() override;
    Chimera::Status_t configure( const Chimera::SPI::DriverConfig &setup ) final override;
    Chimera::Status_t registerConfig( Chimera::SPI::DriverConfig *config ) final override;
    Chimera::Status_t transfer( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) final override;
    Chimera::Status_t transferIT( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) final override;
    Chimera::Status_t transferDMA( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) final override;
    Chimera::Status_t killTransfer() final override;

    void attachISRWakeup( Chimera::Threading::BinarySemaphore *const wakeup );

    HWTransfer getTransferBlock();

  protected:
    friend void(::SPI1_IRQHandler )();
    friend void(::SPI2_IRQHandler )();
    friend void(::SPI3_IRQHandler )();
    friend void(::SPI4_IRQHandler )();
    friend void(::SPI5_IRQHandler )();
    friend void(::SPI6_IRQHandler )();

    /**
     *  Generic interrupt handler for SPI ISR signals
     *  @return void
     */
    void IRQHandler();

    void enterCriticalSection();
    void exitCriticalSection();

  private:
    RegisterMap *periph;  /**< Memory mapped struct to instance registers */
    uint32_t dmaTXSignal; /**< DMA request signal ID for TX operations */
    uint32_t dmaRXSignal; /**< DMA request signal ID for RX operations */
    IRQn_Type periphIRQn; /**< Instance interrupt request signal number */
    size_t resourceIndex; /**< Derived lookup table index for resource access */
    Chimera::SPI::DriverConfig *periphConfig;

    /*------------------------------------------------
    Asynchronous Event Listeners
    ------------------------------------------------*/
    Chimera::Threading::BinarySemaphore *ISRWakeup_external;

    /*------------------------------------------------
    Transfer Control Blocks
    ------------------------------------------------*/
    HWTransfer txfr;
  };
}    // namespace Thor::Driver::SPI

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */
#endif /* THOR_HW_SPI_DRIVER_HPP */