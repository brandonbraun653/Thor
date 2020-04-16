/********************************************************************************
 *  File Name:
 *    hw_spi_driver.hpp
 *
 *  Description:
 *    Declares the LLD interface to the STM32L4 series SPI hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_DRIVER_STM32L4_HPP
#define THOR_HW_SPI_DRIVER_STM32L4_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/spi>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/common/interrupts/spi_interrupt_vectors.hpp>
#include <Thor/lld/interface/spi/spi_intf.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_types.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_mapping.hpp>

namespace Thor::LLD::SPI
{
  class Driver : public IDriver
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
    void attachISRWakeup( Chimera::Threading::BinarySemaphore *const wakeup ) final override;
    HWTransfer getTransferBlock() final override;

  protected:
    friend void(::SPI1_IRQHandler )();
    friend void(::SPI2_IRQHandler )();
    friend void(::SPI3_IRQHandler )();

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
}    // namespace Thor::LLD::SPI

#endif /* !THOR_HW_SPI_DRIVER_STM32L4_HPP */
