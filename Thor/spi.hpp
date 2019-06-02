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

/* Thor Includes */
#include <Thor/types/spi_types.hpp>
#include <Thor/types/gpio_types.hpp>

/* Chimera Includes */
#include <Chimera/interface/spi_intf.hpp>

#ifdef __cplusplus
extern "C"
{
#endif
  extern void SPI1_IRQHandler();
  extern void SPI2_IRQHandler();
  extern void SPI3_IRQHandler();
  extern void SPI4_IRQHandler();
  extern void SPI5_IRQHandler();
  extern void SPI6_IRQHandler();
#ifdef __cplusplus
}
#endif

namespace Thor::SPI
{
  class SPIClass : public Chimera::SPI::Interface
  {
  public:
    SPIClass();
    ~SPIClass();
    
    SPIClass( SPIClass *var );

    Chimera::Status_t init( const Chimera::SPI::Setup &setup ) final override;

    Chimera::Status_t deInit() final override;
    
    Chimera::SPI::Setup getInit() final override;

    Chimera::Status_t setChipSelect( const Chimera::GPIO::State value ) final override;

    Chimera::Status_t setChipSelectControlMode( const Chimera::SPI::ChipSelectMode mode ) final override;

    Chimera::Status_t writeBytes( const uint8_t *const txBuffer, const size_t length, const uint32_t timeoutMS ) final override;

    Chimera::Status_t readBytes( uint8_t *const rxBuffer, const size_t length, const uint32_t timeoutMS ) final override;

    Chimera::Status_t readWriteBytes( const uint8_t *const txBuffer, uint8_t *const rxBuffer, const size_t length,
                                      const uint32_t timeoutMS ) final override;

    Chimera::Status_t setPeripheralMode( const Chimera::Hardware::SubPeripheral periph,
                                         const Chimera::Hardware::SubPeripheralMode mode ) final override;

    Chimera::Status_t setClockFrequency( const uint32_t freq, const uint32_t tolerance ) final override;

    Chimera::Status_t getClockFrequency( uint32_t &freq ) final override;

    Chimera::Status_t attachNotifier( const Chimera::Event::Trigger_t event, volatile uint8_t *const notifier ) final override;

    Chimera::Status_t detachNotifier( const Chimera::Event::Trigger_t event, volatile uint8_t *const notifier ) final override;

    Chimera::Status_t attachCallback( const Chimera::Event::Trigger_t trigger,
                                      const Chimera::Function::void_func_void func ) final override;

    Chimera::Status_t detachCallback( const Chimera::Event::Trigger_t trigger ) final override;

#if defined( USING_FREERTOS )
    Chimera::Status_t attachNotifier( const Chimera::Event::Trigger_t event, SemaphoreHandle_t *const semphr ) final override;

    Chimera::Status_t detachNotifier( const Chimera::Event::Trigger_t event, SemaphoreHandle_t *const semphr ) final override;
#endif

    /**
     *  Normal interrupt based ISR handler
     *
     *  @return void
     */
    void IRQHandler();

    /**
     *  DMA TX ISR handler
     *
     *  @return void
     */
    void IRQHandler_TXDMA();

    /**
     *  DMA RX ISR handler
     *
     *  @return void
     */
    void IRQHandler_RXDMA();

  protected:
    friend void(::HAL_SPI_TxCpltCallback )( SPI_HandleTypeDef *hspi );
    friend void(::HAL_SPI_TxRxCpltCallback )( SPI_HandleTypeDef *hspi );
    friend void(::HAL_SPI_ErrorCallback )( SPI_HandleTypeDef *hspi );

    static uint32_t getPrescaler( const int &channel, const uint32_t &freq );

    static uint32_t getFrequency( const int &channel, const uint32_t &prescaler );

    Chimera::Status_t transfer_blocking( const uint8_t *const txBuffer, uint8_t *const rxBuffer, const size_t length, const uint32_t timeoutMS );

    Chimera::Status_t transfer_interrupt( const uint8_t *const txBuffer, uint8_t *const rxBuffer, const size_t length, const uint32_t timeoutMS );

    Chimera::Status_t transfer_dma( const uint8_t *const txBuffer, uint8_t *const rxBuffer, const size_t length, const uint32_t timeoutMS );
    
  private:

    bool hardwareChipSelect;    /**< If true, the chip select line is controlled by the SPI peripheral */
    uint8_t spi_channel;        /**< The hardware channel this class is bound to */
    
    uint8_t dmaRXReqSig;
    uint8_t dmaTXReqSig;

    Chimera::SPI::Setup cachedSetup;
    Chimera::SPI::ChipSelectMode chipSelectMode;
    Chimera::Hardware::SubPeripheralMode txMode;
    Chimera::Hardware::SubPeripheralMode rxMode;

    volatile bool transfer_complete = false;

    struct SPIStatus
    {
      bool gpio_initialized          = false;
      bool spi_initialized           = false;
      bool spi_interrupts_enabled    = false;
      bool dma_enabled_tx            = false;
      bool dma_enabled_rx            = false;
      bool dma_interrupts_enabled_tx = false;
      bool dma_interrupts_enabled_rx = false;
    } periphStatus;

    SPI_HandleTypeDef spi_handle;
    DMA_HandleTypeDef hdma_spi_tx;
    DMA_HandleTypeDef hdma_spi_rx;

    Thor::Interrupt::Initializer ITSettingsHW;
    Thor::Interrupt::Initializer ITSettings_DMA_TX;
    Thor::Interrupt::Initializer ITSettings_DMA_RX;

    Thor::GPIO::GPIOClass_uPtr MOSI;
    Thor::GPIO::GPIOClass_uPtr MISO;
    Thor::GPIO::GPIOClass_uPtr SCK;
    Thor::GPIO::GPIOClass_uPtr CS;

    void SPI_GPIO_Init();
    void SPI_GPIO_DeInit();

    void SPI_Init();
    void SPI_DeInit();
    void SPI_EnableClock();
    void SPI_DisableClock();
    void SPI_DMA_EnableClock();
    void SPI_EnableInterrupts();
    void SPI_DisableInterrupts();

    void SPI_DMA_Init( const Chimera::Hardware::SubPeripheral periph );
    void SPI_DMA_DeInit( const Chimera::Hardware::SubPeripheral periph );
    void SPI_DMA_EnableInterrupts( const Chimera::Hardware::SubPeripheral periph );
    void SPI_DMA_DisableInterrupts( const Chimera::Hardware::SubPeripheral periph );
  };
}    // namespace Thor::SPI

#endif /* SPI_H_*/
