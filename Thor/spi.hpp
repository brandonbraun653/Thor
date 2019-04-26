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

#if 0
namespace Thor::SPI
{
  class SPIClass : public Chimera::SPI::Interface
  {
  public:
    SPIClass();
    ~SPIClass();

    Chimera::Status_t init( const Chimera::SPI::Setup &setupStruct ) final override;

    Chimera::Status_t deInit() final override;

    Chimera::Status_t setChipSelect( const Chimera::GPIO::State &value ) final override;

    Chimera::Status_t setChipSelectControlMode( const Chimera::SPI::ChipSelectMode &mode ) final override;

    Chimera::Status_t writeBytes( const uint8_t *const txBuffer, const size_t length, const uint32_t timeoutMS ) final override;

    Chimera::Status_t readBytes( uint8_t *const rxBuffer, const size_t length, const uint32_t timeoutMS ) final override;

    Chimera::Status_t readWriteBytes( const uint8_t *const txBuffer, uint8_t *const rxBuffer, const size_t length,
                                      const uint32_t timeoutMS ) final override;

    Chimera::Status_t setPeripheralMode( const Chimera::SPI::SubPeripheral periph,
                                         const Chimera::SPI::SubPeripheralMode mode ) final override;

    Chimera::Status_t setClockFrequency( const uint32_t freq, const uint32_t tolerance ) final override;

    Chimera::Status_t getClockFrequency( uint32_t &freq ) final override;

    Chimera::Status_t onWriteCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

    Chimera::Status_t onReadCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

    Chimera::Status_t onReadWriteCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

    Chimera::Status_t onErrorCallback( const Chimera::Function::void_func_uint32_t func ) final override;


    /**
     *  A factory method to create a new SPIClass object
     *
     *	This method intentionally replaces the typical constructor for the purpose of allowing
     *	the SPI ISR handlers to deduce at runtime which class generated the interrupt. The new
     *	object is internally registered with a static vector that keeps track of this.
     *
     *	@param[in] channel      Hardware SPI peripheral channel number (i.e. 1 for SPI1, 2 for SPI2, etc)
     *	@return const SPIClass_sPtr&
     */
    static const Chimera::SPI::SPIClass_sPtr& create( const uint8_t channel );


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

    Chimera::Status_t transfer_interrupt( const uint8_t *const txBuffer, uint8_t *const rxBuffer, const size_t length );

    Chimera::Status_t transfer_dma( const uint8_t *const txBuffer, uint8_t *const rxBuffer, const size_t length );

  private:
    /**
     *  @brief Construct a new SPIClass object
     *
     * 	This is kept private so users properly manage the class object with
     *  shared_ptr instances.
     * 	The public constructor is Thor::Peripheral::SPI::SPIClass::create.
     *
     *  @param[in]	channel 	Which hardware peripheral to control with the class
     */
    SPIClass( const uint8_t channel );

    bool alternateCS;
    uint8_t spi_channel;

    Chimera::SPI::ChipSelectMode chipSelectMode;
    Chimera::SPI::SubPeripheralMode txMode;
    Chimera::SPI::SubPeripheralMode rxMode;

    struct SPIClassStatus
    {
      bool gpio_enabled              = false;
      bool spi_enabled               = false;
      bool spi_interrupts_enabled    = false;
      bool dma_enabled_tx            = false;
      bool dma_enabled_rx            = false;
      bool dma_interrupts_enabled_tx = false;
      bool dma_interrupts_enabled_rx = false;
    } hardwareStatus;

    SPI_HandleTypeDef spi_handle;
    DMA_HandleTypeDef hdma_spi_tx;
    DMA_HandleTypeDef hdma_spi_rx;

    Thor::Interrupt::Initializer ITSettingsHW;
    Thor::Interrupt::Initializer ITSettings_DMA_TX;
    Thor::Interrupt::Initializer ITSettings_DMA_RX;

    Thor::GPIO::GPIOClass_sPtr externalCS;
    Thor::GPIO::GPIOClass_sPtr MOSI, MISO, SCK, CS;

    void resetISRFlags();

    void SPI_GPIO_Init();
    void SPI_GPIO_DeInit();

    void SPI_Init();
    void SPI_DeInit();
    void SPI_EnableClock();
    void SPI_DisableClock();
    void SPI_DMA_EnableClock();
    void SPI_EnableInterrupts();
    void SPI_DisableInterrupts();

    void SPI_DMA_Init( const Chimera::SPI::SubPeripheral periph );
    void SPI_DMA_DeInit( const Chimera::SPI::SubPeripheral periph );
    void SPI_DMA_EnableInterrupts( const Chimera::SPI::SubPeripheral periph );
    void SPI_DMA_DisableInterrupts( const Chimera::SPI::SubPeripheral periph );
  };
}    // namespace Thor::SPI
#endif 

#endif /* SPI_H_*/
