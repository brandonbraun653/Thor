/********************************************************************************
 *   File Name:
 *    thor_spi.cpp
 *
 *   Description:
 *    Thor SPI Interface
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C/C++ Includes */
#include <cstdlib>
#include <cstring>
#include <cmath>

/* Boost Includes */
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>

/* Thor Includes */
#include <Thor/core.hpp>
#include <Thor/dma.hpp>
#include <Thor/gpio.hpp>
#include <Thor/spi.hpp>
#include <Thor/defaults/spi_defaults.hpp>
#include <Thor/definitions/spi_definitions.hpp>

/* Mock Includes */
#if defined( GMOCK_TEST )
#include "mock_stm32_hal_spi.hpp"
#endif

#if defined( USING_FREERTOS )
#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
}
#endif
#endif

/*------------------------------------------------
Stores references to available SPIClass objects
-------------------------------------------------*/
// static Thor::SPI::SPIClass_sPtr spiObjects[ Thor::SPI::MAX_SPI_CHANNELS + 1 ];
static Thor::SPI::SPIClass *spiObjects[ Thor::SPI::MAX_SPI_CHANNELS + 1 ];

/*------------------------------------------------
Directly maps the HAL SPI Instance pointer to a possible SPIClass object
-------------------------------------------------*/
static auto getSPIClassRef( SPI_TypeDef *instance )
{
  auto i = reinterpret_cast<std::uintptr_t>( instance );

  switch ( i )
  {
#if defined( SPI1 )
    case SPI1_BASE:
      return spiObjects[ 1 ];
      break;
#endif
#if defined( SPI2 )
    case SPI2_BASE:
      return spiObjects[ 2 ];
      break;
#endif
#if defined( SPI3 )
    case SPI3_BASE:
      return spiObjects[ 3 ];
      break;
#endif
#if defined( SPI4 )
    case SPI4_BASE:
      return spiObjects[ 4 ];
      break;
#endif
#if defined( SPI5 )
    case SPI5_BASE:
      return spiObjects[ 5 ];
      break;
#endif
#if defined( SPI6 )
    case SPI6_BASE:
      return spiObjects[ 6 ];
      break;
#endif

    /* If we get here, something went wrong and the program will likely crash */
    default:
      return spiObjects[ 0 ];
      break;
  };
}

/*------------------------------------------------
Directly maps the HAL SPI Instance pointer to the correct bit mask for
enabling/disabling the peripheral clock
-------------------------------------------------*/
static uint32_t spiClockMask( SPI_TypeDef *instance )
{
  auto i = reinterpret_cast<std::uintptr_t>( instance );

  switch ( i )
  {
#if defined( TARGET_STM32F4 ) || defined( TARGET_STM32F7 )
#if defined( SPI1 )
    case SPI1_BASE:
      return RCC_APB2ENR_SPI1EN;
      break;
#endif
#if defined( SPI2 )
    case SPI2_BASE:
      return RCC_APB1ENR_SPI2EN;
      break;
#endif
#if defined( SPI3 )
    case SPI3_BASE:
      return RCC_APB1ENR_SPI3EN;
      break;
#endif
#if defined( SPI4 )
    case SPI4_BASE:
      return RCC_APB2ENR_SPI4EN;
      break;
#endif
#endif

#if defined( TARGET_STM32F7 )
#if defined( SPI5 )
    case SPI5_BASE:
      return RCC_APB2ENR_SPI5EN;
      break;
#endif
#if defined( SPI6 )
    case SPI6_BASE:
      return RCC_APB2ENR_SPI6EN;
      break;
#endif
#endif /* !TARGET_STM32F7 */

    /* If we get here, something went wrong */
    default:
      return 0u;
      break;
  };
}

/*------------------------------------------------
Directly maps the HAL SPI Instance pointer to the correct register for
enabling/disabling the peripheral clock.
-------------------------------------------------*/
static volatile uint32_t *spiClockRegister( SPI_TypeDef *instance )
{
  auto i = reinterpret_cast<std::uintptr_t>( instance );

  switch ( i )
  {
#if defined( TARGET_STM32F4 ) || defined( TARGET_STM32F7 )
#if defined( SPI1 )
    case SPI1_BASE:
      return &( RCC->APB2ENR );
      break;
#endif
#if defined( SPI2 )
    case SPI2_BASE:
      return &( RCC->APB1ENR );
      break;
#endif
#if defined( SPI3 )
    case SPI3_BASE:
      return &( RCC->APB1ENR );
      break;
#endif
#if defined( SPI4 )
    case SPI4_BASE:
      return &( RCC->APB2ENR );
      break;
#endif
#endif

#if defined( TARGET_STM32F7 )
#if defined( SPI5 )
    case SPI5_BASE:
      return &( RCC->APB2ENR );
      break;
#endif
#if defined( SPI6 )
    case SPI6_BASE:
      return &( RCC->APB2ENR );
      break;
#endif
#endif /* !TARGET_STM32F7 */

    /* If we get here, something went wrong */
    default:
      return nullptr;
      break;
  };
}

/*------------------------------------------------
Functions for converting Chimera parameters into STM32HAL
-------------------------------------------------*/
static inline uint32_t convertMode( const Chimera::SPI::Mode mode )
{
  uint32_t result = SPI_MODE_MASTER;

  if ( mode == Chimera::SPI::Mode::SLAVE )
  {
    result = SPI_MODE_SLAVE;
  }

  return result;
}

static inline uint32_t convertDataSize( const Chimera::SPI::DataSize size )
{
  uint32_t result = 0u;

  if ( size == Chimera::SPI::DataSize::SZ_8BIT )
  {
    result = SPI_DATASIZE_8BIT;
  }
  else if ( size == Chimera::SPI::DataSize::SZ_16BIT )
  {
    result = SPI_DATASIZE_16BIT;
  }

  return result;
}

static inline uint32_t convertBitOrder( const Chimera::SPI::BitOrder order )
{
  uint32_t result = 0u;

  if ( order == Chimera::SPI::BitOrder::LSB_FIRST )
  {
    result = SPI_FIRSTBIT_LSB;
  }
  else if ( order == Chimera::SPI::BitOrder::MSB_FIRST )
  {
    result = SPI_FIRSTBIT_MSB;
  }

  return result;
}

static inline uint32_t convertClockPhase( const Chimera::SPI::ClockMode mode )
{
  uint32_t result = 0u;

  if ( ( mode == Chimera::SPI::ClockMode::MODE0 ) || ( mode == Chimera::SPI::ClockMode::MODE2 ) )
  {
    result = SPI_PHASE_1EDGE;
  }
  else if ( ( mode == Chimera::SPI::ClockMode::MODE1 ) || ( mode == Chimera::SPI::ClockMode::MODE3 ) )
  {
    result = SPI_PHASE_2EDGE;
  }

  return result;
}

static inline uint32_t convertClockPolarity( const Chimera::SPI::ClockMode mode )
{
  uint32_t result = 0u;

  if ( ( mode == Chimera::SPI::ClockMode::MODE0 ) || ( mode == Chimera::SPI::ClockMode::MODE2 ) )
  {
    result = SPI_POLARITY_LOW;
  }
  else if ( ( mode == Chimera::SPI::ClockMode::MODE1 ) || ( mode == Chimera::SPI::ClockMode::MODE3 ) )
  {
    result = SPI_POLARITY_HIGH;
  }

  return result;
}


namespace Thor::SPI
{
  SPIClass::SPIClass( SPIClass *var )
  {
  }

  SPIClass::SPIClass() : MOSI(nullptr), MISO(nullptr), SCK(nullptr), CS(nullptr)
  {
    hardwareChipSelect = false;
    transfer_complete  = true;
    spi_handle.Init    = dflt_SPI_Init;

    hdma_spi_tx.Init = dflt_DMA_Init_TX;
    hdma_spi_rx.Init = dflt_DMA_Init_RX;

    dmaRXReqSig = Thor::DMA::Source::NONE;
    dmaTXReqSig = Thor::DMA::Source::NONE;

#if defined( GMOCK_TEST )
    if ( !STM32HAL_Mock::spiMockObj )
    {
      STM32HAL_Mock::spiMockObj = std::make_shared<STM32HAL_Mock::SPINiceMock>();
    }
#endif /* GMOCK_TEST */
  }

  SPIClass::~SPIClass()
  {
    spiObjects[ spi_channel ] = nullptr;
  }

  Chimera::Status_t SPIClass::init( const Chimera::SPI::Setup &setup )
  {
    using namespace Thor::GPIO;
    using namespace Chimera::SPI;
    using namespace Chimera::Hardware;

    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( setup.channel > MAX_SPI_CHANNELS )
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      spi_channel = setup.channel;
      cachedSetup = setup;

      /*------------------------------------------------
      MOSI
      ------------------------------------------------*/
      auto pin  = convertPinNum( setup.MOSI.pin );
      auto port = convertPort( setup.MOSI.port );

      if ( ( pin != PinNum::NOT_A_PIN ) && port )
      {
        MOSI = std::make_unique<Thor::GPIO::GPIOClass>();
        MOSI->initAdvanced( port, pin, PinSpeed::ULTRA_SPD, setup.MOSI.alternate );
      }
      else
      {
        MOSI.reset( nullptr );
      }

      /*------------------------------------------------
      MISO
      ------------------------------------------------*/
      pin  = convertPinNum( setup.MISO.pin );
      port = convertPort( setup.MISO.port );

      if ( ( pin != PinNum::NOT_A_PIN ) && port )
      {
        MISO = std::make_unique<Thor::GPIO::GPIOClass>();
        MISO->initAdvanced( port, pin, PinSpeed::ULTRA_SPD, setup.MISO.alternate );
      }
      else
      {
        MISO.reset( nullptr );
      }

      /*------------------------------------------------
      SCK
      ------------------------------------------------*/
      pin  = convertPinNum( setup.SCK.pin );
      port = convertPort( setup.SCK.port );

      if ( ( pin != PinNum::NOT_A_PIN ) && port )
      {
        SCK = std::make_unique<Thor::GPIO::GPIOClass>();
        SCK->initAdvanced( port, pin, PinSpeed::ULTRA_SPD, setup.SCK.alternate );
      }
      else
      {
        SCK.reset( nullptr );
      }

      /*------------------------------------------------
      CS
      ------------------------------------------------*/
      pin  = convertPinNum( setup.CS.pin );
      port = convertPort( setup.CS.port );

      if ( ( pin != PinNum::NOT_A_PIN ) && port )
      {
        CS = std::make_unique<Thor::GPIO::GPIOClass>();
        CS->initAdvanced( port, pin, PinSpeed::ULTRA_SPD, setup.CS.alternate );

        if ( setup.CS.alternate )
        {
          hardwareChipSelect = true;
        }
      }
      else
      {
        CS.reset( nullptr );
      }

      /*------------------------------------------------
      Initialize all the GPIO to the proper states
      ------------------------------------------------*/
      SPI_GPIO_Init();

      /*------------------------------------------------
      Initialize the SPI peripheral
      ------------------------------------------------*/
      /* Interrupt Control Blocks */
      ITSettingsHW      = hwConfig[ spi_channel ]->IT_HW;
      ITSettings_DMA_TX = hwConfig[ spi_channel ]->dmaIT_TX;
      ITSettings_DMA_RX = hwConfig[ spi_channel ]->dmaIT_RX;

      /* DMA Control Blocks */
      hdma_spi_tx.Init.Channel = hwConfig[ spi_channel ]->dmaTX.channel;
      hdma_spi_rx.Init.Channel = hwConfig[ spi_channel ]->dmaRX.channel;
      hdma_spi_tx.Instance     = hwConfig[ spi_channel ]->dmaTX.Instance;
      hdma_spi_rx.Instance     = hwConfig[ spi_channel ]->dmaRX.Instance;

      /* Hardware Initialization */
      spi_handle.Instance               = hwConfig[ spi_channel ]->instance;
      spi_handle.Init.BaudRatePrescaler = getPrescaler( spi_channel, setup.clockFrequency );
      spi_handle.Init.CLKPhase          = convertClockPhase( setup.clockMode );
      spi_handle.Init.CLKPolarity       = convertClockPolarity( setup.clockMode );
      spi_handle.Init.DataSize          = convertDataSize( setup.dataSize );
      spi_handle.Init.FirstBit          = convertBitOrder( setup.bitOrder );
      spi_handle.Init.Mode              = convertMode( setup.mode );

      SPI_Init();
      setPeripheralMode( SubPeripheral::TX, SubPeripheralMode::BLOCKING );
      setPeripheralMode( SubPeripheral::RX, SubPeripheralMode::BLOCKING );

      /*------------------------------------------------
      Figure out what DMA signals we are using
      ------------------------------------------------*/
      dmaTXReqSig = Thor::SPI::DMATXRequestSignal[ spi_channel ];
      dmaRXReqSig = Thor::SPI::DMARXRequestSignal[ spi_channel ];

      /*------------------------------------------------
      Register this instance so we can figure out ISR information at runtime
      ------------------------------------------------*/
      spiObjects[ spi_channel ] = this;
    }

    return error;
  }

  Chimera::Status_t SPIClass::deInit()
  {
    using namespace Chimera::SPI;
    using namespace Chimera::Hardware;

    SPI_GPIO_DeInit();
    SPI_DisableInterrupts();
    SPI_DMA_DeInit( SubPeripheral::TX );
    SPI_DMA_DeInit( SubPeripheral::RX );

    txMode = SubPeripheralMode::UNKNOWN_MODE;
    rxMode = SubPeripheralMode::UNKNOWN_MODE;

    spiObjects[ spi_channel ] = nullptr;

    return Chimera::CommonStatusCodes::OK;
  }


  Chimera::SPI::Setup SPIClass::getInit()
  {
    return cachedSetup;
  }

  Chimera::Status_t SPIClass::setChipSelect( const Chimera::GPIO::State value )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !periphStatus.gpio_initialized )
    {
      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else
    {
      if ( !hardwareChipSelect )
      {
        error = CS->setState( value );
      }
      else
      {
        /**
         *  This is only useful when the peripheral's dedicated NSS pin is used,
         *  configured for hardware management, and the SSOE bit in CR1 is set. (By
         *  default this is how Thor is set up) In this configuration, the NSS pin is
         *  automatically driven low upon transmission start and will STAY low until
         *  the SPI hardware is disabled. Rather quirky...
         *
         *  Note:   Don't bother trying to use software NSS management as described
         *          in the datasheet. It will cause a nasty mode fault if you are in
         *          master mode.
         */
        if ( static_cast<bool>( value ) )
        {
          __HAL_SPI_DISABLE( &spi_handle );
        }
      }
    }

    return error;
  }

  Chimera::Status_t SPIClass::setChipSelectControlMode( const Chimera::SPI::ChipSelectMode mode )
  {
    chipSelectMode = mode;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t SPIClass::writeBytes( const uint8_t *const txBuffer, const size_t length, const uint32_t timeoutMS )
  {
    return readWriteBytes( txBuffer, nullptr, length, timeoutMS );
  }

  Chimera::Status_t SPIClass::readBytes( uint8_t *const rxBuffer, const size_t length, const uint32_t timeoutMS )
  {
    return readWriteBytes( nullptr, rxBuffer, length, timeoutMS );
  }

  Chimera::Status_t SPIClass::readWriteBytes( const uint8_t *const txBuffer, uint8_t *const rxBuffer, const size_t length,
                                              const uint32_t timeoutMS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !periphStatus.gpio_initialized || !periphStatus.spi_initialized )
    {
      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else if ( !length || ( !txBuffer && !rxBuffer ) )
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      switch ( txMode )
      {
        case Chimera::Hardware::SubPeripheralMode::BLOCKING:
          error = transfer_blocking( txBuffer, rxBuffer, length, timeoutMS );
          break;

        case Chimera::Hardware::SubPeripheralMode::INTERRUPT:
          error = transfer_interrupt( txBuffer, rxBuffer, length, timeoutMS );
          break;

        case Chimera::Hardware::SubPeripheralMode::DMA:
          error = transfer_dma( txBuffer, rxBuffer, length, timeoutMS );
          break;

        default:
          error = Chimera::CommonStatusCodes::FAIL;
          break;
      }
    }

    return error;
  }

  Chimera::Status_t SPIClass::setPeripheralMode( const Chimera::Hardware::SubPeripheral periph,
                                                 const Chimera::Hardware::SubPeripheralMode mode )
  {
    using namespace Chimera::SPI;
    using namespace Chimera::Hardware;

    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !periphStatus.spi_initialized )
    {
      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else
    {
      if ( ( periph == SubPeripheral::TXRX ) || ( periph == SubPeripheral::TX ) )
      {
        switch ( mode )
        {
          case SubPeripheralMode::BLOCKING:
            txMode = mode;

            /*------------------------------------------------
            Disable interrupts as both periphs are now in blocking mode
            -------------------------------------------------*/
            if ( rxMode == SubPeripheralMode::BLOCKING )
            {
              SPI_DisableInterrupts();
            }

            SPI_DMA_DeInit( SubPeripheral::TX );
            break;

          case SubPeripheralMode::INTERRUPT:
            txMode = mode;

            SPI_EnableInterrupts();
            SPI_DMA_DeInit( SubPeripheral::TX );
            break;

          case SubPeripheralMode::DMA:
            txMode = mode;

            SPI_EnableInterrupts();
            SPI_DMA_Init( SubPeripheral::TX );
            break;

          default:
            txMode = SubPeripheralMode::UNKNOWN_MODE;
            error  = Chimera::CommonStatusCodes::FAIL;
            break;
        }
      }

      if ( ( periph == SubPeripheral::TXRX ) || ( periph == SubPeripheral::RX ) )
      {
        switch ( mode )
        {
          case SubPeripheralMode::BLOCKING:
            rxMode = mode;

            /*------------------------------------------------
            Disable interrupts as both periphs are now in blocking mode
            -------------------------------------------------*/
            if ( txMode == SubPeripheralMode::BLOCKING )
            {
              SPI_DisableInterrupts();
            }

            SPI_DMA_DeInit( SubPeripheral::RX );
            break;

          case SubPeripheralMode::INTERRUPT:
            rxMode = mode;

            SPI_EnableInterrupts();
            SPI_DMA_DeInit( SubPeripheral::RX );
            break;

          case SubPeripheralMode::DMA:
            rxMode = mode;

            SPI_EnableInterrupts();
            SPI_DMA_Init( SubPeripheral::RX );
            break;

          default:
            rxMode = SubPeripheralMode::UNKNOWN_MODE;
            error  = Chimera::CommonStatusCodes::FAIL;
            break;
        }
      }
    }

    return error;
  }

  Chimera::Status_t SPIClass::setClockFrequency( const uint32_t freq, const uint32_t tolerance )
  {
    // TODO
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::getClockFrequency( uint32_t &freq )
  {
    freq = getFrequency( spi_channel, spi_handle.Init.BaudRatePrescaler );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t SPIClass::attachNotifier( const Chimera::Event::Trigger event, volatile uint8_t *const notifier )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::detachNotifier( const Chimera::Event::Trigger event, volatile uint8_t *const notifier )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::attachCallback( const Chimera::Event::Trigger trigger,
                                              const Chimera::Function::void_func_void func )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::detachCallback( const Chimera::Event::Trigger trigger )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

#if defined( USING_FREERTOS )
  Chimera::Status_t SPIClass::attachNotifier( const Chimera::Event::Trigger event, SemaphoreHandle_t *const semphr )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::detachNotifier( const Chimera::Event::Trigger event, SemaphoreHandle_t *const semphr )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
#endif

  Chimera::Status_t SPIClass::transfer_blocking( const uint8_t *const txBuffer, uint8_t *const rxBuffer, const size_t length,
                                                 const uint32_t timeoutMS )
  {
    using namespace Chimera::SPI;
    using namespace Chimera::GPIO;
    using namespace Chimera::Hardware;

    HAL_StatusTypeDef stm32Error = HAL_BUSY;
    Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

    if ( lock( timeoutMS ) == Chimera::CommonStatusCodes::OK )
    {
      /*------------------------------------------------
      Activate the chip select line?
      -------------------------------------------------*/
      if ( chipSelectMode != Chimera::SPI::ChipSelectMode::MANUAL )
      {
        setChipSelect( State::LOW );
      }

      /*------------------------------------------------
      Execute the transfer. Nullptr is protected by readWriteBytes()
      -------------------------------------------------*/
      if ( txBuffer && !rxBuffer )
      {
        stm32Error = HAL_SPI_Transmit( &spi_handle, const_cast<uint8_t *>( txBuffer ), length, timeoutMS );
      }
      else if ( rxBuffer && !txBuffer )
      {
        if ( rxMode != txMode )
        {
          /*------------------------------------------------
          In order for TransmitReceive to work, both subperipherals must be in the
          same mode. This will silently clobber previous settings.
          -------------------------------------------------*/
          setPeripheralMode( SubPeripheral::RX, txMode );
        }

        stm32Error = HAL_SPI_TransmitReceive( &spi_handle, const_cast<uint8_t *>( rxBuffer ), const_cast<uint8_t *>( rxBuffer ),
                                              length, timeoutMS );
      }
      else
      {
        if ( rxMode != txMode )
        {
          /*------------------------------------------------
          In order for TransmitReceive to work, both subperipherals must be in the
          same mode. This will silently clobber previous settings.
          -------------------------------------------------*/
          setPeripheralMode( SubPeripheral::RX, txMode );
        }

        stm32Error = HAL_SPI_TransmitReceive( &spi_handle, const_cast<uint8_t *>( txBuffer ), const_cast<uint8_t *>( rxBuffer ),
                                              length, timeoutMS );
      }

      /*------------------------------------------------
      De-activate the chip select line?
      -------------------------------------------------*/
      if ( chipSelectMode != Chimera::SPI::ChipSelectMode::MANUAL )
      {
        setChipSelect( State::HIGH );
      }

      /*------------------------------------------------
      Handle any errors and exit gracefully
      -------------------------------------------------*/
      if ( error != HAL_OK )
      {
        error = convertHALStatus( stm32Error );
      }

      unlock();
    }
    else
    {
      error = Chimera::CommonStatusCodes::LOCKED;
    }

    return error;
  }

  Chimera::Status_t SPIClass::transfer_interrupt( const uint8_t *const txBuffer, uint8_t *const rxBuffer, const size_t length,
                                                  const uint32_t timeoutMS )
  {
    using namespace Chimera::SPI;
    using namespace Chimera::GPIO;
    using namespace Chimera::Hardware;

    HAL_StatusTypeDef stm32Error = HAL_OK;
    Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

    if ( transfer_complete && ( lock( timeoutMS ) == Chimera::CommonStatusCodes::OK ) )
    {
      /*------------------------------------------------
      Activate the chip select line?
      -------------------------------------------------*/
      if ( chipSelectMode != Chimera::SPI::ChipSelectMode::MANUAL )
      {
        setChipSelect( State::LOW );
      }

      /*------------------------------------------------
      Execute the transfer. Apologies for the const_cast-ing.
      The underlying HAL does not modify the buffers.
      -------------------------------------------------*/
      if ( txBuffer && !rxBuffer )
      {
        stm32Error = HAL_SPI_Transmit_IT( &spi_handle, const_cast<uint8_t *>( txBuffer ), length );
      }
      else if ( rxBuffer && !txBuffer )
      {
        /* Technically this would be an Asynchronous Slave Mode operation */
        error = Chimera::CommonStatusCodes::NOT_SUPPORTED;
      }
      else
      {
        if ( rxMode != txMode )
        {
          /*------------------------------------------------
          In order for TransmitReceive to work, both subperipherals must be in the
          same mode. This will silently clobber previous settings.
          -------------------------------------------------*/
          setPeripheralMode( SubPeripheral::RX, txMode );
        }

        stm32Error = HAL_SPI_TransmitReceive_IT( &spi_handle, const_cast<uint8_t *>( txBuffer ),
                                                 const_cast<uint8_t *>( rxBuffer ), length );
      }

      /*------------------------------------------------
      Catch any errors and exit gracefully
      -------------------------------------------------*/
      if ( stm32Error != HAL_OK )
      {
        /*------------------------------------------------
        The ISR will no longer reset the chipselect, so it is handled here
        -------------------------------------------------*/
        if ( chipSelectMode != Chimera::SPI::ChipSelectMode::MANUAL )
        {
          setChipSelect( State::HIGH );
        }
      }

      unlock();
    }
    else
    {
      error = Chimera::CommonStatusCodes::LOCKED;
    }

    return error;
  }

  Chimera::Status_t SPIClass::transfer_dma( const uint8_t *const txBuffer, uint8_t *const rxBuffer, const size_t length,
                                            const uint32_t timeoutMS )
  {
    using namespace Chimera::SPI;
    using namespace Chimera::GPIO;
    using namespace Chimera::Hardware;

    HAL_StatusTypeDef stm32Error = HAL_OK;
    Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

    if ( transfer_complete && ( lock( timeoutMS ) == Chimera::CommonStatusCodes::OK ) )
    {
      /*------------------------------------------------
      Activate the chip select line?
      -------------------------------------------------*/
      if ( chipSelectMode != Chimera::SPI::ChipSelectMode::MANUAL )
      {
        setChipSelect( State::LOW );
      }

      if ( txBuffer && !rxBuffer )
      {
        stm32Error = HAL_SPI_Transmit_DMA( &spi_handle, const_cast<uint8_t *>( txBuffer ), length );
      }
      else if ( rxBuffer && !txBuffer )
      {
        /* Technically this would be an Asynchronous Slave Mode operation */
        error = Chimera::CommonStatusCodes::NOT_SUPPORTED;
      }
      else
      {
        if ( rxMode != txMode )
        {
          /*------------------------------------------------
          In order for TransmitReceive to work, both subperipherals must be in the
          same mode. This will silently clobber previous settings.
          -------------------------------------------------*/
          setPeripheralMode( SubPeripheral::RX, txMode );
        }

        stm32Error = HAL_SPI_TransmitReceive_DMA( &spi_handle, const_cast<uint8_t *>( txBuffer ),
                                                  const_cast<uint8_t *>( rxBuffer ), length );
      }

      /*------------------------------------------------
      Catch any errors and exit gracefully
      -------------------------------------------------*/
      if ( stm32Error != HAL_OK )
      {
        /*------------------------------------------------
        The ISR will no longer reset the chipselect, so it is handled here
        -------------------------------------------------*/
        if ( chipSelectMode != Chimera::SPI::ChipSelectMode::MANUAL )
        {
          setChipSelect( State::HIGH );
        }
      }

      unlock();
    }
    else
    {
      error = Chimera::CommonStatusCodes::LOCKED;
    }

    return error;
  }

  void SPIClass::IRQHandler()
  {
    HAL_SPI_IRQHandler( &spi_handle );
  }

  void SPIClass::IRQHandler_TXDMA()
  {
    HAL_DMA_IRQHandler( spi_handle.hdmatx );
  }

  void SPIClass::IRQHandler_RXDMA()
  {
    HAL_DMA_IRQHandler( spi_handle.hdmarx );
  }

  uint32_t SPIClass::getPrescaler( const int &channel, const uint32_t &freq )
  {
    static constexpr uint8_t numPrescalers = 8;

    uint32_t apbFreq   = 0u;
    uint32_t prescaler = 0u;

    /*------------------------------------------------
    Figure out the APB bus frequency for this channel
    -------------------------------------------------*/
    if ( hwConfig[ channel ]->clockBus == Thor::CLK::ClockBus::APB1_PERIPH )
    {
      apbFreq = HAL_RCC_GetPCLK1Freq();
    }
    else if ( hwConfig[ channel ]->clockBus == Thor::CLK::ClockBus::APB2_PERIPH )
    {
      apbFreq = HAL_RCC_GetPCLK2Freq();
    }

    /*------------------------------------------------
    Calculate the prescaler if the APB bus frequency was found
    -------------------------------------------------*/
    if ( apbFreq )
    {
      /*------------------------------------------------
      Calculate the error between the pre-scaled clock and the desired clock
      -------------------------------------------------*/
      int clockError[ numPrescalers ];
      memset( clockError, std::numeric_limits<int>::max(), numPrescalers );

      for ( int i = 0; i < numPrescalers; i++ )
      {
        clockError[ i ] = abs( static_cast<int>( ( apbFreq / ( 1 << ( i + 1 ) ) - freq ) ) );
      }

      /*------------------------------------------------
      Find the index of the element with lowest error
      -------------------------------------------------*/
      auto idx = std::distance( clockError, std::min_element( clockError, clockError + numPrescalers - 1 ) );

      /*------------------------------------------------
      Grab the correct prescaler
      -------------------------------------------------*/
      switch ( idx )
      {
        case 0:
          prescaler = SPI_BAUDRATEPRESCALER_2;
          break;

        case 1:
          prescaler = SPI_BAUDRATEPRESCALER_4;
          break;

        case 2:
          prescaler = SPI_BAUDRATEPRESCALER_8;
          break;

        case 3:
          prescaler = SPI_BAUDRATEPRESCALER_16;
          break;

        case 4:
          prescaler = SPI_BAUDRATEPRESCALER_32;
          break;

        case 5:
          prescaler = SPI_BAUDRATEPRESCALER_64;
          break;

        case 6:
          prescaler = SPI_BAUDRATEPRESCALER_128;
          break;

        case 7:
          prescaler = SPI_BAUDRATEPRESCALER_256;
          break;

        default:
          prescaler = Thor::SPI::dflt_SPI_Init.BaudRatePrescaler;
          break;
      };
    }
    else
    {
      prescaler = Thor::SPI::dflt_SPI_Init.BaudRatePrescaler;
    }

    return prescaler;
  }

  uint32_t SPIClass::getFrequency( const int &channel, const uint32_t &prescaler )
  {
    uint32_t apbFreq = 0u;
    uint32_t busFreq = 0u;

    /*------------------------------------------------
    Figure out the APB bus frequency for this channel
    -------------------------------------------------*/
    if ( hwConfig[ channel ]->clockBus == Thor::CLK::ClockBus::APB1_PERIPH )
    {
      apbFreq = HAL_RCC_GetPCLK1Freq();
    }
    else if ( hwConfig[ channel ]->clockBus == Thor::CLK::ClockBus::APB2_PERIPH )
    {
      apbFreq = HAL_RCC_GetPCLK2Freq();
    }

    /*------------------------------------------------
    Now get the SPI clock frequency
    -------------------------------------------------*/
    if ( apbFreq )
    {
      switch ( prescaler )
      {
        case SPI_BAUDRATEPRESCALER_2:
          busFreq = apbFreq / 2;
          break;

        case SPI_BAUDRATEPRESCALER_4:
          busFreq = apbFreq / 4;
          break;

        case SPI_BAUDRATEPRESCALER_8:
          busFreq = apbFreq / 8;
          break;

        case SPI_BAUDRATEPRESCALER_16:
          busFreq = apbFreq / 16;
          break;

        case SPI_BAUDRATEPRESCALER_32:
          busFreq = apbFreq / 32;
          break;

        case SPI_BAUDRATEPRESCALER_64:
          busFreq = apbFreq / 64;
          break;

        case SPI_BAUDRATEPRESCALER_128:
          busFreq = apbFreq / 128;
          break;

        case SPI_BAUDRATEPRESCALER_256:
          busFreq = apbFreq / 256;
          break;

        default:
          busFreq = 0u;
      };
    }

    return busFreq;
  }

  void SPIClass::SPI_GPIO_Init()
  {
    using namespace Chimera::GPIO;

    if ( spi_handle.Init.Mode == SPI_MODE_MASTER )
    {
      /*------------------------------------------------
      Setup MISO/MOSI/SCK. Mode must be ALT_PP rather
      than Input in order for reads to work.
      -------------------------------------------------*/
      MISO->setMode( Drive::ALTERNATE_PUSH_PULL, true );
      MOSI->setMode( Drive::ALTERNATE_PUSH_PULL, true );

      if ( spi_handle.Init.CLKPolarity )
      {
        SCK->setMode( Drive::ALTERNATE_PUSH_PULL, true );
      }
      else
      {
        SCK->setMode( Drive::ALTERNATE_PUSH_PULL, false );
      }

      /*------------------------------------------------
      Decide how to initialize the CS pin. There are a few options.
      -------------------------------------------------*/
      if ( !hardwareChipSelect )
      {
        /* User controlled pin */
        CS->setMode( Drive::OUTPUT_PUSH_PULL, false );
        CS->setState( State::HIGH );
      }
      else
      {
        /* Peripheral controlled pin */
        CS->setMode( Drive::ALTERNATE_PUSH_PULL, true );
        CS->setState( State::HIGH );
      }
    }
    else
    {
      MISO->setMode( Drive::ALTERNATE_PUSH_PULL, true );
      MOSI->setMode( Drive::ALTERNATE_OPEN_DRAIN, true );
      SCK->setMode( Drive::ALTERNATE_OPEN_DRAIN, true );
      CS->setMode( Drive::ALTERNATE_OPEN_DRAIN, true );
    }

    periphStatus.gpio_initialized = true;
  }

  void SPIClass::SPI_GPIO_DeInit()
  {
    SCK.reset( nullptr );
    MOSI.reset( nullptr );
    MISO.reset( nullptr );
    CS.reset( nullptr );
  }

  void SPIClass::SPI_Init()
  {
    HAL_StatusTypeDef initResult;
    
    SPI_EnableClock();
    initResult = HAL_SPI_Init( &spi_handle );

    assert( initResult == HAL_OK );

    periphStatus.spi_initialized = true;
  }

  void SPIClass::SPI_DeInit()
  {
    HAL_StatusTypeDef deInitResult;
    deInitResult = HAL_SPI_DeInit( &spi_handle );
    SPI_DisableClock();
    
    assert( deInitResult == HAL_OK );

    periphStatus.spi_initialized = false;
  }

  void SPIClass::SPI_EnableClock()
  {
    *spiClockRegister( spi_handle.Instance ) |= spiClockMask( spi_handle.Instance );
  }

  void SPIClass::SPI_DisableClock()
  {
    *spiClockRegister( spi_handle.Instance ) &= ~spiClockMask( spi_handle.Instance );
  }

  void SPIClass::SPI_DMA_EnableClock()
  {
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
  }

  void SPIClass::SPI_EnableInterrupts()
  {
    HAL_NVIC_DisableIRQ( ITSettingsHW.IRQn );
    HAL_NVIC_SetPriority( ITSettingsHW.IRQn, ITSettingsHW.preemptPriority, ITSettingsHW.subPriority );
    HAL_NVIC_EnableIRQ( ITSettingsHW.IRQn );

    if ( rxMode == Chimera::Hardware::SubPeripheralMode::INTERRUPT )
    {
      __HAL_SPI_ENABLE_IT( &spi_handle, SPI_IT_RXNE );
    }

    periphStatus.spi_interrupts_enabled = true;
  }

  void SPIClass::SPI_DisableInterrupts()
  {
    __HAL_SPI_DISABLE_IT( &spi_handle, SPI_IT_RXNE );

    HAL_NVIC_ClearPendingIRQ( ITSettingsHW.IRQn );
    HAL_NVIC_DisableIRQ( ITSettingsHW.IRQn );

    periphStatus.spi_interrupts_enabled = false;
  }

  void SPIClass::SPI_DMA_Init( const Chimera::Hardware::SubPeripheral periph )
  {
    HAL_StatusTypeDef result = HAL_ERROR;
    
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      /*------------------------------------------------
      Perform all the low level initialization
      ------------------------------------------------*/
      SPI_DMA_EnableClock();
      result = HAL_DMA_Init( &hdma_spi_tx );
      assert( result == HAL_OK );
      __HAL_LINKDMA( &spi_handle, hdmatx, hdma_spi_tx );

      /*------------------------------------------------
      Bind the class specific DMA interrupt handler to the
      hardware peripheral DMA TX request signal.
      ------------------------------------------------*/
      if ( dmaTXReqSig != Thor::DMA::Source::NONE )
      {
        Thor::DMA::requestHandlers[ dmaTXReqSig ] = boost::bind( &SPIClass::IRQHandler_TXDMA, this );
      }

      /*------------------------------------------------
      Now that the handler has been bound, we are ready to consume interrupts
      ------------------------------------------------*/
      SPI_DMA_EnableInterrupts( periph );
      periphStatus.dma_enabled_tx = true;
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      /*------------------------------------------------
      Perform all the low level initialization
      ------------------------------------------------*/
      SPI_DMA_EnableClock();
      result = HAL_DMA_Init( &hdma_spi_rx );
      assert( result == HAL_OK );
      __HAL_LINKDMA( &spi_handle, hdmarx, hdma_spi_rx );

      /*------------------------------------------------
      Bind the class specific DMA interrupt handler to the
      hardware peripheral DMA RX request signal.
      ------------------------------------------------*/
      if ( dmaRXReqSig != Thor::DMA::Source::NONE )
      {
        Thor::DMA::requestHandlers[ dmaRXReqSig ] = boost::bind( &SPIClass::IRQHandler_RXDMA, this );
      }

      /*------------------------------------------------
      Now that the handler has been bound, we are ready to consume interrupts
      ------------------------------------------------*/
      SPI_DMA_EnableInterrupts( periph );
      periphStatus.dma_enabled_rx = true;
    }
  }

  void SPIClass::SPI_DMA_DeInit( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      if ( !periphStatus.dma_enabled_tx )
      {
        return;
      }

      HAL_DMA_Abort( spi_handle.hdmatx );
      HAL_DMA_DeInit( spi_handle.hdmatx );
      SPI_DMA_DisableInterrupts( periph );
      Thor::DMA::requestHandlers[ dmaTXReqSig ].clear();

      periphStatus.dma_enabled_tx = false;
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      if ( !periphStatus.dma_enabled_rx )
      {
        return;
      }

      HAL_DMA_Abort( spi_handle.hdmarx );
      HAL_DMA_DeInit( spi_handle.hdmarx );
      SPI_DMA_DisableInterrupts( periph );
      Thor::DMA::requestHandlers[ dmaRXReqSig ].clear();

      periphStatus.dma_enabled_rx = false;
    }
  }

  void SPIClass::SPI_DMA_EnableInterrupts( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      HAL_NVIC_DisableIRQ( ITSettings_DMA_TX.IRQn );
      HAL_NVIC_ClearPendingIRQ( ITSettings_DMA_TX.IRQn );
      HAL_NVIC_SetPriority( ITSettings_DMA_TX.IRQn, ITSettings_DMA_TX.preemptPriority, ITSettings_DMA_TX.subPriority );
      HAL_NVIC_EnableIRQ( ITSettings_DMA_TX.IRQn );

      periphStatus.dma_interrupts_enabled_tx = true;
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      HAL_NVIC_DisableIRQ( ITSettings_DMA_RX.IRQn );
      HAL_NVIC_ClearPendingIRQ( ITSettings_DMA_RX.IRQn );
      HAL_NVIC_SetPriority( ITSettings_DMA_RX.IRQn, ITSettings_DMA_RX.preemptPriority, ITSettings_DMA_RX.subPriority );
      HAL_NVIC_EnableIRQ( ITSettings_DMA_RX.IRQn );

      periphStatus.dma_interrupts_enabled_rx = true;
    }
  }

  void SPIClass::SPI_DMA_DisableInterrupts( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      HAL_NVIC_ClearPendingIRQ( ITSettings_DMA_TX.IRQn );
      HAL_NVIC_DisableIRQ( ITSettings_DMA_TX.IRQn );

      periphStatus.dma_interrupts_enabled_tx = false;
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      HAL_NVIC_ClearPendingIRQ( ITSettings_DMA_RX.IRQn );
      HAL_NVIC_DisableIRQ( ITSettings_DMA_RX.IRQn );

      periphStatus.dma_interrupts_enabled_rx = false;
    }
  }

}    // namespace Thor::SPI

#if !defined( GMOCK_TEST )
void HAL_SPI_TxCpltCallback( SPI_HandleTypeDef *hspi )
{
  using namespace Chimera::GPIO;
  using namespace Chimera::SPI;

  /*------------------------------------------------
  Determine which object actually triggered this callback
  -------------------------------------------------*/
  auto spi = getSPIClassRef( hspi->Instance );

  if ( spi )
  {
    spi->transfer_complete = true;

    if ( spi->chipSelectMode != ChipSelectMode::MANUAL )
    {
      spi->setChipSelect( State::HIGH );
    }

    /*------------------------------------------------
    Notify event occurred
    -------------------------------------------------*/
#if defined( USING_FREERTOS )
    // spiTaskTrigger.logEvent( TX_COMPLETE, &spiTaskTrigger );
#endif
  }
}

void HAL_SPI_RxCpltCallback( SPI_HandleTypeDef *hspi )
{
}

void HAL_SPI_TxRxCpltCallback( SPI_HandleTypeDef *hspi )
{
  using namespace Chimera::GPIO;
  using namespace Chimera::SPI;

  /*------------------------------------------------
  Determine which object actually triggered this callback
  -------------------------------------------------*/
  auto spi = getSPIClassRef( hspi->Instance );

  if ( spi )
  {
    spi->transfer_complete = true;

    if ( spi->chipSelectMode != ChipSelectMode::MANUAL )
    {
      spi->setChipSelect( State::HIGH );
    }

    /*------------------------------------------------
    Notify event occurred
    -------------------------------------------------*/
#if defined( USING_FREERTOS )
    // spiTaskTrigger.logEvent( TX_COMPLETE, &spiTaskTrigger );
#endif
  }
}

void HAL_SPI_TxHalfCpltCallback( SPI_HandleTypeDef *hspi )
{
}

void HAL_SPI_RxHalfCpltCallback( SPI_HandleTypeDef *hspi )
{
}

void HAL_SPI_TxRxHalfCpltCallback( SPI_HandleTypeDef *hspi )
{
}

void HAL_SPI_ErrorCallback( SPI_HandleTypeDef *hspi )
{
}
#endif /* !GMOCK_TEST */

void SPI1_IRQHandler()
{
  if ( spiObjects[ 1 ] )
  {
    spiObjects[ 1 ]->IRQHandler();
  }
}

void SPI2_IRQHandler()
{
  if ( spiObjects[ 2 ] )
  {
    spiObjects[ 2 ]->IRQHandler();
  }
}

void SPI3_IRQHandler()
{
  if ( spiObjects[ 3 ] )
  {
    spiObjects[ 3 ]->IRQHandler();
  }
}

void SPI4_IRQHandler()
{
  if ( spiObjects[ 4 ] )
  {
    spiObjects[ 4 ]->IRQHandler();
  }
}

void SPI5_IRQHandler()
{
  if ( spiObjects[ 5 ] )
  {
    spiObjects[ 5 ]->IRQHandler();
  }
}

void SPI6_IRQHandler()
{
  if ( spiObjects[ 6 ] )
  {
    spiObjects[ 6 ]->IRQHandler();
  }
}
