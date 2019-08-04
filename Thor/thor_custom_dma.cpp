/********************************************************************************
 * File Name:
 *   thor_custom_dma.cpp
 *
 * Description:
 *   Implements DMA for Thor using the custom low level drivers.
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/threading.hpp>

/* Thor Includes */
#include <Thor/thor.hpp>
#include <Thor/dma.hpp>
#include <Thor/drivers/dma.hpp>

namespace DMADriver = Thor::Driver::DMA;

/*------------------------------------------------
Static Functions
------------------------------------------------*/
static void USART1ISRPostProcessorThread( void *argument );
static void USART2ISRPostProcessorThread( void *argument );
static void USART3ISRPostProcessorThread( void *argument );
static void USART6ISRPostProcessorThread( void *argument );


static std::array<DMADriver::Driver *, DMADriver::NUM_DMA_PERIPHS> dmaPeriphs;

static std::shared_ptr<Thor::DMA::DMAClass> dmaSingleton = nullptr;

namespace Thor::DMA
{
  DMAClass::DMAClass()
  {
  }

  DMAClass::~DMAClass()
  {
  }

  std::shared_ptr<DMAClass> DMAClass::get()
  {
    /* Gets around the private constructor issue with shared_ptr */
    struct A : public DMAClass{};

    if ( !dmaSingleton )
    {
      dmaSingleton = std::make_shared<A>();
    }

    return dmaSingleton;
  }

  Chimera::Status_t DMAClass::init()
  {
    /*------------------------------------------------
    Ensure an instance of DMA peripherals are created
    and initialized properly.
    ------------------------------------------------*/
    for ( uint8_t x = 0; x < dmaPeriphs.size(); x++ )
    {
      if ( !dmaPeriphs[ x ] )
      {
        dmaPeriphs[ x ] = new DMADriver::Driver();
        dmaPeriphs[ x ]->attach( DMADriver::periphInstanceList[ x ] );
      }

      if ( dmaPeriphs[ x ]->init() != Chimera::CommonStatusCodes::OK )
      {
        return Chimera::CommonStatusCodes::FAIL;
      }
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t DMAClass::reset()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DMAClass::start( const Chimera::DMA::Init &config, const Chimera::DMA::TCB &transfer, const size_t timeout,
                                     Chimera::DMA::TransferHandle_t *const handle )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DMAClass::abort( Chimera::DMA::TransferHandle_t handle, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DMAClass::status( Chimera::DMA::TransferHandle_t handle, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DMAClass::registerListener( Chimera::Event::Actionable &listener, const size_t timeout, size_t &registrationID )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DMAClass::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

}    // namespace Thor::DMA
