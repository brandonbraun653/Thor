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

static DMADriver::Driver *currentDMAInstance = nullptr;
static DMADriver::StreamX *currentStream = nullptr;

namespace Thor::DMA
{
  static DMADriver::Driver *const getDriver( const Chimera::DMA::Init &config );
  static DMADriver::StreamX *const getStream( const Chimera::DMA::Init &config );
  static DMADriver::StreamConfig convertConfig( const Chimera::DMA::Init &config );
  static DMADriver::TCB convertTCB( const Chimera::DMA::TCB &transfer );

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

  Chimera::Status_t DMAClass::start()
  {
    if ( currentDMAInstance && currentStream ) 
    {
      return currentDMAInstance->start( currentStream );
    }

    return Chimera::CommonStatusCodes::FAIL;
  }

  Chimera::Status_t DMAClass::configure( const Chimera::DMA::Init &config, const Chimera::DMA::TCB &transfer,
                                         const size_t timeout, Chimera::DMA::TransferHandle_t *const handle )
  {
    auto result = Chimera::CommonStatusCodes::FAIL;

    auto dma = getDriver( config );
    auto stream = getStream( config );
    auto cfg = convertConfig( config );
    auto tcb = convertTCB( transfer );

    // Manually set the stream and dma driver just for testing purposes.

    currentDMAInstance = dmaPeriphs[ 0 ];
    currentStream = DMADriver::DMA1_STREAM3;

    if ( currentDMAInstance ) 
    {
      result = currentDMAInstance->configure( currentStream, &cfg, &tcb );
    }

    return result;
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

  static DMADriver::Driver *const getDriver( const Chimera::DMA::Init &config )
  {
    // Figure out the DMA peripheral instance (driver) from the source
  }

  static DMADriver::StreamX *const getStream( const Chimera::DMA::Init &config )
  {
    // Figure out the stream instance from the source
  }

  static DMADriver::StreamConfig convertConfig( const Chimera::DMA::Init &config )
  {
    using namespace DMADriver;

    StreamConfig scfg;

    scfg.Channel             = Configuration::ChannelSelect::Channel4;
    scfg.Direction           = TransferMap[ static_cast<size_t>( config.direction ) ];
    scfg.MemBurst            = Configuration::MemoryBurst::Single;
    scfg.MemDataAlignment    = MemoryAlignmentMap[ static_cast<size_t>( config.mAlign ) ];
    scfg.MemInc              = MemoryIncrementMap[ static_cast<size_t>( config.mInc ) ];
    scfg.Mode                = ModeMap[ static_cast<size_t>( config.mode ) ];
    scfg.PeriphBurst         = Configuration::PeriphBurst::Single;
    scfg.PeriphInc           = PeripheralIncrementMap[ static_cast<size_t>( config.pInc ) ];
    scfg.PeriphDataAlignment = PeripheralAlignmentMap[ static_cast<size_t>( config.pAlign ) ];
    scfg.Priority            = PriorityMap[ static_cast<uint32_t>( config.priority ) ];

    // TODO: These two are a bit of a weirdo...how to abstract this higher?
    scfg.FIFOMode            = Configuration::FIFODirectMode::Enabled;
    scfg.FIFOThreshold       = Configuration::FIFOThreshold::Threshold_4_4;

    return scfg;
  }

  static DMADriver::TCB convertTCB( const Chimera::DMA::TCB &transfer )
  {
    using namespace DMADriver;

    TCB tcb;
    tcb.srcAddress = transfer.srcAddress;
    tcb.dstAddress = transfer.dstAddress;
    tcb.transferSize = transfer.transferSize;

    return tcb;
  }
}    // namespace Thor::DMA