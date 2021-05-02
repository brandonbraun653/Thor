/********************************************************************************
 *  File Name:
 *    hld_gpio_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera DMA driver hooks
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/dma>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/dma>

namespace Chimera::DMA::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }


  RequestId constructPipe( const PipeConfig &config )
  {
    return INVALID_REQUEST;
  }


  RequestId transfer( const MemTransfer &transfer )
  {
    return INVALID_REQUEST;
  }


  RequestId transfer( const PipeTransfer &transfer )
  {
    return INVALID_REQUEST;
  }

  Chimera::Status_t registerDriver( Chimera::DMA::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_DMA )
    registry.isSupported   = true;
    registry.initialize    = initialize;
    registry.reset         = reset;
    registry.constructPipe = constructPipe;
    registry.memTransfer   = transfer;
    registry.pipeTransfer  = transfer;

    return Chimera::Status::OK;
#else
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif /* THOR_HLD_DMA */
  }
}    // namespace Chimera::DMA::Backend