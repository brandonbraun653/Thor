/********************************************************************************
 *  File Name:
 *    hld_dma_chimera.cpp
 *
 *  Description:
 *    Implementation of Chimera DMA driver registration hooks
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

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
    return Thor::DMA::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::DMA::reset();
  }


  RequestId constructPipe( const PipeConfig &config )
  {
    return Thor::DMA::constructPipe( config );
  }


  RequestId transfer( const MemTransfer &transfer )
  {
    return Thor::DMA::transfer( transfer );
  }


  RequestId transfer( const PipeTransfer &transfer )
  {
    return Thor::DMA::transfer( transfer );
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
