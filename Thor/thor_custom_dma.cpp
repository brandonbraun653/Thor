/********************************************************************************
 * File Name:
 *   thor_custom_dma.cpp
 *
 * Description:
 *   Implements DMA for Thor using the custom low level drivers.
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/thor.hpp>
#include <Thor/dma.hpp>

namespace Thor::DMA
{
  DMAClass::DMAClass()
  {

  }

  DMAClass::~DMAClass()
  {
  }

  Chimera::Status_t DMAClass::init( const Chimera::DMA::Init &config, const size_t timeout,
                                    Chimera::DMA::TransferHandle_t handle )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DMAClass::start( Chimera::DMA::TransferHandle_t handle, const Chimera::DMA::TCB &transfer,
                                     const size_t timeout )
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
}    // namespace Thor
