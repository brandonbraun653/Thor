/********************************************************************************
 *  File Name:
 *    hld_gpio_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera DMA driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
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
  Chimera::Status_t initialize()
  {
    return Thor::DMA::initialize();
  }

  Chimera::Status_t reset()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::DMA::DMA_sPtr create_shared_ptr()
  {
    return Thor::DMA::DMAClass::get();
  }

  Chimera::DMA::DMA_uPtr create_unique_ptr()
  {
    // This is not allowed because Thor DMA is a singleton
    return nullptr;
  }

//   Chimera::Status_t registerDriver( Chimera::DMA::Backend::DriverConfig &registry )
//   {
// #if defined( THOR_HLD_DMA )
//     registry.isSupported  = true;
//     registry.createShared = create_shared_ptr;
//     registry.createUnique = create_unique_ptr;
//     registry.initialize   = initialize;
//     registry.reset        = reset;
//     return Chimera::CommonStatusCodes::OK;
// #else
//     registry.isSupported  = false;
//     registry.createShared = nullptr;
//     registry.createUnique = nullptr;
//     registry.initialize   = nullptr;
//     registry.reset        = nullptr;
//     return Chimera::CommonStatusCodes::NOT_SUPPORTED;
// #endif /* THOR_HLD_DMA */
//  }
}    // namespace Chimera::DMA::Backend