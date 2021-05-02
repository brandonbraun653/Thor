/********************************************************************************
 *  File Name:
 *    dma_common_driver.cpp
 *
 *  Description:
 *    Shared driver for DMA across multiple STM32 chips
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/interface/inc/dma>

namespace Thor::LLD::DMA
{

  Chimera::Status_t Driver::clockEnable()
  {
    // auto rcc   = Thor::LLD::RCC::getPeriphClockCtrl();
    // auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    // rcc->enableClock( Chimera::Peripheral::Type::PERIPH_DMA, index );
    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::clockDisable()
  {
    // auto rcc   = Thor::LLD::RCC::getPeriphClockCtrl();
    // auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    // rcc->disableClock( Chimera::Peripheral::Type::PERIPH_DMA, index );
    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::reset()
  {
    // auto rcc   = Thor::LLD::RCC::getPeriphClockCtrl();
    // auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    // rcc->reset( Chimera::Peripheral::Type::PERIPH_DMA, index );
    return Chimera::Status::OK;
  }

}  // namespace Thor::LLD::DMA
